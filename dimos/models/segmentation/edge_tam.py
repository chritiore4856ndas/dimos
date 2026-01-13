# Copyright 2025-2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""EdgeTAM video segmentation model for object tracking."""

from __future__ import annotations

from pathlib import Path
import shutil

import cv2
from hydra.utils import instantiate
import numpy as np
from omegaconf import OmegaConf
from PIL import Image as PILImage
from sam2.sam2_video_predictor import SAM2VideoPredictor  # type: ignore[import-untyped]
import torch
from tqdm import tqdm

from dimos.msgs.sensor_msgs import Image
from dimos.perception.detection.detectors.types import Detector
from dimos.perception.detection.type import ImageDetections2D
from dimos.perception.detection.type.detection2d.seg import Detection2DSeg
from dimos.utils.data import get_data
# Note: Don't use dimos.utils.gpu_utils.is_cuda_available - it uses pycuda, not torch
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


# Monkey patch tqdm to be silent
def silent_tqdm(*args, **kwargs):  # type: ignore[no-untyped-def]
    kwargs["disable"] = True
    return tqdm(*args, **kwargs)


import sam2.sam2_video_predictor  # type: ignore[import-untyped]
import sam2.utils.misc  # type: ignore[import-untyped]

sam2.sam2_video_predictor.tqdm = silent_tqdm
sam2.utils.misc.tqdm = silent_tqdm


def _load_checkpoint(model: SAM2VideoPredictor, ckpt_path: str) -> None:
    if ckpt_path is not None:
        sd = torch.load(ckpt_path, map_location="cpu", weights_only=True)["model"]
        missing_keys, unexpected_keys = model.load_state_dict(sd)
        if missing_keys:
            logger.error(f"Missing keys: {missing_keys}")
            raise RuntimeError("Missing keys in checkpoint")
        if unexpected_keys:
            logger.error(f"Unexpected keys: {unexpected_keys}")
            raise RuntimeError("Unexpected keys in checkpoint")
        logger.info("Loaded checkpoint successfully")


class EdgeTAMProcessor(Detector):
    """EdgeTAM video object segmentation processor.

    Uses SAM2 architecture with EdgeTAM weights for efficient video
    object tracking and segmentation.
    """

    def __init__(
        self,
        model_path: str = "models_edgetam",
        model_name: str = "edgetam.pt",
        config_name: str = "edgetam.yaml",
        device: str | None = None,
        use_bf16: bool = True,
    ) -> None:
        self.checkpoint_path = get_data(model_path) / model_name

        current_dir = Path(__file__).parent
        local_config_path = current_dir / "configs" / config_name

        if not local_config_path.exists():
            raise FileNotFoundError(f"EdgeTAM config not found at {local_config_path}")

        if device:
            self.device = device
        elif torch.cuda.is_available():
            self.device = "cuda"
            logger.info("Using CUDA for EdgeTAM")
        else:
            self.device = "cpu"
            logger.info("Using CPU for EdgeTAM")

        self.use_bf16 = use_bf16 and self.device == "cuda"

        # Build model manually
        cfg = OmegaConf.load(local_config_path)

        overrides = {
            "model.sam_mask_decoder_extra_args.dynamic_multimask_via_stability": True,
            "model.sam_mask_decoder_extra_args.dynamic_multimask_stability_delta": 0.05,
            "model.sam_mask_decoder_extra_args.dynamic_multimask_stability_thresh": 0.98,
            "model.binarize_mask_from_pts_for_mem_enc": True,
            "model.fill_hole_area": 8,
        }

        for key, value in overrides.items():
            OmegaConf.update(cfg, key, value)

        logger.info("Instantiating EdgeTAM model...")
        if cfg.model._target_ != "sam2.sam2_video_predictor.SAM2VideoPredictor":
            logger.warning(
                f"Config target is {cfg.model._target_}, forcing SAM2VideoPredictor"
            )
            cfg.model._target_ = "sam2.sam2_video_predictor.SAM2VideoPredictor"

        self.predictor: SAM2VideoPredictor = instantiate(cfg.model, _recursive_=True)

        logger.info(f"Loading checkpoint from {self.checkpoint_path}")
        _load_checkpoint(self.predictor, str(self.checkpoint_path))

        self.predictor = self.predictor.to(self.device)
        self.predictor.eval()

        # NOTE: torch.compile disabled - was causing 100x slowdown
        # The CUDA graphs from mode="reduce-overhead" may conflict with SAM2's dynamic control flow
        # if self.device == "cuda":
        #     try:
        #         self.predictor = torch.compile(self.predictor, mode="reduce-overhead")
        #         logger.info("Model compiled with torch.compile()")
        #     except Exception as e:
        #         logger.warning(f"torch.compile() failed, using eager mode: {e}")

        logger.info("EdgeTAM model ready")

        self.inference_state: dict | None = None  # type: ignore[type-arg]
        self.frame_count = 0
        self.temp_dir = Path("/tmp/dimos_edgetam_frames")
        self.is_tracking = False
        self.buffer_size = 100  # Keep last N frames in memory to avoid OOM

        if self.temp_dir.exists():
            shutil.rmtree(self.temp_dir)
        self.temp_dir.mkdir(parents=True, exist_ok=True)

    def _prepare_frame(self, image: Image) -> torch.Tensor:
        """Prepare frame for SAM2 (resize, normalize, convert to tensor)."""
        cv_image = image.to_opencv()
        rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        pil_image = PILImage.fromarray(rgb_image)

        img_np = np.array(
            pil_image.resize((self.predictor.image_size, self.predictor.image_size))
        )
        img_np = img_np.astype(np.float32) / 255.0

        img_mean = np.array([0.485, 0.456, 0.406], dtype=np.float32).reshape(1, 1, 3)
        img_std = np.array([0.229, 0.224, 0.225], dtype=np.float32).reshape(1, 1, 3)
        img_np -= img_mean
        img_np /= img_std

        img_tensor = torch.from_numpy(img_np).permute(2, 0, 1).float()

        if self.device == "cuda":
            img_tensor = img_tensor.cuda()
            if self.use_bf16:
                img_tensor = img_tensor.to(torch.bfloat16)

        return img_tensor

    def init_track(
        self,
        image: Image,
        points: np.ndarray | None = None,  # type: ignore[type-arg]
        labels: np.ndarray | None = None,  # type: ignore[type-arg]
        box: np.ndarray | None = None,  # type: ignore[type-arg]
        obj_id: int = 1,
    ) -> ImageDetections2D:  # type: ignore[type-arg]
        """Initialize tracking with a prompt (points or box).

        Args:
            image: Initial frame to start tracking from
            points: Point prompts for segmentation (Nx2 array of [x, y] coordinates)
            labels: Labels for points (1 = foreground, 0 = background)
            box: Bounding box prompt in [x1, y1, x2, y2] format
            obj_id: Object ID for tracking

        Returns:
            ImageDetections2D with initial segmentation mask
        """
        # Reset state properly - we must ensure temp dir is clean
        if self.inference_state is not None:
            self.stop()

        # Re-create temp directory immediately
        if self.temp_dir.exists():
            shutil.rmtree(self.temp_dir)
        self.temp_dir.mkdir(parents=True, exist_ok=True)

        self.frame_count = 0

        frame_path = self.temp_dir / f"{self.frame_count:05d}.jpg"
        image.save(str(frame_path))

        # Now we can safely init state with the fresh directory containing 1 frame
        self.inference_state = self.predictor.init_state(video_path=str(self.temp_dir))
        self.predictor.reset_state(self.inference_state)

        if torch.is_tensor(self.inference_state["images"]):
            self.inference_state["images"] = [self.inference_state["images"][0]]

        self.is_tracking = True

        if points is not None:
            points = points.astype(np.float32)
        if labels is not None:
            labels = labels.astype(np.int32)
        if box is not None:
            box = box.astype(np.float32)

        with torch.no_grad():
            if self.use_bf16:
                with torch.autocast("cuda", dtype=torch.bfloat16):
                    _, out_obj_ids, out_mask_logits = self.predictor.add_new_points_or_box(
                        inference_state=self.inference_state,
                        frame_idx=0,
                        obj_id=obj_id,
                        points=points,
                        labels=labels,
                        box=box,
                    )
            else:
                _, out_obj_ids, out_mask_logits = self.predictor.add_new_points_or_box(
                    inference_state=self.inference_state,
                    frame_idx=0,
                    obj_id=obj_id,
                    points=points,
                    labels=labels,
                    box=box,
                )

        return self._process_results(image, out_obj_ids, out_mask_logits)

    def process_image(self, image: Image) -> ImageDetections2D:  # type: ignore[type-arg]
        """Process a new video frame and propagate tracking.

        Args:
            image: New frame to process

        Returns:
            ImageDetections2D with tracked object segmentation masks
        """
        import time as _time

        if not self.is_tracking or self.inference_state is None:
            return ImageDetections2D(image=image)

        self.frame_count += 1

        # Append new frame to inference state
        t0 = _time.perf_counter()
        new_frame_tensor = self._prepare_frame(image)
        t1 = _time.perf_counter()
        self.inference_state["images"].append(new_frame_tensor)
        self.inference_state["num_frames"] += 1
        logger.info(f"_prepare_frame took {(t1-t0)*1000:.1f}ms, num_frames={self.inference_state['num_frames']}")

        # Memory management
        cached_features = self.inference_state["cached_features"]
        if len(cached_features) > self.buffer_size:
            oldest_frame = min(cached_features.keys())
            if oldest_frame < self.frame_count - self.buffer_size:
                del cached_features[oldest_frame]

        if len(self.inference_state["images"]) > self.buffer_size + 10:
            idx_to_drop = self.frame_count - self.buffer_size - 5
            if idx_to_drop >= 0 and idx_to_drop < len(self.inference_state["images"]):
                if self.inference_state["images"][idx_to_drop] is not None:
                    self.inference_state["images"][idx_to_drop] = None

        detections: ImageDetections2D = ImageDetections2D(image=image)  # type: ignore[type-arg]

        t2 = _time.perf_counter()
        with torch.no_grad():
            if self.use_bf16:
                with torch.autocast("cuda", dtype=torch.bfloat16):
                    for out_frame_idx, out_obj_ids, out_mask_logits in self.predictor.propagate_in_video(
                        self.inference_state, start_frame_idx=self.frame_count, max_frame_num_to_track=1
                    ):
                        if out_frame_idx == self.frame_count:
                            t3 = _time.perf_counter()
                            logger.info(f"propagate_in_video took {(t3-t2)*1000:.1f}ms")
                            return self._process_results(image, out_obj_ids, out_mask_logits)
            else:
                for out_frame_idx, out_obj_ids, out_mask_logits in self.predictor.propagate_in_video(
                    self.inference_state, start_frame_idx=self.frame_count, max_frame_num_to_track=1
                ):
                    if out_frame_idx == self.frame_count:
                        t3 = _time.perf_counter()
                        logger.info(f"propagate_in_video took {(t3-t2)*1000:.1f}ms")
                        return self._process_results(image, out_obj_ids, out_mask_logits)

        return detections

    def _process_results(
        self,
        image: Image,
        obj_ids: list[int],
        mask_logits: torch.Tensor | np.ndarray,  # type: ignore[type-arg]
    ) -> ImageDetections2D:  # type: ignore[type-arg]
        """Process model outputs into ImageDetections2D."""
        detections: ImageDetections2D = ImageDetections2D(image=image)  # type: ignore[type-arg]

        if len(obj_ids) == 0:
            return detections

        if isinstance(mask_logits, torch.Tensor):
            mask_logits = mask_logits.cpu().numpy()

        for i, obj_id in enumerate(obj_ids):
            mask = mask_logits[i]
            seg = Detection2DSeg.from_sam2_result(
                mask=mask,
                obj_id=obj_id,
                image=image,
                name="object",
            )

            # Only add if valid (non-empty)
            if seg.is_valid():
                detections.detections.append(seg)

        return detections

    def stop(self) -> None:
        """Stop tracking and clean up resources."""
        self.is_tracking = False
        self.inference_state = None
        if self.temp_dir.exists():
            try:
                shutil.rmtree(self.temp_dir)
            except OSError as e:
                logger.warning(f"Failed to remove temp dir {self.temp_dir}: {e}")


__all__ = ["EdgeTAMProcessor"]
