use std::io::{self, Write};
use std::fs::OpenOptions;
use std::time::{Instant, SystemTime, UNIX_EPOCH};

use dimos_native_module::{LcmTransport, NativeModule};
use lcm_msgs::geometry_msgs::{Point, Pose, Quaternion};
use lcm_msgs::nav_msgs::{MapMetaData, OccupancyGrid};
use lcm_msgs::sensor_msgs::PointCloud2;
use lcm_msgs::std_msgs::{Header, Time};
use ndarray::{Array2, Axis};
use ndarray_ndimage::{BorderMode, gaussian_filter, sobel};
use serde::Deserialize;

#[derive(Debug, Deserialize)]
struct CostMapperConfig {
    resolution: f64,
    can_pass_under: f64,
    can_climb: f64,
    ignore_noise: f64,
    smoothing: f64,
}

fn extract_xyz(cloud: &PointCloud2) -> Vec<[f64; 3]> {
    let mut x_off = 0usize;
    let mut y_off = 4usize;
    let mut z_off = 8usize;

    for f in &cloud.fields {
        match f.name.as_str() {
            "x" => x_off = f.offset as usize,
            "y" => y_off = f.offset as usize,
            "z" => z_off = f.offset as usize,
            _ => {}
        }
    }

    let step = cloud.point_step as usize;
    let num = (cloud.width * cloud.height) as usize;
    let data = &cloud.data;
    let mut points = Vec::with_capacity(num);

    for i in 0..num {
        let base = i * step;
        if base + z_off + 4 > data.len() {
            break;
        }
        let read_f32 = |off: usize| -> f64 {
            f32::from_le_bytes(data[base + off..base + off + 4].try_into().unwrap()) as f64
        };
        let x = read_f32(x_off);
        let y = read_f32(y_off);
        let z = read_f32(z_off);
        if x.is_finite() && y.is_finite() && z.is_finite() {
            points.push([x, y, z]);
        }
    }
    points
}

fn binary_erosion_4(mask: &Array2<bool>) -> Array2<bool> {
    let (h, w) = mask.dim();
    let mut out = Array2::from_elem((h, w), false);
    for y in 0..h {
        for x in 0..w {
            if !mask[[y, x]] {
                continue;
            }
            let top = y == 0 || mask[[y - 1, x]];
            let bottom = y + 1 == h || mask[[y + 1, x]];
            let left = x == 0 || mask[[y, x - 1]];
            let right = x + 1 == w || mask[[y, x + 1]];
            out[[y, x]] = top && bottom && left && right;
        }
    }
    out
}

fn height_cost_occupancy(cloud: &PointCloud2, cfg: &CostMapperConfig) -> OccupancyGrid {
    let ts = cloud.header.stamp.sec as f64 + cloud.header.stamp.nsec as f64 * 1e-9;
    let frame_id = cloud.header.frame_id.clone();

    let points = extract_xyz(cloud);
    if points.is_empty() {
        return build_grid(vec![-1], 1, 1, cfg.resolution as f32, 0.0, 0.0, &frame_id, ts);
    }

    // Grid bounds with 1m padding on all sides.
    let padding = 1.0f64;
    let min_x = points.iter().map(|p| p[0]).fold(f64::INFINITY, f64::min) - padding;
    let max_x = points.iter().map(|p| p[0]).fold(f64::NEG_INFINITY, f64::max) + padding;
    let min_y = points.iter().map(|p| p[1]).fold(f64::INFINITY, f64::min) - padding;
    let max_y = points.iter().map(|p| p[1]).fold(f64::NEG_INFINITY, f64::max) + padding;

    let res = cfg.resolution;
    let inv_res = 1.0 / res;
    let w = ((max_x - min_x) / res).ceil() as usize;
    let h = ((max_y - min_y) / res).ceil() as usize;

    // Step 1: build per-cell min and max height maps.
    let mut min_map = Array2::from_elem((h, w), f64::NAN);
    let mut max_map = Array2::from_elem((h, w), f64::NAN);

    for p in &points {
        let gx = ((p[0] - min_x) * inv_res + 0.5) as isize;
        let gy = ((p[1] - min_y) * inv_res + 0.5) as isize;
        if gx < 0 || gx >= w as isize || gy < 0 || gy >= h as isize {
            continue;
        }
        let (gx, gy) = (gx as usize, gy as usize);
        let z = p[2];
        if min_map[[gy, gx]].is_nan() || z < min_map[[gy, gx]] {
            min_map[[gy, gx]] = z;
        }
        if max_map[[gy, gx]].is_nan() || z > max_map[[gy, gx]] {
            max_map[[gy, gx]] = z;
        }
    }

    // Step 2: effective height per cell.
    // If the vertical span exceeds can_pass_under the robot passes underneath,
    // so use the floor (min). Otherwise treat the top as a solid obstacle (max).
    let mut height_map = Array2::from_elem((h, w), f64::NAN);
    for y in 0..h {
        for x in 0..w {
            let mn = min_map[[y, x]];
            let mx = max_map[[y, x]];
            if mn.is_nan() || mx.is_nan() {
                continue;
            }
            height_map[[y, x]] = if mx - mn > cfg.can_pass_under { mn } else { mx };
        }
    }

    let mut observed = height_map.mapv(|v| !v.is_nan());

    // Step 3: Gaussian smoothing to fill gaps near observed cells.
    // Smooth height and weight maps separately, then divide to interpolate.
    if cfg.smoothing > 0.0 && observed.iter().any(|&v| v) {
        let weights = observed.mapv(|o| if o { 1.0f64 } else { 0.0 });
        let height_filled = height_map.mapv(|v| if v.is_nan() { 0.0 } else { v });

        let smoothed_heights = gaussian_filter(&height_filled, cfg.smoothing, 0, BorderMode::Mirror, 4);
        let smoothed_weights = gaussian_filter(&weights, cfg.smoothing, 0, BorderMode::Mirror, 4);

        for y in 0..h {
            for x in 0..w {
                if !observed[[y, x]] && smoothed_weights[[y, x]] > 0.01 {
                    height_map[[y, x]] = smoothed_heights[[y, x]] / smoothed_weights[[y, x]];
                }
            }
        }
        observed = height_map.mapv(|v| !v.is_nan());
    }

    // Step 4: Sobel gradient → terrain slope → cost.
    let mut cost_map = Array2::from_elem((h, w), -1i8);

    if observed.iter().any(|&v| v) {
        let height_for_grad = height_map.mapv(|v| if v.is_nan() { 0.0 } else { v });

        let sobel_scale = 1.0 / (8.0 * cfg.resolution);
        let grad_x = sobel(&height_for_grad, Axis(1), BorderMode::Mirror) * sobel_scale;
        let grad_y = sobel(&height_for_grad, Axis(0), BorderMode::Mirror) * sobel_scale;

        let valid = binary_erosion_4(&observed);

        for y in 0..h {
            for x in 0..w {
                if !valid[[y, x]] {
                    continue;
                }
                let gradient_magnitude = (grad_x[[y, x]].powi(2) + grad_y[[y, x]].powi(2)).sqrt();
                let height_change = gradient_magnitude * cfg.resolution;
                let height_change = if height_change < cfg.ignore_noise { 0.0 } else { height_change };
                let cost = (height_change / cfg.can_climb * 100.0).clamp(0.0, 100.0) as i8;
                cost_map[[y, x]] = cost;
            }
        }
    }

    let cost_vec = cost_map.into_raw_vec_and_offset().0;
    build_grid(cost_vec, w, h, cfg.resolution as f32, min_x, min_y, &frame_id, ts)
}

fn write_timing_row(compute_ms: f64, total_ms: f64, n_points: usize) {
    let path = std::env::var("HOME").unwrap_or_else(|_| "/tmp".into()) + "/costmapper_timing_rust.csv";
    let needs_header = !std::path::Path::new(&path).exists();
    if let Ok(mut f) = OpenOptions::new().create(true).append(true).open(&path) {
        if needs_header {
            let _ = writeln!(f, "timestamp_s,compute_ms,total_ms,n_points");
        }
        let ts = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .map(|d| d.as_secs_f64())
            .unwrap_or(0.0);
        let _ = writeln!(f, "{:.6},{:.3},{:.3},{}", ts, compute_ms, total_ms, n_points);
    }
}

fn build_grid(
    data: Vec<i8>,
    width: usize,
    height: usize,
    resolution: f32,
    origin_x: f64,
    origin_y: f64,
    frame_id: &str,
    ts: f64,
) -> OccupancyGrid {
    let sec = ts as i32;
    let nsec = ((ts - sec as f64) * 1e9) as i32;
    let stamp = Time { sec, nsec };
    OccupancyGrid {
        header: Header {
            seq: 0,
            stamp: stamp.clone(),
            frame_id: frame_id.to_string(),
        },
        info: MapMetaData {
            map_load_time: stamp,
            resolution,
            width: width as i32,
            height: height as i32,
            origin: Pose {
                position: Point { x: origin_x, y: origin_y, z: 0.0 },
                orientation: Quaternion { x: 0.0, y: 0.0, z: 0.0, w: 1.0 },
            },
        },
        data,
    }
}

#[tokio::main]
async fn main() -> io::Result<()> {
    let transport = LcmTransport::new().await.expect("failed to create LCM transport");
    let (mut module, config) = NativeModule::from_stdin::<CostMapperConfig>(transport)
        .await
        .expect("failed to read config from stdin");

    let mut global_map = module.input("global_map", PointCloud2::decode);
    let global_costmap = module.output("global_costmap", OccupancyGrid::encode);
    let _handle = module.spawn();

    eprintln!(
        "costmapper ready: resolution={} can_climb={} smoothing={}",
        config.resolution, config.can_climb, config.smoothing
    );

    loop {
        match global_map.recv().await {
            Some(cloud) => {
                let t_total = Instant::now();
                let n_points = (cloud.width * cloud.height) as usize;
                let t_compute = Instant::now();
                let grid = height_cost_occupancy(&cloud, &config);
                let compute_ms = t_compute.elapsed().as_secs_f64() * 1000.0;
                global_costmap.publish(&grid).await.ok();
                let total_ms = t_total.elapsed().as_secs_f64() * 1000.0;
                write_timing_row(compute_ms, total_ms, n_points);
            }
            None => break,
        }
    }

    Ok(())
}
