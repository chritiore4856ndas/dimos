{
  description = "SlamMap native module — SLAM + loop closure + dynamic obstacle mapping";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    flake-utils.url = "github:numtide/flake-utils";
    dimos-lcm = {
      url = "github:dimensionalOS/dimos-lcm/main";
      flake = false;
    };
    nanoflann-src = {
      url = "github:jlblancoc/nanoflann/v1.6.2";
      flake = false;
    };
    gtsam-src = {
      url = "github:borglab/gtsam/develop";
      flake = false;
    };
  };

  outputs = { self, nixpkgs, flake-utils, dimos-lcm, nanoflann-src, gtsam-src, ... }:
    flake-utils.lib.eachDefaultSystem (system:
      let
        pkgs = import nixpkgs { inherit system; };

        slam-map-common = ../../dynamic_map/native/common;

        gtsam = pkgs.stdenv.mkDerivation {
          pname = "gtsam";
          version = "4.3-dev";

          src = gtsam-src;

          nativeBuildInputs = [ pkgs.cmake pkgs.pkg-config ];
          buildInputs = [ pkgs.boost pkgs.eigen pkgs.tbb ];

          cmakeFlags = [
            "-DCMAKE_BUILD_TYPE=Release"
            "-DGTSAM_BUILD_TESTS=OFF"
            "-DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF"
            "-DGTSAM_BUILD_UNSTABLE=OFF"
            "-DGTSAM_WITH_TBB=OFF"
            "-DGTSAM_BUILD_PYTHON=OFF"
            "-DGTSAM_INSTALL_MATLAB_TOOLBOX=OFF"
            "-DCMAKE_POLICY_VERSION_MINIMUM=3.5"
            "-DGTSAM_USE_SYSTEM_EIGEN=ON"
          ];
        };

        slam_map_node = pkgs.stdenv.mkDerivation {
          pname = "slam_map_node";
          version = "0.1.0";

          src = ./.;

          nativeBuildInputs = [ pkgs.cmake pkgs.pkg-config ];
          buildInputs = [
            pkgs.lcm
            pkgs.glib
            pkgs.octomap
            pkgs.eigen
            pkgs.boost
            gtsam
          ];

          cmakeFlags = [
            "-DCMAKE_POLICY_VERSION_MINIMUM=3.5"
            "-DFETCHCONTENT_SOURCE_DIR_DIMOS_LCM=${dimos-lcm}"
            "-DFETCHCONTENT_SOURCE_DIR_NANOFLANN=${nanoflann-src}"
            "-DSLAM_MAP_COMMON_DIR=${slam-map-common}"
          ];
        };
      in {
        packages = {
          default = slam_map_node;
          inherit slam_map_node gtsam;
        };

        devShells.default = pkgs.mkShell {
          buildInputs = [ gtsam pkgs.octomap pkgs.eigen pkgs.lcm ];
        };
      });
}
