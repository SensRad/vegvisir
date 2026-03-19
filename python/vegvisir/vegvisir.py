# Copyright (c) Sensrad 2026
"""Python wrapper around the C++ vegvisir::Vegvisir pybind11 binding."""

from typing import Optional

import numpy as np

from .pybind.vegvisir_pybind import (
    Mode,  # noqa: F401
    _Vegvisir,
    _VegvisirConfig,
)


class VegvisirConfig:
    """Runtime configuration for the Vegvisir engine.

    All defaults match the original hardcoded values — zero behavioral change
    when no overrides are provided.

    Args:
        voxel_size: Voxel grid cell size in meters.
        splitting_distance_slam: Distance between keyframes in SLAM mode (meters).
        splitting_distance_localization: Distance between submaps in localization mode (meters).
        overlap_threshold: Minimum overlap fraction to accept a loop closure.
        pgo_max_iterations: Maximum iterations for pose graph optimization.
        inliers_threshold: Minimum number of feature inliers to consider a closure candidate.
    """

    def __init__(
        self,
        voxel_size: float = 0.8,
        splitting_distance_slam: float = 50.0,
        splitting_distance_localization: float = 5.0,
        overlap_threshold: float = 0.10,
        pgo_max_iterations: int = 10,
        inliers_threshold: int = 10,
    ):
        self._impl = _VegvisirConfig()
        self._impl.voxel_size = voxel_size
        self._impl.splitting_distance_slam = splitting_distance_slam
        self._impl.splitting_distance_localization = splitting_distance_localization
        self._impl.overlap_threshold = overlap_threshold
        self._impl.pgo_max_iterations = pgo_max_iterations
        self._impl.inliers_threshold = inliers_threshold

    @property
    def voxel_size(self) -> float:
        return self._impl.voxel_size

    @voxel_size.setter
    def voxel_size(self, value: float) -> None:
        self._impl.voxel_size = value

    @property
    def splitting_distance_slam(self) -> float:
        return self._impl.splitting_distance_slam

    @splitting_distance_slam.setter
    def splitting_distance_slam(self, value: float) -> None:
        self._impl.splitting_distance_slam = value

    @property
    def splitting_distance_localization(self) -> float:
        return self._impl.splitting_distance_localization

    @splitting_distance_localization.setter
    def splitting_distance_localization(self, value: float) -> None:
        self._impl.splitting_distance_localization = value

    @property
    def overlap_threshold(self) -> float:
        return self._impl.overlap_threshold

    @overlap_threshold.setter
    def overlap_threshold(self, value: float) -> None:
        self._impl.overlap_threshold = value

    @property
    def pgo_max_iterations(self) -> int:
        return self._impl.pgo_max_iterations

    @pgo_max_iterations.setter
    def pgo_max_iterations(self, value: int) -> None:
        self._impl.pgo_max_iterations = value

    @property
    def inliers_threshold(self) -> int:
        return self._impl.inliers_threshold

    @inliers_threshold.setter
    def inliers_threshold(self, value: int) -> None:
        self._impl.inliers_threshold = value


class Vegvisir:
    """Python interface to the C++ Vegvisir (SLAM / Localization).

    Mirrors the C++ ``Vegvisir::update()`` called from
    ``VegvisirNode::process()``.

    Args:
        map_database_path: Path to map directory for saving/loading.
        mode: ``Mode.SLAM`` (default) or ``Mode.LOCALIZATION``.
        config: Optional runtime configuration. Uses defaults if not provided.
    """

    def __init__(
        self,
        map_database_path: str = "",
        mode=Mode.SLAM,
        config: Optional[VegvisirConfig] = None,
    ):
        cpp_config = config._impl if config is not None else _VegvisirConfig()
        self._impl = _Vegvisir(map_database_path, mode, cpp_config)

    def update(
        self,
        points: np.ndarray,
        absolute_pose: np.ndarray,
    ):
        """Feed one scan into the localizer.

        Args:
            points: Nx3 array of 3D points [x, y, z].
            absolute_pose: 4x4 odometry pose (T_odom_base).
        """
        self._impl.update(
            np.ascontiguousarray(points, dtype=np.float64),
            np.ascontiguousarray(absolute_pose, dtype=np.float64),
        )

    def save_database(self) -> bool:
        """Save the map database (SLAM mode only)."""
        return self._impl.save_database()

    def get_keyposes(self):
        """Return list of keypose 4x4 matrices."""
        graph = self._impl.get_local_map_graph()
        return graph.get_all_keyposes()

    def get_closures(self):
        """Return list of (source_id, target_id, pose_4x4) tuples."""
        closures = self._impl.get_closures()
        return [(c.source_id, c.target_id, c.pose) for c in closures]

    def get_num_closures(self) -> int:
        return self._impl.get_num_closures()

    @property
    def poses(self):
        """Per-frame poses (keypose @ relative), like KissSLAM.poses."""
        return self._impl.poses

    def get_local_map_graph(self):
        """Access the C++ LocalMapGraph (read-only reference)."""
        return self._impl.get_local_map_graph()

    def get_map_to_odom_transform(self) -> np.ndarray:
        return self._impl.get_map_to_odom_transform()

    def get_base_in_map_frame(self) -> np.ndarray:
        return self._impl.get_base_in_map_frame()

    def get_covariance(self) -> np.ndarray:
        return self._impl.get_covariance()

    def get_mode(self):
        return self._impl.get_mode()

    # -- Map closures forwarding --

    def get_density_map(self, map_id: int) -> np.ndarray:
        """Get the 2D density map for a specific map ID."""
        return self._impl.get_density_map(map_id)

    def get_density_map_metadata(self, map_id: int) -> dict:
        """Get metadata for a specific density map."""
        return self._impl.get_density_map_metadata(map_id)

    def get_all_map_ids(self):
        """Get list of all map IDs in the local map graph."""
        graph = self._impl.get_local_map_graph()
        return list(graph.get_all_ids())

    def get_available_density_map_ids(self):
        """Get list of all map IDs that have density maps in the database."""
        return self._impl.get_available_map_ids()

    def get_ground_alignment(self, map_id: int) -> np.ndarray:
        """Get the ground alignment transform for a specific map."""
        return self._impl.get_ground_alignment(map_id)

    def get_reference_poses(self) -> dict:
        """Get all reference poses {map_id: 4x4 matrix}."""
        return self._impl.get_reference_poses()

    def get_reference_pose(self, map_id: int) -> np.ndarray:
        """Get the reference pose for a specific map ID."""
        return self._impl.get_reference_pose(map_id)

    def has_local_map_points(self, map_id: int) -> bool:
        return self._impl.has_local_map_points(map_id)

    def get_local_map_points(self, map_id: int) -> np.ndarray:
        """Get Nx3 point cloud for a specific local map."""
        return self._impl.get_local_map_points(map_id)

    def fine_grained_optimization(self):
        """Per-frame PGO over the full trajectory (runs in C++).

        If GNSS measurements have been added via add_gnss_measurement(),
        they will be used as constraints in the optimization.

        Returns (optimized_poses, None).
        """
        poses = self._impl.fine_grained_optimization()
        if not poses:
            return [np.eye(4)], None
        return poses, None

    def fine_grained_optimization_and_update_keyposes(self):
        """Per-frame PGO AND update internal keyposes to match optimized poses.

        This is the recommended method when using GNSS anchoring. It ensures
        that the keyposes stored in the database match the PGO-optimized
        trajectory, providing frame consistency between mapping and localization.

        After calling this method:
        - LocalMapGraph keyposes are updated to match optimized poses
        - MapClosures reference poses are updated
        - local_map_points are re-transformed to the new keypose frames

        If GNSS measurements have been added via add_gnss_measurement(),
        they will be used as constraints in the optimization.

        Returns (optimized_poses, None).
        """
        poses = self._impl.fine_grained_optimization_and_update_keyposes()
        if not poses:
            return [np.eye(4)], None
        return poses, None

    def get_reference_keyposes(self):
        """Get reference keyposes from the loaded map (localization mode).

        Returns a list of 4x4 matrices representing the reference map's keyposes.
        In SLAM mode, this returns the same as get_keyposes().
        In localization mode, this returns the keyposes stored in the loaded database.
        """
        ref_poses = self._impl.get_reference_poses()
        if not ref_poses:
            return []
        # Return as list sorted by map_id
        return [ref_poses[k] for k in sorted(ref_poses.keys())]

    def add_gnss_measurement(
        self, pose_index: int, position_enu: np.ndarray, information_matrix: np.ndarray
    ):
        """Add a GNSS measurement for fusion during pose graph optimization.

        Args:
            pose_index: The index of the pose (frame) this measurement corresponds to
            position_enu: 3-element array [east, north, up] GNSS position in ENU frame
            information_matrix: 3x3 information matrix (inverse covariance)
        """
        self._impl.add_gnss_measurement(
            pose_index,
            np.ascontiguousarray(position_enu, dtype=np.float64),
            np.ascontiguousarray(information_matrix, dtype=np.float64),
        )

    def clear_gnss_measurements(self):
        """Clear all stored GNSS measurements."""
        self._impl.clear_gnss_measurements()

    def get_num_gnss_measurements(self) -> int:
        """Get the number of stored GNSS measurements."""
        return self._impl.get_num_gnss_measurements()

    def get_optimized_alignment_transform(self) -> np.ndarray:
        """Get the optimized T_ENU_map alignment transform after PGO.

        This is the transform that maps positions from map frame to ENU frame:
        p_ENU = T_ENU_map * p_map

        Returns Identity if no GNSS-fused optimization has been run.
        """
        return self._impl.get_optimized_alignment_transform()

    def set_initial_alignment_estimate(self, t_enu_map: np.ndarray):
        """Set initial estimate for T_ENU_map alignment before PGO.

        Call this before fine_grained_optimization if you have a good initial
        guess (e.g., computed from trajectory principal directions).
        If not set, PGO will initialize from Identity.

        Args:
            t_enu_map: 4x4 initial estimate for the ENU-to-map alignment transform
        """
        self._impl.set_initial_alignment_estimate(np.ascontiguousarray(t_enu_map, dtype=np.float64))

    def set_gnss_anchor_transform(self, t_enu_map: np.ndarray):
        """Set the GNSS anchor transform to be saved with the map.

        Call this before save_database() to include GNSS anchoring in metadata.
        """
        self._impl.set_gnss_anchor_transform(np.ascontiguousarray(t_enu_map, dtype=np.float64))

    def set_gnss_origin(self, lat0: float, lon0: float, alt0: float):
        """Set the GNSS origin (WGS84) used for ENU conversion.

        Call this before save_database() to include the origin in metadata.
        """
        self._impl.set_gnss_origin(lat0, lon0, alt0)

    def get_gnss_anchor_transform(self) -> np.ndarray:
        """Get the GNSS anchor transform from loaded metadata."""
        return self._impl.get_gnss_anchor_transform()

    def has_gnss_anchor(self) -> bool:
        """Check if GNSS anchor data is available in the loaded map."""
        return self._impl.has_gnss_anchor()

    def get_gnss_origin(self):
        """Get the GNSS origin from loaded metadata."""
        return self._impl.get_gnss_origin()
