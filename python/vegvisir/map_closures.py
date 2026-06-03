# Copyright (c) Sensrad 2026
# pylint: disable=no-name-in-module
from dataclasses import dataclass

import numpy as np

from vegvisir.pybind import vegvisir_pybind

type ClosureCandidate = vegvisir_pybind.ClosureCandidate

LOCAL_MAPS_TO_SKIP: int = vegvisir_pybind.LOCAL_MAPS_TO_SKIP
MIN_NUMBER_OF_MATCHES: int = vegvisir_pybind.MIN_NUMBER_OF_MATCHES


@dataclass
class MapClosuresConfig:
    density_map_resolution: float = 0.5
    density_threshold: float = 0.05
    sift_match_ratio: float = 0.85
    density_map_gamma: float = 0.3
    lbd_min_line_length: float = 15.0
    lbd_match_ratio: float = 0.80
    lbd_num_octaves: int = 2
    lbd_scale: int = 2
    lbd_weight: float = 1.0
    ransac_inlier_threshold_m: float = 1.0


class MapClosures:
    def __init__(self, config: MapClosuresConfig | None = None):
        if config is None:
            self.pipeline = vegvisir_pybind.MapClosuresCore()
        else:
            cfg_dict = {
                "density_map_resolution": config.density_map_resolution,
                "density_threshold": config.density_threshold,
                "sift_match_ratio": config.sift_match_ratio,
                "density_map_gamma": config.density_map_gamma,
                "lbd_min_line_length": config.lbd_min_line_length,
                "lbd_match_ratio": config.lbd_match_ratio,
                "lbd_num_octaves": config.lbd_num_octaves,
                "lbd_scale": config.lbd_scale,
                "lbd_weight": config.lbd_weight,
                "ransac_inlier_threshold_m": config.ransac_inlier_threshold_m,
            }
            self.pipeline = vegvisir_pybind.MapClosuresCore(cfg_dict)

    def get_top_k_closures(
        self, query_idx: int, local_map: np.ndarray, k: int
    ) -> list[ClosureCandidate]:
        pcd = vegvisir_pybind.Vector3dVector(local_map.astype(np.float64))
        return self.pipeline.get_top_k_closures(query_idx, pcd, k)

    def get_closures(self, query_idx: int, local_map: np.ndarray) -> list[ClosureCandidate]:
        pcd = vegvisir_pybind.Vector3dVector(local_map.astype(np.float64))
        return self.pipeline.get_closures(query_idx, pcd)

    def query_top_k_closures(
        self, query_idx: int, local_map: np.ndarray, k: int
    ) -> list[ClosureCandidate]:
        """Query for top-k closures without adding the map to the database."""
        pcd = vegvisir_pybind.Vector3dVector(local_map.astype(np.float64))
        return self.pipeline.query_top_k_closures(query_idx, pcd, k)

    def query_closures(self, query_idx: int, local_map: np.ndarray) -> list[ClosureCandidate]:
        """Query for all closures without adding the map to the database."""
        pcd = vegvisir_pybind.Vector3dVector(local_map.astype(np.float64))
        return self.pipeline.query_closures(query_idx, pcd)

    def get_stored_closures(
        self, query_id: int, k: int = -1, ignore_skip: bool = False
    ) -> list[ClosureCandidate]:
        """Match an already-stored segment against the other stored segments.

        No re-extraction (uses the stored features). ``ignore_skip`` disables the
        sequential LOCAL_MAPS_TO_SKIP filter -- needed when matching across two maps
        whose ids are merely offset (not one continuous trajectory).
        """
        return self.pipeline.get_stored_closures(query_id, k, ignore_skip)

    def get_density_map_from_id(self, map_id: int) -> np.ndarray:
        return self.pipeline.get_density_map_from_id(map_id)

    def get_density_map_metadata(self, map_id: int) -> dict:
        return self.pipeline.get_density_map_metadata(map_id)

    def get_available_map_ids(self) -> list[int]:
        return self.pipeline.get_available_map_ids()

    def get_ground_alignment(self, map_id: int) -> np.ndarray:
        return self.pipeline.get_ground_alignment(map_id)

    def get_reference_poses(self) -> dict[int, np.ndarray]:
        return self.pipeline.get_reference_poses()

    def get_reference_pose(self, map_id: int) -> np.ndarray:
        return self.pipeline.get_reference_pose(map_id)

    def set_reference_pose(self, map_id: int, pose: np.ndarray):
        self.pipeline.set_reference_pose(map_id, pose)

    def set_ground_alignment(self, map_id: int, ground_alignment: np.ndarray):
        self.pipeline.set_ground_alignment(
            map_id, np.ascontiguousarray(ground_alignment, dtype=np.float64)
        )

    def merge_maps(self, target_id: int, source_id: int, relative_pose: np.ndarray):
        """Merge source_id's features into target_id, reframing by relative_pose.

        relative_pose maps the source's local frame to the target's local frame.
        Features are concatenated (not re-extracted); the source map is removed.
        """
        self.pipeline.merge_maps(
            target_id, source_id, np.ascontiguousarray(relative_pose, dtype=np.float64)
        )

    def import_segments(self, other: "MapClosures", id_offset: int):
        """Copy all of ``other``'s segments (features, density maps, ground
        alignments) into this map with ids shifted by ``id_offset``. Segments stay
        separate and keep their own local frames; the caller repositions the matching
        keyposes/points. Used to combine two maps into one.
        """
        self.pipeline.import_segments(other.pipeline, id_offset)

    def get_feature_counts(self, map_id: int) -> dict:
        """Per-layer feature counts for a map, e.g. {"sift": n, "lbd": m}."""
        counts = self.pipeline.get_feature_counts(map_id)
        return {
            "sift": counts[0] if len(counts) > 0 else 0,
            "lbd": counts[1] if len(counts) > 1 else 0,
        }

    def get_local_map_points(self, map_id: int) -> np.ndarray:
        points = self.pipeline.get_local_map_points(map_id)
        return np.asarray(points).astype(np.float64)

    def has_local_map_points(self, map_id: int) -> bool:
        return self.pipeline.has_local_map_points(map_id)

    def save(self, file_path: str) -> bool:
        return self.pipeline.save(file_path)

    def load(self, file_path: str) -> bool:
        return self.pipeline.load(file_path)

    def load_reference_poses(self, file_path: str, keypose_ids: list[int]) -> bool:
        return self.pipeline.load_reference_poses(file_path, keypose_ids)

    def load_local_map_points(self, file_path: str) -> bool:
        return self.pipeline.load_local_map_points(file_path)

    def get_feature_info(self, map_id: int) -> dict:
        return self.pipeline.get_sift_keypoints_viz(map_id)


def align_map_to_local_ground(pointcloud: np.ndarray, resolution: float) -> np.ndarray:
    pcd = vegvisir_pybind.Vector3dVector(pointcloud.astype(np.float64))
    return vegvisir_pybind.align_map_to_local_ground(pcd, resolution)
