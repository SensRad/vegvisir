# Copyright (c) Sensrad 2026
from dataclasses import dataclass

import numpy as np

from vegvisir.pybind import vegvisir_pybind

type ClosureCandidate = vegvisir_pybind._ClosureCandidate

LOCAL_MAPS_TO_SKIP: int = vegvisir_pybind.LOCAL_MAPS_TO_SKIP
MIN_NUMBER_OF_MATCHES: int = vegvisir_pybind.MIN_NUMBER_OF_MATCHES
RANSAC_INLIER_THRESHOLD: float = vegvisir_pybind.RANSAC_INLIER_THRESHOLD


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


class MapClosures:
    def __init__(self, config: MapClosuresConfig | None = None):
        if config is None:
            self._pipeline = vegvisir_pybind._MapClosures()
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
            }
            self._pipeline = vegvisir_pybind._MapClosures(cfg_dict)

    def get_top_k_closures(
        self, query_idx: int, local_map: np.ndarray, k: int
    ) -> list[ClosureCandidate]:
        pcd = vegvisir_pybind._Vector3dVector(local_map.astype(np.float64))
        return self._pipeline._GetTopKClosures(query_idx, pcd, k)

    def get_closures(
        self, query_idx: int, local_map: np.ndarray
    ) -> list[ClosureCandidate]:
        pcd = vegvisir_pybind._Vector3dVector(local_map.astype(np.float64))
        return self._pipeline._GetClosures(query_idx, pcd)

    def query_top_k_closures(
        self, query_idx: int, local_map: np.ndarray, k: int
    ) -> list[ClosureCandidate]:
        """Query for top-k closures without adding the map to the database."""
        pcd = vegvisir_pybind._Vector3dVector(local_map.astype(np.float64))
        return self._pipeline._QueryTopKClosures(query_idx, pcd, k)

    def query_closures(
        self, query_idx: int, local_map: np.ndarray
    ) -> list[ClosureCandidate]:
        """Query for all closures without adding the map to the database."""
        pcd = vegvisir_pybind._Vector3dVector(local_map.astype(np.float64))
        return self._pipeline._QueryClosures(query_idx, pcd)

    def get_density_map_from_id(self, map_id: int) -> np.ndarray:
        return self._pipeline._getDensityMapFromId(map_id)

    def get_density_map_metadata(self, map_id: int) -> dict:
        return self._pipeline._getDensityMapMetadata(map_id)

    def get_available_map_ids(self) -> list[int]:
        return self._pipeline._getAvailableMapIds()

    def get_ground_alignment(self, map_id: int) -> np.ndarray:
        return self._pipeline._getGroundAlignment(map_id)

    def get_reference_poses(self) -> dict[int, np.ndarray]:
        return self._pipeline._getReferencePoses()

    def get_reference_pose(self, map_id: int) -> np.ndarray:
        return self._pipeline._getReferencePose(map_id)

    def set_reference_pose(self, map_id: int, pose: np.ndarray):
        self._pipeline._setReferencePose(map_id, pose)

    def get_local_map_points(self, map_id: int) -> np.ndarray:
        points = self._pipeline._getLocalMapPoints(map_id)
        return np.asarray(points).astype(np.float64)

    def has_local_map_points(self, map_id: int) -> bool:
        return self._pipeline._hasLocalMapPoints(map_id)

    def save(self, file_path: str) -> bool:
        return self._pipeline._save(file_path)

    def load(self, file_path: str) -> bool:
        return self._pipeline._load(file_path)

    def load_reference_poses(self, file_path: str) -> bool:
        return self._pipeline._loadReferencePoses(file_path)

    def load_local_map_points(self, file_path: str) -> bool:
        return self._pipeline._loadLocalMapPoints(file_path)

    def get_feature_info(self, map_id: int) -> dict:
        return self._pipeline._getSiftKeypointsViz(map_id)


def align_map_to_local_ground(pointcloud: np.ndarray, resolution: float) -> np.ndarray:
    pcd = vegvisir_pybind._Vector3dVector(pointcloud.astype(np.float64))
    return vegvisir_pybind._align_map_to_local_ground(pcd, resolution)
