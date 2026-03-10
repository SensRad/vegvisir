# Copyright (c) Sensrad 2026
import numpy as np

from vegvisir.pybind import vegvisir_pybind


class VoxelMap:
    def __init__(self, voxel_size: float):
        self.map = vegvisir_pybind._VoxelMap(voxel_size)

    def integrate_frame(self, points: np.ndarray, pose: np.ndarray):
        vector3dvector = vegvisir_pybind._Vector3dVector(points.astype(np.float64))
        self.map._integrate_frame(vector3dvector, pose)

    def add_points(self, points: np.ndarray):
        vector3dvector = vegvisir_pybind._Vector3dVector(points.astype(np.float64))
        self.map._add_points(vector3dvector)

    def point_cloud(self) -> np.ndarray:
        return np.asarray(self.map._point_cloud()).astype(np.float64)

    def clear(self):
        self.map._clear()

    def num_voxels(self) -> int:
        return self.map._num_voxels()

    def per_voxel_point_and_normal(self):
        points, normals = self.map._per_voxel_point_and_normal()
        return np.asarray(points), np.asarray(normals)
