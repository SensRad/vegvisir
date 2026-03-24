# Copyright (c) Sensrad 2026
# pylint: disable=no-name-in-module
import numpy as np

from vegvisir.pybind import vegvisir_pybind


class VoxelMap:
    def __init__(self, voxel_size: float):
        self.map = vegvisir_pybind.VoxelMapCore(voxel_size)

    def integrate_frame(self, points: np.ndarray, pose: np.ndarray):
        vector3dvector = vegvisir_pybind.Vector3dVector(points.astype(np.float64))
        self.map.integrate_frame(vector3dvector, pose)

    def add_points(self, points: np.ndarray):
        vector3dvector = vegvisir_pybind.Vector3dVector(points.astype(np.float64))
        self.map.add_points(vector3dvector)

    def point_cloud(self) -> np.ndarray:
        return np.asarray(self.map.point_cloud()).astype(np.float64)

    def clear(self):
        self.map.clear()

    def num_voxels(self) -> int:
        return self.map.num_voxels()

    def per_voxel_point_and_normal(self):
        points, normals = self.map.per_voxel_point_and_normal()
        return np.asarray(points), np.asarray(normals)
