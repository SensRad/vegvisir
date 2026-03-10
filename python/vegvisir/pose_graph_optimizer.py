# Copyright (c) Sensrad 2026
import numpy as np

from vegvisir.pybind import vegvisir_pybind


class PoseGraphOptimizer:
    def __init__(self, max_iterations: int = 10):
        self.pgo = vegvisir_pybind._PoseGraphOptimizer(max_iterations)

    def add_variable(self, id_: int, pose: np.ndarray):
        self.pgo._add_variable(id_, pose)

    def fix_variable(self, id_: int):
        self.pgo._fix_variable(id_)

    def add_factor(self, id_source, id_target, relative_pose, information_matrix):
        self.pgo._add_factor(id_source, id_target, relative_pose, information_matrix)

    def add_gnss_constraint(self, id_: int, position: np.ndarray, information_matrix: np.ndarray):
        """Add a GNSS XYZ position constraint to a pose vertex.

        Args:
            id_: The vertex ID to constrain
            position: 3-element array [x, y, z] GNSS position in ENU frame
            information_matrix: 3x3 information matrix (inverse covariance)
        """
        self.pgo._add_gnss_constraint(id_, position, information_matrix)

    def optimize(self):
        self.pgo._optimize()

    def estimates(self):
        return self.pgo._estimates()

    def read_graph(self, filename: str):
        self.pgo._read_graph(filename)

    def write_graph(self, filename: str):
        self.pgo._write_graph(filename)
