# Copyright (c) Sensrad 2026
# pylint: disable=no-name-in-module
import numpy as np

from vegvisir.pybind import vegvisir_pybind


class PoseGraphOptimizer:
    def __init__(self, max_iterations: int = 10):
        self.pgo = vegvisir_pybind.PoseGraphOptimizerCore(max_iterations)

    def add_variable(self, id_: int, pose: np.ndarray):
        self.pgo.add_variable(id_, pose)

    def fix_variable(self, id_: int):
        self.pgo.fix_variable(id_)

    def add_factor(self, id_source, id_target, relative_pose, information_matrix):
        self.pgo.add_factor(id_source, id_target, relative_pose, information_matrix)

    def add_gnss_constraint(self, id_: int, position: np.ndarray, information_matrix: np.ndarray):
        """Add a GNSS XYZ position constraint to a pose vertex.

        Args:
            id_: The vertex ID to constrain
            position: 3-element array [x, y, z] GNSS position in ENU frame
            information_matrix: 3x3 information matrix (inverse covariance)
        """
        self.pgo.add_gnss_constraint(id_, position, information_matrix)

    def optimize(self):
        self.pgo.optimize()

    def estimates(self):
        return self.pgo.estimates()

    def read_graph(self, filename: str):
        self.pgo.read_graph(filename)

    def write_graph(self, filename: str):
        self.pgo.write_graph(filename)
