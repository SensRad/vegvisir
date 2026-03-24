// Copyright (c) Sensrad 2025-2026
#include <memory>
#include <tuple>
#include <vector>

#include <Eigen/Core>
#include <opencv2/core.hpp>
#include <opencv2/core/eigen.hpp>
#include <pybind11/eigen.h>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/stl_bind.h>
#include <sophus/se3.hpp>

#include "LocalMapGraph.hpp"
#include "Vegvisir.hpp"
#include "VegvisirConfig.hpp"
#include "VegvisirIO.hpp"
#include "map_closures/AlignRansac2D.hpp"
#include "map_closures/GroundAlign.hpp"
#include "map_closures/MapClosures.hpp"
#include "pgo/pose_graph_optimizer.hpp"
#include "stl_vector_eigen.h"
#include "voxel_map/VoxelMap.hpp"

PYBIND11_MAKE_OPAQUE(std::vector<Eigen::Vector3d>);
PYBIND11_MAKE_OPAQUE(std::vector<Eigen::Vector3f>);
PYBIND11_MAKE_OPAQUE(std::vector<Eigen::Vector3i>);
PYBIND11_MAKE_OPAQUE(pgo::PoseGraphOptimizer::PoseIDMap);

namespace py = pybind11;
using namespace py::literals;

namespace map_closures {
Config GetConfigFromDict(const py::dict& cfg) {
  Config cpp_config;
  if (cfg.contains("density_threshold")) {
    cpp_config.density_threshold = cfg["density_threshold"].cast<float>();
  }
  if (cfg.contains("density_map_resolution")) {
    cpp_config.density_map_resolution = cfg["density_map_resolution"].cast<float>();
  }
  if (cfg.contains("sift_match_ratio")) {
    cpp_config.sift_match_ratio = cfg["sift_match_ratio"].cast<float>();
  }
  if (cfg.contains("density_map_gamma")) {
    cpp_config.density_map_gamma = cfg["density_map_gamma"].cast<float>();
  }
  if (cfg.contains("lbd_min_line_length")) {
    cpp_config.lbd_min_line_length = cfg["lbd_min_line_length"].cast<float>();
  }
  if (cfg.contains("lbd_match_ratio")) {
    cpp_config.lbd_match_ratio = cfg["lbd_match_ratio"].cast<float>();
  }
  if (cfg.contains("lbd_num_octaves")) {
    cpp_config.lbd_num_octaves = cfg["lbd_num_octaves"].cast<int>();
  }
  if (cfg.contains("lbd_scale")) {
    cpp_config.lbd_scale = cfg["lbd_scale"].cast<int>();
  }
  if (cfg.contains("lbd_weight")) {
    cpp_config.lbd_weight = cfg["lbd_weight"].cast<float>();
  }
  return cpp_config;
}
}  // namespace map_closures

// Helper: convert 4x4 numpy matrix to Sophus::SE3d
static Sophus::SE3d matrix4d_to_se3(const Eigen::Matrix4d& T) {
  Eigen::Matrix3d R = T.block<3, 3>(0, 0);
  Eigen::Vector3d t = T.block<3, 1>(0, 3);
  return Sophus::SE3d(Eigen::Quaterniond(R), t);
}

PYBIND11_MODULE(vegvisir_pybind, m) {
  // ---- Vector types ----
  auto vector3dvector = pybind_eigen_vector_of_vector<Eigen::Vector3d>(
      m, "Vector3dVector", "std::vector<Eigen::Vector3d>",
      py::py_array_to_vectors_double<Eigen::Vector3d>);
  auto vector3fvector = pybind_eigen_vector_of_vector<Eigen::Vector3f>(
      m, "Vector3fVector", "std::vector<Eigen::Vector3f>",
      py::py_array_to_vectors_float<Eigen::Vector3f>);
  auto vector3ivector = pybind_eigen_vector_of_vector<Eigen::Vector3i>(
      m, "Vector3iVector", "std::vector<Eigen::Vector3i>",
      py::py_array_to_vectors_int<Eigen::Vector3i>);

  // ---- PoseIDMap ----
  py::bind_map<pgo::PoseGraphOptimizer::PoseIDMap>(m, "PoseIDMap");

  // ---- PoseGraphOptimizer ----
  {
    using namespace pgo;
    py::class_<PoseGraphOptimizer> pgo_cls(m, "PoseGraphOptimizerCore", "");
    pgo_cls.def(py::init<int>(), "max_iterations"_a)
        .def("add_variable", &PoseGraphOptimizer::addVariable, "id"_a, "T"_a)
        .def("fix_variable", &PoseGraphOptimizer::fixVariable, "id"_a)
        .def("add_factor", &PoseGraphOptimizer::addFactor, "id_source"_a, "id_target"_a, "T"_a,
             "omega"_a)
        .def("optimize", &PoseGraphOptimizer::optimize)
        .def("estimates", &PoseGraphOptimizer::estimates)
        .def("read_graph", &PoseGraphOptimizer::readGraph, "filename"_a)
        .def("write_graph", &PoseGraphOptimizer::writeGraph, "filename"_a);
  }

  // ---- VoxelMap ----
  {
    using namespace voxel_map;
    py::class_<VoxelMap> vmap(m, "VoxelMapCore", "");
    vmap.def(py::init<double>(), "voxel_size"_a)
        .def("integrate_frame", &VoxelMap::integrateFrame, "points"_a, "pose"_a)
        .def("add_points", &VoxelMap::addPoints, "points"_a)
        .def("point_cloud", &VoxelMap::pointcloud)
        .def("clear", &VoxelMap::clear)
        .def("num_voxels", &VoxelMap::numVoxels)
        .def("per_voxel_point_and_normal", &VoxelMap::perVoxelPointAndNormal);
  }

  // ---- ClosureCandidate ----
  {
    using namespace map_closures;
    py::class_<ClosureCandidate> closure_candidate(m, "ClosureCandidate");
    closure_candidate.def(py::init<>())
        .def_readwrite("source_id", &ClosureCandidate::source_id)
        .def_readwrite("target_id", &ClosureCandidate::target_id)
        .def_readwrite("pose", &ClosureCandidate::pose)
        .def_readwrite("number_of_inliers", &ClosureCandidate::number_of_inliers)
        .def_readwrite("weighted_score", &ClosureCandidate::weighted_score)
        .def_readwrite("sift_inliers", &ClosureCandidate::sift_inliers)
        .def_readwrite("lbd_inliers", &ClosureCandidate::lbd_inliers);
  }

  // ---- MapClosures ----
  {
    using namespace map_closures;
    py::class_<MapClosures, std::shared_ptr<MapClosures>> mc(m, "MapClosuresCore", "");
    mc.def(py::init([]() { return std::make_shared<MapClosures>(); }))
        .def(py::init([](const py::dict& cfg) {
               auto config = GetConfigFromDict(cfg);
               return std::make_shared<MapClosures>(config);
             }),
             "config"_a)
        .def("get_top_k_closures", &MapClosures::getTopKClosures, "query_id"_a, "local_map"_a,
             "k"_a)
        .def("get_closures", &MapClosures::getClosures, "query_id"_a, "local_map"_a)
        .def("query_top_k_closures", &MapClosures::queryTopKClosures, "query_id"_a, "local_map"_a,
             "k"_a)
        .def("query_closures", &MapClosures::queryClosures, "query_id"_a, "local_map"_a)
        .def("get_density_map_from_id",
             [](MapClosures& self, const int& map_id) {
               const auto& density_map = self.getDensityMapFromId(map_id);
               Eigen::MatrixXf density_map_eigen;
               cv::cv2eigen(density_map.grid, density_map_eigen);
               return density_map_eigen;
             })
        .def("get_density_map_metadata",
             [](MapClosures& self, const int& map_id) {
               const auto& density_map = self.getDensityMapFromId(map_id);
               py::dict metadata;
               metadata["resolution"] = density_map.resolution;
               metadata["lower_bound"] = density_map.lower_bound;
               metadata["rows"] = density_map.grid.rows;
               metadata["cols"] = density_map.grid.cols;
               return metadata;
             })
        .def("get_available_map_ids", [](MapClosures& self) { return self.getAvailableMapIds(); })
        .def("get_ground_alignment",
             [](MapClosures& self, const int& map_id) { return self.getGroundAlignment(map_id); })
        .def("get_reference_poses",
             [](MapClosures& self) {
               const auto& ref = self.getReferencePoses();
               std::map<int, Eigen::Matrix4d> ordered(ref.begin(), ref.end());
               return ordered;
             })
        .def("get_reference_pose",
             [](MapClosures& self, const int& map_id) { return self.getReferencePose(map_id); })
        .def(
            "set_reference_pose",
            [](MapClosures& self, const int& map_id, const Eigen::Matrix4d& pose) {
              self.setReferencePose(map_id, pose);
            },
            "map_id"_a, "pose"_a)
        .def("get_local_map_points",
             [](MapClosures& self, const int& map_id) { return self.getLocalMapPoints(map_id); })
        .def("has_local_map_points",
             [](MapClosures& self, const int& map_id) { return self.hasLocalMapPoints(map_id); })
        .def("save", &MapClosures::save, "file_path"_a)
        .def("load", &MapClosures::load, "file_path"_a)
        .def("load_reference_poses", &MapClosures::loadReferencePoses, "file_path"_a)
        .def("load_local_map_points", &MapClosures::loadLocalMapPoints, "file_path"_a)
        .def("get_sift_keypoints_viz", [](MapClosures& self, int map_id) {
          const auto& dm = self.getDensityMapFromId(map_id);
          const auto& lb = dm.lower_bound;
          py::dict info;
          info["lower_bound_x"] = lb.x();
          info["lower_bound_y"] = lb.y();
          return info;
        });
  }

  // ---- Mode enum ----
  {
    using namespace vegvisir;
    py::enum_<Mode>(m, "Mode")
        .value("LOCALIZATION", Mode::LOCALIZATION)
        .value("SLAM", Mode::SLAM)
        .export_values();
  }

  // ---- LocalMap ----
  {
    using namespace vegvisir;
    py::class_<LocalMap>(m, "LocalMap")
        .def_property_readonly("id", &LocalMap::id)
        .def_property(
            "keypose", [](const LocalMap& lm) -> Eigen::Matrix4d { return lm.keypose(); },
            [](LocalMap& lm, const Eigen::Matrix4d& kp) { lm.keypose() = kp; })
        .def_property_readonly("local_trajectory",
                               [](const LocalMap& lm) { return lm.localTrajectory(); })
        .def_property_readonly("has_point_cloud", &LocalMap::hasPointCloud)
        .def("point_cloud",
             [](const LocalMap& lm) -> py::object {
               if (!lm.hasPointCloud()) {
                 return py::none();
               }
               const auto& pts = lm.pointCloud();
               Eigen::MatrixXd mat(pts.size(), 3);
               for (size_t i = 0; i < pts.size(); ++i) {
                 mat.row(i) = pts[i].transpose();
               }
               return py::cast(mat);
             })
        .def("endpose", &LocalMap::endpose);
  }

  // ---- LocalMapGraph ----
  {
    using namespace vegvisir;
    py::class_<LocalMapGraph>(m, "LocalMapGraph")
        .def(
            "__getitem__", [](LocalMapGraph& g, uint64_t key) -> LocalMap& { return g[key]; },
            py::return_value_policy::reference_internal)
        .def("has_local_map", &LocalMapGraph::hasLocalMap)
        .def("size", &LocalMapGraph::size)
        .def("empty", &LocalMapGraph::empty)
        .def("last_id", &LocalMapGraph::lastId)
        .def("last_keypose",
             [](const LocalMapGraph& g) -> Eigen::Matrix4d { return g.lastKeypose(); })
        .def("get_all_keyposes", &LocalMapGraph::getAllKeyposes)
        .def("get_all_ids", &LocalMapGraph::getAllIds)
        .def("local_maps", [](LocalMapGraph& g) {
          py::list result;
          for (auto& [id, lm] : g) {
            result.append(py::cast(&lm, py::return_value_policy::reference_internal, py::cast(g)));
          }
          return result;
        });
  }

  // ---- GnssOrigin ----
  {
    using namespace vegvisir;
    py::class_<GnssOrigin>(m, "GnssOrigin")
        .def(py::init<>())
        .def_readwrite("lat0", &GnssOrigin::lat0)
        .def_readwrite("lon0", &GnssOrigin::lon0)
        .def_readwrite("alt0", &GnssOrigin::alt0)
        .def_readwrite("valid", &GnssOrigin::valid);
  }

  // ---- MapMetadata ----
  {
    using namespace vegvisir;
    py::class_<MapMetadata>(m, "MapMetadata")
        .def(py::init<>())
        .def_readwrite("name", &MapMetadata::name)
        .def_readwrite("location", &MapMetadata::location)
        .def_readwrite("notes", &MapMetadata::notes)
        .def_readwrite("gnss_anchor_transform", &MapMetadata::gnss_anchor_transform)
        .def_readwrite("has_gnss_anchor", &MapMetadata::has_gnss_anchor)
        .def_readwrite("gnss_origin", &MapMetadata::gnss_origin);
  }

  // ---- VegvisirConfig ----
  {
    using namespace vegvisir;
    py::class_<VegvisirConfig>(m, "VegvisirConfigCore")
        .def(py::init<>())
        .def_readwrite("voxel_size", &VegvisirConfig::voxel_size)
        .def_readwrite("splitting_distance_slam", &VegvisirConfig::splitting_distance_slam)
        .def_readwrite("splitting_distance_localization",
                       &VegvisirConfig::splitting_distance_localization)
        .def_readwrite("overlap_threshold", &VegvisirConfig::overlap_threshold)
        .def_readwrite("pgo_max_iterations", &VegvisirConfig::pgo_max_iterations)
        .def_readwrite("inliers_threshold", &VegvisirConfig::inliers_threshold)
        .def_readwrite("icp_refinement_voxel_size", &VegvisirConfig::icp_refinement_voxel_size)
        .def_readwrite("icp_max_iterations", &VegvisirConfig::icp_max_iterations)
        .def_readwrite("icp_convergence_criterion", &VegvisirConfig::icp_convergence_criterion)
        .def_readwrite("icp_max_correspondence_distance",
                       &VegvisirConfig::icp_max_correspondence_distance);
  }

  // ---- Vegvisir ----
  {
    using namespace vegvisir;
    py::class_<Vegvisir>(m, "VegvisirCore")
        .def(py::init<const std::string&, Mode, const VegvisirConfig&>(), "map_database_path"_a,
             "mode"_a = Mode::SLAM, "config"_a = VegvisirConfig{})
        .def(
            "update",
            [](Vegvisir& self, const std::vector<Eigen::Vector3d>& points,
               const Eigen::Matrix4d& absolute_pose) {
              auto abs_se3 = matrix4d_to_se3(absolute_pose);
              self.update(points, abs_se3);
            },
            "points"_a, "absolute_pose"_a,
            "Update the localizer with 3D points and absolute pose (4x4)")
        .def("save_database", &Vegvisir::saveDatabase)
        .def("fine_grained_optimization", &Vegvisir::fineGrainedOptimization)
        .def("fine_grained_optimization_and_update_keyposes",
             &Vegvisir::fineGrainedOptimizationAndUpdateKeyposes,
             "Run PGO and update internal keyposes to match optimized poses. "
             "This ensures frame consistency between the saved map and "
             "localization.")
        .def("add_gnss_measurement", &Vegvisir::addGnssMeasurement, "pose_index"_a,
             "position_enu"_a, "information_matrix"_a,
             "Add GNSS measurement for pose graph optimization")
        .def("clear_gnss_measurements", &Vegvisir::clearGnssMeasurements)
        .def("get_num_gnss_measurements", &Vegvisir::getNumGnssMeasurements)
        .def("get_optimized_alignment_transform", &Vegvisir::getOptimizedAlignmentTransform,
             "Get the optimized T_ENU_map alignment transform after PGO. "
             "This is the transform that maps positions from map frame to ENU "
             "frame: p_ENU = T_ENU_map * p_map")
        .def("set_initial_alignment_estimate", &Vegvisir::setInitialAlignmentEstimate,
             "T_enu_map"_a,
             "Set initial estimate for T_ENU_map alignment before PGO. "
             "Call this before fine_grained_optimization if you have a good "
             "initial guess. If not set, PGO will initialize from Identity.")
        .def("set_gnss_anchor_transform", &Vegvisir::setGnssAnchorTransform, "T_enu_map"_a,
             "Set the GNSS anchor transform (T_ENU_map) to be saved with the "
             "map. Call this before save_database() to include GNSS anchoring "
             "in metadata.")
        .def("set_gnss_origin", &Vegvisir::setGnssOrigin, "lat0"_a, "lon0"_a, "alt0"_a,
             "Set the GNSS origin (WGS84 coordinates) used for ENU conversion. "
             "Call this before save_database() to include the origin in "
             "metadata.")
        .def("get_gnss_anchor_transform", &Vegvisir::getGnssAnchorTransform,
             "Get the GNSS anchor transform from loaded metadata. "
             "Returns identity if no anchor is set.")
        .def("has_gnss_anchor", &Vegvisir::hasGnssAnchor,
             "Check if GNSS anchor data is available in the loaded map.")
        .def("get_gnss_origin", &Vegvisir::getGnssOrigin,
             "Get the GNSS origin from loaded metadata.")
        .def("get_mode", &Vegvisir::getMode)
        .def("get_map_to_odom_transform", &Vegvisir::getMapToOdomTransform)
        .def("get_covariance", &Vegvisir::getCovariance)
        .def("get_base_in_map_frame", &Vegvisir::getBaseInMapFrame)
        .def("get_current_odom_base",
             [](const Vegvisir& self) -> Eigen::Matrix4d {
               return self.getCurrentOdomBase().matrix();
             })
        .def("get_num_closures", &Vegvisir::getNumClosures)
        .def(
            "get_closures", [](const Vegvisir& self) { return self.getClosures(); },
            py::return_value_policy::reference_internal)
        .def("get_available_map_ids", &Vegvisir::getAvailableMapIds)
        .def(
            "get_local_map_graph",
            [](const Vegvisir& self) -> const LocalMapGraph& { return self.getLocalMapGraph(); },
            py::return_value_policy::reference_internal)
        .def("get_reference_poses",
             [](const Vegvisir& self) {
               const auto& ref = self.getReferencePoses();
               std::map<int, Eigen::Matrix4d> ordered(ref.begin(), ref.end());
               return ordered;
             })
        .def(
            "get_reference_pose",
            [](const Vegvisir& self, int map_id) { return self.getReferencePose(map_id); },
            "map_id"_a)
        .def(
            "get_ground_alignment",
            [](const Vegvisir& self, int map_id) { return self.getGroundAlignment(map_id); },
            "map_id"_a)
        .def(
            "get_density_map",
            [](const Vegvisir& self, int map_id) {
              const auto& density_map = self.getDensityMap(map_id);
              Eigen::MatrixXf density_map_eigen;
              cv::cv2eigen(density_map.grid, density_map_eigen);
              return density_map_eigen;
            },
            "map_id"_a)
        .def(
            "get_density_map_metadata",
            [](const Vegvisir& self, int map_id) {
              const auto& density_map = self.getDensityMap(map_id);
              py::dict metadata;
              metadata["resolution"] = density_map.resolution;
              metadata["lower_bound"] = density_map.lower_bound;
              metadata["rows"] = density_map.grid.rows;
              metadata["cols"] = density_map.grid.cols;
              return metadata;
            },
            "map_id"_a)
        .def("has_local_map_points", &Vegvisir::hasLocalMapPoints, "map_id"_a)
        .def(
            "get_local_map_points",
            [](const Vegvisir& self, int map_id) {
              const auto& pts = self.getLocalMapPoints(map_id);
              Eigen::MatrixXd mat(pts.size(), 3);
              for (size_t i = 0; i < pts.size(); ++i) {
                mat.row(i) = pts[i].transpose();
              }
              return mat;
            },
            "map_id"_a)
        .def_property(
            "map_metadata", [](const Vegvisir& self) { return self.getMapMetadata(); },
            [](Vegvisir& self, const MapMetadata& md) { self.setMapMetadata(md); })
        .def_property_readonly("poses", [](const Vegvisir& self) {
          // Return poses with same indexing as fine-grained PGO:
          // - id=0: first keypose
          // - id=1+: keypose * traj[i+1] for each frame
          const auto& graph = self.getLocalMapGraph();
          std::vector<Eigen::Matrix4d> poses;
          bool first = true;
          for (auto it = graph.cbegin(); it != graph.cend(); ++it) {
            const auto& node = it->second;
            const auto& traj = node.localTrajectory();
            if (first) {
              // First pose is the first keypose (matches PGO id=0)
              poses.push_back(node.keypose());
              first = false;
            }
            // Same loop as PGO: i from 0 to traj.size()-2, add traj[i+1]
            for (size_t i = 0; i + 1 < traj.size(); ++i) {
              poses.push_back(node.keypose() * traj[i + 1]);
            }
          }
          return poses;
        });
  }

  // ---- Free functions ----
  m.def("align_map_to_local_ground", &map_closures::alignToLocalGround, "pointcloud"_a,
        "resolution"_a);

  // ---- Constants ----
  m.attr("LOCAL_MAPS_TO_SKIP") = map_closures::LOCAL_MAPS_TO_SKIP;
  m.attr("MIN_NUMBER_OF_MATCHES") = map_closures::MIN_NUMBER_OF_MATCHES;
  m.attr("RANSAC_INLIER_THRESHOLD_M") = map_closures::RANSAC_INLIER_THRESHOLD_M;
}
