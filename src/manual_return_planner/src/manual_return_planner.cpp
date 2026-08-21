#include "manual_return_planner/manual_return_planner.h"
#include "manual_return_planner/safe_rdp.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>

namespace manual_return_planner {
namespace {

bool finitePoint(const TrajectoryPoint& p) {
  return std::isfinite(p.timestamp) && p.position.allFinite();
}

bool finiteAuxiliaryState(const TrajectoryPoint& p) {
  return p.velocity.allFinite() && std::isfinite(p.roll) &&
         std::isfinite(p.pitch) && std::isfinite(p.yaw);
}

double unwrapNear(double angle, double reference) {
  while (angle - reference > M_PI) angle -= 2.0 * M_PI;
  while (angle - reference < -M_PI) angle += 2.0 * M_PI;
  return angle;
}

std::vector<std::string> splitCsv(const std::string& line) {
  std::vector<std::string> fields;
  std::stringstream stream(line);
  std::string field;
  while (std::getline(stream, field, ',')) {
    if (!field.empty() && field.back() == '\r') field.pop_back();
    const std::size_t first = field.find_first_not_of(" \t");
    const std::size_t last = field.find_last_not_of(" \t");
    fields.push_back(first == std::string::npos ? ""
                                                : field.substr(first, last - first + 1));
  }
  return fields;
}

}  // namespace

bool TrajectoryPreprocessor::detectPositionJump(
    const TrajectoryPoint& previous, const TrajectoryPoint& current,
    const ManualReturnConfig& config, double* observed_speed) const {
  const double dt = current.timestamp - previous.timestamp;
  if (dt <= 1e-9) {
    if (observed_speed) *observed_speed = std::numeric_limits<double>::infinity();
    return false;
  }
  const double speed = (current.position - previous.position).norm() / dt;
  if (observed_speed) *observed_speed = speed;
  return speed > config.max_reasonable_speed;
}

bool TrajectoryPreprocessor::preprocess(
    const std::vector<TrajectoryPoint>& input, const ManualReturnConfig& config,
    std::vector<TrajectoryPoint>* output, std::string* warning_message,
    ManualReturnStatus* failure_status) const {
  if (!output || !warning_message || !failure_status) return false;
  output->clear();
  warning_message->clear();
  *failure_status = ManualReturnStatus::INVALID_INPUT;
  if (input.empty()) {
    *failure_status = ManualReturnStatus::TOO_FEW_POINTS;
    return false;
  }
  if (!std::isfinite(config.min_point_spacing) ||
      !std::isfinite(config.rdp_epsilon) ||
      !std::isfinite(config.max_segment_length) ||
      !std::isfinite(config.max_reasonable_speed) ||
      config.min_point_spacing < 0.0 || config.rdp_epsilon < 0.0 ||
      config.max_segment_length <= 0.0 || config.max_reasonable_speed <= 0.0) {
    *warning_message = "trajectory preprocessing parameters are invalid";
    return false;
  }

  std::ostringstream warnings;
  std::size_t jump_count = 0;
  for (std::size_t i = 0; i < input.size(); ++i) {
    const TrajectoryPoint& point = input[i];
    if (!finitePoint(point)) {
      *failure_status = ManualReturnStatus::INVALID_INPUT;
      *warning_message = "timestamp and position must be finite (row " +
                         std::to_string(i) + ")";
      return false;
    }
    if (!finiteAuxiliaryState(point)) {
      warnings << "non-finite velocity/attitude at row " << i
               << " (position retained); ";
    }
    if (i > 0 && point.timestamp + 1e-9 < input[i - 1].timestamp) {
      *failure_status = ManualReturnStatus::INVALID_TIMESTAMP;
      *warning_message = "trajectory timestamps are not ordered at row " +
                         std::to_string(i);
      return false;
    }
    if (i > 0) {
      double speed = 0.0;
      if (detectPositionJump(input[i - 1], point, config, &speed)) {
        ++jump_count;
        warnings << "position jump at row " << i << " (" << speed
                 << " m/s); ";
      }
    }
    if (output->empty() ||
        (point.position - output->back().position).norm() >=
            config.min_point_spacing) {
      output->push_back(point);
    }
  }
  // The final measured position is never allowed to disappear due to spacing.
  if (output->empty() || output->back().position != input.back().position) {
    output->push_back(input.back());
  }
  if (output->size() < 2) {
    *failure_status = ManualReturnStatus::TOO_FEW_POINTS;
    *warning_message = "at least two distinct trajectory points are required";
    return false;
  }
  if (jump_count > 0) {
    *warning_message = warnings.str();
  } else if (!warnings.str().empty()) {
    *warning_message = warnings.str();
  }
  *failure_status = ManualReturnStatus::SUCCESS;
  return true;
}

double RdpSimplifier::pointToSegmentDistance(const Eigen::Vector3d& point,
                                             const Eigen::Vector3d& start,
                                             const Eigen::Vector3d& end) {
  const Eigen::Vector3d segment = end - start;
  const double squared_length = segment.squaredNorm();
  if (squared_length <= 1e-18) return (point - start).norm();
  double t = (point - start).dot(segment) / squared_length;
  t = std::max(0.0, std::min(1.0, t));
  return (point - (start + t * segment)).norm();
}

std::vector<TrajectoryPoint> RdpSimplifier::simplify(
    const std::vector<TrajectoryPoint>& points, double epsilon,
    double* max_deviation) {
  if (max_deviation) *max_deviation = 0.0;
  if (points.size() <= 2 || epsilon < 0.0) return points;

  std::vector<bool> keep(points.size(), false);
  keep.front() = true;
  keep.back() = true;
  struct Segment { std::size_t first; std::size_t last; };
  std::vector<Segment> stack;
  stack.push_back({0, points.size() - 1});
  while (!stack.empty()) {
    const Segment segment = stack.back();
    stack.pop_back();
    if (segment.last <= segment.first + 1) continue;
    double largest = -1.0;
    std::size_t largest_index = segment.first;
    for (std::size_t i = segment.first + 1; i < segment.last; ++i) {
      const double distance = pointToSegmentDistance(
          points[i].position, points[segment.first].position,
          points[segment.last].position);
      if (distance > largest) {
        largest = distance;
        largest_index = i;
      }
    }
    if (max_deviation) *max_deviation = std::max(*max_deviation, largest);
    if (largest > epsilon) {
      keep[largest_index] = true;
      stack.push_back({segment.first, largest_index});
      stack.push_back({largest_index, segment.last});
    }
  }
  std::vector<TrajectoryPoint> result;
  result.reserve(points.size());
  for (std::size_t i = 0; i < points.size(); ++i) {
    if (keep[i]) result.push_back(points[i]);
  }
  // Report the deviation of the returned polyline, rather than the largest
  // deviation of an intermediate stack segment (which may have been split
  // later).  This makes the metric directly comparable with epsilon.
  if (max_deviation) {
    *max_deviation = 0.0;
    std::size_t simplified_index = 0;
    for (std::size_t i = 0; i < points.size(); ++i) {
      if (simplified_index + 1 >= result.size()) break;
      if (points[i].position == result[simplified_index + 1].position &&
          points[i].timestamp == result[simplified_index + 1].timestamp) {
        ++simplified_index;
        continue;
      }
      *max_deviation = std::max(
          *max_deviation,
          pointToSegmentDistance(points[i].position,
                                 result[simplified_index].position,
                                 result[simplified_index + 1].position));
    }
  }
  return result;
}

bool CsvTrajectoryReader::read(const std::string& path,
                               std::vector<TrajectoryPoint>* points,
                               std::string* error_message) {
  if (!points || !error_message) return false;
  points->clear();
  std::ifstream input(path.c_str());
  if (!input.is_open()) {
    *error_message = "cannot open CSV: " + path;
    return false;
  }
  std::string line;
  if (!std::getline(input, line)) {
    *error_message = "CSV is empty";
    return false;
  }
  std::vector<std::string> header = splitCsv(line);
  // Accept the UTF-8 BOM emitted by some spreadsheet tools while keeping the
  // actual schema strict and deterministic.
  if (!header.empty() && header.front().size() >= 3 &&
      static_cast<unsigned char>(header.front()[0]) == 0xEF &&
      static_cast<unsigned char>(header.front()[1]) == 0xBB &&
      static_cast<unsigned char>(header.front()[2]) == 0xBF) {
    header.front().erase(0, 3);
  }
  const std::vector<std::string> expected = {
      "timestamp", "x", "y", "z", "vx", "vy", "vz", "roll", "pitch", "yaw"};
  if (header != expected) {
    *error_message = "CSV header must be timestamp,x,y,z,vx,vy,vz,roll,pitch,yaw";
    return false;
  }
  std::size_t row = 1;
  while (std::getline(input, line)) {
    ++row;
    if (line.empty()) continue;
    const std::vector<std::string> fields = splitCsv(line);
    if (fields.size() != 10) {
      *error_message = "CSV row " + std::to_string(row) + " has " +
                       std::to_string(fields.size()) + " fields";
      return false;
    }
    try {
      TrajectoryPoint point;
      point.timestamp = std::stod(fields[0]);
      point.position = Eigen::Vector3d(std::stod(fields[1]), std::stod(fields[2]),
                                       std::stod(fields[3]));
      point.velocity = Eigen::Vector3d(std::stod(fields[4]), std::stod(fields[5]),
                                       std::stod(fields[6]));
      point.roll = std::stod(fields[7]);
      point.pitch = std::stod(fields[8]);
      point.yaw = std::stod(fields[9]);
      points->push_back(point);
    } catch (const std::exception& exception) {
      *error_message = "invalid numeric value at CSV row " +
                       std::to_string(row) + ": " + exception.what();
      return false;
    }
  }
  if (points->empty()) {
    *error_message = "CSV contains no trajectory points";
    return false;
  }
  return true;
}

bool CsvTrajectoryReader::write(const std::string& path,
                                const std::vector<TrajectoryPoint>& points,
                                std::string* error_message) {
  if (!error_message) return false;
  std::ofstream output(path.c_str());
  if (!output.is_open()) {
    *error_message = "cannot write CSV: " + path;
    return false;
  }
  output << "timestamp,x,y,z,vx,vy,vz,roll,pitch,yaw\n";
  output << std::setprecision(12);
  for (const TrajectoryPoint& point : points) {
    output << point.timestamp << ',' << point.position.x() << ','
           << point.position.y() << ',' << point.position.z() << ','
           << point.velocity.x() << ',' << point.velocity.y() << ','
           << point.velocity.z() << ',' << point.roll << ',' << point.pitch
           << ',' << point.yaw << '\n';
  }
  return true;
}

namespace {

// Find the takeoff hover point: the highest-z sample in the opening segment
// that stays within `xy_tol` of the first position.  Returns its index, or 0
// if there is no takeoff climb (the opening z never rises by `min_rise`), in
// which case the original first point remains Home.
std::size_t takeoffHomeIndex(const std::vector<TrajectoryPoint>& points,
                             double xy_tol, double min_rise) {
  if (points.size() < 2) return 0;
  const Eigen::Vector3d start = points.front().position;
  const double z_start = start.z();
  std::size_t best = 0;
  double best_z = z_start;
  for (std::size_t i = 1; i < points.size(); ++i) {
    const double xy = (points[i].position - start).head<2>().norm();
    if (xy > xy_tol) break;  // left the launch point; takeoff segment ended
    if (points[i].position.z() > best_z) {
      best_z = points[i].position.z();
      best = i;
    }
  }
  // No real climb: keep the original first point as Home.
  if (best_z - z_start < min_rise) return 0;
  return best;
}

}  // namespace

double ManualReturnPlanner::pathLength(
    const std::vector<TrajectoryPoint>& points) {
  double length = 0.0;
  for (std::size_t i = 1; i < points.size(); ++i)
    length += (points[i].position - points[i - 1].position).norm();
  return length;
}

double ManualReturnPlanner::maxDeviation(
    const std::vector<TrajectoryPoint>& source,
    const std::vector<TrajectoryPoint>& simplified) {
  if (source.size() <= 2 || simplified.size() < 2) return 0.0;
  double maximum = 0.0;
  std::size_t source_start = 0;
  for (std::size_t segment = 1; segment < simplified.size(); ++segment) {
    std::size_t source_end = source_start;
    while (source_end + 1 < source.size() &&
           (source[source_end].position - simplified[segment].position).norm() > 1e-9) {
      ++source_end;
    }
    for (std::size_t i = source_start; i <= source_end && i < source.size(); ++i) {
      maximum = std::max(maximum, RdpSimplifier::pointToSegmentDistance(
          source[i].position, simplified[segment - 1].position,
          simplified[segment].position));
    }
    source_start = source_end;
  }
  return maximum;
}

std::vector<TrajectoryPoint> ManualReturnPlanner::enforceMaxSegmentLength(
    const std::vector<TrajectoryPoint>& simplified,
    const std::vector<TrajectoryPoint>& history, double max_segment_length) {
  if (simplified.size() < 2 || max_segment_length <= 0.0) return simplified;
  std::vector<TrajectoryPoint> result;
  result.push_back(simplified.front());
  std::size_t history_index = 0;
  for (std::size_t i = 1; i < simplified.size(); ++i) {
    // RDP returns copies of history points.  Match by timestamp and position
    // so that inserted points always come from the measured trajectory.
    std::size_t target_index = history_index;
    for (std::size_t h = history_index + 1; h < history.size(); ++h) {
      if (history[h].timestamp == simplified[i].timestamp &&
          history[h].position == simplified[i].position) {
        target_index = h;
        break;
      }
    }
    if (target_index == history_index) {
      for (std::size_t h = history_index + 1; h < history.size(); ++h) {
        if (history[h].position == simplified[i].position) {
          target_index = h;
          break;
        }
      }
    }

    // Pick the furthest historical sample that is still within L_max of the
    // current output point.  Repeating this guarantees every achievable
    // segment is bounded without inventing points in free space.
    std::size_t current_index = history_index;
    while ((simplified[i].position - result.back().position).norm() >
           max_segment_length + 1e-9 && current_index < target_index) {
      std::size_t best_index = current_index;
      for (std::size_t h = current_index + 1; h < target_index; ++h) {
        if ((history[h].position - result.back().position).norm() <=
            max_segment_length + 1e-9) {
          best_index = h;
        }
      }
      // If the recorded sampling itself has a gap larger than L_max, retain
      // the next real sample.  The planner validates this below and fails
      // explicitly instead of silently creating a spatial shortcut.
      if (best_index == current_index) best_index = current_index + 1;
      result.push_back(history[best_index]);
      current_index = best_index;
    }
    result.push_back(simplified[i]);
    history_index = target_index;
  }
  return result;
}

std::vector<ReturnWaypoint> ManualReturnPlanner::makeWaypoints(
    const std::vector<TrajectoryPoint>& reversed) {
  std::vector<ReturnWaypoint> waypoints;
  waypoints.reserve(reversed.size());
  double previous_yaw = reversed.empty() ? 0.0 : reversed.front().yaw;
  for (std::size_t i = 0; i < reversed.size(); ++i) {
    ReturnWaypoint waypoint;
    waypoint.position = reversed[i].position;
    if (i + 1 < reversed.size()) {
      const Eigen::Vector3d delta = reversed[i + 1].position - reversed[i].position;
      if (delta.head<2>().norm() > 1e-9)
        waypoint.yaw = std::atan2(delta.y(), delta.x());
      else
        waypoint.yaw = previous_yaw;
    } else {
      waypoint.yaw = reversed[i].yaw;
    }
    waypoint.yaw = unwrapNear(waypoint.yaw, previous_yaw);
    previous_yaw = waypoint.yaw;
    waypoints.push_back(waypoint);
  }
  return waypoints;
}

ReturnPlanResult ManualReturnPlanner::planManualReturn(
    const std::vector<TrajectoryPoint>& flown_trajectory,
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& map_cloud,
    const ManualReturnConfig& config) const {
  ReturnPlanResult result;
  result.original_points = flown_trajectory;
  result.raw_point_num = flown_trajectory.size();
  const std::chrono::steady_clock::time_point start =
      std::chrono::steady_clock::now();
  if (!std::isfinite(config.rdp_epsilon) || config.rdp_epsilon < 0.0 ||
      !std::isfinite(config.max_segment_length) ||
      config.max_segment_length <= 0.0) {
    result.status = ManualReturnStatus::INVALID_INPUT;
    result.message = "rdp_epsilon and max_segment_length must be finite and positive";
    result.planning_time_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - start).count();
    return result;
  }
  TrajectoryPreprocessor preprocessor;
  ManualReturnStatus failure = ManualReturnStatus::INVALID_INPUT;
  std::string warning;
  if (!preprocessor.preprocess(flown_trajectory, config,
                               &result.preprocessed_points, &warning,
                               &failure)) {
    result.status = failure;
    result.message = warning;
    result.planning_time_ms =
        std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - start).count();
    return result;
  }
  result.preprocessed_point_num = result.preprocessed_points.size();
  result.original_path_length = pathLength(result.original_points);

  // Strip the pre-takeoff climb so Home becomes the hover point rather than
  // the initial (low/negative-z) pose recorded before takeoff.  See
  // ManualReturnConfig::takeoff_xy_tolerance / takeoff_min_rise.
  {
    const std::size_t home_index = takeoffHomeIndex(
        result.preprocessed_points, config.takeoff_xy_tolerance,
        config.takeoff_min_rise);
    if (home_index > 0 && home_index < result.preprocessed_points.size()) {
      result.preprocessed_points.erase(
          result.preprocessed_points.begin(),
          result.preprocessed_points.begin() +
              static_cast<std::ptrdiff_t>(home_index));
    }
  }

  result.reversed_points = result.preprocessed_points;
  std::reverse(result.reversed_points.begin(), result.reversed_points.end());
  double rdp_deviation = 0.0;
  std::vector<TrajectoryPoint> simplified = RdpSimplifier::simplify(
      result.reversed_points, config.rdp_epsilon, &rdp_deviation);
  result.return_waypoints = makeWaypoints(enforceMaxSegmentLength(
      simplified, result.reversed_points, config.max_segment_length));
  for (std::size_t i = 1; i < result.return_waypoints.size(); ++i) {
    const double segment_length =
        (result.return_waypoints[i].position -
         result.return_waypoints[i - 1].position).norm();
    if (!std::isfinite(segment_length) ||
        segment_length > config.max_segment_length + 1e-6) {
      result.status = ManualReturnStatus::TRAJECTORY_DISCONTINUITY;
      result.message = "historical samples cannot satisfy max_segment_length";
      result.planning_time_ms = std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - start).count();
      return result;
    }
  }
  result.final_point_num = result.return_waypoints.size();
  result.return_path_length = 0.0;
  for (std::size_t i = 1; i < result.return_waypoints.size(); ++i)
    result.return_path_length +=
        (result.return_waypoints[i].position - result.return_waypoints[i - 1].position).norm();
  result.max_rdp_deviation = rdp_deviation;
  result.compression_ratio = result.raw_point_num == 0
                                 ? 0.0
                                 : static_cast<double>(result.final_point_num) /
                                       static_cast<double>(result.raw_point_num);
      // --- Safe-RDP V1.1.0: validate the compressed path against the PCD. ---
    result.safe_waypoints = result.return_waypoints;
    result.safe_rdp_enabled = config.safe_rdp.enabled;
    result.original_rdp_point_num = result.return_waypoints.size();
    result.safe_point_num = result.return_waypoints.size();
    result.shortcut_count = 0;  // V1.1.0 validates only; no shortcut optimization

    if (config.safe_rdp.enabled && map_cloud && !map_cloud->empty()) {
      SafeRdpPlanner safe_planner;
      const SafeRdpResult safe =
          safe_planner.validate(result.return_waypoints, map_cloud,
                                config.safe_rdp);
      result.clearance_available = safe.clearance_available;
      result.min_clearance_m = safe.min_clearance_m;
      result.collision_check_count = safe.collision_check_count;
      result.unsafe_segments = safe.unsafe_segments;
      result.validated_segments = safe.validated_segments;
      result.voxelized_cloud_size = safe.voxelized_cloud_size;
      result.safe_waypoints = safe.safe_path;
      result.safe_point_num = result.safe_waypoints.size();
      if (!safe.safe) {
        result.status = ManualReturnStatus::NO_SAFE_PATH;
        result.message = safe.message;
        result.planning_time_ms =
            std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - start).count();
        return result;
      }
    } else {
      result.clearance_available = false;
      result.min_clearance_m = 0.0;
      result.collision_check_count = 0;
      result.unsafe_segments = 0;
      result.validated_segments = 0;
      result.voxelized_cloud_size = 0;
    }

result.status = result.final_point_num < result.preprocessed_point_num
                      ? ManualReturnStatus::SUCCESS_RDP
                      : ManualReturnStatus::SUCCESS_DENSE_BACKTRACK;
  result.message = warning.empty() ? "manual return path planned" : warning;
  result.planning_time_ms =
      std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - start).count();
  return result;
}

double ReturnMetricsAnalyzer::pointToPolylineDistance(
    const Eigen::Vector3d& point,
    const std::vector<TrajectoryPoint>& polyline) {
  if (polyline.empty()) return std::numeric_limits<double>::infinity();
  if (polyline.size() == 1) return (point - polyline[0].position).norm();
  double best = std::numeric_limits<double>::infinity();
  for (std::size_t i = 0; i + 1 < polyline.size(); ++i) {
    best = std::min(best, RdpSimplifier::pointToSegmentDistance(
                              point, polyline[i].position,
                              polyline[i + 1].position));
  }
  return best;
}

ReturnMetricsAnalyzer::DeviationStats ReturnMetricsAnalyzer::computePathDeviation(
    const std::vector<TrajectoryPoint>& simplified_return_path,
    const std::vector<TrajectoryPoint>& history) {
  DeviationStats stats;
  if (simplified_return_path.size() < 2 || history.size() < 2) {
    return stats;
  }
  std::vector<double> deviations;
  // Densely sample each return segment so straight-line simplifications that
  // cut across a curved flown corridor still show a non-zero deviation.
  const double step = 0.05;  // [m]
  for (std::size_t i = 0; i + 1 < simplified_return_path.size(); ++i) {
    const Eigen::Vector3d a = simplified_return_path[i].position;
    const Eigen::Vector3d b = simplified_return_path[i + 1].position;
    const double length = (b - a).norm();
    const int samples = std::max(1, static_cast<int>(std::ceil(length / step)));
    for (int k = 0; k <= samples; ++k) {
      const double t = static_cast<double>(k) / static_cast<double>(samples);
      const Eigen::Vector3d p = a + t * (b - a);
      deviations.push_back(pointToPolylineDistance(p, history));
    }
  }
  if (deviations.empty()) return stats;
  std::sort(deviations.begin(), deviations.end());
  stats.max = deviations.back();
  double sum = 0.0;
  for (double d : deviations) sum += d;
  stats.mean = sum / static_cast<double>(deviations.size());
  stats.p95 = deviations[static_cast<std::size_t>(
      0.95 * static_cast<double>(deviations.size() - 1))];
  return stats;
}

const char* statusToString(ManualReturnStatus status) {
  switch (status) {
    case ManualReturnStatus::SUCCESS: return "SUCCESS";
    case ManualReturnStatus::SUCCESS_RDP: return "SUCCESS_RDP";
    case ManualReturnStatus::SUCCESS_DENSE_BACKTRACK: return "SUCCESS_DENSE_BACKTRACK";
    case ManualReturnStatus::INVALID_INPUT: return "INVALID_INPUT";
    case ManualReturnStatus::TOO_FEW_POINTS: return "TOO_FEW_POINTS";
    case ManualReturnStatus::INVALID_TIMESTAMP: return "INVALID_TIMESTAMP";
    case ManualReturnStatus::TRAJECTORY_DISCONTINUITY: return "TRAJECTORY_DISCONTINUITY";
    case ManualReturnStatus::FRAME_ERROR: return "FRAME_ERROR";
    case ManualReturnStatus::EXECUTION_ERROR: return "EXECUTION_ERROR";
    case ManualReturnStatus::NO_SAFE_PATH: return "NO_SAFE_PATH";
    case ManualReturnStatus::INTERNAL_ERROR: return "INTERNAL_ERROR";
  }
  return "UNKNOWN";
}

}  // namespace manual_return_planner
