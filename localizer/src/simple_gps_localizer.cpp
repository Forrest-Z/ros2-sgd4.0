
#include "localizer/simple_gps_localizer.hpp"

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "message_filters/subscriber.h"
#include "nav2_amcl/angleutils.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_amcl/pf/pf.hpp"
#include "nav2_util/string_utils.hpp"
#include "nav2_amcl/sensors/laser/laser.hpp"
#include "tf2/convert.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/message_filter.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/create_timer_ros.h"

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#include "tf2/utils.h"
#pragma GCC diagnostic pop

//#include "portable_utils.h"

using namespace std;
using namespace std::placeholders;
using namespace std::chrono_literals;

using nav2_util::geometry_utils::orientationAroundZAxis;

class Simple_GPS_Localizer : public rclcpp::Node
{
public:
  Simple_GPS_Localizer() : Node("simple_gps_localizer")
  {
    RCLCPP_INFO(get_logger(), "Creating");
    
    
  }

  ~Simple_GPS_Localizer()
  {

  }

  nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & /*state*/)
  {
    RCLCPP_INFO(get_logger(), "Configuring");

    InitParameters();
    InitTransforms();
    InitMessageFilters();
    InitPubSub();
    InitServices();
    InitOdometry();
    InitParticleFilter();
    InitLaserScan();

    return nav2_util::CallbackReturn::SUCCESS;
  }

  void WaitForTransforms()
  {
    std::string tf_error;

    RCLCPP_INFO(get_logger(), "Checking that transform thread is ready");

    while (rclcpp::ok() &&
      !tf_buffer_->canTransform(
        global_frame_id_, odom_frame_id_, tf2::TimePointZero,
        transform_tolerance_, &tf_error))
    {
      RCLCPP_INFO(
        get_logger(), "Timed out waiting for transform from %s to %s"
        " to become available, tf error: %s",
        odom_frame_id_.c_str(), global_frame_id_.c_str(), tf_error.c_str());

      // The error string will accumulate and errors will typically be the same, so the last
      // will do for the warning above. Reset the string here to avoid accumulation.
      tf_error.clear();
    }
  }

  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & /*state*/)
  {
    RCLCPP_INFO(get_logger(), "Activating");

    // Lifecycle publishers must be explicitly activated
    pose_pub_->on_activate();
    particlecloud_pub_->on_activate();
    particle_cloud_pub_->on_activate();

    RCLCPP_WARN(
      get_logger(),
      "Publishing the particle cloud as geometry_msgs/PoseArray msg is deprecated, "
      "will be published as nav2_msgs/ParticleCloud in the future");

    bool first_pose_sent_ = false;

    // Keep track of whether we're in the active state. We won't
    // process incoming callbacks until we are
    isActive = true;

    if (set_initial_pose) {
      auto msg = std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>();

      msg->header.stamp = now();
      msg->header.frame_id = global_frame_id;
      msg->pose.pose.position.x = initial_pose_x;
      msg->pose.pose.position.y = initial_pose_y;
      msg->pose.pose.position.z = initial_pose_z;
      msg->pose.pose.orientation = orientationAroundZAxis(initial_pose_yaw);

      InitialPoseReceived(msg);
    } else if (init_pose_received_on_inactive) {
      handleInitialPose(last_published_pose_);
    }

    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & /*state*/)
  {
    RCLCPP_INFO(get_logger(), "Deactivating");

    isActive = false;

    // Lifecycle publishers must be explicitly deactivated
    pose_pub_->on_deactivate();
    particlecloud_pub_->on_deactivate();
    particle_cloud_pub_->on_deactivate();

    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & /*state*/)
  {
    RCLCPP_INFO(get_logger(), "Cleaning up");

    // Get rid of the inputs first (services and message filter input), so we
    // don't continue to process incoming messages
    global_loc_srv_.reset();
    nomotion_update_srv_.reset();
    initial_pose_sub_.reset();
    laser_scan_connection_.disconnect();
    laser_scan_filter_.reset();
    laser_scan_sub_.reset();

    // Map
    map_free(map_);
    map_ = nullptr;
    first_map_received_ = false;
    free_space_indices.resize(0);

    // Transforms
    tf_broadcaster_.reset();
    tf_listener_.reset();
    tf_buffer_.reset();

    // PubSub
    pose_pub_.reset();
    particlecloud_pub_.reset();
    particle_cloud_pub_.reset();

    // Odometry
    motion_model_.reset();

    // Particle Filter
    pf_free(pf_);
    pf_ = nullptr;

    if (set_initial_pose) {
      set_parameter(
        rclcpp::Parameter(
          "initial_pose.x",
          rclcpp::ParameterValue(last_published_pose_.pose.pose.position.x)));
      set_parameter(
        rclcpp::Parameter(
          "initial_pose.y",
          rclcpp::ParameterValue(last_published_pose_.pose.pose.position.y)));
      set_parameter(
        rclcpp::Parameter(
          "initial_pose.z",
          rclcpp::ParameterValue(last_published_pose_.pose.pose.position.z)));
      set_parameter(
        rclcpp::Parameter(
          "initial_pose.yaw",
          rclcpp::ParameterValue(tf2::getYaw(last_published_pose_.pose.pose.orientation))));
    }

    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & /*state*/)
  {
    RCLCPP_INFO(get_logger(), "Shutting down");
    return nav2_util::CallbackReturn::SUCCESS;
  }

  void CheckLaserReceived()
  {
  }

  bool CheckElapsedTime(std::chrono::seconds check_interval, rclcpp::Time last_time)
  {
    rclcpp::Duration elapsed_time = now() - last_time;
    if (elapsed_time.nanoseconds() * 1e-9 > check_interval.count()) {
      return true;
    }
    return false;
  }

  #if NEW_UNIFORM_SAMPLING
  std::vector<std::pair<int, int>> AmclNode::free_space_indices;
  #endif

  bool GetOdomPose(
    geometry_msgs::msg::PoseStamped & odom_pose,
    double & x, double & y, double & yaw,
    const rclcpp::Time & sensor_timestamp, const std::string & frame_id)
  {
    // Get the robot's pose
    geometry_msgs::msg::PoseStamped ident;
    ident.header.frame_id = nav2_util::strip_leading_slash(frame_id);
    ident.header.stamp = sensor_timestamp;
    tf2::toMsg(tf2::Transform::getIdentity(), ident.pose);

    try {
      tf_buffer_->transform(ident, odom_pose, odom_frame_id_);
    } catch (tf2::TransformException & e) {
      
      RCLCPP_ERROR(
        get_logger(), "Error in GetOdomPose()",
        e.what());
      
      return false;
    }

    x = odom_pose.pose.position.x;
    y = odom_pose.pose.position.y;
    yaw = tf2::getYaw(odom_pose.pose.orientation);

    return true;
  }

  pf_vector_t UniformPoseGenerator(void * arg)
  {
    map_t * map = reinterpret_cast<map_t *>(arg);
    pf_vector_t p;

    double min_x, max_x, min_y, max_y;

    min_x = (map->size_x * map->scale) / 2.0 - map->origin_x;
    max_x = (map->size_x * map->scale) / 2.0 + map->origin_x;
    min_y = (map->size_y * map->scale) / 2.0 - map->origin_y;
    max_y = (map->size_y * map->scale) / 2.0 + map->origin_y;

    RCLCPP_DEBUG(get_logger(), "Generating new uniform sample");
    for (;; ) {
      p.v[0] = min_x + drand48() * (max_x - min_x);
      p.v[1] = min_y + drand48() * (max_y - min_y);
      p.v[2] = drand48() * 2 * M_PI - M_PI;
      // Check that it's a free cell
      int i, j;
      i = MAP_GXWX(map, p.v[0]);
      j = MAP_GYWY(map, p.v[1]);
      if (MAP_VALID(map, i, j) && (map->cells[MAP_INDEX(map, i, j)].occ_state == -1)) {
        break;
      }
    }
    
    return p;
  }

  void GlobalLocalizationCallback(
    const std::shared_ptr<rmw_request_id_t>/*request_header*/,
    const std::shared_ptr<std_srvs::srv::Empty::Request>/*req*/,
    std::shared_ptr<std_srvs::srv::Empty::Response>/*res*/)
  {
    RCLCPP_INFO(get_logger(), "Initializing with uniform distribution");

    pf_init_model(
      pf_, (pf_init_model_fn_t)UniformPoseGenerator,
      reinterpret_cast<void *>(map_));
    RCLCPP_INFO(get_logger(), "Global initialisation done!");
    initial_pose_is_known_ = true;
    pf_init_ = false;
  }

  // force nomotion updates (amcl updating without requiring motion)
  void NomotionUpdateCallback(
    const std::shared_ptr<rmw_request_id_t>/*request_header*/,
    const std::shared_ptr<std_srvs::srv::Empty::Request>/*req*/,
    std::shared_ptr<std_srvs::srv::Empty::Response>/*res*/)
  {
    RCLCPP_INFO(get_logger(), "Requesting no-motion update");
    force_update_ = true;
  }

  void InitialPoseReceived(geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    RCLCPP_INFO(get_logger(), "initialPoseReceived");

    if (msg->header.frame_id == "") {
      // This should be removed at some point
      RCLCPP_WARN(
        get_logger(),
        "Received initial pose with empty frame_id. You should always supply a frame_id.");
      return;
    }
    if (nav2_util::strip_leading_slash(msg->header.frame_id) != global_frame_id) {
      RCLCPP_WARN(
        get_logger(),
        "Ignoring initial pose in frame \"%s\"; initial poses must be in the global frame, \"%s\"",
        nav2_util::strip_leading_slash(msg->header.frame_id).c_str(),
        global_frame_id_.c_str());
      return;
    }
    // Overriding last published pose to initial pose
    last_published_pose_ = *msg;

    if (!isActive) {
      init_pose_received_on_inactive = true;
      RCLCPP_WARN(
        get_logger(), "Received initial pose request, "
        "but AMCL is not yet in the active state");
      return;
    }
    HandleInitialPose(*msg);
  }

  void HandleInitialPose(geometry_msgs::msg::PoseWithCovarianceStamped & msg)
  {
    // In case the client sent us a pose estimate in the past, integrate the
    // intervening odometric change.
    geometry_msgs::msg::TransformStamped tx_odom;
    try {
      rclcpp::Time rclcpp_time = now();
      tf2::TimePoint tf2_time(std::chrono::nanoseconds(rclcpp_time.nanoseconds()));

      // Check if the transform is available
      tx_odom = tf_buffer_->lookupTransform(
        base_frame_id, tf2_ros::fromMsg(msg.header.stamp),
        base_frame_id, tf2_time, odom_frame_id);
    } catch (tf2::TransformException & e) {
      // If we've never sent a transform, then this is normal, because the
      // global_frame_id_ frame doesn't exist.  We only care about in-time
      // transformation for on-the-move pose-setting, so ignoring this
      // startup condition doesn't really cost us anything.
      if (sent_first_transform_) {
        RCLCPP_WARN(get_logger(), "Failed to transform initial pose in time (%s)", e.what());
      }
      tf2::impl::Converter<false, true>::convert(tf2::Transform::getIdentity(), tx_odom.transform);
    }

    tf2::Transform tx_odom_tf2;
    tf2::impl::Converter<true, false>::convert(tx_odom.transform, tx_odom_tf2);

    tf2::Transform pose_old;
    tf2::impl::Converter<true, false>::convert(msg.pose.pose, pose_old);

    tf2::Transform pose_new = pose_old * tx_odom_tf2;

    // Transform into the global frame

    RCLCPP_INFO(
      get_logger(), "Setting pose (%.6f): %.3f %.3f %.3f",
      now().nanoseconds() * 1e-9,
      pose_new.getOrigin().x(),
      pose_new.getOrigin().y(),
      tf2::getYaw(pose_new.getRotation()));

    // Re-initialize the filter
    pf_vector_t pf_init_pose_mean = pf_vector_zero();
    pf_init_pose_mean.v[0] = pose_new.getOrigin().x();
    pf_init_pose_mean.v[1] = pose_new.getOrigin().y();
    pf_init_pose_mean.v[2] = tf2::getYaw(pose_new.getRotation());

    pf_matrix_t pf_init_pose_cov = pf_matrix_zero();
    // Copy in the covariance, converting from 6-D to 3-D
    for (int i = 0; i < 2; i++) {
      for (int j = 0; j < 2; j++) {
        pf_init_pose_cov.m[i][j] = msg.pose.covariance[6 * i + j];
      }
    }

    pf_init_pose_cov.m[2][2] = msg.pose.covariance[6 * 5 + 5];

    pf_init(pf_, pf_init_pose_mean, pf_init_pose_cov);
    pf_init_ = false;
    init_pose_received_on_inactive = false;
    initial_pose_is_known_ = true;
  }

  void LaserReceived(sensor_msgs::msg::LaserScan::ConstSharedPtr laser_scan)
  {
    // Since the sensor data is continually being published by the simulator or robot,
    // we don't want our callbacks to fire until we're in the active state
    if (!isActive) {return;}
    if (!first_map_received) {
      if (checkElapsedTime(2s, last_time_printed_msg_)) {
        RCLCPP_WARN(get_logger(), "Waiting for map....");
        last_time_printed_msg_ = now();
      }
      return;
    }

    std::string laser_scan_frame_id = nav2_util::strip_leading_slash(laser_scan->header.frame_id);
    last_laser_received_ts_ = now();
    int laser_index = -1;
    geometry_msgs::msg::PoseStamped laser_pose;

    // Do we have the base->base_laser Tx yet?
    if (frame_to_laser_.find(laser_scan_frame_id) == frame_to_laser_.end()) {
      if (!AddNewScanner(laser_index, laser_scan, laser_scan_frame_id, laser_pose)) {
        return;  // could not find transform
      }
    } else {
      // we have the laser pose, retrieve laser index
      laser_index = frame_to_laser_[laser_scan->header.frame_id];
    }

    // Where was the robot when this scan was taken?
    pf_vector_t pose;
    if (!GetOdomPose(
        latest_odom_pose_, pose.v[0], pose.v[1], pose.v[2],
        laser_scan->header.stamp, base_frame_id))
    {
      RCLCPP_ERROR(get_logger(), "Couldn't determine robot's pose associated with laser scan");
      return;
    }

    pf_vector_t delta = pf_vector_zero();
    bool force_publication = false;
    if (!pf_init_) {
      // Pose at last filter update
      pf_odom_pose_ = pose;
      pf_init_ = true;

      for (unsigned int i = 0; i < lasers_update_.size(); i++) {
        lasers_update_[i] = true;
      }

      force_publication = true;
      resample_count_ = 0;
    } else {
      // Set the laser update flags
      if (shouldUpdateFilter(pose, delta)) {
        for (unsigned int i = 0; i < lasers_update_.size(); i++) {
          lasers_update_[i] = true;
        }
      }
      if (lasers_update_[laser_index]) {
        motion_model_->odometryUpdate(pf_, pose, delta);
      }
      force_update_ = false;
    }

    bool resampled = false;

    // If the robot has moved, update the filter
    if (lasers_update_[laser_index]) {
      updateFilter(laser_index, laser_scan, pose);

      // Resample the particles
      if (!(++resample_count_ % resample_interval_)) {
        pf_update_resample(pf_);
        resampled = true;
      }

      pf_sample_set_t * set = pf_->sets + pf_->current_set;
      RCLCPP_DEBUG(get_logger(), "Num samples: %d\n", set->sample_count);

      if (!force_update_) {
        publishParticleCloud(set);
      }
    }
    if (resampled || force_publication || !first_pose_sent_) {
      amcl_hyp_t max_weight_hyps;
      std::vector<amcl_hyp_t> hyps;
      int max_weight_hyp = -1;
      if (getMaxWeightHyp(hyps, max_weight_hyps, max_weight_hyp)) {
        publishAmclPose(laser_scan, hyps, max_weight_hyp);
        calculateMaptoOdomTransform(laser_scan, hyps, max_weight_hyp);

        if (tf_broadcast_ == true) {
          // We want to send a transform that is good up until a
          // tolerance time so that odom can be used
          auto stamp = tf2_ros::fromMsg(laser_scan->header.stamp);
          tf2::TimePoint transform_expiration = stamp + transform_tolerance_;
          sendMapToOdomTransform(transform_expiration);
          sent_first_transform_ = true;
        }
      } else {
        RCLCPP_ERROR(get_logger(), "No pose!");
      }
    } else if (latest_tf_valid_) {
      if (tf_broadcast_ == true) {
        // Nothing changed, so we'll just republish the last transform, to keep
        // everybody happy.
        tf2::TimePoint transform_expiration = tf2_ros::fromMsg(laser_scan->header.stamp) +
          transform_tolerance_;
        sendMapToOdomTransform(transform_expiration);
      }
    }
  }

  bool AddNewScanner(
    int & laser_index,
    const sensor_msgs::msg::LaserScan::ConstSharedPtr & laser_scan,
    const std::string & laser_scan_frame_id,
    geometry_msgs::msg::PoseStamped & laser_pose)
  {
    lasers_.push_back(createLaserObject());
    lasers_update_.push_back(true);
    laser_index = frame_to_laser_.size();

    geometry_msgs::msg::PoseStamped ident;
    ident.header.frame_id = laser_scan_frame_id;
    ident.header.stamp = rclcpp::Time();
    tf2::toMsg(tf2::Transform::getIdentity(), ident.pose);
    try {
      tf_buffer_->transform(ident, laser_pose, base_frame_id_, transform_tolerance_);
    } catch (tf2::TransformException & e) {
      RCLCPP_ERROR(
        get_logger(), "Couldn't transform from %s to %s, "
        "even though the message notifier is in use: (%s)",
        laser_scan->header.frame_id.c_str(),
        base_frame_id_.c_str(), e.what());
      return false;
    }

    pf_vector_t laser_pose_v;
    laser_pose_v.v[0] = laser_pose.pose.position.x;
    laser_pose_v.v[1] = laser_pose.pose.position.y;
    // laser mounting angle gets computed later -> set to 0 here!
    laser_pose_v.v[2] = 0;
    lasers_[laser_index]->SetLaserPose(laser_pose_v);
    frame_to_laser_[laser_scan->header.frame_id] = laser_index;
    return true;
  }

  bool ShouldUpdateFilter(const pf_vector_t pose, pf_vector_t & delta)
  {
    delta.v[0] = pose.v[0] - pf_odom_pose_.v[0];
    delta.v[1] = pose.v[1] - pf_odom_pose_.v[1];
    delta.v[2] = angleutils::angle_diff(pose.v[2], pf_odom_pose_.v[2]);

    // See if we should update the filter
    bool update = fabs(delta.v[0]) > d_thresh_ ||
      fabs(delta.v[1]) > d_thresh_ ||
      fabs(delta.v[2]) > a_thresh_;
    update = update || force_update_;
    return update;
  }

  bool UpdateFilter(
    const int & laser_index,
    const sensor_msgs::msg::LaserScan::ConstSharedPtr & laser_scan,
    const pf_vector_t & pose)
  {
    nav2_amcl::LaserData ldata;
    ldata.laser = lasers_[laser_index];
    ldata.range_count = laser_scan->ranges.size();
    // To account for lasers that are mounted upside-down, we determine the
    // min, max, and increment angles of the laser in the base frame.
    //
    // Construct min and max angles of laser, in the base_link frame.
    // Here we set the roll pich yaw of the lasers.  We assume roll and pich are zero.
    geometry_msgs::msg::QuaternionStamped min_q, inc_q;
    min_q.header.stamp = laser_scan->header.stamp;
    min_q.header.frame_id = nav2_util::strip_leading_slash(laser_scan->header.frame_id);
    min_q.quaternion = orientationAroundZAxis(laser_scan->angle_min);

    inc_q.header = min_q.header;
    inc_q.quaternion = orientationAroundZAxis(laser_scan->angle_min + laser_scan->angle_increment);
    try {
      tf_buffer_->transform(min_q, min_q, base_frame_id_);
      tf_buffer_->transform(inc_q, inc_q, base_frame_id_);
    } catch (tf2::TransformException & e) {
      RCLCPP_WARN(
        get_logger(), "Unable to transform min/max laser angles into base frame: %s",
        e.what());
      return false;
    }
    double angle_min = tf2::getYaw(min_q.quaternion);
    double angle_increment = tf2::getYaw(inc_q.quaternion) - angle_min;

    // wrapping angle to [-pi .. pi]
    angle_increment = fmod(angle_increment + 5 * M_PI, 2 * M_PI) - M_PI;

    RCLCPP_DEBUG(
      get_logger(), "Laser %d angles in base frame: min: %.3f inc: %.3f", laser_index, angle_min,
      angle_increment);

    // Apply range min/max thresholds, if the user supplied them
    if (laser_max_range_ > 0.0) {
      ldata.range_max = std::min(laser_scan->range_max, static_cast<float>(laser_max_range_));
    } else {
      ldata.range_max = laser_scan->range_max;
    }
    double range_min;
    if (laser_min_range_ > 0.0) {
      range_min = std::max(laser_scan->range_min, static_cast<float>(laser_min_range_));
    } else {
      range_min = laser_scan->range_min;
    }

    // The LaserData destructor will free this memory
    ldata.ranges = new double[ldata.range_count][2];
    for (int i = 0; i < ldata.range_count; i++) {
      // amcl doesn't (yet) have a concept of min range.  So we'll map short
      // readings to max range.
      if (laser_scan->ranges[i] <= range_min) {
        ldata.ranges[i][0] = ldata.range_max;
      } else {
        ldata.ranges[i][0] = laser_scan->ranges[i];
      }
      // Compute bearing
      ldata.ranges[i][1] = angle_min +
        (i * angle_increment);
    }
    lasers_[laser_index]->sensorUpdate(pf_, reinterpret_cast<nav2_amcl::LaserData *>(&ldata));
    lasers_update_[laser_index] = false;
    pf_odom_pose_ = pose;
    return true;
  }

  void PublishParticleCloud(const pf_sample_set_t * set)
  {
    // If initial pose is not known, AMCL does not know the current pose
    if (!initial_pose_is_known_) {return;}
    auto cloud_with_weights_msg = std::make_unique<nav2_msgs::msg::ParticleCloud>();
    cloud_with_weights_msg->header.stamp = this->now();
    cloud_with_weights_msg->header.frame_id = global_frame_id_;
    cloud_with_weights_msg->particles.resize(set->sample_count);

    auto cloud_msg = std::make_unique<geometry_msgs::msg::PoseArray>();
    cloud_msg->header.stamp = this->now();
    cloud_msg->header.frame_id = global_frame_id_;
    cloud_msg->poses.resize(set->sample_count);
    for (int i = 0; i < set->sample_count; i++) {
      cloud_msg->poses[i].position.x = set->samples[i].pose.v[0];
      cloud_msg->poses[i].position.y = set->samples[i].pose.v[1];
      cloud_msg->poses[i].position.z = 0;
      cloud_msg->poses[i].orientation = orientationAroundZAxis(set->samples[i].pose.v[2]);
      cloud_with_weights_msg->particles[i].pose = (*cloud_msg).poses[i];
      cloud_with_weights_msg->particles[i].weight = set->samples[i].weight;
    }
    particlecloud_pub_->publish(std::move(cloud_msg));
    particle_cloud_pub_->publish(std::move(cloud_with_weights_msg));
  }

  bool GetMaxWeightHyp(
    std::vector<amcl_hyp_t> & hyps, amcl_hyp_t & max_weight_hyps,
    int & max_weight_hyp)
  {
    // Read out the current hypotheses
    double max_weight = 0.0;
    hyps.resize(pf_->sets[pf_->current_set].cluster_count);
    for (int hyp_count = 0;
      hyp_count < pf_->sets[pf_->current_set].cluster_count; hyp_count++)
    {
      double weight;
      pf_vector_t pose_mean;
      pf_matrix_t pose_cov;
      if (!pf_get_cluster_stats(pf_, hyp_count, &weight, &pose_mean, &pose_cov)) {
        RCLCPP_ERROR(get_logger(), "Couldn't get stats on cluster %d", hyp_count);
        return false;
      }

      hyps[hyp_count].weight = weight;
      hyps[hyp_count].pf_pose_mean = pose_mean;
      hyps[hyp_count].pf_pose_cov = pose_cov;

      if (hyps[hyp_count].weight > max_weight) {
        max_weight = hyps[hyp_count].weight;
        max_weight_hyp = hyp_count;
      }
    }

    if (max_weight > 0.0) {
      RCLCPP_DEBUG(
        get_logger(), "Max weight pose: %.3f %.3f %.3f",
        hyps[max_weight_hyp].pf_pose_mean.v[0],
        hyps[max_weight_hyp].pf_pose_mean.v[1],
        hyps[max_weight_hyp].pf_pose_mean.v[2]);

      max_weight_hyps = hyps[max_weight_hyp];
      return true;
    }
    return false;
  }

  void PublishAmclPose(
    const sensor_msgs::msg::LaserScan::ConstSharedPtr & laser_scan,
    const std::vector<amcl_hyp_t> & hyps, const int & max_weight_hyp)
  {
    // If initial pose is not known, AMCL does not know the current pose
    if (!initial_pose_is_known_) {
      if (checkElapsedTime(2s, last_time_printed_msg_)) {
        RCLCPP_WARN(
          get_logger(), "ACML cannot publish a pose or update the transform. "
          "Please set the initial pose...");
        last_time_printed_msg_ = now();
      }
      return;
    }

    auto p = std::make_unique<geometry_msgs::msg::PoseWithCovarianceStamped>();
    // Fill in the header
    p->header.frame_id = global_frame_id_;
    p->header.stamp = laser_scan->header.stamp;
    // Copy in the pose
    p->pose.pose.position.x = hyps[max_weight_hyp].pf_pose_mean.v[0];
    p->pose.pose.position.y = hyps[max_weight_hyp].pf_pose_mean.v[1];
    p->pose.pose.orientation = orientationAroundZAxis(hyps[max_weight_hyp].pf_pose_mean.v[2]);
    // Copy in the covariance, converting from 3-D to 6-D
    pf_sample_set_t * set = pf_->sets + pf_->current_set;
    for (int i = 0; i < 2; i++) {
      for (int j = 0; j < 2; j++) {
        // Report the overall filter covariance, rather than the
        // covariance for the highest-weight cluster
        // p->covariance[6*i+j] = hyps[max_weight_hyp].pf_pose_cov.m[i][j];
        p->pose.covariance[6 * i + j] = set->cov.m[i][j];
      }
    }
    p->pose.covariance[6 * 5 + 5] = set->cov.m[2][2];
    float temp = 0.0;
    for (auto covariance_value : p->pose.covariance) {
      temp += covariance_value;
    }
    temp += p->pose.pose.position.x + p->pose.pose.position.y;
    if (!std::isnan(temp)) {
      RCLCPP_DEBUG(get_logger(), "Publishing pose");
      last_published_pose_ = *p;
      first_pose_sent_ = true;
      pose_pub_->publish(std::move(p));
    } else {
      RCLCPP_WARN(
        get_logger(), "AMCL covariance or pose is NaN, likely due to an invalid "
        "configuration or faulty sensor measurements! Pose is not available!");
    }

    RCLCPP_DEBUG(
      get_logger(), "New pose: %6.3f %6.3f %6.3f",
      hyps[max_weight_hyp].pf_pose_mean.v[0],
      hyps[max_weight_hyp].pf_pose_mean.v[1],
      hyps[max_weight_hyp].pf_pose_mean.v[2]);
  }

  void CalculateMaptoOdomTransform(
    const sensor_msgs::msg::LaserScan::ConstSharedPtr & laser_scan,
    const std::vector<amcl_hyp_t> & hyps, const int & max_weight_hyp)
  {
    // subtracting base to odom from map to base and send map to odom instead
    geometry_msgs::msg::PoseStamped odom_to_map;
    try {
      tf2::Quaternion q;
      q.setRPY(0, 0, hyps[max_weight_hyp].pf_pose_mean.v[2]);
      tf2::Transform tmp_tf(q, tf2::Vector3(
          hyps[max_weight_hyp].pf_pose_mean.v[0],
          hyps[max_weight_hyp].pf_pose_mean.v[1],
          0.0));

      geometry_msgs::msg::PoseStamped tmp_tf_stamped;
      tmp_tf_stamped.header.frame_id = base_frame_id_;
      tmp_tf_stamped.header.stamp = laser_scan->header.stamp;
      tf2::toMsg(tmp_tf.inverse(), tmp_tf_stamped.pose);

      tf_buffer_->transform(tmp_tf_stamped, odom_to_map, odom_frame_id_);
    } catch (tf2::TransformException & e) {
      RCLCPP_DEBUG(get_logger(), "Failed to subtract base to odom transform: (%s)", e.what());
      return;
    }

    tf2::impl::Converter<true, false>::convert(odom_to_map.pose, latest_tf_);
    latest_tf_valid_ = true;
  }

  void SendMapToOdomTransform(const tf2::TimePoint & transform_expiration)
  {
    // AMCL will update transform only when it has knowledge about robot's initial position
    if (!initial_pose_is_known_) {return;}
    geometry_msgs::msg::TransformStamped tmp_tf_stamped;
    tmp_tf_stamped.header.frame_id = global_frame_id_;
    tmp_tf_stamped.header.stamp = tf2_ros::toMsg(transform_expiration);
    tmp_tf_stamped.child_frame_id = odom_frame_id_;
    tf2::impl::Converter<false, true>::convert(latest_tf_.inverse(), tmp_tf_stamped.transform);
    tf_broadcaster_->sendTransform(tmp_tf_stamped);
  }

  nav2_amcl::Laser CreateLaserObject()
  {
    RCLCPP_INFO(get_logger(), "createLaserObject");

    if (sensor_model_type_ == "beam") {
      return new nav2_amcl::BeamModel(
        z_hit_, z_short_, z_max_, z_rand_, sigma_hit_, lambda_short_,
        0.0, max_beams_, map_);
    }

    if (sensor_model_type_ == "likelihood_field_prob") {
      return new nav2_amcl::LikelihoodFieldModelProb(
        z_hit_, z_rand_, sigma_hit_,
        laser_likelihood_max_dist_, do_beamskip_, beam_skip_distance_, beam_skip_threshold_,
        beam_skip_error_threshold_, max_beams_, map_);
    }

    return new nav2_amcl::LikelihoodFieldModel(
      z_hit_, z_rand_, sigma_hit_,
      laser_likelihood_max_dist_, max_beams_, map_);
  }

  void InitParameters()
  {
    // configure parameters

  }

  void MapReceived(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    RCLCPP_DEBUG(get_logger(), "AmclNode: A new map was received.");
    if (first_map_only_ && first_map_received_) {
      return;
    }
    handleMapMessage(*msg);
    first_map_received_ = true;
  }

  void HandleMapMessage(const nav_msgs::msg::OccupancyGrid & msg)
  {
    std::lock_guard<std::recursive_mutex> cfl(configuration_mutex_);

    RCLCPP_INFO(
      get_logger(), "Received a %d X %d map @ %.3f m/pix",
      msg.info.width,
      msg.info.height,
      msg.info.resolution);
    if (msg.header.frame_id != global_frame_id_) {
      RCLCPP_WARN(
        get_logger(), "Frame_id of map received:'%s' doesn't match global_frame_id:'%s'. This could"
        " cause issues with reading published topics",
        msg.header.frame_id.c_str(),
        global_frame_id_.c_str());
    }
    freeMapDependentMemory();
    map_ = convertMap(msg);

  #if NEW_UNIFORM_SAMPLING
    createFreeSpaceVector();
  #endif
  }

  void CreateFreeSpaceVector()
  {
    // Index of free space
    free_space_indices.resize(0);
    for (int i = 0; i < map_->size_x; i++) {
      for (int j = 0; j < map_->size_y; j++) {
        if (map_->cells[MAP_INDEX(map_, i, j)].occ_state == -1) {
          free_space_indices.push_back(std::make_pair(i, j));
        }
      }
    }
  }

  void FreeMapDependentMemory()
  {
    if (map_ != NULL) {
      map_free(map_);
      map_ = NULL;
    }

    // Clear queued laser objects because they hold pointers to the existing
    // map, #5202.
    lasers_.clear();
    lasers_update_.clear();
    frame_to_laser_.clear();
  }

  // Convert an OccupancyGrid map message into the internal representation. This function
  // allocates a map_t and returns it.
  map_t ConvertMap(const nav_msgs::msg::OccupancyGrid & map_msg)
  {
    map_t * map = map_alloc();

    map->size_x = map_msg.info.width;
    map->size_y = map_msg.info.height;
    map->scale = map_msg.info.resolution;
    map->origin_x = map_msg.info.origin.position.x + (map->size_x / 2) * map->scale;
    map->origin_y = map_msg.info.origin.position.y + (map->size_y / 2) * map->scale;

    map->cells =
      reinterpret_cast<map_cell_t *>(malloc(sizeof(map_cell_t) * map->size_x * map->size_y));

    // Convert to player format
    for (int i = 0; i < map->size_x * map->size_y; i++) {
      if (map_msg.data[i] == 0) {
        map->cells[i].occ_state = -1;
      } else if (map_msg.data[i] == 100) {
        map->cells[i].occ_state = +1;
      } else {
        map->cells[i].occ_state = 0;
      }
    }

    return map;
  }

  void InitTransforms()
  {
    RCLCPP_INFO(get_logger(), "initTransforms");

    // Initialize transform listener and broadcaster
    tf_buffer = std::make_shared<tf2_ros::Buffer>(rclcpp_node_->get_clock());
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
      rclcpp_node_->get_node_base_interface(),
      rclcpp_node_->get_node_timers_interface());
    tf_buffer->setCreateTimerInterface(timer_interface);
    tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(rclcpp_node_);

    sent_first_transform_ = false;
    latest_tf_valid_ = false;
    latest_tf_ = tf2::Transform::getIdentity();
  }

  void InitMessageFilters()
  {
    laser_scan_sub_ = std::make_unique<message_filters::Subscriber<sensor_msgs::msg::LaserScan>>(
      rclcpp_node_.get(), scan_topic_, rmw_qos_profile_sensor_data);

    laser_scan_filter_ = std::make_unique<tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan>>(
      *laser_scan_sub_, *tf_buffer_, odom_frame_id_, 10, rclcpp_node_);

    laser_scan_connection_ = laser_scan_filter_->registerCallback(
      std::bind(
        &AmclNode::laserReceived,
        this, std::placeholders::_1));
  }

  void InitPubSub()
  {
    RCLCPP_INFO(get_logger(), "initPubSub");

    particlecloud_pub_ = create_publisher<geometry_msgs::msg::PoseArray>(
      "particlecloud",
      rclcpp::SensorDataQoS());

    particle_cloud_pub_ = create_publisher<nav2_msgs::msg::ParticleCloud>(
      "particle_cloud",
      rclcpp::SensorDataQoS());

    pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "amcl_pose",
      rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

    initial_pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "initialpose", rclcpp::SystemDefaultsQoS(),
      std::bind(&AmclNode::initialPoseReceived, this, std::placeholders::_1));

    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      map_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
      std::bind(&AmclNode::mapReceived, this, std::placeholders::_1));

    RCLCPP_INFO(get_logger(), "Subscribed to map topic.");
  }

  void InitServices()
  {
    global_loc_srv_ = create_service<std_srvs::srv::Empty>(
      "reinitialize_global_localization",
      std::bind(&AmclNode::globalLocalizationCallback, this, _1, _2, _3));

    nomotion_update_srv_ = create_service<std_srvs::srv::Empty>(
      "request_nomotion_update",
      std::bind(&AmclNode::nomotionUpdateCallback, this, _1, _2, _3));
  }

  void InitOdometry()
  {
    // TODO(mjeronimo): We should handle persistance of the last known pose of the robot. We could
    // then read that pose here and initialize using that.

    // When pausing and resuming, remember the last robot pose so we don't start at 0:0 again
    init_pose_[0] = last_published_pose_.pose.pose.position.x;
    init_pose_[1] = last_published_pose_.pose.pose.position.y;
    init_pose_[2] = tf2::getYaw(last_published_pose_.pose.pose.orientation);

    if (!initial_pose_is_known_) {
      init_cov_[0] = 0.5 * 0.5;
      init_cov_[1] = 0.5 * 0.5;
      init_cov_[2] = (M_PI / 12.0) * (M_PI / 12.0);
    } else {
      init_cov_[0] = last_published_pose_.pose.covariance[0];
      init_cov_[1] = last_published_pose_.pose.covariance[7];
      init_cov_[2] = last_published_pose_.pose.covariance[35];
    }

    motion_model_ = std::unique_ptr<nav2_amcl::MotionModel>(
      nav2_amcl::MotionModel::createMotionModel(
        robot_model_type_, alpha1_, alpha2_, alpha3_, alpha4_, alpha5_));

    latest_odom_pose_ = geometry_msgs::msg::PoseStamped();
  }

  void InitParticleFilter()
  {
    // Create the particle filter
    pf_ = pf_alloc(
      min_particles_, max_particles_, alpha_slow_, alpha_fast_,
      (pf_init_model_fn_t)AmclNode::uniformPoseGenerator,
      reinterpret_cast<void *>(map_));
    pf_->pop_err = pf_err_;
    pf_->pop_z = pf_z_;

    // Initialize the filter
    pf_vector_t pf_init_pose_mean = pf_vector_zero();
    pf_init_pose_mean.v[0] = init_pose_[0];
    pf_init_pose_mean.v[1] = init_pose_[1];
    pf_init_pose_mean.v[2] = init_pose_[2];

    pf_matrix_t pf_init_pose_cov = pf_matrix_zero();
    pf_init_pose_cov.m[0][0] = init_cov_[0];
    pf_init_pose_cov.m[1][1] = init_cov_[1];
    pf_init_pose_cov.m[2][2] = init_cov_[2];

    pf_init(pf_, pf_init_pose_mean, pf_init_pose_cov);

    pf_init_ = false;
    resample_count_ = 0;
    memset(&pf_odom_pose_, 0, sizeof(pf_odom_pose_));
  }

  void InitLaserScan()
  {
    scan_error_count_ = 0;
    last_laser_received_ts_ = rclcpp::Time(0);
  }

private:
  tf_buffer_;

  // Node state
  bool isActive;

  // Publisher & Subscriber
  particlecloud_pub_;
  particle_cloud_pub_;
  pose_pub_;
  initial_pose_sub_;
  map_sub_;

  last_published_pose_;

  // Parameters

  double alpha1 = 0.2;
  double alpha2 = 0.2;
  double alpha3 = 0.2;
  double alpha4 = 0.2;
  double alpha5 = 0.2;
  
  

  double beam_skip_distance = 0.5;
  double beam_skip_error_threshold = 0.9;
  double beam_skip_threshold = 0.3;
  bool do_beamskip = false;

  

  double lambda_short = 0.1;

  double laser_likelihood_max_dist = 2.0;
  double laser_max_range = 100.0;
  double laser_min_range = -1.0;

  string laser_model_type = "likelihood_field";

  bool set_initial_pose = false;
  auto initial_pose = geometry_msgs::msg::Point();
  initial_pose_x = 0.0;
  initial_pose_y = 0.0;
  initial_pose_z = 0.0;
  initial_pose_yaw = 0.0;

  //initial_pose.yaw = 0.0;

  double max_beams = 60;
  double max_particles = 2000;

  double min_particles = 500;

  

  double pf_err = 0.05;

  double recovery_alpha_fast = 0.0;

  double recovery_alpha_slow = 0.0;

  int resample_interval = 1;

  string robot_model_type = "differential";

  double save_pose_rate = 0.5;

  double sigma_hit = 0.2;

  bool tf_broadcast = true;

  double transform_tolerance = 1.0;

  double update_min_a = 0.2;
  double update_min_d = 0.25;

  double z_hit = 0.5;
  double z_max = 0.05;
  double z_rand = 0.5;
  double z_short = 0.05;

  bool always_reset_initial_pose = false;

  // InitParameters()
  string odom_frame_id = "odom";
  string base_frame_id = "base_footprint";
  string global_frame_id = "map";
  string scan_topic = "scan";
  string map_topic = "map";

  double save_pose_rate;
  double tmp_tol;

  tf2::Duration save_pose_period_ = tf2::durationFromSec(1.0 / save_pose_rate);
  tf2::Duration transform_tolerance_ = tf2::durationFromSec(tmp_tol);

  rclcpp::Time last_time_printed_msg_ = now();

  // InitTransforms
  tf2_ros::Buffer tf_buffer;
  tf2_ros::TransformListener tf_listener;
  tf2_ros::TransformBroadcaster tf_broadcaster;

};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Simple_GPS_Localizer>());
  rclcpp::shutdown();

  return 0;
}