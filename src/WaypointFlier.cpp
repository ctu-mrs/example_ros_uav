/* include header file of this class */
#include "WaypointFlier.h"

/* every nodelet must include macros which export the class as a nodelet plugin */
#include <pluginlib/class_list_macros.h>

namespace waypoint_flier
{

/* onInit() //{ */

void WaypointFlier::onInit() {

  // | ---------------- set my booleans to false ---------------- |
  // but remember, always set them to their default value in the header file
  // because, when you add new one later, you might forger to come back here
  got_odom_uav_     = false;
  got_odom_gt_      = false;
  got_tracker_diag_ = false;

  is_tracking_      = false;
  waypoints_loaded_ = false;

  /* obtain node handle */
  ros::NodeHandle nh("~");

  /* waits for the ROS to publish clock */
  ros::Time::waitForValid();

  // | ------------------- load ros parameters ------------------ |
  /* (mrs_lib implementation checks whether the parameter was loaded or not) */
  mrs_lib::ParamLoader param_loader(nh, "WaypointFlier");

  param_loader.load_param("simulation", _simulation_);
  param_loader.load_param("land_at_the_end", _land_end_);
  param_loader.load_param("n_loops", _n_loops_);
  param_loader.load_param("waypoint_idle_time", _waypoint_idle_time_);
  param_loader.load_param("rate/publish_dist_to_waypoint", _rate_timer_publish_dist_to_waypoint_);
  param_loader.load_param("rate/check_subscribers", _rate_timer_check_subscribers_);
  param_loader.load_param("rate/publish_reference", _rate_timer_publisher_reference_);

  /* load waypoints as a half-dynamic matrix from config file */
  Eigen::MatrixXd waypoint_matrix;
  param_loader.load_matrix_dynamic("waypoints", waypoint_matrix, -1, 4);  // -1 indicates the dynamic dimension
  waypoints_            = matrixToPoints(waypoint_matrix);
  n_waypoints_          = waypoints_.size();
  waypoints_loaded_     = true;
  idx_current_waypoint_ = 0;
  c_loop_               = 0;
  ROS_INFO_STREAM_ONCE("[WaypointFlier]: " << n_waypoints_ << " waypoints loaded");
  ROS_INFO_STREAM_ONCE("[WaypointFlier]: " << _n_loops_ << " loops requested");

  /* load offset of all waypoints as a static matrix from config file */
  param_loader.load_matrix_static("offset", _offset_, 1, 4);
  offsetPoints(waypoints_, _offset_);

  // | ------------------ initialize subscribers ----------------- |
  sub_odom_uav_     = nh.subscribe("odom_uav_in", 1, &WaypointFlier::callbackOdomUav, this, ros::TransportHints().tcpNoDelay());
  sub_tracker_diag_ = nh.subscribe("tracker_diagnostics_in", 1, &WaypointFlier::callbackTrackerDiag, this, ros::TransportHints().tcpNoDelay());

  /* subscribe ground truth only in simulation, where it is available */
  if (_simulation_) {
    sub_odom_gt_ = nh.subscribe("odom_gt_in", 1, &WaypointFlier::callbackOdomGt, this, ros::TransportHints().tcpNoDelay());
  }

  // | ------------------ initialize publishers ----------------- |
  pub_dist_to_waypoint_ = nh.advertise<mrs_msgs::Float64Stamped>("dist_to_waypoint_out", 1);
  pub_reference_        = nh.advertise<mrs_msgs::ReferenceStamped>("reference_out", 1);

  // | -------------------- initialize timers ------------------- |
  timer_publish_dist_to_waypoint_ = nh.createTimer(ros::Rate(_rate_timer_publish_dist_to_waypoint_), &WaypointFlier::callbackTimerPublishDistToWaypoint, this);
  timer_check_subscribers_        = nh.createTimer(ros::Rate(_rate_timer_check_subscribers_), &WaypointFlier::callbackTimerCheckSubscribers, this);
  // you can disable autostarting of the timer by the last argument
  timer_publisher_reference_ =
      nh.createTimer(ros::Rate(_rate_timer_publisher_reference_), &WaypointFlier::callbackTimerPublishSetReference, this, false, false);

  // | --------------- initialize service servers --------------- |
  srv_server_start_waypoints_following_ = nh.advertiseService("start_waypoints_following_in", &WaypointFlier::callbackStartWaypointFollowing, this);
  srv_server_stop_waypoints_following_  = nh.advertiseService("stop_waypoints_following_in", &WaypointFlier::callbackStopWaypointFollowing, this);
  srv_server_fly_to_first_waypoint_     = nh.advertiseService("fly_to_first_waypoint_in", &WaypointFlier::callbackFlyToFirstWaypoint, this);

  // | --------------- initialize service clients --------------- |
  srv_client_land_ = nh.serviceClient<std_srvs::Trigger>("land_out");

  // | ---------- initialize dynamic reconfigure server --------- |
  reconfigure_server_.reset(new ReconfigureServer(mutex_dynamic_reconfigure_, nh));
  ReconfigureServer::CallbackType f = boost::bind(&WaypointFlier::callbackDynamicReconfigure, this, _1, _2);
  reconfigure_server_->setCallback(f);

  /* set the default value of dynamic reconfigure server to the value of parameter with the same name */
  {
    std::scoped_lock lock(mutex_waypoint_idle_time_);
    last_drs_config_.waypoint_idle_time = _waypoint_idle_time_;
  }
  reconfigure_server_->updateConfig(last_drs_config_);

  ROS_INFO_ONCE("[WaypointFlier]: initialized");

  is_initialized_ = true;
}
//}

// | ---------------------- msg callbacks --------------------- |

/* callbackOdomUav() //{ */

void WaypointFlier::callbackOdomUav(const nav_msgs::OdometryConstPtr& msg) {

  /* do not continue if the nodelet is not initialized */
  if (!is_initialized_)
    return;

  {
    std::scoped_lock lock(mutex_odom_uav_);
    odom_uav_ = *msg;
  }

  if (!got_odom_uav_) {
    got_odom_uav_ = true;
    ROS_INFO("[WaypointFlier]: Received first odom uav msg");
  }

  time_last_odom_uav_ = ros::Time::now();
}

//}

/* callbackTrackerDiag() //{ */

void WaypointFlier::callbackTrackerDiag(const mrs_msgs::MpcTrackerDiagnosticsConstPtr& msg) {

  /* do not continue if the nodelet is not initialized */
  if (!is_initialized_)
    return;

  if (!got_tracker_diag_) {
    got_tracker_diag_ = true;
    ROS_INFO("[WaypointFlier]: Received first tracker diagnostics msg");
  }

  if (is_tracking_ && !msg->tracking_trajectory) {
    ROS_INFO("[WaypointFlier]: Waypoint reached.");
    std::scoped_lock lock(mutex_is_tracking_);
    is_tracking_ = false;

    /* start idling at the reached waypoint */
    is_idling_ = true;
    ros::NodeHandle nh("~");
    timer_idling_ = nh.createTimer(ros::Duration(_waypoint_idle_time_), &WaypointFlier::callbackTimerIdling, this,
                                   true);  // the last boolean argument makes the timer run only once
    ROS_INFO("[WaypointFlier]: Idling for %2.2f seconds.", _waypoint_idle_time_);
  }

  time_last_tracker_diagnostics_ = ros::Time::now();
}

//}

/* callbackPoseGt() //{ */

void WaypointFlier::callbackOdomGt(const nav_msgs::OdometryConstPtr& msg) {

  /* do not continue if the nodelet is not initialized */
  if (!is_initialized_)
    return;

  {
    std::scoped_lock lock(mutex_odom_gt_);
    odom_gt_ = *msg;
  }

  if (!got_odom_gt_) {
    got_odom_gt_ = true;
    ROS_INFO("[WaypointFlier]: Received first ground truth odom msg");
  }

  time_last_odom_gt_ = ros::Time::now();
}

//}

// | --------------------- timer callbacks -------------------- |

/* callbackTimerPublishSetReference() //{ */

void WaypointFlier::callbackTimerPublishSetReference([[maybe_unused]] const ros::TimerEvent& te) {

  if (!is_initialized_)
    return;

  /* return if the uav is still flying to the previous waypoints */
  if (is_tracking_)
    return;

  /* return if the UAV is idling at a waypoint */
  if (is_idling_)
    return;

  /* shutdown node after flying through all the waypoints (call land service before) */
  if (idx_current_waypoint_ >= n_waypoints_) {

    c_loop_++;

    ROS_INFO("[WaypointFlier]: Finished loop %d/%d", c_loop_, _n_loops_);

    if (c_loop_ >= _n_loops_) {

      ROS_INFO("[WaypointFlier]: Finished %d loops of %d waypoints.", _n_loops_, n_waypoints_);

      if (_land_end_) {
        ROS_INFO("[WaypointFlier]: Calling land service.");
        std_srvs::Trigger srv_land_call;
        srv_client_land_.call(srv_land_call);
      }

      ROS_INFO("[WaypointFlier]: Shutting down.");
      ros::shutdown();
      return;

    } else {
      ROS_INFO("[WaypointFlier]: Starting loop %d/%d", c_loop_ + 1, _n_loops_);
      idx_current_waypoint_ = 0;
    }
  }

  /* create new waypoint msg */
  mrs_msgs::ReferenceStamped new_waypoint;
    // it is important to set the frame id correctly !!
    // -- "" means the frame currently used for control
  new_waypoint.header.frame_id = "";
  new_waypoint.header.stamp    = ros::Time::now();

  {
    std::scoped_lock lock(mutex_current_waypoint_);

    current_waypoint_                 = waypoints_.at(idx_current_waypoint_);
    new_waypoint.reference.position.x = current_waypoint_.x;
    new_waypoint.reference.position.y = current_waypoint_.y;
    new_waypoint.reference.position.z = current_waypoint_.z;
    new_waypoint.reference.yaw        = current_waypoint_.yaw;
  }

  ROS_INFO("[WaypointFlier]: Flying to waypoint %d: x: %2.2f y: %2.2f z: %2.2f yaw: %2.2f", idx_current_waypoint_ + 1, new_waypoint.reference.position.x,
           new_waypoint.reference.position.y, new_waypoint.reference.position.z, new_waypoint.reference.yaw);

  try {
    pub_reference_.publish(new_waypoint);
  }
  catch (...) {
    ROS_ERROR("Exception caught during publishing topic %s.", pub_reference_.getTopic().c_str());
  }

  idx_current_waypoint_++;

  {
    std::scoped_lock lock(mutex_is_tracking_);

    is_tracking_ = true;
  }
}

//}

/* callbackTimerPublishDistToWaypoint() //{ */

void WaypointFlier::callbackTimerPublishDistToWaypoint([[maybe_unused]] const ros::TimerEvent& te) {

  if (!is_initialized_)
    return;

  /* do not publish distance to waypoint when the uav is not flying towards a waypoint */
  if (!is_tracking_)
    return;

  mrs_msgs::TrackerPoint tmp_current_waypoint;
  {
    std::scoped_lock lock(mutex_current_waypoint_);

    tmp_current_waypoint = current_waypoint_;
  }

  geometry_msgs::Pose tmp_pose_uav;
  {
    std::scoped_lock lock(mutex_odom_uav_);

    tmp_pose_uav = odom_uav_.pose.pose;
  }

  double dist = distance(tmp_current_waypoint, tmp_pose_uav);
  ROS_INFO("[WaypointFlier]: Distance to waypoint: %2.2f", dist);

  mrs_msgs::Float64Stamped dist_msg;
    // it is important to set the frame id correctly !!
    // -- "" means the frame currently used for control
  dist_msg.header.frame_id = "";
  dist_msg.header.stamp    = ros::Time::now();
  dist_msg.value           = dist;

  try {
    pub_dist_to_waypoint_.publish(dist_msg);
  }
  catch (...) {
    ROS_ERROR("Exception caught during publishing topic %s.", pub_dist_to_waypoint_.getTopic().c_str());
  }
}

//}

/* callbackTimerCheckSubscribers() //{ */

void WaypointFlier::callbackTimerCheckSubscribers([[maybe_unused]] const ros::TimerEvent& te) {

  if (!is_initialized_)
    return;

  ros::Time time_now = ros::Time::now();

  /* check whether uav odometry msgs are coming */
  if (!got_odom_uav_) {
    ROS_WARN_THROTTLE(1.0, "Not received uav odom msg since node launch.");
  } else {
    if ((time_now - time_last_odom_uav_).toSec() > 1.0) {
      ROS_WARN_THROTTLE(1.0, "Not received uav odom msg for %f sec.", (time_now - time_last_odom_uav_).toSec());
    }
  }

  /* check whether tracker diagnostics msgs are coming */
  if (!got_tracker_diag_) {
    ROS_WARN_THROTTLE(1.0, "Not received tracker diagnostics msg since node launch.");
  } else {
    if ((time_now - time_last_tracker_diagnostics_).toSec() > 1.0) {
      ROS_WARN_THROTTLE(1.0, "Not received uav odom msg for %f sec.", (time_now - time_last_tracker_diagnostics_).toSec());
    }
  }

  /* check whether ground truth pose msgs are coming */
  if (_simulation_) {
    if (!got_odom_gt_) {
      ROS_WARN_THROTTLE(1.0, "Not received ground truth odom msg since node launch.");
    } else {
      if ((time_now - time_last_odom_gt_).toSec() > 1.0) {
        ROS_WARN_THROTTLE(1.0, "Not received ground truth odom msg for %f sec.", (time_now - time_last_odom_gt_).toSec());
      }
    }
  }
}

//}

/* callbackTimerIdling() //{ */
void WaypointFlier::callbackTimerIdling([[maybe_unused]] const ros::TimerEvent& te) {

  ROS_INFO("[WaypointFlier]: Idling finished");
  is_idling_ = false;
}
//}

// | -------------------- service callbacks ------------------- |

/* //{ callbackStartWaypointFollowing() */

bool WaypointFlier::callbackStartWaypointFollowing([[maybe_unused]] std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {

  if (!is_initialized_) {

    res.success = false;
    res.message = "Waypoint flier not initialized!";
    ROS_WARN("[WaypointFlier]: Cannot start waypoint following, nodelet is not initialized.");
    return true;
  }

  if (waypoints_loaded_) {

    timer_publisher_reference_.start();

    ROS_INFO("[WaypointFlier]: Starting waypoint following.");

    res.success = true;
    res.message = "Starting waypoint following.";

  } else {

    ROS_WARN("[WaypointFlier]: Cannot start waypoint following, waypoints are not set.");
    res.success = false;
    res.message = "Waypoints not set.";
  }

  return true;
}

//}

/* //{ callbackStopWaypointFollowing() */

bool WaypointFlier::callbackStopWaypointFollowing([[maybe_unused]] std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {

  if (!is_initialized_) {

    res.success = false;
    res.message = "Waypoint flier not initialized!";
    ROS_WARN("[WaypointFlier]: Cannot stop waypoint following, nodelet is not initialized.");
    return true;
  }

  timer_publisher_reference_.stop();

  ROS_INFO("[WaypointFlier]: Waypoint following stopped.");

  res.success = true;
  res.message = "Waypoint following stopped.";

  return true;
}

//}

/* //{ callbackFlyToFirstWaypoint() */

bool WaypointFlier::callbackFlyToFirstWaypoint([[maybe_unused]] std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res) {

  if (!is_initialized_) {

    res.success = false;
    res.message = "Waypoint flier not initialized!";
    ROS_WARN("[WaypointFlier]: Cannot stop waypoint following, nodelet is not initialized.");
    return true;
  }

  if (waypoints_loaded_) {

    /* create new waypoint msg */
    mrs_msgs::ReferenceStamped new_waypoint;

    // it is important to set the frame id correctly !!
    // -- "" means the frame currently used for control
    new_waypoint.header.frame_id = "";

    new_waypoint.header.stamp = ros::Time::now();
    {
      std::scoped_lock lock(mutex_current_waypoint_);

      current_waypoint_                 = waypoints_.at(0);
      new_waypoint.reference.position.x = current_waypoint_.x;
      new_waypoint.reference.position.y = current_waypoint_.y;
      new_waypoint.reference.position.z = current_waypoint_.z;
      new_waypoint.reference.yaw        = current_waypoint_.yaw;
    }

    idx_current_waypoint_ = 0;
    c_loop_               = 0;

    {
      std::scoped_lock lock(mutex_is_tracking_);
      is_tracking_ = true;
    }

    try {
      pub_reference_.publish(new_waypoint);
    }
    catch (...) {
      ROS_ERROR("Exception caught during publishing topic %s.", pub_reference_.getTopic().c_str());
    }

    ROS_INFO("[WaypointFlier]: Flying to first waypoint: x: %2.2f y: %2.2f z: %2.2f yaw: %2.2f", new_waypoint.reference.position.x,
             new_waypoint.reference.position.y, new_waypoint.reference.position.z, new_waypoint.reference.yaw);

    char temp_str[100];
    sprintf((char*)&temp_str, "Flying to first waypoint: x: %2.2f, y: %2.2f, z: %2.2f, yaw: %2.2f", new_waypoint.reference.position.x,
            new_waypoint.reference.position.y, new_waypoint.reference.position.z, new_waypoint.reference.yaw);

    res.success = true;
    res.message = temp_str;

  } else {

    ROS_WARN("[WaypointFlier]: Cannot fly to first waypoint, waypoints not loaded!");

    res.success = false;
    res.message = "Waypoints not loaded";
  }

  return true;
}

//}

// | -------------- dynamic reconfigure callback -------------- |

/* //{ callbackDynamicReconfigure() */
void WaypointFlier::callbackDynamicReconfigure([[maybe_unused]] Config& config, [[maybe_unused]] uint32_t level) {

  if (!is_initialized_)
    return;

  ROS_INFO(
      "[WaypointFlier]:"
      "Reconfigure Request: "
      "Waypoint idle time: %2.2f",
      config.waypoint_idle_time);

  {
    std::scoped_lock lock(mutex_waypoint_idle_time_);

    _waypoint_idle_time_ = config.waypoint_idle_time;
  }
}
//}

// | -------------------- support functions ------------------- |

/* matrixToPoints() //{ */

std::vector<mrs_msgs::TrackerPoint> WaypointFlier::matrixToPoints(const Eigen::MatrixXd& matrix) {
  std::vector<mrs_msgs::TrackerPoint> points;

  for (int i = 0; i < matrix.rows(); i++) {
    mrs_msgs::TrackerPoint point;
    point.x   = matrix.row(i)(0);
    point.y   = matrix.row(i)(1);
    point.z   = matrix.row(i)(2);
    point.yaw = matrix.row(i)(3);
    points.push_back(point);
  }

  return points;
}

//}

/* offsetPoints() //{ */

void WaypointFlier::offsetPoints(std::vector<mrs_msgs::TrackerPoint>& points, const Eigen::MatrixXd& offset) {

  for (size_t i = 0; i < points.size(); i++) {
    points.at(i).x += offset(0);
    points.at(i).y += offset(1);
    points.at(i).z += offset(2);
    points.at(i).yaw += offset(3);
  }
}

//}

/* distance() //{ */

double WaypointFlier::distance(const mrs_msgs::TrackerPoint& waypoint, const geometry_msgs::Pose& pose) {

  return std::sqrt(std::pow(waypoint.x - pose.position.x, 2) + std::pow(waypoint.y - pose.position.y, 2) + std::pow(waypoint.z - pose.position.z, 2));
}

//}

}  // namespace waypoint_flier

/* every nodelet must export its class as nodelet plugin */
PLUGINLIB_EXPORT_CLASS(waypoint_flier::WaypointFlier, nodelet::Nodelet);
