#pragma once
#ifndef WAYPOINTFLIER_WAYPOINTFLIER_H
#define WAYPOINTFLIER_WAYPOINTFLIER_H

/* includes //{ */

/* each ros package must have these */
#include <ros/ros.h>
#include <ros/package.h>
#include <nodelet/nodelet.h>

/* for loading dynamic parameters while the nodelet is running */
#include <dynamic_reconfigure/server.h>

/* this header file is created during compilation from python script dynparam.cfg */
#include <waypoint_flier/dynparamConfig.h>

/* for smart pointers (do not use raw pointers) */
#include <memory>

/* for protecting variables from simultaneous by from multiple threads */
#include <mutex>

/* for writing and reading from streams */
#include <fstream>
#include <iostream>

/* for storing information about the state of the uav (position) */
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>

/* for storing information about the state of the uav (position, twist) + covariances */
#include <nav_msgs/Odometry.h>

/* custom msgs of MRS group */
#include <mrs_msgs/ControlManagerDiagnostics.h>
#include <mrs_msgs/Float64Stamped.h>
#include <mrs_msgs/ReferenceStamped.h>

/* custom helper functions from our library */
#include <mrs_lib/param_loader.h>
#include <mrs_lib/subscribe_handler.h>
#include <mrs_lib/mutex.h>
#include <mrs_lib/attitude_converter.h>
#include <mrs_lib/msg_extractor.h>

/* for calling simple ros services */
#include <std_srvs/Trigger.h>

/* for operations with matrices */
#include <Eigen/Dense>

//}

namespace waypoint_flier
{

/* class WaypointFlier //{ */
class WaypointFlier : public nodelet::Nodelet {

public:
  /* onInit() is called when nodelet is launched (similar to main() in regular node) */
  virtual void onInit();

private:
  /* flags */
  bool is_initialized_ = false;

  /* ros parameters */
  bool        _simulation_;
  std::string _uav_name_;

  // | ---------------------- msg callbacks --------------------- |

  mrs_lib::SubscribeHandler<nav_msgs::Odometry>                  sh_odometry_;
  mrs_lib::SubscribeHandler<nav_msgs::Odometry>                  sh_ground_truth_;
  mrs_lib::SubscribeHandler<mrs_msgs::ControlManagerDiagnostics> sh_control_manager_diag_;

  void callbackControlManagerDiag(mrs_lib::SubscribeHandler<mrs_msgs::ControlManagerDiagnostics>& sh);
  bool have_goal_ = false;

  // | --------------------- timer callbacks -------------------- |

  void           callbackTimerPublishDistToWaypoint(const ros::TimerEvent& te);
  ros::Publisher pub_dist_to_waypoint_;
  ros::Timer     timer_publish_dist_to_waypoint_;
  int            _rate_timer_publish_dist_to_waypoint_;

  void           callbackTimerPublishSetReference(const ros::TimerEvent& te);
  ros::Publisher pub_reference_;
  ros::Timer     timer_publisher_reference_;
  int            _rate_timer_publisher_reference_;

  void       callbackTimerCheckSubscribers(const ros::TimerEvent& te);
  ros::Timer timer_check_subscribers_;
  int        _rate_timer_check_subscribers_;

  // | ---------------- service server callbacks ---------------- |

  bool               callbackStartWaypointFollowing(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
  ros::ServiceServer srv_server_start_waypoints_following_;

  bool               callbackStopWaypointFollowing(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
  ros::ServiceServer srv_server_stop_waypoints_following_;

  bool               callbackFlyToFirstWaypoint(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);
  ros::ServiceServer srv_server_fly_to_first_waypoint_;

  // | --------------------- service clients -------------------- |

  ros::ServiceClient srv_client_land_;
  bool               _land_end_;

  // | -------------------- loading waypoints ------------------- |

  std::vector<mrs_msgs::Reference> waypoints_;
  std::string                      _waypoints_frame_;
  bool                             waypoints_loaded_ = false;
  mrs_msgs::Reference              current_waypoint_;
  std::mutex                       mutex_current_waypoint_;
  int                              idx_current_waypoint_;
  int                              n_waypoints_;
  int                              _n_loops_;
  int                              c_loop_;
  std::mutex                       mutex_waypoint_idle_time_;
  Eigen::MatrixXd                  _offset_;

  // | ------------------- dynamic reconfigure ------------------ |

  typedef waypoint_flier::dynparamConfig                              Config;
  typedef dynamic_reconfigure::Server<waypoint_flier::dynparamConfig> ReconfigureServer;
  boost::recursive_mutex                                              mutex_dynamic_reconfigure_;
  boost::shared_ptr<ReconfigureServer>                                reconfigure_server_;
  void                                                                callbackDynamicReconfigure(Config& config, uint32_t level);
  waypoint_flier::dynparamConfig                                      last_drs_config_;

  // | --------------------- waypoint idling -------------------- |

  bool       is_idling_ = false;
  ros::Timer timer_idling_;
  double     _waypoint_idle_time_;
  void       callbackTimerIdling(const ros::TimerEvent& te);

  // | -------------------- support functions ------------------- |

  std::vector<mrs_msgs::Reference> matrixToPoints(const Eigen::MatrixXd& matrix);

  void offsetPoints(std::vector<mrs_msgs::Reference>& points, const Eigen::MatrixXd& offset);

  double distance(const mrs_msgs::Reference& waypoint, const geometry_msgs::Pose& pose);
};
//}

}  // namespace waypoint_flier
#endif
