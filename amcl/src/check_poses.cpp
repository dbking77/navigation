/*
 *  Copyright (c) 2008, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  This library is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
 *
 *  This library is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public
 *  License along with this library; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

/* Author: Brian Gerkey */

#include <algorithm>
#include <vector>
#include <map>
#include <cmath>

#include <boost/bind.hpp>
#include <boost/thread/mutex.hpp>

// Signal handling
#include <signal.h>

#include "map/map.h"
#include "pf/pf.h"
#include "sensors/amcl_odom.h"
#include "sensors/amcl_laser.h"

#include "ros/assert.h"

// roscpp
#include "ros/ros.h"
#include "actionlib/server/simple_action_server.h"

// Messages that I need
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Pose.h"
#include "nav_msgs/GetMap.h"
#include "std_srvs/Empty.h"
#include "amcl/CheckPosesAction.h"

// For transform support
#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"
#include "tf/message_filter.h"
#include "tf/tf.h"
#include "message_filters/subscriber.h"

//
#include "message_filters/cache.h"

using namespace amcl;

static double
normalize(double z)
{
  return atan2(sin(z),cos(z));
}
static double
angle_diff(double a, double b)
{
  double d1, d2;
  a = normalize(a);
  b = normalize(b);
  d1 = a-b;
  d2 = 2*M_PI - fabs(d1);
  if(d1 > 0)
    d2 *= -1.0;
  if(fabs(d1) < fabs(d2))
    return(d1);
  else
    return(d2);
}

static const std::string scan_topic_ = "scan";

class AmclNode
{
  public:
    AmclNode();
    ~AmclNode();

  private:
    tf::TransformBroadcaster* tfb_;  // DEBUG ONLY
    tf::TransformListener* tf_;

    void checkPoses(std::vector<double> &weights, const geometry_msgs::PoseArray &poses);
    void laserReceived(const sensor_msgs::LaserScanConstPtr& laser_scan);
    void initialPoseReceived(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg);
    void mapReceived(const nav_msgs::OccupancyGridConstPtr& msg);

    void handleMapMessage(const nav_msgs::OccupancyGrid& msg);
    void freeMapDependentMemory();
    map_t* convertMap( const nav_msgs::OccupancyGrid& map_msg );

    double getYaw(tf::Pose& t);

    // DEBUG ONLY
    std::string odom_frame_id_;

    //parameter for what base to use
    std::string base_frame_id_;
    std::string global_frame_id_;

    bool use_map_topic_;
    bool first_map_only_;

    map_t* map_;
    char* mapdata;
    int sx, sy;
    double resolution;

    message_filters::Subscriber<sensor_msgs::LaserScan>* laser_scan_sub_;
    message_filters::Cache<sensor_msgs::LaserScan>* laser_scan_cache_;

    // DEBUG : used to trigger pose check before action is implemented
    ros::Subscriber initial_pose_sub_;

    std::vector< AMCLLaser* > lasers_;
    std::vector< bool > lasers_update_;
    std::map< std::string, int > frame_to_laser_;

    // Particle filter
    double laser_min_range_;
    double laser_max_range_;

    //Nomotion update control
    bool m_force_update;  // used to temporarily let amcl update samples even when no motion occurs...

    AMCLLaser* laser_;

    void requestMap();

    // Helper to get odometric pose from transform system
    bool getOdomPose(tf::Stamped<tf::Pose>& pose,
                     double& x, double& y, double& yaw,
                     const ros::Time& t, const std::string& f);

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Subscriber map_sub_;

    bool first_map_received_;

    boost::recursive_mutex configuration_mutex_;
    ros::Timer check_laser_timer_;

    int max_beams_, min_particles_, max_particles_;
    double z_hit_, z_short_, z_max_, z_rand_, sigma_hit_, lambda_short_;
  //beam skip related params
    bool do_beamskip_;
    double beam_skip_distance_, beam_skip_threshold_, beam_skip_error_threshold_;
    double laser_likelihood_max_dist_;
    laser_model_t laser_model_type_;

    ros::Time last_laser_received_ts_;
    ros::Duration laser_check_interval_;
    void checkLaserReceived(const ros::TimerEvent& event);

  // Check poses action server
  boost::shared_ptr< actionlib::SimpleActionServer<amcl::CheckPosesAction> > action_server_;
  // Action callback function
  void checkPosesActionCB(const amcl::CheckPosesGoalConstPtr &poses);
};


#define USAGE "USAGE: amcl"

boost::shared_ptr<AmclNode> amcl_node_ptr;

int
main(int argc, char** argv)
{
  ros::init(argc, argv, "pose_check");
  ros::NodeHandle nh;

  // Make our node available to sigintHandler
  amcl_node_ptr.reset(new AmclNode());

  ros::spin();

  // Without this, our boost locks are not shut down nicely
  amcl_node_ptr.reset();

  // To quote Morgan, Hooray!
  return(0);
}

AmclNode::AmclNode() :
        map_(NULL),
        laser_(NULL),
              private_nh_("~"),
        first_map_received_(false)
{
  boost::recursive_mutex::scoped_lock l(configuration_mutex_);

  // Grab params off the param server
  private_nh_.param("use_map_topic", use_map_topic_, false);
  private_nh_.param("first_map_only", first_map_only_, false);

  private_nh_.param("laser_min_range", laser_min_range_, -1.0);
  private_nh_.param("laser_max_range", laser_max_range_, -1.0);
  private_nh_.param("laser_max_beams", max_beams_, 30);

  private_nh_.param("do_beamskip", do_beamskip_, false);
  private_nh_.param("beam_skip_distance", beam_skip_distance_, 0.5);
  private_nh_.param("beam_skip_threshold", beam_skip_threshold_, 0.3);
  private_nh_.param("beam_skip_error_threshold_", beam_skip_error_threshold_, 0.9);

  private_nh_.param("laser_z_hit", z_hit_, 0.95);
  private_nh_.param("laser_z_short", z_short_, 0.1);
  private_nh_.param("laser_z_max", z_max_, 0.05);
  private_nh_.param("laser_z_rand", z_rand_, 0.05);
  private_nh_.param("laser_sigma_hit", sigma_hit_, 0.2);
  private_nh_.param("laser_lambda_short", lambda_short_, 0.1);
  private_nh_.param("laser_likelihood_max_dist", laser_likelihood_max_dist_, 2.0);
  std::string tmp_model_type;
  private_nh_.param("laser_model_type", tmp_model_type, std::string("likelihood_field"));
  if(tmp_model_type == "beam")
    laser_model_type_ = LASER_MODEL_BEAM;
  else if(tmp_model_type == "likelihood_field")
    laser_model_type_ = LASER_MODEL_LIKELIHOOD_FIELD;
  else if(tmp_model_type == "likelihood_field_prob"){
    laser_model_type_ = LASER_MODEL_LIKELIHOOD_FIELD_PROB;
  }
  else
  {
    ROS_WARN("Unknown laser model type \"%s\"; defaulting to likelihood_field model",
             tmp_model_type.c_str());
    laser_model_type_ = LASER_MODEL_LIKELIHOOD_FIELD;
  }

  private_nh_.param("odom_frame_id", odom_frame_id_, std::string("odom"));
  private_nh_.param("base_frame_id", base_frame_id_, std::string("base_link"));
  private_nh_.param("global_frame_id", global_frame_id_, std::string("map"));

  tfb_ = NULL; //new tf::TransformBroadcaster(); // DEBUG ONLY
  tf_ = new tf::TransformListener();

  laser_scan_sub_ = new message_filters::Subscriber<sensor_msgs::LaserScan>(nh_, scan_topic_, 100);
  laser_scan_cache_ = new message_filters::Cache<sensor_msgs::LaserScan>(*laser_scan_sub_, 100);

  // DEBUG : use initial pose message to trigger checkPoses
  //initial_pose_sub_ = nh_.subscribe("initialpose", 2, &AmclNode::initialPoseReceived, this);

  if(use_map_topic_) {
    map_sub_ = nh_.subscribe("map", 1, &AmclNode::mapReceived, this);
    ROS_INFO("Subscribed to map topic.");
  } else {
    requestMap();
  }

  // 15s timer to warn on lack of receipt of laser scans, #5209
  laser_check_interval_ = ros::Duration(15.0);
  check_laser_timer_ = nh_.createTimer(laser_check_interval_,
                                       boost::bind(&AmclNode::checkLaserReceived, this, _1));

  // Start action server
  action_server_.reset( new actionlib::SimpleActionServer<amcl::CheckPosesAction>(private_nh_, "check_poses", boost::bind(&AmclNode::checkPosesActionCB, this, _1), false) );
  action_server_->start();
}


void
AmclNode::checkLaserReceived(const ros::TimerEvent& event)
{
  ros::Duration d = ros::Time::now() - last_laser_received_ts_;
  if(d > laser_check_interval_)
  {
    ROS_WARN("No laser scan received (and thus no pose updates have been published) for %f seconds.  Verify that data is being published on the %s topic.",
             d.toSec(),
             ros::names::resolve(scan_topic_).c_str());
  }
}

void
AmclNode::requestMap()
{
  boost::recursive_mutex::scoped_lock ml(configuration_mutex_);

  // get map via RPC
  nav_msgs::GetMap::Request  req;
  nav_msgs::GetMap::Response resp;
  ROS_INFO("Requesting the map...");
  while(!ros::service::call("static_map", req, resp))
  {
    ROS_WARN("Request for map failed; trying again...");
    ros::Duration d(0.5);
    d.sleep();
  }
  handleMapMessage( resp.map );
}

void
AmclNode::mapReceived(const nav_msgs::OccupancyGridConstPtr& msg)
{
  if( first_map_only_ && first_map_received_ ) {
    return;
  }

  handleMapMessage( *msg );

  first_map_received_ = true;
}

void
AmclNode::handleMapMessage(const nav_msgs::OccupancyGrid& msg)
{
  boost::recursive_mutex::scoped_lock cfl(configuration_mutex_);

  ROS_INFO("Received a %d X %d map @ %.3f m/pix\n",
           msg.info.width,
           msg.info.height,
           msg.info.resolution);

  freeMapDependentMemory();
  // Clear queued laser objects because they hold pointers to the existing
  // map, #5202.
  lasers_.clear();
  lasers_update_.clear();
  frame_to_laser_.clear();

  map_ = convertMap(msg);

  // Instantiate the sensor objects
  // Odometry

  // Laser
  delete laser_;
  laser_ = new AMCLLaser(max_beams_, map_);
  ROS_ASSERT(laser_);
  if(laser_model_type_ == LASER_MODEL_BEAM)
    laser_->SetModelBeam(z_hit_, z_short_, z_max_, z_rand_,
                         sigma_hit_, lambda_short_, 0.0);
  else if(laser_model_type_ == LASER_MODEL_LIKELIHOOD_FIELD_PROB){
    ROS_INFO("Initializing likelihood field model; this can take some time on large maps...");
    laser_->SetModelLikelihoodFieldProb(z_hit_, z_rand_, sigma_hit_,
                                        laser_likelihood_max_dist_,
                                        do_beamskip_, beam_skip_distance_,
                                        beam_skip_threshold_, beam_skip_error_threshold_);
    ROS_INFO("Done initializing likelihood field model.");
  }
  else
  {
    ROS_INFO("Initializing likelihood field model; this can take some time on large maps...");
    laser_->SetModelLikelihoodField(z_hit_, z_rand_, sigma_hit_,
                                    laser_likelihood_max_dist_);
    ROS_INFO("Done initializing likelihood field model.");
  }

  ROS_INFO("Map processing completed");
}

void
AmclNode::freeMapDependentMemory()
{
  if( map_ != NULL ) {
    map_free( map_ );
    map_ = NULL;
  }
  delete laser_;
  laser_ = NULL;
}

/**
 * Convert an OccupancyGrid map message into the internal
 * representation.  This allocates a map_t and returns it.
 */
map_t*
AmclNode::convertMap( const nav_msgs::OccupancyGrid& map_msg )
{
  map_t* map = map_alloc();
  ROS_ASSERT(map);

  map->size_x = map_msg.info.width;
  map->size_y = map_msg.info.height;
  map->scale = map_msg.info.resolution;
  map->origin_x = map_msg.info.origin.position.x + (map->size_x / 2) * map->scale;
  map->origin_y = map_msg.info.origin.position.y + (map->size_y / 2) * map->scale;
  // Convert to player format
  map->cells = (map_cell_t*)malloc(sizeof(map_cell_t)*map->size_x*map->size_y);
  ROS_ASSERT(map->cells);
  for(int i=0;i<map->size_x * map->size_y;i++)
  {
    if(map_msg.data[i] == 0)
      map->cells[i].occ_state = -1;
    else if(map_msg.data[i] == 100)
      map->cells[i].occ_state = +1;
    else
      map->cells[i].occ_state = 0;
  }

  return map;
}

AmclNode::~AmclNode()
{
  freeMapDependentMemory();
  delete laser_scan_cache_;
  delete laser_scan_sub_;
  delete tfb_;
  delete tf_;
  // TODO: delete everything allocated in constructor
}



// Debug : Use to test check Pose, TODO, use action instead
void
AmclNode::initialPoseReceived(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)
{
  boost::recursive_mutex::scoped_lock prl(configuration_mutex_);
  if(msg->header.frame_id == "")
  {
    // This should be removed at some point
    ROS_WARN("Received initial pose with empty frame_id.  You should always supply a frame_id.");
  }
  // We only accept initial pose estimates in the global frame, #5148.
  else if(tf_->resolve(msg->header.frame_id) != tf_->resolve(global_frame_id_))
  {
    ROS_WARN("Ignoring initial pose in frame \"%s\"; initial poses must be in the global frame, \"%s\"",
             msg->header.frame_id.c_str(),
             global_frame_id_.c_str());
    return;
  }

  geometry_msgs::Pose best_pose = msg->pose.pose;
  std::vector<double> weights;

  // Slowly iterate to best solution
  double delta = 0.5;
  for (int iteration=0; iteration<30; ++iteration)
  {
    geometry_msgs::PoseArray poses;
    poses.header = msg->header;
    double angle = 2.0 * atan2(best_pose.orientation.z, best_pose.orientation.w);
    for (int dx = -3; dx <= 3; ++dx)
    {
      for (int dy = -3; dy <= 3; ++dy)
      {
        for (int da = -3; da <= 3; ++da)
        {
          geometry_msgs::Pose pose = best_pose;
          pose.position.x += dx*delta;
          pose.position.y += dy*delta;
          pose.orientation.x = 0.0;
          pose.orientation.y = 0.0;
          double new_angle = angle + da*delta;
          pose.orientation.z = sin(new_angle/2);
          pose.orientation.w = cos(new_angle/2);
          poses.poses.push_back(pose);
        }
      }
    }

    delta *= 0.9;

    checkPoses(weights, poses);

    // Find best (highest weight) sample
    double best_weight = weights[0];
    size_t best_index = 0;
    for (size_t sample_index = 0; sample_index<poses.poses.size(); ++sample_index)
    {
      if (weights[sample_index] > best_weight)
      {
        best_weight = weights[sample_index];
        best_index = sample_index;
      }
    }
    best_pose = poses.poses[best_index];
    ROS_INFO_STREAM(iteration
                    << " index " << best_index
                    << " x " << best_pose.position.x
                    << " y " << best_pose.position.y
                    << " delta " << delta
                    << " weight " << best_weight
                    );

    // DEBUG ONLY
    if (tfb_)
    {
      // subtracting base to odom from map to base and send map to odom instead
      tf::Stamped<tf::Pose> odom_to_map;
      try
      {
        tf::Transform tmp_tf(tf::Quaternion(best_pose.orientation.x,
                                            best_pose.orientation.y,
                                            best_pose.orientation.z,
                                            best_pose.orientation.w),
                             tf::Vector3(best_pose.position.x,
                                         best_pose.position.y,
                                         0.0));
        tf::Stamped<tf::Pose> tmp_tf_stamped (tmp_tf.inverse(),
                                              msg->header.stamp,
                                              base_frame_id_);
        this->tf_->transformPose(odom_frame_id_,
                                 tmp_tf_stamped,
                                 odom_to_map);
      }
      catch(tf::TransformException)
      {
        ROS_ERROR("Failed to subtract base to odom transform");
        return;
      }

      tf::Transform latest_tf = tf::Transform(tf::Quaternion(odom_to_map.getRotation()),
                                              tf::Point(odom_to_map.getOrigin()));

      ros::Time transform_expiration = ros::Time::now() + ros::Duration(0.2);
      //laser_scan->header.stamp + ros::Duration(1.0);
      tf::StampedTransform tmp_tf_stamped(latest_tf.inverse(),
                                          transform_expiration,
                                          global_frame_id_, odom_frame_id_);
      tfb_->sendTransform(tmp_tf_stamped);
      //ROS_INFO("Published transform");
      ros::WallDuration(0.1).sleep();
    }
  }
}


void AmclNode::checkPosesActionCB(const amcl::CheckPosesGoalConstPtr &goal)
{
  ROS_INFO("CheckPosesAction with %d poses", int(goal->poses.poses.size()));

  // TODO break pose into chuncks of 1000, but anything smaller is pretty fast
  // so just use normal check poses.
  amcl::CheckPosesResult result;
  result.is_valid = true;

  try
  {
    // TODO : lock with time-out
    checkPoses(result.weights, goal->poses);
  }
  catch (std::exception &ex)
  {
    ROS_ERROR_STREAM("Catch exception while processing goal : " << ex.what());
    result.is_valid = false;
    result.message = ex.what();
  }

  action_server_->setSucceeded(result);
}


void AmclNode::checkPoses(std::vector<double> &weights, const geometry_msgs::PoseArray &poses)
{
  ros::WallTime start = ros::WallTime::now();

  if(poses.poses.size() == 0)
  {
    throw std::runtime_error("PoseArray must have at least one pose");
  }

  if(tf_->resolve(poses.header.frame_id) != tf_->resolve(global_frame_id_))
  {
    std::stringstream ss;
    ss << "Poses must use global frame ID : "
       << " request_frame=" << poses.header.frame_id
       << " global_frame=" << global_frame_id_;
    throw std::runtime_error(ss.str());
  }

  boost::recursive_mutex::scoped_lock lr(configuration_mutex_);

  // Find laser scan that matches timestamp in poses header
  sensor_msgs::LaserScanConstPtr laser_scan = laser_scan_cache_->getElemBeforeTime(poses.header.stamp);

  if (!laser_scan)
  {
    std::stringstream ss;
    ss << "No laser scan for :"
       << " time=" << poses.header.stamp
       << " oldest=" << laser_scan_cache_->getLatestTime()
       << " latest=" << laser_scan_cache_->getOldestTime();
    throw std::runtime_error(ss.str());
  }

  int laser_index = -1;

  // Do we have the base->base_laser Tx yet?
  if(frame_to_laser_.find(laser_scan->header.frame_id) == frame_to_laser_.end())
  {
    ROS_INFO("Setting up laser %d (frame_id=%s)\n",
             (int)frame_to_laser_.size(), laser_scan->header.frame_id.c_str());
    lasers_.push_back(new AMCLLaser(*laser_));
    lasers_update_.push_back(true);
    laser_index = frame_to_laser_.size();

    tf::Stamped<tf::Pose> ident (tf::Transform(tf::createIdentityQuaternion(),
                                             tf::Vector3(0,0,0)),
                                 ros::Time(), laser_scan->header.frame_id);
    tf::Stamped<tf::Pose> laser_pose;

    try
    {
      this->tf_->transformPose(base_frame_id_, ident, laser_pose);
    }
    catch(tf::TransformException& ex)
    {
      std::stringstream ss;
      ss << "Couldn't transform : "
         << " from " << laser_scan->header.frame_id
         << " to " << base_frame_id_
         << " : " << ex.what();
      throw std::runtime_error(ss.str());
    }

    pf_vector_t laser_pose_v;
    laser_pose_v.v[0] = laser_pose.getOrigin().x();
    laser_pose_v.v[1] = laser_pose.getOrigin().y();
    // laser mounting angle gets computed later -> set to 0 here!
    laser_pose_v.v[2] = 0;
    lasers_[laser_index]->SetLaserPose(laser_pose_v);
    ROS_INFO("Received laser's pose wrt robot: %.3f %.3f %.3f",
              laser_pose_v.v[0],
              laser_pose_v.v[1],
              laser_pose_v.v[2]);

    frame_to_laser_[laser_scan->header.frame_id] = laser_index;
  } else {
    // we have the laser pose, retrieve laser index
    laser_index = frame_to_laser_[laser_scan->header.frame_id];
  }

  // Convert laser scan start and stop angles
  AMCLLaserData ldata;
  ldata.sensor = lasers_[laser_index];
  ldata.range_count = laser_scan->ranges.size();

  // To account for lasers that are mounted upside-down, we determine the
  // min, max, and increment angles of the laser in the base frame.
  //
  // Construct min and max angles of laser, in the base_link frame.
  tf::Quaternion q;
  q.setRPY(0.0, 0.0, laser_scan->angle_min);
  tf::Stamped<tf::Quaternion> min_q(q, laser_scan->header.stamp,
                                    laser_scan->header.frame_id);
  q.setRPY(0.0, 0.0, laser_scan->angle_min + laser_scan->angle_increment);
  tf::Stamped<tf::Quaternion> inc_q(q, laser_scan->header.stamp,
                                    laser_scan->header.frame_id);
  try
  {
    tf_->transformQuaternion(base_frame_id_, min_q, min_q);
    tf_->transformQuaternion(base_frame_id_, inc_q, inc_q);
  }
  catch(tf::TransformException& ex)
  {
    std::stringstream ss;
    ss << "Unable to transform min/max laser angles into base frame : "
       << " base_frame_id=" << base_frame_id_
       << " : " << ex.what();
    throw std::runtime_error(ss.str());
  }

  double angle_min = tf::getYaw(min_q);
  double angle_increment = tf::getYaw(inc_q) - angle_min;

  // wrapping angle to [-pi .. pi]
  angle_increment = fmod(angle_increment + 5*M_PI, 2*M_PI) - M_PI;

  // Apply range min/max thresholds, if the user supplied them
  if(laser_max_range_ > 0.0)
    ldata.range_max = std::min(laser_scan->range_max, (float)laser_max_range_);
  else
    ldata.range_max = laser_scan->range_max;
  double range_min;
  if(laser_min_range_ > 0.0)
    range_min = std::max(laser_scan->range_min, (float)laser_min_range_);
  else
    range_min = laser_scan->range_min;
  // The AMCLLaserData destructor will free this memory
  ldata.ranges = new double[ldata.range_count][2];

  if (ldata.ranges == NULL)
  {
    throw std::runtime_error("Couldn't allocate buffer for laser data");
  }

  for(int i=0;i<ldata.range_count;i++)
  {
    // amcl doesn't (yet) have a concept of min range.  So we'll map short
    // readings to max range.
    if(laser_scan->ranges[i] <= range_min)
      ldata.ranges[i][0] = ldata.range_max;
    else
      ldata.ranges[i][0] = laser_scan->ranges[i];
    // Compute bearing
    ldata.ranges[i][1] = angle_min +
      (i * angle_increment);
  }

  // Generate set of particles from pose array
  pf_sample_set_t pf_set;

  // AFIAK no laser update use kdtree, or cluster but make them null
  // so a segfault occurs if laser update function tries to use it
  pf_set.kdtree = NULL;
  pf_set.clusters = NULL;
  pf_set.cluster_count = 0;
  pf_set.cluster_max_count = 0;
  pf_set.converged = 0; //?
  // Copy poses into particle positions
  pf_set.sample_count = poses.poses.size();
  pf_set.samples = new pf_sample_t[poses.poses.size()];

  if (pf_set.samples == NULL)
  {
    throw std::runtime_error("Couldn't allocate buffer for samples");
  }

  for (size_t ii = 0; ii < poses.poses.size(); ++ii)
  {
    pf_set.samples[ii].pose.v[0] = poses.poses[ii].position.x;
    pf_set.samples[ii].pose.v[1] = poses.poses[ii].position.y;

    double yaw, pitch, roll;
    const geometry_msgs::Quaternion &q(poses.poses[ii].orientation);
    tf::Matrix3x3(tf::Quaternion(q.x, q.y, q.z, q.w)).getEulerYPR(yaw,pitch,roll);
    pf_set.samples[ii].pose.v[2] = yaw;

    pf_set.samples[ii].weight = 1.0;
  }

  double total_weight = lasers_[laser_index]->UpdateSensor(&pf_set, &ldata);

  weights.resize(poses.poses.size());
  for (size_t ii = 0; ii<poses.poses.size(); ++ii)
  {
    weights[ii] = pf_set.samples[ii].weight;
  }

  delete[] pf_set.samples;

  ros::WallTime end = ros::WallTime::now();
  ROS_INFO("Took %f seconds to process %d poses", (end-start).toSec(), int(weights.size()));
}


double
AmclNode::getYaw(tf::Pose& t)
{
  double yaw, pitch, roll;
  t.getBasis().getEulerYPR(yaw,pitch,roll);
  return yaw;
}
