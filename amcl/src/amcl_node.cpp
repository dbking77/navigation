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
#include <boost/shared_ptr.hpp>

// Signal handling
#include <signal.h>

#include "map/map.h"
#include "pf/pf.h"
#include "sensors/amcl_odom.h"
#include "sensors/amcl_laser.h"

#include "ros/assert.h"

// roscpp
#include "ros/ros.h"

// Messages that I need
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Pose.h"
#include "nav_msgs/GetMap.h"
#include "nav_msgs/SetMap.h"
#include "std_srvs/Empty.h"

// For TF2 transform support
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/message_filter.h"
//#include "tf2_ros/Transform.h"
#include "tf2/LinearMath/Transform.h"

// For transform support
#include "tf/transform_broadcaster.h"
#include "tf/transform_listener.h"
#include "tf/message_filter.h"
#include "tf/tf.h"
#include "message_filters/subscriber.h"

// Dynamic_reconfigure
#include "dynamic_reconfigure/server.h"
#include "amcl/AMCLConfig.h"

// Stand-alone configuration
#include <yaml-cpp/yaml.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>
#include <boost/filesystem.hpp>
#include <map_server/image_loader.h>

#define NEW_UNIFORM_SAMPLING 1

using namespace amcl;

// Pose hypothesis
typedef struct
{
  // Total weight (weights sum to 1)
  double weight;

  // Mean of pose esimate
  pf_vector_t pf_pose_mean;

  // Covariance of pose estimate
  pf_matrix_t pf_pose_cov;

} amcl_hyp_t;

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

namespace tf2
{

/**
 * @brief Convert geometry_msgs::PoseStamped into tf2::Transform
 */
void fromMsg (const geometry_msgs::TransformStamped &msg, tf2::Transform &out)
{
  tf2::Quaternion q(msg.transform.rotation.x,
                    msg.transform.rotation.y,
                    msg.transform.rotation.z,
                    msg.transform.rotation.w);
  tf2::Vector3 t(msg.transform.translation.x,
                  msg.transform.translation.y,
                  msg.transform.translation.z);
  out = tf2::Transform(q,t);
}

} //end namespace tf2

class AmclNode
{
  public:
    AmclNode();

    // Runs
    AmclNode(const std::string &config_yaml_fn,
             const std::string &map_yaml_fn,
             const std::string &initial_pose_yaml_fn,
             bool publish_to_ros);

    void loadMapYaml(const std::string &map_yaml_fn);

    void run(const std::string &in_bag_fn,
             const std::string &out_bag_fn);

    ~AmclNode();

    int process();
    void savePoseToServer();

    static double getYaw(const tf2::Quaternion &q);

  private:
    // Keep all information associated with a single laser in one struct
    struct LaserInfo : boost::noncopyable
    {
      LaserInfo() : laser(NULL), update(true) { }
      AMCLLaser* laser;
      double angle_min;
      double angle_increment;
      bool update;
    };

    typedef boost::shared_ptr<LaserInfo> LaserInfoPtr;

    tf::TransformBroadcaster* tfb_;
    tf::TransformListener* tf_;

    tf2_ros::Buffer tf_buffer_;

    // Bag file where output poses can be stored
    boost::shared_ptr<rosbag::Bag> out_bag_;

    bool sent_first_transform_;

    /* This is latest transfrom from odom frame with respect to (w.r.t.) map frame
     * note that this was opposite of latest_tf_ which was calculation of map wrt odom
     */
    tf::StampedTransform latest_tf_odom_wrt_map_;
    bool latest_tf_valid_;

    // Pose-generating function used to uniformly distribute particles over
    // the map
    static pf_vector_t uniformPoseGenerator(void* arg);
#if NEW_UNIFORM_SAMPLING
    static std::vector<std::pair<int,int> > free_space_indices;
#endif
    // Callbacks
    bool globalLocalizationCallback(std_srvs::Empty::Request& req,
                                    std_srvs::Empty::Response& res);
    bool nomotionUpdateCallback(std_srvs::Empty::Request& req,
                                    std_srvs::Empty::Response& res);
    bool setMapCallback(nav_msgs::SetMap::Request& req,
                        nav_msgs::SetMap::Response& res);

    void laserReceived(const sensor_msgs::LaserScanConstPtr& laser_scan);
    void bagLaserReceived(const sensor_msgs::LaserScanConstPtr& laser_scan);

    void initialPoseReceived(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg);
    void handleInitialPoseMessage(const geometry_msgs::PoseWithCovarianceStamped& msg);
    void mapReceived(const nav_msgs::OccupancyGridConstPtr& msg);


    bool transformOdomToMap(const pf_vector_t &pose,
                            const ros::Time &time,
                            tf::StampedTransform &tf_odom_wrt_map);

    bool bagTransformOdomToMap(const pf_vector_t &pose,
                               const ros::Time &time,
                               tf::StampedTransform &tf_odom_wrt_map);

    void updateParticleFilter(const pf_vector_t &pose,
                              const sensor_msgs::LaserScanConstPtr& laser_scan,
                              LaserInfoPtr laser_info);

    void handleMapMessage(const nav_msgs::OccupancyGrid& msg);
    void freeMapDependentMemory();
    map_t* convertMap( const nav_msgs::OccupancyGrid& map_msg );
    void updatePoseFromServer();
    void updatePoseFromYaml(YAML::Node &node);
    void applyInitialPose();

    static double getYaw(tf::Pose& t);

    //parameter for what odom to use
    std::string odom_frame_id_;

    //paramater to store latest odom pose
    tf::Stamped<tf::Pose> latest_odom_pose_;

    //parameter for what base to use
    std::string base_frame_id_;
    std::string global_frame_id_;

    /// True if running from bag file
    bool running_from_bag_;

    bool use_map_topic_;
    bool first_map_only_;

    ros::Duration gui_publish_period;
    ros::Time save_pose_last_time;
    ros::Duration save_pose_period;

    geometry_msgs::PoseWithCovarianceStamped last_published_pose;

    map_t* map_;
    char* mapdata;
    int sx, sy;
    double resolution;

    message_filters::Subscriber<sensor_msgs::LaserScan>* laser_scan_sub_;
    tf::MessageFilter<sensor_msgs::LaserScan>* laser_scan_filter_;
    ros::Subscriber initial_pose_sub_;

    /// Mapping from laser frame ID to laser data
    typedef std::map< std::string, LaserInfoPtr > frame_to_laser_t;
    frame_to_laser_t frame_to_laser_;

    // Particle filter
    pf_t *pf_;
    double pf_err_, pf_z_;
    bool pf_init_;
    pf_vector_t pf_odom_pose_;
    double d_thresh_, a_thresh_;
    int resample_interval_;
    int resample_count_;
    double laser_min_range_;
    double laser_max_range_;

    //Nomotion update control
    bool m_force_update;  // used to temporarily let amcl update samples even when no motion occurs...

    AMCLOdom* odom_;
    AMCLLaser* laser_;

    ros::Duration cloud_pub_interval;
    ros::Time last_cloud_pub_time;

    void requestMap();

    // Helper to get odometric pose from TF2 transform systemsx
    bool getBagOdomPose(double& x, double& y, double& yaw,
                        const ros::Time& time, const std::string& source_frame_id);

    // Helper to get odometric pose from transform system
    bool getOdomPose(tf::Stamped<tf::Pose>& pose,
                     double& x, double& y, double& yaw,
                     const ros::Time& t, const std::string& f);

    //time for tolerance on the published transform,
    //basically defines how long a map->odom transform is good for
    ros::Duration transform_tolerance_;

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Publisher pose_pub_;
    ros::Publisher particlecloud_pub_;
    ros::ServiceServer global_loc_srv_;
    ros::ServiceServer nomotion_update_srv_; //to let amcl update samples without requiring motion
    ros::ServiceServer set_map_srv_;
    ros::Subscriber initial_pose_sub_old_;
    ros::Subscriber map_sub_;

    amcl_hyp_t* initial_pose_hyp_;
    bool first_map_received_;
    bool first_reconfigure_call_;

    boost::recursive_mutex configuration_mutex_;
    dynamic_reconfigure::Server<amcl::AMCLConfig> *dsrv_;
    amcl::AMCLConfig default_config_;
    ros::Timer check_laser_timer_;

    int max_beams_, min_particles_, max_particles_;
    double alpha1_, alpha2_, alpha3_, alpha4_, alpha5_;
    double alpha_slow_, alpha_fast_;
    double z_hit_, z_short_, z_max_, z_rand_, sigma_hit_, lambda_short_;
  //beam skip related params
    bool do_beamskip_;
    double beam_skip_distance_, beam_skip_threshold_, beam_skip_error_threshold_;
    double laser_likelihood_max_dist_;
    odom_model_t odom_model_type_;
    double init_pose_[3];
    double init_cov_[3];
    laser_model_t laser_model_type_;
    bool tf_broadcast_;

    void reconfigureCB(amcl::AMCLConfig &config, uint32_t level);

    ros::Time last_laser_received_ts_;
    ros::Duration laser_check_interval_;
    void checkLaserReceived(const ros::TimerEvent& event);
};

std::vector<std::pair<int,int> > AmclNode::free_space_indices;

//#define USAGE "USAGE: amcl"

boost::shared_ptr<AmclNode> amcl_node_ptr;

void sigintHandler(int sig)
{
  // Save latest pose as we're shutting down.
  amcl_node_ptr->savePoseToServer();
  ros::shutdown();
}

void usage()
{
  std::cerr << "Usage : amcl | amcl <config.yaml> <map.yaml> <initial_pose.yaml> <in.bag> <out.bag>" << std::endl;
}

int
main(int argc, char** argv)
{
  ros::init(argc, argv, "amcl");
  ros::NodeHandle nh;

  // Override default sigint handler
  signal(SIGINT, sigintHandler);

  // Make our node available to sigintHandler
  if (argc == 1)
  {
    amcl_node_ptr.reset(new AmclNode());
  }
  else if (argc == 6)
  {
    amcl_node_ptr.reset(new AmclNode(argv[1], argv[2], argv[3], true));
    amcl_node_ptr->run(argv[4], argv[5]);
  }
  else
  {
    usage();
    ROS_ERROR("Incorrect number of arguments");
    exit(1);
  }

  ros::spin();

  // Without this, our boost locks are not shut down nicely
  amcl_node_ptr.reset();

  // To quote Morgan, Hooray!
  return(0);
}

AmclNode::AmclNode() :
        tfb_(NULL),
        tf_(NULL),
        running_from_bag_(false),
        sent_first_transform_(false),
        latest_tf_valid_(false),
        map_(NULL),
        pf_(NULL),
        resample_count_(0),
        odom_(NULL),
        laser_(NULL),
        private_nh_("~"),
        initial_pose_hyp_(NULL),
        first_map_received_(false),
        first_reconfigure_call_(true)
{
  boost::recursive_mutex::scoped_lock l(configuration_mutex_);

  // Grab params off the param server
  private_nh_.param("use_map_topic", use_map_topic_, false);
  private_nh_.param("first_map_only", first_map_only_, false);

  double tmp;
  private_nh_.param("gui_publish_rate", tmp, -1.0);
  gui_publish_period = ros::Duration(1.0/tmp);
  private_nh_.param("save_pose_rate", tmp, 0.5);
  save_pose_period = ros::Duration(1.0/tmp);

  private_nh_.param("laser_min_range", laser_min_range_, -1.0);
  private_nh_.param("laser_max_range", laser_max_range_, -1.0);
  private_nh_.param("laser_max_beams", max_beams_, 30);
  private_nh_.param("min_particles", min_particles_, 100);
  private_nh_.param("max_particles", max_particles_, 5000);
  private_nh_.param("kld_err", pf_err_, 0.01);
  private_nh_.param("kld_z", pf_z_, 0.99);
  private_nh_.param("odom_alpha1", alpha1_, 0.2);
  private_nh_.param("odom_alpha2", alpha2_, 0.2);
  private_nh_.param("odom_alpha3", alpha3_, 0.2);
  private_nh_.param("odom_alpha4", alpha4_, 0.2);
  private_nh_.param("odom_alpha5", alpha5_, 0.2);

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

  private_nh_.param("odom_model_type", tmp_model_type, std::string("diff"));
  if(tmp_model_type == "diff")
    odom_model_type_ = ODOM_MODEL_DIFF;
  else if(tmp_model_type == "omni")
    odom_model_type_ = ODOM_MODEL_OMNI;
  else if(tmp_model_type == "diff-corrected")
    odom_model_type_ = ODOM_MODEL_DIFF_CORRECTED;
  else if(tmp_model_type == "omni-corrected")
    odom_model_type_ = ODOM_MODEL_OMNI_CORRECTED;
  else
  {
    ROS_WARN("Unknown odom model type \"%s\"; defaulting to diff model",
             tmp_model_type.c_str());
    odom_model_type_ = ODOM_MODEL_DIFF;
  }

  private_nh_.param("update_min_d", d_thresh_, 0.2);
  private_nh_.param("update_min_a", a_thresh_, M_PI/6.0);
  private_nh_.param("odom_frame_id", odom_frame_id_, std::string("odom"));
  private_nh_.param("base_frame_id", base_frame_id_, std::string("base_link"));
  private_nh_.param("global_frame_id", global_frame_id_, std::string("map"));
  private_nh_.param("resample_interval", resample_interval_, 2);
  double tmp_tol;
  private_nh_.param("transform_tolerance", tmp_tol, 0.1);
  private_nh_.param("recovery_alpha_slow", alpha_slow_, 0.001);
  private_nh_.param("recovery_alpha_fast", alpha_fast_, 0.1);
  private_nh_.param("tf_broadcast", tf_broadcast_, true);

  transform_tolerance_.fromSec(tmp_tol);

  updatePoseFromServer();

  cloud_pub_interval.fromSec(1.0);
  tfb_ = new tf::TransformBroadcaster();
  tf_ = new tf::TransformListener();

  pose_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("amcl_pose", 2, true);
  particlecloud_pub_ = nh_.advertise<geometry_msgs::PoseArray>("particlecloud", 2, true);
  global_loc_srv_ = nh_.advertiseService("global_localization",
                                         &AmclNode::globalLocalizationCallback,
                                         this);
  nomotion_update_srv_= nh_.advertiseService("request_nomotion_update", &AmclNode::nomotionUpdateCallback, this);
  set_map_srv_= nh_.advertiseService("set_map", &AmclNode::setMapCallback, this);

  laser_scan_sub_ = new message_filters::Subscriber<sensor_msgs::LaserScan>(nh_, scan_topic_, 100);
  laser_scan_filter_ =
          new tf::MessageFilter<sensor_msgs::LaserScan>(*laser_scan_sub_,
                                                        *tf_,
                                                        odom_frame_id_,
                                                        100);
  laser_scan_filter_->registerCallback(boost::bind(&AmclNode::laserReceived,
                                                   this, _1));
  initial_pose_sub_ = nh_.subscribe("initialpose", 2, &AmclNode::initialPoseReceived, this);

  if(use_map_topic_) {
    map_sub_ = nh_.subscribe("map", 1, &AmclNode::mapReceived, this);
    ROS_INFO("Subscribed to map topic.");
  } else {
    requestMap();
  }
  m_force_update = false;

  dsrv_ = new dynamic_reconfigure::Server<amcl::AMCLConfig>(ros::NodeHandle("~"));
  dynamic_reconfigure::Server<amcl::AMCLConfig>::CallbackType cb = boost::bind(&AmclNode::reconfigureCB, this, _1, _2);
  dsrv_->setCallback(cb);

  // 15s timer to warn on lack of receipt of laser scans, #5209
  laser_check_interval_ = ros::Duration(15.0);
  check_laser_timer_ = nh_.createTimer(laser_check_interval_,
                                       boost::bind(&AmclNode::checkLaserReceived, this, _1));
}


void AmclNode::loadMapYaml(const std::string &map_yaml_fn)
{
  YAML::Node map = YAML::LoadFile(map_yaml_fn);
  std::string image_fn = map["image"].as<std::string>();
  double resolution = map["resolution"].as<double>();
  double origin[3];
  for (int ii=0; ii<3; ++ii)
  {
    origin[ii] = map["origin"][ii].as<double>();
  }
  bool negate = bool(map["negate"].as<int>());
  double occupied_thresh = map["occupied_thresh"].as<double>();
  double free_thresh = map["free_thresh"].as<double>();
  bool trinary = true;
  if (map["trinary"])
  {
    bool trinary = map["trinary"].as<bool>();
  }

  // Provide fully qualified path for image
  if(image_fn.size() == 0)
  {
    throw std::runtime_error("The image tag cannot be an empty string.");
  }
  if(image_fn[0] != '/')
  {
    image_fn = boost::filesystem::path(map_yaml_fn).parent_path().string() + "/" + image_fn;
    std::cerr << "Absolute image fn : " << image_fn << std::endl;
  }

  nav_msgs::GetMap::Response resp;
  map_server::loadMapFromFile(&resp, image_fn.c_str(), resolution, negate,
                              occupied_thresh, free_thresh, origin, trinary);

  handleMapMessage( resp.map );
}


void AmclNode::run(const std::string &in_bag_fn,
                   const std::string &out_bag_fn)
{
  std::cerr << "run" << std::endl;

  rosbag::Bag bag;
  bag.open(in_bag_fn, rosbag::bagmode::Read);

  if (!out_bag_fn.empty())
  {
    std::cerr << "Opening " << out_bag_fn << " for saving data" << std::endl;
    out_bag_.reset(new rosbag::Bag);
    out_bag_->open(out_bag_fn, rosbag::bagmode::Write);
  }

  std::vector<std::string> topics;
  topics.push_back(std::string("tf"));
  topics.push_back(std::string("base_scan"));

  std::cerr << "message_filter" << std::endl;
  tf2_ros::MessageFilter<sensor_msgs::LaserScan>
    laser_scan_filter(tf_buffer_,
                      odom_frame_id_,
                      100,
                      NULL);

  std::cerr << "register_callback" << std::endl;
  laser_scan_filter.registerCallback(boost::bind(&AmclNode::bagLaserReceived, this, _1));

  rosbag::View view(bag, rosbag::TopicQuery(topics));
  BOOST_FOREACH(rosbag::MessageInstance const msg, view)
  {
    tf2_msgs::TFMessage::ConstPtr tf_msg = msg.instantiate<tf2_msgs::TFMessage>();
    if (tf_msg != NULL)
    {
      for (size_t ii=0; ii<tf_msg->transforms.size(); ++ii)
      {
        tf_buffer_.setTransform(tf_msg->transforms[ii], "rosbag_authority");
      }
      continue;
    }

    sensor_msgs::LaserScan::ConstPtr base_scan = msg.instantiate<sensor_msgs::LaserScan>();
    if (base_scan != NULL)
    {
      laser_scan_filter.add(base_scan);
      continue;
    }

    std::cerr << "unsupported message type" << msg.getTopic() << std::endl;
  }


  if (out_bag_)
  {
    std::cerr << "closing output bag file" << std::endl;
    // closing output bag might take a while since it needs to store index
    out_bag_->close();
  }

  bag.close();

  std::cerr << "shutting down" << std::endl;
  ros::shutdown();
}


// Runs
AmclNode::AmclNode(const std::string &config_yaml_fn,
                   const std::string &map_yaml_fn,
                   const std::string &initial_pose_yaml_fn,
                   bool publish_to_ros) :
        tfb_(NULL),
        tf_(NULL),
        running_from_bag_(true),
        sent_first_transform_(false),
        latest_tf_valid_(false),
        map_(NULL),
        pf_(NULL),
        resample_count_(0),
        odom_(NULL),
        laser_(NULL),
              private_nh_("~"),
        initial_pose_hyp_(NULL),
        first_map_received_(false),
        first_reconfigure_call_(true)
{
  std::cerr << "Loading config file : " << config_yaml_fn << std::endl;
  YAML::Node config = YAML::LoadFile(config_yaml_fn);
  std::cerr << "Done loading config file" << std::endl;


  std::cerr << "Loading initial pose file : " << initial_pose_yaml_fn << std::endl;
  YAML::Node initial_pose = YAML::LoadFile(initial_pose_yaml_fn);
  updatePoseFromYaml(initial_pose);
  std::cerr << "Done loading initial pose " << std::endl;

  double tmp = config["gui_publish_rate"].as<double>(); //-1.0
  gui_publish_period = ros::Duration(1.0/tmp);

  laser_min_range_ = config["laser_min_range"].as<double>(); //, -1.0);
  laser_max_range_ = config["laser_max_range"].as<double>(); //-1.0);
  max_beams_ = config["laser_max_beams"].as<unsigned>(); // 30);
  min_particles_ = config["min_particles"].as<unsigned>(); //, 100);
  max_particles_ = config["max_particles"].as<unsigned>(); //, 5000);
  pf_err_ = config["kld_err"].as<double>(); // , 0.01);
  pf_z_ = config["kld_z"].as<double>(); //, 0.99);

  alpha1_ = config["odom_alpha1"].as<double>(); // 0.2);
  alpha2_ = config["odom_alpha2"].as<double>(); // 0.2);
  alpha3_ = config["odom_alpha3"].as<double>(); //  0.2);
  alpha4_ = config["odom_alpha4"].as<double>(); // 0.2);
  alpha5_ = config["odom_alpha5"].as<double>();  // 0.2);

  do_beamskip_ = config["do_beamskip"].as<bool>(); //false);
  beam_skip_distance_ = config["beam_skip_distance"].as<double>();  //0.5);
  beam_skip_threshold_ = config["beam_skip_threshold"].as<double>();  //0.3);
  beam_skip_error_threshold_ = config["beam_skip_error_threshold"].as<double>();  //0.9);

  z_hit_ = config["laser_z_hit"].as<double>(); // 0.95);
  z_short_ = config["laser_z_short"].as<double>(); //0.1);
  z_max_ = config["laser_z_max"].as<double>();  //0.05);
  z_rand_= config["laser_z_rand"].as<double>();  //0.05);
  sigma_hit_ = config["laser_sigma_hit"].as<double>();  //0.2);
  lambda_short_ = config["laser_lambda_short"].as<double>();  //0.1);
  laser_likelihood_max_dist_ = config["laser_likelihood_max_dist"].as<double>();  //2.0);

  std::string tmp_model_type;
  tmp_model_type = config["laser_model_type"].as<std::string>(); // "likelihood_field"));
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

  tmp_model_type = config["odom_model_type"].as<std::string>();  // std::string("diff"));
  if(tmp_model_type == "diff")
    odom_model_type_ = ODOM_MODEL_DIFF;
  else if(tmp_model_type == "omni")
    odom_model_type_ = ODOM_MODEL_OMNI;
  else if(tmp_model_type == "diff-corrected")
    odom_model_type_ = ODOM_MODEL_DIFF_CORRECTED;
  else if(tmp_model_type == "omni-corrected")
    odom_model_type_ = ODOM_MODEL_OMNI_CORRECTED;
  else
  {
    ROS_WARN("Unknown odom model type \"%s\"; defaulting to diff model",
             tmp_model_type.c_str());
    odom_model_type_ = ODOM_MODEL_DIFF;
  }

  d_thresh_ = config["update_min_d"].as<double>();  // 0.2);
  a_thresh_ = config["update_min_a"].as<double>();  // M_PI/6.0);
  odom_frame_id_ = config["odom_frame_id"].as<std::string>();  // std::string("odom"));
  base_frame_id_ = config["base_frame_id"].as<std::string>();  // std::string("base_link"));
  global_frame_id_ = config["global_frame_id"].as<std::string>();  // std::string("map"));
  resample_interval_ = config["resample_interval"].as<unsigned>();  // 2);

  double tmp_tol;
  tmp_tol = config["transform_tolerance"].as<double>();  // 0.1);
  alpha_slow_ = config["recovery_alpha_slow"].as<double>();  // 0.001);
  alpha_fast_ = config["recovery_alpha_fast"].as<double>();  // 0.1);
  tf_broadcast_ = config["tf_broadcast"].as<bool>();  // true);

  transform_tolerance_.fromSec(tmp_tol);

  // TODO get initial pose
  //updatePoseFromServer();

  cloud_pub_interval.fromSec(1.0);

  loadMapYaml(map_yaml_fn);

  if (publish_to_ros)
  {
    tfb_ = new tf::TransformBroadcaster();
    pose_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("amcl_pose", 2, true);
    particlecloud_pub_ = nh_.advertise<geometry_msgs::PoseArray>("particlecloud", 2, true);
  }

  m_force_update = false;
}



void AmclNode::reconfigureCB(AMCLConfig &config, uint32_t level)
{
  boost::recursive_mutex::scoped_lock cfl(configuration_mutex_);

  //we don't want to do anything on the first call
  //which corresponds to startup
  if(first_reconfigure_call_)
  {
    first_reconfigure_call_ = false;
    default_config_ = config;
    return;
  }

  if(config.restore_defaults){
    config = default_config_;
    //avoid looping
    config.restore_defaults = false;
  }

  d_thresh_ = config.update_min_d;
  a_thresh_ = config.update_min_a;

  resample_interval_ = config.resample_interval;

  laser_min_range_ = config.laser_min_range;
  laser_max_range_ = config.laser_max_range;

  gui_publish_period = ros::Duration(1.0/config.gui_publish_rate);
  save_pose_period = ros::Duration(1.0/config.save_pose_rate);

  transform_tolerance_.fromSec(config.transform_tolerance);

  max_beams_ = config.laser_max_beams;
  alpha1_ = config.odom_alpha1;
  alpha2_ = config.odom_alpha2;
  alpha3_ = config.odom_alpha3;
  alpha4_ = config.odom_alpha4;
  alpha5_ = config.odom_alpha5;

  z_hit_ = config.laser_z_hit;
  z_short_ = config.laser_z_short;
  z_max_ = config.laser_z_max;
  z_rand_ = config.laser_z_rand;
  sigma_hit_ = config.laser_sigma_hit;
  lambda_short_ = config.laser_lambda_short;
  laser_likelihood_max_dist_ = config.laser_likelihood_max_dist;

  if(config.laser_model_type == "beam")
    laser_model_type_ = LASER_MODEL_BEAM;
  else if(config.laser_model_type == "likelihood_field")
    laser_model_type_ = LASER_MODEL_LIKELIHOOD_FIELD;
  else if(config.laser_model_type == "likelihood_field_prob")
    laser_model_type_ = LASER_MODEL_LIKELIHOOD_FIELD_PROB;

  if(config.odom_model_type == "diff")
    odom_model_type_ = ODOM_MODEL_DIFF;
  else if(config.odom_model_type == "omni")
    odom_model_type_ = ODOM_MODEL_OMNI;
  else if(config.odom_model_type == "diff-corrected")
    odom_model_type_ = ODOM_MODEL_DIFF_CORRECTED;
  else if(config.odom_model_type == "omni-corrected")
    odom_model_type_ = ODOM_MODEL_OMNI_CORRECTED;

  if(config.min_particles > config.max_particles)
  {
    ROS_WARN("You've set min_particles to be greater than max particles, this isn't allowed so they'll be set to be equal.");
    config.max_particles = config.min_particles;
  }

  min_particles_ = config.min_particles;
  max_particles_ = config.max_particles;
  alpha_slow_ = config.recovery_alpha_slow;
  alpha_fast_ = config.recovery_alpha_fast;
  tf_broadcast_ = config.tf_broadcast;

  do_beamskip_= config.do_beamskip;
  beam_skip_distance_ = config.beam_skip_distance;
  beam_skip_threshold_ = config.beam_skip_threshold;

  pf_ = pf_alloc(min_particles_, max_particles_,
                 alpha_slow_, alpha_fast_,
                 (pf_init_model_fn_t)AmclNode::uniformPoseGenerator,
                 (void *)map_);
  pf_err_ = config.kld_err;
  pf_z_ = config.kld_z;
  pf_->pop_err = pf_err_;
  pf_->pop_z = pf_z_;

  // Initialize the filter
  pf_vector_t pf_init_pose_mean = pf_vector_zero();
  pf_init_pose_mean.v[0] = last_published_pose.pose.pose.position.x;
  pf_init_pose_mean.v[1] = last_published_pose.pose.pose.position.y;
  pf_init_pose_mean.v[2] = tf::getYaw(last_published_pose.pose.pose.orientation);
  pf_matrix_t pf_init_pose_cov = pf_matrix_zero();
  pf_init_pose_cov.m[0][0] = last_published_pose.pose.covariance[6*0+0];
  pf_init_pose_cov.m[1][1] = last_published_pose.pose.covariance[6*1+1];
  pf_init_pose_cov.m[2][2] = last_published_pose.pose.covariance[6*5+5];
  pf_init(pf_, pf_init_pose_mean, pf_init_pose_cov);
  pf_init_ = false;

  // Instantiate the sensor objects
  // Odometry
  delete odom_;
  odom_ = new AMCLOdom();
  ROS_ASSERT(odom_);
  odom_->SetModel( odom_model_type_, alpha1_, alpha2_, alpha3_, alpha4_, alpha5_ );
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
    ROS_INFO("Done initializing likelihood field model with probabilities.");
  }
  else if(laser_model_type_ == LASER_MODEL_LIKELIHOOD_FIELD){
    ROS_INFO("Initializing likelihood field model; this can take some time on large maps...");
    laser_->SetModelLikelihoodField(z_hit_, z_rand_, sigma_hit_,
                                    laser_likelihood_max_dist_);
    ROS_INFO("Done initializing likelihood field model.");
  }

  odom_frame_id_ = config.odom_frame_id;
  base_frame_id_ = config.base_frame_id;
  global_frame_id_ = config.global_frame_id;

  delete laser_scan_filter_;
  laser_scan_filter_ =
          new tf::MessageFilter<sensor_msgs::LaserScan>(*laser_scan_sub_,
                                                        *tf_,
                                                        odom_frame_id_,
                                                        100);
  laser_scan_filter_->registerCallback(boost::bind(&AmclNode::laserReceived,
                                                   this, _1));

  initial_pose_sub_ = nh_.subscribe("initialpose", 2, &AmclNode::initialPoseReceived, this);
}

void AmclNode::savePoseToServer()
{
  // We need to apply the last transform to the latest odom pose to get
  // the latest map pose to store.  We'll take the covariance from
  // last_published_pose.
  tf::Pose map_pose = latest_tf_odom_wrt_map_ * latest_odom_pose_;
  double yaw,pitch,roll;
  map_pose.getBasis().getEulerYPR(yaw, pitch, roll);

  ROS_DEBUG("Saving pose to server. x: %.3f, y: %.3f", map_pose.getOrigin().x(), map_pose.getOrigin().y() );

  private_nh_.setParam("initial_pose_x", map_pose.getOrigin().x());
  private_nh_.setParam("initial_pose_y", map_pose.getOrigin().y());
  private_nh_.setParam("initial_pose_a", yaw);
  private_nh_.setParam("initial_cov_xx",
                                  last_published_pose.pose.covariance[6*0+0]);
  private_nh_.setParam("initial_cov_yy",
                                  last_published_pose.pose.covariance[6*1+1]);
  private_nh_.setParam("initial_cov_aa",
                                  last_published_pose.pose.covariance[6*5+5]);
}


void AmclNode::updatePoseFromYaml(YAML::Node &node)
{
  std::cerr << "Update Pose From Yaml" << std::endl;
  init_pose_[0] = node["pose_x"].as<double>();
  init_pose_[1] = node["pose_y"].as<double>();
  init_pose_[2] = node["pose_theta"].as<double>();
  init_cov_[0] = node["cov_x"].as<double>();
  init_cov_[1] = node["cov_y"].as<double>();
  init_cov_[2] = node["cov_theta"].as<double>();
}

void AmclNode::updatePoseFromServer()
{
  init_pose_[0] = 0.0;
  init_pose_[1] = 0.0;
  init_pose_[2] = 0.0;
  init_cov_[0] = 0.5 * 0.5;
  init_cov_[1] = 0.5 * 0.5;
  init_cov_[2] = (M_PI/12.0) * (M_PI/12.0);
  // Check for NAN on input from param server, #5239
  double tmp_pos;
  private_nh_.param("initial_pose_x", tmp_pos, init_pose_[0]);
  if(!std::isnan(tmp_pos))
    init_pose_[0] = tmp_pos;
  else
    ROS_WARN("ignoring NAN in initial pose X position");
  private_nh_.param("initial_pose_y", tmp_pos, init_pose_[1]);
  if(!std::isnan(tmp_pos))
    init_pose_[1] = tmp_pos;
  else
    ROS_WARN("ignoring NAN in initial pose Y position");
  private_nh_.param("initial_pose_a", tmp_pos, init_pose_[2]);
  if(!std::isnan(tmp_pos))
    init_pose_[2] = tmp_pos;
  else
    ROS_WARN("ignoring NAN in initial pose Yaw");
  private_nh_.param("initial_cov_xx", tmp_pos, init_cov_[0]);
  if(!std::isnan(tmp_pos))
    init_cov_[0] =tmp_pos;
  else
    ROS_WARN("ignoring NAN in initial covariance XX");
  private_nh_.param("initial_cov_yy", tmp_pos, init_cov_[1]);
  if(!std::isnan(tmp_pos))
    init_cov_[1] = tmp_pos;
  else
    ROS_WARN("ignoring NAN in initial covariance YY");
  private_nh_.param("initial_cov_aa", tmp_pos, init_cov_[2]);
  if(!std::isnan(tmp_pos))
    init_cov_[2] = tmp_pos;
  else
    ROS_WARN("ignoring NAN in initial covariance AA");
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
  //lasers_.clear();
  //lasers_update_.clear();
  // TODO this cause
  frame_to_laser_.clear();

  map_ = convertMap(msg);

#if NEW_UNIFORM_SAMPLING
  // Index of free space
  free_space_indices.resize(0);
  for(int i = 0; i < map_->size_x; i++)
    for(int j = 0; j < map_->size_y; j++)
      if(map_->cells[MAP_INDEX(map_,i,j)].occ_state == -1)
        free_space_indices.push_back(std::make_pair(i,j));
#endif
  // Create the particle filter
  pf_ = pf_alloc(min_particles_, max_particles_,
                 alpha_slow_, alpha_fast_,
                 (pf_init_model_fn_t)AmclNode::uniformPoseGenerator,
                 (void *)map_);
  pf_->pop_err = pf_err_;
  pf_->pop_z = pf_z_;

  if (!running_from_bag_)
  {
    updatePoseFromServer();
  }

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

  // Instantiate the sensor objects
  // Odometry
  delete odom_;
  odom_ = new AMCLOdom();
  ROS_ASSERT(odom_);
  odom_->SetModel( odom_model_type_, alpha1_, alpha2_, alpha3_, alpha4_, alpha5_ );
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

  // In case the initial pose message arrived before the first map,
  // try to apply the initial pose now that the map has arrived.
  applyInitialPose();

}

void
AmclNode::freeMapDependentMemory()
{
  if( map_ != NULL ) {
    map_free( map_ );
    map_ = NULL;
  }
  if( pf_ != NULL ) {
    pf_free( pf_ );
    pf_ = NULL;
  }
  delete odom_;
  odom_ = NULL;
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
  delete dsrv_;
  freeMapDependentMemory();
  delete laser_scan_filter_;
  delete laser_scan_sub_;
  delete tfb_;
  delete tf_;
  // TODO: delete everything allocated in constructor
}

bool
AmclNode::getOdomPose(tf::Stamped<tf::Pose>& odom_pose,
                      double& x, double& y, double& yaw,
                      const ros::Time& t, const std::string& f)
{
  // Get the robot's pose
  tf::Stamped<tf::Pose> ident (tf::Transform(tf::createIdentityQuaternion(),
                                           tf::Vector3(0,0,0)), t, f);
  try
  {
    this->tf_->transformPose(odom_frame_id_, ident, odom_pose);
  }
  catch(tf::TransformException e)
  {
    ROS_WARN("Failed to compute odom pose, skipping scan (%s)", e.what());
    return false;
  }
  x = odom_pose.getOrigin().x();
  y = odom_pose.getOrigin().y();
  double pitch,roll;
  odom_pose.getBasis().getEulerYPR(yaw, pitch, roll);

  //ROS_ERROR("%s to odom, %f %f %f", f.c_str(), x, y, yaw);

  return true;
}

/**
 * @brief Get transform from source frame to odom frame at time
 * @returns true if transform was sucessfull, false otherwise
 * @param x reference to variable where x coordinate will be stored
 * @param y reference to variable where y coordinate will be stored
 */
bool
AmclNode::getBagOdomPose(double& x, double& y, double& yaw,
                         const ros::Time& time, const std::string& source_frame_id)
{
  geometry_msgs::TransformStamped transform;
  try
  {
    transform = tf_buffer_.lookupTransform(odom_frame_id_, source_frame_id, time);
  }
  catch(tf2::TransformException ex)
  {
    ROS_WARN("Failed to compute odom pose, skipping scan (%s)", ex.what());
    return false;
  }

  x = transform.transform.translation.x;
  y = transform.transform.translation.y;
  // TODO check that transform is purely around z-axis
  yaw = 2.0*atan2(transform.transform.rotation.z,
                  transform.transform.rotation.w);

  //ROS_ERROR("%s to odom, %f %f %f", source_frame_id.c_str(), x, y, yaw);

  return true;
}


pf_vector_t
AmclNode::uniformPoseGenerator(void* arg)
{
  map_t* map = (map_t*)arg;
#if NEW_UNIFORM_SAMPLING
  unsigned int rand_index = drand48() * free_space_indices.size();
  std::pair<int,int> free_point = free_space_indices[rand_index];
  pf_vector_t p;
  p.v[0] = MAP_WXGX(map, free_point.first);
  p.v[1] = MAP_WYGY(map, free_point.second);
  p.v[2] = drand48() * 2 * M_PI - M_PI;
#else
  double min_x, max_x, min_y, max_y;

  min_x = (map->size_x * map->scale)/2.0 - map->origin_x;
  max_x = (map->size_x * map->scale)/2.0 + map->origin_x;
  min_y = (map->size_y * map->scale)/2.0 - map->origin_y;
  max_y = (map->size_y * map->scale)/2.0 + map->origin_y;

  pf_vector_t p;

  ROS_DEBUG("Generating new uniform sample");
  for(;;)
  {
    p.v[0] = min_x + drand48() * (max_x - min_x);
    p.v[1] = min_y + drand48() * (max_y - min_y);
    p.v[2] = drand48() * 2 * M_PI - M_PI;
    // Check that it's a free cell
    int i,j;
    i = MAP_GXWX(map, p.v[0]);
    j = MAP_GYWY(map, p.v[1]);
    if(MAP_VALID(map,i,j) && (map->cells[MAP_INDEX(map,i,j)].occ_state == -1))
      break;
  }
#endif
  return p;
}

bool
AmclNode::globalLocalizationCallback(std_srvs::Empty::Request& req,
                                     std_srvs::Empty::Response& res)
{
  if( map_ == NULL ) {
    return true;
  }
  boost::recursive_mutex::scoped_lock gl(configuration_mutex_);
  ROS_INFO("Initializing with uniform distribution");
  pf_init_model(pf_, (pf_init_model_fn_t)AmclNode::uniformPoseGenerator,
                (void *)map_);
  ROS_INFO("Global initialisation done!");
  pf_init_ = false;
  return true;
}

// force nomotion updates (amcl updating without requiring motion)
bool
AmclNode::nomotionUpdateCallback(std_srvs::Empty::Request& req,
                                     std_srvs::Empty::Response& res)
{
        m_force_update = true;
        //ROS_INFO("Requesting no-motion update");
        return true;
}

bool
AmclNode::setMapCallback(nav_msgs::SetMap::Request& req,
                         nav_msgs::SetMap::Response& res)
{
  handleMapMessage(req.map);
  handleInitialPoseMessage(req.initial_pose);
  res.success = true;
  return true;
}

double AmclNode::getYaw(const tf2::Quaternion &q)
{
  double yaw, pitch, roll;
  tf2::Matrix3x3(q).getEulerYPR(yaw,pitch,roll);
  return yaw;
}

void printRotation(const char* name, const tf2::Quaternion &q)
{
  ROS_ERROR("Rotation %s, %f, %f, %f, %f",
            name,
            q.getAxis()[0],
            q.getAxis()[1],
            q.getAxis()[2],
            q.getAngle()
            );
}


/**
 * @brief Callback from bag-file based laser processing.  Should use tf2 instead of tf.
 */
void AmclNode::bagLaserReceived(const sensor_msgs::LaserScanConstPtr& laser_scan)
{
  if ((laser_scan->header.seq % 100) == 0)
  {
    std::cerr << "bagLaserReceived : " << laser_scan->header.seq << std::endl;
  }

  last_laser_received_ts_ = ros::Time::now();
  if( map_ == NULL )
  {
    std::cerr << "No map? : " << std::endl;
    return;
  }

  // Do we have the base->base_laser Tx yet?
  if(frame_to_laser_.find(laser_scan->header.frame_id) == frame_to_laser_.end())
  {
    ROS_DEBUG("Setting up laser %d (frame_id=%s)\n", (int)frame_to_laser_.size(), laser_scan->header.frame_id.c_str());

    tf2::Transform transform;
    try
    {
      geometry_msgs::TransformStamped geo_msg =
        tf_buffer_.lookupTransform(base_frame_id_, laser_scan->header.frame_id, ros::Time());
      tf2::fromMsg(geo_msg, transform);
      /*
      transform =
        tf2::Transform(tf2::Quaternion(geo_msg.transform.rotation.x,
                                       geo_msg.transform.rotation.y,
                                       geo_msg.transform.rotation.z,
                                       geo_msg.transform.rotation.w),
                       tf2::Vector3(geo_msg.transform.translation.x,
                                    geo_msg.transform.translation.y,
                                    geo_msg.transform.translation.z));
      */
    }
    catch(tf2::TransformException& ex)
    {
      ROS_ERROR("Couldn't transform from %s to %s, "
                "even though the message notifier is in use : %s",
                laser_scan->header.frame_id.c_str(),
                base_frame_id_.c_str(),
                ex.what());
      return;
    }

    double angle_min = laser_scan->angle_min;
    double angle_two = angle_min + laser_scan->angle_increment;
    tf2::Quaternion q_min(tf2::Vector3(0,0,1),angle_min);
    tf2::Quaternion q_two(tf2::Vector3(0,0,1),angle_two);
    q_min = transform*q_min;
    q_two = transform*q_two;
    angle_min = getYaw(q_min);
    angle_two = getYaw(q_two);
    // TODO use angles
    double angle_increment = fmod((angle_two-angle_min) + 5*M_PI, 2*M_PI) - M_PI;

    ROS_ERROR("Laser %s angles in base frame: min: %f inc: %f",
              laser_scan->header.frame_id.c_str(), angle_min, angle_increment);

    LaserInfoPtr laser_info(new LaserInfo);
    laser_info->angle_min = angle_min;
    laser_info->angle_increment = angle_increment;
    laser_info->laser = new AMCLLaser(*laser_);

    pf_vector_t laser_pose_v;
    laser_pose_v.v[0] = transform.getOrigin()[0]; //0.235;
    laser_pose_v.v[1] = transform.getOrigin()[1]; //0.0;
    // laser mounting angle is part of angle_min and angle_incremetn -> set to 0 here!
    laser_pose_v.v[2] = 0;
    ROS_ERROR("Received laser's pose wrt robot: %.3f %.3f %.3f",
              laser_pose_v.v[0],
              laser_pose_v.v[1],
              laser_pose_v.v[2]);

    frame_to_laser_[laser_scan->header.frame_id] = laser_info;
  }

  // Where was the robot when this scan was taken?
  pf_vector_t pose;
  if(!getBagOdomPose(pose.v[0], pose.v[1], pose.v[2],
                     laser_scan->header.stamp, base_frame_id_))
  {
    ROS_ERROR("Couldn't determine robot's pose associated with laser scan");
    return;
  }

  updateParticleFilter(pose, laser_scan, frame_to_laser_[laser_scan->header.frame_id]);
}

void printRotation(const char* name, const tf::Stamped<tf::Quaternion> &q)
{
  ROS_ERROR("Rotation %s, %f, %f, %f, %f",
            name,
            q.getAxis()[0],
            q.getAxis()[1],
            q.getAxis()[2],
            q.getAngle()
            );
}

void printPose(const char* name, const tf::Stamped<tf::Pose> &pose)
{
  double angle = tf::getYaw(pose.getRotation());
  ROS_ERROR("Transform %s wrt %s : x=%f y=%f angle=%f",
            name,
            pose.frame_id_.c_str(),
            pose.getOrigin()[0],
            pose.getOrigin()[1],
            (angle*180/M_PI)
            );
}


void printTransform(const char* name,
                    const std::string target,
                    const std::string source,
                    const tf::StampedTransform &t)
{
  double angle = tf::getYaw(t.getRotation());
  ROS_ERROR("Transform %s : target=%s, source=%s : frame=%s, child=%s : x=%f y=%f angle=%f",
            name,
            target.c_str(),
            source.c_str(),
            t.frame_id_.c_str(),
            t.child_frame_id_.c_str(),
            t.getOrigin()[0],
            t.getOrigin()[1],
            (angle*180/M_PI)
            );
}


void
AmclNode::laserReceived(const sensor_msgs::LaserScanConstPtr& laser_scan)
{
  last_laser_received_ts_ = ros::Time::now();
  if( map_ == NULL ) {
    return;
  }
  boost::recursive_mutex::scoped_lock lr(configuration_mutex_);

  // Do we have the base->base_laser Tx yet?
  if(frame_to_laser_.find(laser_scan->header.frame_id) == frame_to_laser_.end())
  {
    ROS_DEBUG("Setting up laser %d (frame_id=%s)\n", (int)frame_to_laser_.size(), laser_scan->header.frame_id.c_str());

    ROS_ERROR("Laser %s angles in base frame: min: %f inc: %f",
              laser_scan->header.frame_id.c_str(),
              laser_scan->angle_min, laser_scan->angle_increment);

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

    printRotation("inc_q", inc_q);
    printRotation("min_q", min_q);

    tf::Stamped<tf::Pose> ident (tf::Transform(tf::createIdentityQuaternion(),
                                             tf::Vector3(0,0,0)),
                                 ros::Time(), laser_scan->header.frame_id);
    tf::Stamped<tf::Pose> laser_pose;
    try
    {
      tf_->transformPose(base_frame_id_, ident, laser_pose);
      tf_->transformQuaternion(base_frame_id_, min_q, min_q);
      tf_->transformQuaternion(base_frame_id_, inc_q, inc_q);
    }
    catch(tf::TransformException& e)
    {
      ROS_ERROR("Couldn't transform from %s to %s, "
                "even though the message notifier is in use",
                laser_scan->header.frame_id.c_str(),
                base_frame_id_.c_str());
      return;
    }

    printRotation("inc_q2", inc_q);
    printRotation("min_q2", min_q);

    double angle_min = tf::getYaw(min_q);
    double angle_increment = tf::getYaw(inc_q) - angle_min;

    // wrapping angle to [-pi .. pi]
    // TODO use angles
    angle_increment = fmod(angle_increment + 5*M_PI, 2*M_PI) - M_PI;

    ROS_ERROR("Laser %s angles in base frame: min: %f inc: %f",
              laser_scan->header.frame_id.c_str(), angle_min, angle_increment);

    LaserInfoPtr laser_info(new LaserInfo);
    laser_info->angle_min = angle_min;
    laser_info->angle_increment = angle_increment;
    laser_info->laser = new AMCLLaser(*laser_);

    pf_vector_t laser_pose_v;
    laser_pose_v.v[0] = laser_pose.getOrigin().x();
    laser_pose_v.v[1] = laser_pose.getOrigin().y();
    // laser mounting angle comes from angle_min and angle_max
    laser_pose_v.v[2] = 0;
    laser_info->laser->SetLaserPose(laser_pose_v);
    ROS_ERROR("Received laser's pose wrt robot: %.3f %.3f %.3f %.3f",
              laser_pose_v.v[0],
              laser_pose_v.v[1],
              angle_min,
              angle_increment);

    frame_to_laser_[laser_scan->header.frame_id] = laser_info;
  }

  // Where was the robot when this scan was taken?
  pf_vector_t pose;
  if(!getOdomPose(latest_odom_pose_, pose.v[0], pose.v[1], pose.v[2],
                  laser_scan->header.stamp, base_frame_id_))
  {
    ROS_ERROR("Couldn't determine robot's pose associated with laser scan");
    return;
  }

  updateParticleFilter(pose, laser_scan, frame_to_laser_[laser_scan->header.frame_id]);
}


/**
 * @brief Calculate transform of odom frame wrt map frame using particle position
 * @param pose particle filter pose which is represents trasnform from base frame to map frame
 * @param time to lookup transform for
 * @param output where result transform will be stored
 * @returns Returns true if transform can be calculated, false otherwise
 */
bool AmclNode::transformOdomToMap(const pf_vector_t &pose,
                                  const ros::Time &time,
                                  tf::StampedTransform &tf_odom_wrt_map)
{
  try
  {
    // Lookup transform from odom w.r.t base : T(odom-base)
    tf::StampedTransform tf_odom_wrt_base;
    this->tf_->lookupTransform(base_frame_id_, odom_frame_id_, time, tf_odom_wrt_base);

    // Particle pose represents the position of the robot base w.r.t. the map : T(base-map)
    tf::Transform tf_base_wrt_map(tf::createQuaternionFromYaw(pose.v[2]),
                                  tf::Vector3(pose.v[0], pose.v[1], 0.0));

    // This is transform of odom w.r.t map : T(odom-map)
    // T(odom-map) = T(base-map) * T(odom-base)
    //tf::Transform tmp = tf::StampedTransform(tf_base_wrt_map,time,"","") * tf_odom_wrt_base;
    tf::Transform tmp = tf_base_wrt_map * tf_odom_wrt_base;
    tf_odom_wrt_map = tf::StampedTransform(tmp, time, global_frame_id_, odom_frame_id_);

    printTransform("tf_odom_wrt_map", "", "", tf_odom_wrt_map);
  }
  catch(tf::TransformException)
  {
    ROS_DEBUG("Failed to subtract base to odom transform");
    return false;
  }
  return true;
}


/**
 * @brief Calculate transform of odom frame wrt map frame using particle position
 * @param pose particle filter pose which is represents trasnform from base frame to map frame
 * @param time to lookup transform for
 * @param output where result transform will be stored
 * @returns Returns true if transform can be calculated, false otherwise
 *
 * Uses tf2 buffer, instead of tf::Listener
 */
bool AmclNode::bagTransformOdomToMap(const pf_vector_t &pose,
                                     const ros::Time &time,
                                     tf::StampedTransform &tf_odom_wrt_map)
{
  try
  {
    // Lookup transform from odom w.r.t base : T(odom-base)
    tf2::Transform tf_odom_wrt_base;
    tf2::fromMsg(tf_buffer_.lookupTransform(base_frame_id_, odom_frame_id_, time), tf_odom_wrt_base);

    // Particle pose represents the position of the robot base w.r.t. the map : T(base-map)
    tf2::Transform tf_base_wrt_map(tf2::Quaternion(tf2::Vector3(0,0,1),pose.v[2]),
                                   tf2::Vector3(pose.v[0], pose.v[1], 0.0));

    // This is transform of odom w.r.t map : T(odom-map)
    // T(odom-map) = T(base-map) * T(odom-base)
    //tf::Transform tmp = tf::StampedTransform(tf_base_wrt_map,time,"","") * tf_odom_wrt_base;
    tf2::Transform tmp = tf_base_wrt_map * tf_odom_wrt_base;

    tf::Quaternion q(tmp.getRotation()[0],
                     tmp.getRotation()[1],
                     tmp.getRotation()[2],
                     tmp.getRotation()[3]);
    tf::Vector3 t(tmp.getOrigin()[0],
                  tmp.getOrigin()[1],
                  tmp.getOrigin()[2]);
    tf_odom_wrt_map = tf::StampedTransform(tf::Transform(q,t), time, global_frame_id_, odom_frame_id_);

    //printTransform("tf_odom_wrt_map", "", "", tf_odom_wrt_map);
  }
  catch(tf2::TransformException ex)
  {
    ROS_ERROR("Failed to determine trasform from odom to map");
    return false;
  }
  return true;
}




/**
 * @brief pose new pose of robot base
 * @brief laser_scan new laser scan data
 * @brief laser_info info about laser which has new data
 */
void AmclNode::updateParticleFilter(const pf_vector_t &pose,
                                    const sensor_msgs::LaserScanConstPtr& laser_scan,
                                    LaserInfoPtr laser_info)
{
  pf_vector_t delta = pf_vector_zero();

  if(pf_init_)
  {
    // Compute change in pose
    //delta = pf_vector_coord_sub(pose, pf_odom_pose_);
    delta.v[0] = pose.v[0] - pf_odom_pose_.v[0];
    delta.v[1] = pose.v[1] - pf_odom_pose_.v[1];
    delta.v[2] = angle_diff(pose.v[2], pf_odom_pose_.v[2]);

    // See if we should update the filter
    bool update = fabs(delta.v[0]) > d_thresh_ ||
                  fabs(delta.v[1]) > d_thresh_ ||
                  fabs(delta.v[2]) > a_thresh_;
    update = update || m_force_update;
    m_force_update=false;

    // Set update flag for all lasers
    if(update)
    {
      BOOST_FOREACH(frame_to_laser_t::value_type& kv, frame_to_laser_)
      {
        kv.second->update = true;
      }
    }
  }

  bool force_publication = false;
  if(!pf_init_)
  {
    // Pose at last filter update
    pf_odom_pose_ = pose;

    // Filter is now initialized
    pf_init_ = true;

    // Set update flag for all lasers
    BOOST_FOREACH(frame_to_laser_t::value_type& kv, frame_to_laser_)
    {
      kv.second->update = true;
    }

    force_publication = true;

    resample_count_ = 0;
  }
  // If the robot has moved, update the filter
  else if(pf_init_ && laser_info->update)
  {
    //printf("pose\n");
    //pf_vector_fprintf(pose, stdout, "%.3f");

    AMCLOdomData odata;
    odata.pose = pose;
    // HACK
    // Modify the delta in the action data so the filter gets
    // updated correctly
    odata.delta = delta;

    // Use the action data to update the filter
    odom_->UpdateAction(pf_, (AMCLSensorData*)&odata);

    // Pose at last filter update
    //this->pf_odom_pose = pose;
  }

  bool resampled = false;
  // If the robot has moved, update the filter
  if(laser_info->update)
  {
    AMCLLaserData ldata;
    ldata.sensor = laser_info->laser;
    ldata.range_count = laser_scan->ranges.size();

    double angle_min = laser_info->angle_min;
    double angle_increment = laser_info->angle_increment;

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
    ROS_ASSERT(ldata.ranges);
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

    laser_info->laser->UpdateSensor(pf_, (AMCLSensorData*)&ldata);

    laser_info->update = false;

    pf_odom_pose_ = pose;

    // Resample the particles
    if(!(++resample_count_ % resample_interval_))
    {
      pf_update_resample(pf_);
      resampled = true;
    }

    pf_sample_set_t* set = pf_->sets + pf_->current_set;
    ROS_DEBUG("Num samples: %d\n", set->sample_count);

    // Publish the resulting cloud
    // TODO: set maximum rate for publishing
    if (!m_force_update) {
      geometry_msgs::PoseArray cloud_msg;
      cloud_msg.header.stamp = ros::Time::now();
      cloud_msg.header.frame_id = global_frame_id_;
      cloud_msg.poses.resize(set->sample_count);
      for(int i=0;i<set->sample_count;i++)
      {
        tf::poseTFToMsg(tf::Pose(tf::createQuaternionFromYaw(set->samples[i].pose.v[2]),
                                 tf::Vector3(set->samples[i].pose.v[0],
                                           set->samples[i].pose.v[1], 0)),
                        cloud_msg.poses[i]);
      }
      particlecloud_pub_.publish(cloud_msg);
    }
  }

  if(resampled || force_publication)
  {
    // Read out the current hypotheses
    double max_weight = 0.0;
    int max_weight_hyp = -1;
    std::vector<amcl_hyp_t> hyps;
    hyps.resize(pf_->sets[pf_->current_set].cluster_count);
    for(int hyp_count = 0;
        hyp_count < pf_->sets[pf_->current_set].cluster_count; hyp_count++)
    {
      double weight;
      pf_vector_t pose_mean;
      pf_matrix_t pose_cov;
      if (!pf_get_cluster_stats(pf_, hyp_count, &weight, &pose_mean, &pose_cov))
      {
        ROS_ERROR("Couldn't get stats on cluster %d", hyp_count);
        break;
      }

      hyps[hyp_count].weight = weight;
      hyps[hyp_count].pf_pose_mean = pose_mean;
      hyps[hyp_count].pf_pose_cov = pose_cov;

      if(hyps[hyp_count].weight > max_weight)
      {
        max_weight = hyps[hyp_count].weight;
        max_weight_hyp = hyp_count;
      }
    }

    if(max_weight > 0.0)
    {
      ROS_DEBUG("Max weight pose: %.3f %.3f %.3f",
                hyps[max_weight_hyp].pf_pose_mean.v[0],
                hyps[max_weight_hyp].pf_pose_mean.v[1],
                hyps[max_weight_hyp].pf_pose_mean.v[2]);

      /*
         puts("");
         pf_matrix_fprintf(hyps[max_weight_hyp].pf_pose_cov, stdout, "%6.3f");
         puts("");
       */

      geometry_msgs::PoseWithCovarianceStamped p;
      // Fill in the header
      p.header.frame_id = global_frame_id_;
      p.header.stamp = laser_scan->header.stamp;
      // Copy in the pose
      p.pose.pose.position.x = hyps[max_weight_hyp].pf_pose_mean.v[0];
      p.pose.pose.position.y = hyps[max_weight_hyp].pf_pose_mean.v[1];
      tf::quaternionTFToMsg(tf::createQuaternionFromYaw(hyps[max_weight_hyp].pf_pose_mean.v[2]),
                            p.pose.pose.orientation);
      // Copy in the covariance, converting from 3-D to 6-D
      pf_sample_set_t* set = pf_->sets + pf_->current_set;
      for(int i=0; i<2; i++)
      {
        for(int j=0; j<2; j++)
        {
          // Report the overall filter covariance, rather than the
          // covariance for the highest-weight cluster
          //p.covariance[6*i+j] = hyps[max_weight_hyp].pf_pose_cov.m[i][j];
          p.pose.covariance[6*i+j] = set->cov.m[i][j];
        }
      }
      // Report the overall filter covariance, rather than the
      // covariance for the highest-weight cluster
      //p.covariance[6*5+5] = hyps[max_weight_hyp].pf_pose_cov.m[2][2];
      p.pose.covariance[6*5+5] = set->cov.m[2][2];

      /*
         printf("cov:\n");
         for(int i=0; i<6; i++)
         {
         for(int j=0; j<6; j++)
         printf("%6.3f ", p.covariance[6*i+j]);
         puts("");
         }
      */

      if (out_bag_)
      {
        out_bag_->write("amcl_pose", p.header.stamp, p);
      }

      pose_pub_.publish(p);
      last_published_pose = p;

      ROS_DEBUG("New pose: %6.3f %6.3f %6.3f",
               hyps[max_weight_hyp].pf_pose_mean.v[0],
               hyps[max_weight_hyp].pf_pose_mean.v[1],
               hyps[max_weight_hyp].pf_pose_mean.v[2]);


      // Use best particle position hypothesis to calcutlate
      // transform from odom to map frame
      if (this->tf_)
      {
        if (!transformOdomToMap(hyps[max_weight_hyp].pf_pose_mean,
                                laser_scan->header.stamp,
                                latest_tf_odom_wrt_map_))
        {
          return;
        }
      }
      else
      {
        // If tf_ is null, then use tf2 instead
        if (!bagTransformOdomToMap(hyps[max_weight_hyp].pf_pose_mean,
                                   laser_scan->header.stamp,
                                   latest_tf_odom_wrt_map_))
        {
          return;
        }
      }

      latest_tf_valid_ = true;
    }
    else
    {
      ROS_ERROR("No pose!");
    }
  }


  if(latest_tf_valid_)
  {
    if (tf_broadcast_ == true)
    {
      // We want to send a transform that is good up until a
      // tolerance time so that odom can be used
      ros::Time transform_expiration = (laser_scan->header.stamp +
                                        transform_tolerance_);
      latest_tf_odom_wrt_map_.stamp_ = transform_expiration;
      //printTransform("broadcast", "", "", latest_tf_odom_wrt_map_);

      this->tfb_->sendTransform(latest_tf_odom_wrt_map_);
      sent_first_transform_ = true;
    }

    // Is it time to save our last pose to the param server
    ros::Time now = ros::Time::now();
    if((save_pose_period.toSec() > 0.0) &&
       (now - save_pose_last_time) >= save_pose_period)
    {
      this->savePoseToServer();
      save_pose_last_time = now;
    }
  }

}

double
AmclNode::getYaw(tf::Pose& t)
{
  double yaw, pitch, roll;
  t.getBasis().getEulerYPR(yaw,pitch,roll);
  return yaw;
}

void
AmclNode::initialPoseReceived(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg)
{
  handleInitialPoseMessage(*msg);
}

void
AmclNode::handleInitialPoseMessage(const geometry_msgs::PoseWithCovarianceStamped& msg)
{
  boost::recursive_mutex::scoped_lock prl(configuration_mutex_);
  if(msg.header.frame_id == "")
  {
    // This should be removed at some point
    ROS_WARN("Received initial pose with empty frame_id.  You should always supply a frame_id.");
  }
  // We only accept initial pose estimates in the global frame, #5148.
  else if(tf_->resolve(msg.header.frame_id) != tf_->resolve(global_frame_id_))
  {
    ROS_WARN("Ignoring initial pose in frame \"%s\"; initial poses must be in the global frame, \"%s\"",
             msg.header.frame_id.c_str(),
             global_frame_id_.c_str());
    return;
  }

  // In case the client sent us a pose estimate in the past, integrate the
  // intervening odometric change.
  tf::StampedTransform tx_odom;
  try
  {
    tf_->lookupTransform(base_frame_id_, ros::Time::now(),
                         base_frame_id_, msg.header.stamp,
                         global_frame_id_, tx_odom);
  }
  catch(tf::TransformException e)
  {
    // If we've never sent a transform, then this is normal, because the
    // global_frame_id_ frame doesn't exist.  We only care about in-time
    // transformation for on-the-move pose-setting, so ignoring this
    // startup condition doesn't really cost us anything.
    if(sent_first_transform_)
      ROS_WARN("Failed to transform initial pose in time (%s)", e.what());
    tx_odom.setIdentity();
  }

  tf::Pose pose_old, pose_new;
  tf::poseMsgToTF(msg.pose.pose, pose_old);
  pose_new = tx_odom.inverse() * pose_old;

  // Transform into the global frame

  ROS_INFO("Setting pose (%.6f): %.3f %.3f %.3f",
           ros::Time::now().toSec(),
           pose_new.getOrigin().x(),
           pose_new.getOrigin().y(),
           getYaw(pose_new));
  // Re-initialize the filter
  pf_vector_t pf_init_pose_mean = pf_vector_zero();
  pf_init_pose_mean.v[0] = pose_new.getOrigin().x();
  pf_init_pose_mean.v[1] = pose_new.getOrigin().y();
  pf_init_pose_mean.v[2] = getYaw(pose_new);
  pf_matrix_t pf_init_pose_cov = pf_matrix_zero();
  // Copy in the covariance, converting from 6-D to 3-D
  for(int i=0; i<2; i++)
  {
    for(int j=0; j<2; j++)
    {
      pf_init_pose_cov.m[i][j] = msg.pose.covariance[6*i+j];
    }
  }
  pf_init_pose_cov.m[2][2] = msg.pose.covariance[6*5+5];

  delete initial_pose_hyp_;
  initial_pose_hyp_ = new amcl_hyp_t();
  initial_pose_hyp_->pf_pose_mean = pf_init_pose_mean;
  initial_pose_hyp_->pf_pose_cov = pf_init_pose_cov;
  applyInitialPose();
}

/**
 * If initial_pose_hyp_ and map_ are both non-null, apply the initial
 * pose to the particle filter state.  initial_pose_hyp_ is deleted
 * and set to NULL after it is used.
 */
void
AmclNode::applyInitialPose()
{
  boost::recursive_mutex::scoped_lock cfl(configuration_mutex_);
  if( initial_pose_hyp_ != NULL && map_ != NULL ) {
    pf_init(pf_, initial_pose_hyp_->pf_pose_mean, initial_pose_hyp_->pf_pose_cov);
    pf_init_ = false;

    delete initial_pose_hyp_;
    initial_pose_hyp_ = NULL;
  }
}
