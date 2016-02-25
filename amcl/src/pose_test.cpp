#include "tf2_ros/buffer.h"

#include <math.h>
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"
#include "tf2/LinearMath/Transform.h"

using std::cerr;
using std::endl;

geometry_msgs::TransformStamped tx2d(double dx, double dy, double dtheta, double dt,
                                     const std::string &parent, const std::string &child)
{
  dtheta *= M_PI/180.0;
  geometry_msgs::TransformStamped tx;
  geometry_msgs::Vector3 &t(tx.transform.translation);
  t.x = dx; t.y = dy; t.z = 0.0;
  geometry_msgs::Quaternion &q(tx.transform.rotation);
  q.x=q.y=0.0; q.z=sin(dtheta/2); q.w=cos(dtheta/2);
  std_msgs::Header &h(tx.header);
  h.stamp = ros::Time(dt);
  h.frame_id = parent;
  tx.child_frame_id = child;
  return tx;
}

/*  Moved forward 2 meters
transforms:
  -
    header:
      seq: 0
      stamp:
        secs: 1456360333
        nsecs: 520700923
      frame_id: odom
    child_frame_id: base_link
    transform:
      translation:
        x: 2.03968531857
        y: 0.00509840534769
        z: 0.0
      rotation:
        x: 0.0
        y: 0.0
        z: 0.0118346724763
        w: 0.999929967811

rosrun tf tf_echo odom base_link
At time 1456360432.660
- Translation: [2.040, 0.005, 0.000]
- Rotation: in Quaternion [0.000, 0.000, 0.012, 1.000]
            in RPY (radian) [0.000, -0.000, 0.024]
            in RPY (degree) [0.000, -0.000, 1.355]
*/


/* Rotate -90 degrees
transforms:
  -
    header:
      seq: 0
      stamp:
        secs: 1456360502
        nsecs: 399830692
      frame_id: odom
    child_frame_id: base_link
    transform:
      translation:
        x: 2.04224327458
        y: 0.00529340522398
        z: 0.0
      rotation:
        x: 0.0
        y: 0.0
        z: -0.704542542177
        w: 0.709661754826

rosrun tf tf_echo odom base_link
At time 1456360603.500
- Translation: [2.042, 0.005, 0.000]
- Rotation: in Quaternion [0.000, 0.000, -0.705, 0.710]
            in RPY (radian) [0.000, 0.000, -1.564]
            in RPY (degree) [0.000, 0.000, -89.585]
*/


/*  Move another 1 meter forward
transforms:
  -
    header:
      seq: 0
      stamp:
        secs: 1456360715
        nsecs: 899764445
      frame_id: odom
    child_frame_id: base_link
    transform:
      translation:
        x: 2.0556281702
        y: -1.01476717053
        z: 0.0
      rotation:
        x: 0.0
        y: 0.0
        z: -0.692537181504
        w: 0.721382181811

At time 1456360669.079
- Translation: [2.056, -1.015, 0.000]
- Rotation: in Quaternion [0.000, 0.000, -0.693, 0.721]
            in RPY (radian) [0.000, 0.000, -1.530]
            in RPY (degree) [0.000, 0.000, -87.663]
*/


/* Final position from base to odom
rosrun tf tf_echo base_link odom
At time 1456360817.360
- Translation: [-1.098, -2.013, 0.000]
- Rotation: in Quaternion [0.000, 0.000, 0.693, 0.721]
            in RPY (radian) [0.000, -0.000, 1.530]
            in RPY (degree) [0.000, -0.000, 87.663]
*/


/*
void Transformer::lookupTransform	(	const std::string &     target_frame,
const std::string &     source_frame,
const ros::Time &       time,
StampedTransform &      transform
)		const
Get the transform between two frames by frame ID.

Parameters:
target_frame	The frame to which data should be transformed
source_frame	The frame where the data originated
time	The time at which the value of the transform is desired. (0 will get the latest)
transform	The transform reference to fill.
*/

double getYaw(const geometry_msgs::Quaternion &q)
{
  return 2.0*atan2(q.z, q.w);
}

void printHdr()
{
  cerr << "Name   "
       << "\tStamp"
       << "\tFrame"
       << "\tChild"
       << "\tx    "
       << "\ty    "
       << "\tangle"
       << endl;
}

void print(const char* name, const geometry_msgs::TransformStamped &msg)
{

  cerr << " " << name
       << std::setprecision(1) << std::fixed
       << "\t"  << msg.header.stamp.toSec()
       << "\t" << msg.header.frame_id
       << "\t" << msg.child_frame_id
       << "\t" << msg.transform.translation.x
       << "\t" << msg.transform.translation.y
       << "\t" << (getYaw(msg.transform.rotation)*180/M_PI)
       << endl;
}


/**
 * @brief Convert geometry_msgs::PoseStamped into tf2::Transform
 *
 * TODO : pull this from somewhere else if it ever implemented
 */
static void fromMsg (const geometry_msgs::TransformStamped &msg, tf2::Transform &out)
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


/**
 * @brief Convert geometry_msgs::PoseStamped into tf2::Transform
 *
 * TODO : pull this from somewhere else if it ever implemented
 */
static void toMsg (const tf2::Transform &in, geometry_msgs::TransformStamped &msg)
{
  const tf2::Quaternion q(in.getRotation());
  msg.transform.rotation.x = q.x();
  msg.transform.rotation.y = q.y();
  msg.transform.rotation.z = q.z();
  msg.transform.rotation.w = q.w();
  msg.transform.translation.x = in.getOrigin()[0];
  msg.transform.translation.y = in.getOrigin()[1];
  msg.transform.translation.z = in.getOrigin()[2];
}


geometry_msgs::TransformStamped
operator *(const geometry_msgs::TransformStamped &a,
           const geometry_msgs::TransformStamped &b)
{
  tf2::Transform a2,b2;
  fromMsg(a,a2);
  fromMsg(b,b2);
  geometry_msgs::TransformStamped axb;
  toMsg(a2*b2, axb);
  return axb;
}

int
main(int argc, char** argv)
{
  tf2_ros::Buffer tfb;

  printHdr();

  // 0 - start                          : base-map  2, 0, 90
  // 1 - we move forward 1 meters,      : base-map  2, 1, 90
  // 2 - then forward another 2 meters, : base-map  2, 3, 90
  // 3 - then rotate 90 degrees,        : base-map  2, 3, 190
  // 4 - then forward another 1 meter   : base-map  1, 3, 180

  std::cerr<< "0" << std::endl;
  tfb.setTransform(tx2d(0, 0, 0, 1, "odom", "base"), "_auth");
  tfb.setTransform(tx2d(-2, -3, -90, 1, "map", "odom"), "_auth");
  print("", tfb.lookupTransform("odom","base", ros::Time()));
  print("", tfb.lookupTransform("map","base", ros::Time()));

  std::cerr<< "1" << std::endl;
  tfb.setTransform(tx2d(1, 0, 0, 2, "odom", "base"), "_auth");
  tfb.setTransform(tx2d(-2, -3, -90, 2, "map", "odom"), "_auth");
  print("", tfb.lookupTransform("odom","base", ros::Time()));
  print("", tfb.lookupTransform("map","base", ros::Time()));

  std::cerr<< "2" << std::endl;
  tfb.setTransform(tx2d(3, 0, 0, 3, "odom", "base"), "_auth");
  tfb.setTransform(tx2d(-2, -3, -90, 3, "map", "odom"), "_auth");
  print("", tfb.lookupTransform("odom","base", ros::Time()));
  print("", tfb.lookupTransform("map","base", ros::Time()));

  std::cerr<< "3" << std::endl;
  tfb.setTransform(tx2d(3, 0, 90, 4, "odom", "base"), "_auth");
  tfb.setTransform(tx2d(-2, -3, -90, 4, "map", "odom"), "_auth");
  print("", tfb.lookupTransform("odom","base", ros::Time()));
  print("", tfb.lookupTransform("map","base", ros::Time()));

  std::cerr<< "4" << std::endl;
  tfb.setTransform(tx2d(3, 1, 90, 5, "odom", "base"), "_auth");
  tfb.setTransform(tx2d(-2, -3, -90, 5, "map", "odom"), "_auth");
  print("", tfb.lookupTransform("odom","base", ros::Time()));
  print("", tfb.lookupTransform("map","base", ros::Time()));


  std::cerr<< "Last" << std::endl;
  geometry_msgs::TransformStamped tf1 =
    tfb.lookupTransform("odom", ros::Time(5),
                        "odom", ros::Time(1),
                        "map");
  print("wrt map", tf1);

  geometry_msgs::TransformStamped tf2 =
    tfb.lookupTransform("base", ros::Time(5),
                        "base", ros::Time(1),
                        "odom");
  print("wrt odom", tf2);


  geometry_msgs::TransformStamped tf3 =
    tfb.lookupTransform("base", ros::Time(1),
                        "base", ros::Time(5),
                        "map");
  print("wrt map", tf3);


  //tfb.setTransform(tx2d(2, 0, 90, 5, "map", "odom"), "_auth");
  //print("", tfb.lookupTransform("odom","base", ros::Time()));
  //print("", tfb.lookupTransform("map","odom", ros::Time()));
  //print("", tfb.lookupTransform("map","base", ros::Time()));

  // Transforms are translation then rotation
}
