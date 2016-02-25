#include "tf2_ros/buffer.h"

#include <math.h>
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"

using std::cerr;
using std::endl;

geometry_msgs::TransformStamped tx2d(double dx, double dy, double dtheta, double dt,
                                     const std::string &parent, const std::string &child)
{
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


int
main(int argc, char** argv)
{
  tf2_ros::Buffer tfb;

  // we move forward 2 meters, then rotate 90 degrees, then forware another 1 meter
  tfb.setTransform(tx2d(0, 0, 0*M_PI/180, 0, "odom", "base"), "_auth");
  cerr << endl << "Move 0 " << endl << tfb.lookupTransform("odom","base", ros::Time()) << endl;

  tfb.setTransform(tx2d(2, 0, 0*M_PI/180, 1, "odom", "base"), "_auth");
  cerr << endl << "Move 1 " << endl << tfb.lookupTransform("odom","base", ros::Time()) << endl;

  tfb.setTransform(tx2d(2, 0, 90*M_PI/180, 2, "odom", "base"), "_auth");
  cerr << endl << "Move 2 " << endl << tfb.lookupTransform("odom","base", ros::Time()) << endl;

  tfb.setTransform(tx2d(2, 1, 90*M_PI/180, 3, "odom", "base"), "_auth");
  cerr << endl << "Move 3 " << endl << tfb.lookupTransform("odom","base", ros::Time()) << endl;

  
  

  // Transforms are translation then rotation
}
