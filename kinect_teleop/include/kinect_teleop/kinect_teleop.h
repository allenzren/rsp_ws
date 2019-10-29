#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>

class KinectTeleop
{
private:
  ros::NodeHandle node_;
  ros::Duration dt_;
  ros::Timer timer_;
  ros::Publisher cmd_pub_;
  tf::TransformListener tf_listener_;
  tf::TransformBroadcaster tf_broadcaster_;
  tf::StampedTransform tf_ur5_ref_;
  bool is_init_;
  double speed_;
  double alpha_;
  std::string base_frame_, tool_frame_, cmd_frame_, kinect_frame_, neck_frame_, hand_frame_;

  const double HZ = 10.0;
  const double SPEED = 1.0;
  const double ALPHA = 0.2;
  const std::string BASE_FRAME    = "/base_link";
  const std::string TOOL_FRAME    = "/ee_link";
  const std::string CMD_FRAME     = "/cmd";
  const std::string KINECT_FRAME  = "/openni_depth_frame";
  const std::string NECK_FRAME    = "/neck_1";
  const std::string HAND_FRAME    = "/left_hand_1";

  void timerCallback(const ros::TimerEvent&);

public:
  KinectTeleop();
};
