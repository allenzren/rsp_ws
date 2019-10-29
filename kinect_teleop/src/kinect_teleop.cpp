#include "kinect_teleop/kinect_teleop.h"

void KinectTeleop::timerCallback(const ros::TimerEvent&)
{
  ros::Time now = ros::Time::now();
  tf::StampedTransform tf_neck_hand;
  tf::Transform tf_cmd;
  tf::Transform tf_ur5_kinect = tf::Transform(tf::Quaternion(0.0, 0.0, 1.0, 0.0), tf::Vector3(0.0, 1.0, 0.0));
  tf::Transform tf_user_neck = tf::Transform(tf::Quaternion(0.5, -0.5, -0.5, 0.5), tf::Vector3(0.0, 0.0, 0.0));
  tf::Vector3 p_hand;
  tf::Vector3 p_hand_est;
  tf::Vector3 dp_cmd;
  tf::Vector3 p_cmd;
  tf::Quaternion q_cmd;
  geometry_msgs::PoseStamped pose_cmd;

  //  broadcast tf from ur5 to kinect
  tf_broadcaster_.sendTransform(tf::StampedTransform(tf_ur5_kinect, now, base_frame_, kinect_frame_));

  try
  {
    //  read transforms
    tf_listener_.waitForTransform(neck_frame_, hand_frame_, now, dt_);
    tf_listener_.lookupTransform(neck_frame_, hand_frame_, now, tf_neck_hand);
    p_hand = (tf_user_neck * tf_neck_hand).getOrigin();

    //  initialize
    if (!is_init_)
    {
      p_hand_est = p_hand;
      p_cmd = tf_ur5_ref_.getOrigin();
      is_init_ = true;
    }

    //  compute the translation
    dp_cmd = speed_ * alpha_ * (p_hand - p_hand_est);
    //if (dp_cmd.length() / dt_.toSec() > 1)
    //  throw std::runtime_error("Moving too fast.");
    p_cmd += dp_cmd;

    //  compute the orientation
    q_cmd = tf_ur5_ref_.getRotation();

    //  combine
    tf_cmd.setOrigin(p_cmd);
    tf_cmd.setRotation(q_cmd);

    //  broadcast tf_cmd
    tf_broadcaster_.sendTransform(tf::StampedTransform(tf_cmd, now, base_frame_, cmd_frame_));

    //  publish pose_cmd
    pose_cmd.header.stamp = now;
    pose_cmd.header.frame_id = base_frame_;
    tf::poseTFToMsg(tf_cmd, pose_cmd.pose);
    cmd_pub_.publish(pose_cmd);

    //  update estimation (exponential smoothing)
    p_hand_est = (1 - alpha_) * p_hand_est + alpha_ * p_hand;
  }
  catch (std::runtime_error &ex)
  {
  }
}

KinectTeleop::KinectTeleop()
{
  double hz;
  node_.param("hz", hz, HZ);
  node_.param("speed", speed_, SPEED);
  node_.param("alpha", alpha_, ALPHA);
  node_.param<std::string>("base_frame",    base_frame_,    BASE_FRAME  );
  node_.param<std::string>("tool_frame",    tool_frame_,    TOOL_FRAME  );
  node_.param<std::string>("cmd_frame",     cmd_frame_,     CMD_FRAME   );
  node_.param<std::string>("kinect_frame",  kinect_frame_,  KINECT_FRAME);
  node_.param<std::string>("neck_frame",    neck_frame_,    NECK_FRAME  );
  node_.param<std::string>("hand_frame",    hand_frame_,    HAND_FRAME  );
  dt_ = ros::Duration(1.0 / hz);
  cmd_pub_ = node_.advertise<geometry_msgs::PoseStamped>("target_pose", 1);
  timer_ = node_.createTimer(dt_, &KinectTeleop::timerCallback, this);
  is_init_ = false;
  for (size_t i = 0; i < 10; i++)
  {
    try
    {
      //  try to get UR5 ref pose
      ros::Time now = ros::Time::now();
      tf_listener_.waitForTransform(base_frame_, tool_frame_, now, ros::Duration(2.0));
      tf_listener_.lookupTransform(base_frame_, tool_frame_, now, tf_ur5_ref_);
    }
    catch (tf::TransformException &ex)
    {
      //  default UR5 ref pose
      tf_ur5_ref_.setOrigin(tf::Vector3(0.0, 0.4, 0.4));
      tf_ur5_ref_.setRotation(tf::Quaternion(0.0, 1.0, 0.0, 0.0));
    }
  }
  ROS_INFO("UR5 reference pose: \n(%.3f, %.3f, %.3f), (%.3f, %.3f, %.3f, %.3f))", 
    tf_ur5_ref_.getOrigin().x(), 
    tf_ur5_ref_.getOrigin().y(), 
    tf_ur5_ref_.getOrigin().z(), 
    tf_ur5_ref_.getRotation().x(), 
    tf_ur5_ref_.getRotation().y(), 
    tf_ur5_ref_.getRotation().z(), 
    tf_ur5_ref_.getRotation().w());
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "kinect_teleop");
  KinectTeleop kinect_teleop;
  ros::spin();;
  return 0;
}
