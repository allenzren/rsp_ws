#include <ros/ros.h>

#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>


class central
{
  public:

  int run();

  central();
  virtual ~central();


  private:

  ros::NodeHandle n_;

  // intermediate poses
  std::vector<geometry_msgs::Pose> waypoints;

  // object location
  geometry_msgs::Pose cub1;
  geometry_msgs::Pose cub2;
  geometry_msgs::Pose cub3;

  // target location
  geometry_msgs::Pose dest;

  //
  ros::Publisher tar_pose_pub_;
  ros::Publisher cur_joints_pub_;

  // ros::Publisher tran_pub_;

  // ros::Subscriber cub1_sub_;
  // ros::Subscriber cub2_sub_;
  // ros::Subscriber cub3_sub_;
  // ros::Subscriber dest_sub_;


};

central::central():
n_()
{
  cur_joints_pub_ = n_.advertise<sensor_msgs::JointState>("/move_group/fake_controller_joint_states", 1);
  tar_pose_pub_ = n_.advertise<geometry_msgs::PoseStamped>("target_pose", 1);

  // tran_pub_ = n_.advertise<geometry_msgs::PoseStamped>("tran", 1);
}

central::~central()
{
  
}

// after kinect, planning, kinematics all done, start
int central::run()
{
  usleep(5 * 1000 * 1000);

  // move robot to a good starting pose first
  sensor_msgs::JointState initial_sim_joints;
  initial_sim_joints.name.resize(6);
  initial_sim_joints.name[0] = "shoulder_pan_joint";
  initial_sim_joints.name[1] = "shoulder_lift_joint";
  initial_sim_joints.name[2] = "elbow_joint";
  initial_sim_joints.name[3] = "wrist_1_joint";
  initial_sim_joints.name[4] = "wrist_2_joint";
  initial_sim_joints.name[5] = "wrist_3_joint";

  initial_sim_joints.position.resize(6);
  initial_sim_joints.position[0] = -1;
  initial_sim_joints.position[1] = -1.5;
  initial_sim_joints.position[2] = -1.5;
  initial_sim_joints.position[3] = -1.5;
  initial_sim_joints.position[4] = 1.5;
  initial_sim_joints.position[5] = 0;
  cur_joints_pub_.publish(initial_sim_joints);
  ROS_INFO("Initial simulation joint values published!");

  // while(1);
  
  // wait for planning initialization to be ready
  bool kinematics_ready = false;
  bool planning_ready = false;
  while(!(ros::param::get("kinematics_ready", kinematics_ready)  && kinematics_ready));
  while(!(ros::param::get("planning_ready", planning_ready) && planning_ready));
  ros::param::set("start", true);

  ROS_INFO("MAPPING STARTS!");

  /* 
  * mapping process
  */
  bool mapping_done = false;

  while(1)
  {

    // wait until stop gesture
    usleep(5*1000*1000);

    mapping_done = true;
    ros::param::set("mapping_done", mapping_done);
    break;
  }

  ROS_INFO("PICK AND PLACE STARTS!");

  /* 
  * Pick and Place
  */

  boost::shared_ptr<geometry_msgs::Pose const> sharedPtr1;
  boost::shared_ptr<geometry_msgs::Pose const> sharedPtr2;
  boost::shared_ptr<geometry_msgs::Pose const> sharedPtr3;
  boost::shared_ptr<geometry_msgs::Pose const> sharedPtr4;
  ros::Duration marker_timeout(0.1);

  while(1)
  {
    // look for Aruco tag poses
    sharedPtr1 = ros::topic::waitForMessage<geometry_msgs::Pose>("/aruco_simple/cub1", marker_timeout);
    if(sharedPtr1 != NULL)
    {
      cub1 = *sharedPtr1;
      ROS_INFO("Found cube 1!");
    }
    sharedPtr2 = ros::topic::waitForMessage<geometry_msgs::Pose>("/aruco_simple/cub2", marker_timeout);
    if(sharedPtr2 != NULL)
    {
      cub2 = *sharedPtr2;
      ROS_INFO("Found cube 2!");
    }
    sharedPtr3 = ros::topic::waitForMessage<geometry_msgs::Pose>("/aruco_simple/cub3", marker_timeout);
    if(sharedPtr3 != NULL)
    {
      cub3 = *sharedPtr3;
      ROS_INFO("Found cube 3!");
    }
    sharedPtr4 = ros::topic::waitForMessage<geometry_msgs::Pose>("/aruco_simple/dest", marker_timeout);
    if(sharedPtr4 != NULL)
    {
      dest = *sharedPtr4;
      ROS_INFO("Found cube 4!");
    }
    if(sharedPtr1 != NULL && sharedPtr2 != NULL && sharedPtr3 != NULL && sharedPtr4 != NULL)
    {
      ROS_INFO("Found all cubes and target.");
      break;
    }
    // if(sharedPtr1 == NULL || sharedPtr2 == NULL || sharedPtr3 == NULL || sharedPtr4 == NULL)
    // {
    //   ROS_INFO("Cannot find all cubes and target!");
    // }
    ROS_INFO("looking for transform...");
  }

  tf::Matrix3x3 cub_rot(1, 0, 0, 0, -1, 0, 0, 0, -1);
  tf::Vector3   cub_trans(0, 0, 0.28);
  tf::Transform cub_tf(cub_rot, cub_trans);

  tf::Quaternion cub1_quat(cub1.orientation.x, cub1.orientation.y, cub1.orientation.z, cub1.orientation.w);
  tf::Vector3    cub1_trans(cub1.position.x, cub1.position.y, cub1.position.z);
  tf::Transform cub1_tf(cub1_quat, cub1_trans);
  // tf::Transform tmp = cub1_tf*cub_tf;

  geometry_msgs::Pose msg;
  // tf::Stamped<t grasp_tf_pose(tmp);
  tf::poseTFToMsg(cub1_tf*cub_tf, msg);
  geometry_msgs::PoseStamped msg_stamped;
  msg_stamped.header.frame_id = "base_link";
  msg_stamped.pose = msg;

  ROS_INFO("Target: %f %f %f", msg.position.x, msg.position.y, msg.position.z);
  for(int i = 0; i <= 2; i++)
  {
    // tar_pose_pub_.publish(msg_stamped);
  }
  ROS_INFO("Task finished!");
  return 0;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "central");
  // ros::AsyncSpinner spinner(4);
  // spinner.start();

  central central_node;

  return central_node.run();
}