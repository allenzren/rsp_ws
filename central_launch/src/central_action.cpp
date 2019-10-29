#include <ros/ros.h>

#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#include <central_launch/centralAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<central_launch::centralAction> Client;

class central
{
  public:

  int run();

  central();
  virtual ~central();


  private:

  ros::NodeHandle n_;

  // intermediate poses
  std::vector<geometry_msgs::Pose> prm_samples;

  tf::StampedTransform cub1_tf_stamped;
  tf::StampedTransform cub2_tf_stamped;
  tf::StampedTransform cub3_tf_stamped;
  tf::StampedTransform dest_tf_stamped;

  // object location
  geometry_msgs::Pose cub1;
  geometry_msgs::Pose cub2;
  geometry_msgs::Pose cub3;
  geometry_msgs::Pose dest;

  // set intermediate pose, stop mapping
  ros::Subscriber kinect_teleop_sub_;

  // gripper
  ros::Publisher gripper_pub_;
  void openGripper();
  void closeGripper();

  // action
  Client *client;
  void sendGoal(geometry_msgs::PoseStamped pose, int type);

  // helper functions
  std::vector<geometry_msgs::PoseStamped> findAbove(geometry_msgs::Pose pose);

  tf::TransformListener listener;
};

central::central():
n_()
{
  // tar_pose_pub_ = n_.advertise<geometry_msgs::PoseStamped>("target_pose", 1);
  // tran_pub_ = n_.advertise<geometry_msgs::PoseStamped>("tran", 1);

  gripper_pub_ = n_.advertise<std_msgs::String>("/ur_driver/URScript", 1);

  // initialize action client
  client = new Client("/ur5_planning_action/centralAction", true);
  // wait for action server to come up
  while(!client->waitForServer(ros::Duration(5.0))){
    ROS_INFO("Node waiting for the centralAction server");
  }
}

central::~central()
{
  
}

int central::run()
{
  usleep(5 * 1000 * 1000);
  
  // wait for planning initialization to be ready
  // bool kinematics_ready = false;
  bool planning_ready = false;
  // while(!(ros::param::get("kinematics_ready", kinematics_ready)  && kinematics_ready));
  while(!(ros::param::get("planning_ready", planning_ready) && planning_ready));
  ros::param::set("start", true);

  ROS_INFO("MAPPING STARTS!");

  /* 
  * -----------------------------------------------------------------------------
  * mapping process
  */
  bool mapping_done = false;
  boost::shared_ptr<std_msgs::String const> setPosePtr;
  ros::Duration setPose_timeout(0.2);

  while(1)
  {
    setPosePtr = ros::topic::waitForMessage<std_msgs::String>("/kinect/teleop/gesture", setPose_timeout);
    ROS_INFO("Waiting for for stop!");

    if(setPosePtr == NULL)
    {
      continue;
    }
    else if((setPosePtr->data) == "SET")
    {
      ROS_WARN("SET.");
      // record joint states
    }
    else if((setPosePtr->data) == "STOP")
    {
      ROS_WARN("STOP.");
      break;
    }
  }
  mapping_done = true;
  ros::param::set("mapping_done", mapping_done);

  /* 
  * -----------------------------------------------------------------------------
  * Pick and Place
  */
  ROS_INFO("PICK AND PLACE STARTS!");
  usleep(3*1000*1000);

  // find all markers first.
  while(1)
  {
    // cube1Ptr = ros::topic::waitForMessage<geometry_msgs::Pose>("/aruco_simple/cub1", marker_timeout);
    ros::Time now = ros::Time::now();
    
    try
    {
      listener.waitForTransform("base_link", "cube_1_frame", now, ros::Duration(0.1));
      listener.lookupTransform("base_link", "cube_1_frame", now, cub1_tf_stamped);

      listener.waitForTransform("base_link", "cube_2_frame", now, ros::Duration(0.1));
      listener.lookupTransform("base_link", "cube_2_frame", now, cub2_tf_stamped);

      listener.waitForTransform("base_link", "dest_frame", now, ros::Duration(0.1));
      listener.lookupTransform("base_link", "dest_frame", now, dest_tf_stamped);
    }
    catch(tf::TransformException &ex)
    {
      ROS_INFO("Cannot find both cube and target.");
      continue;
    }

    ROS_INFO("Found all cubes and target!");
    break;
  }

  tf::poseTFToMsg(cub1_tf_stamped, cub1);
  tf::poseTFToMsg(cub2_tf_stamped, cub2);
  tf::poseTFToMsg(dest_tf_stamped, dest);


  std::vector<geometry_msgs::PoseStamped> cub1_abv = findAbove(cub1);
  std::vector<geometry_msgs::PoseStamped> cub2_abv = findAbove(cub2);
  // std::vector<geometry_msgs::PoseStamped> cub3_abv = findAbove(cub3);
  std::vector<geometry_msgs::PoseStamped> dest_abv = findAbove(dest);

  /*
  * ------------------------------------------------------------------
  * Move to pick up cub1
  */

  // make sure gripper is open first
  openGripper();

  // ROS_INFO("Target: %f %f %f", cub1_abv[0].pose.position.x, cub1_abv[0].pose.position.y, cub1_abv[0].pose.position.z);
  // ROS_INFO("Target: %f %f %f %f", cub1_abv[0].pose.orientation.x, cub1_abv[0].pose.orientation.y,
  //   cub1_abv[0].pose.orientation.z, cub1_abv[0].pose.orientation.w);

  usleep(1*1000*1000);

  // move to above cube 1
  sendGoal(cub1_abv[0], 1);

  client->waitForResult(ros::Duration(0.0)); // infinite timeout
  if(client->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    ROS_INFO("Successfully moved to above cube 1.");
  }
  else
  {
    ROS_WARN("Cannot move to above cube 1");
    while(1);
  }

  usleep(1*1000*1000);

  // move downwards
  sendGoal(cub1_abv[1],2);

  client->waitForResult(ros::Duration(0.0)); // infinite timeout
  if(client->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    ROS_INFO("Successfully moved to cube 1.");
  }
  else
  {
    ROS_WARN("Cannot move to cube 1");
    while(1);
  }

  usleep(1*1000*1000);

  closeGripper();

  usleep(1*1000*1000);

  // move to above cube 1
  sendGoal(cub1_abv[0], 2);

  client->waitForResult(ros::Duration(0.0)); // infinite timeout
  if(client->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    ROS_INFO("Successfully moved to above cube 1.");
  }
  else
  {
    ROS_WARN("Cannot move to above cube 1");
    while(1);
  }

  usleep(1*1000*1000);

  
  // * ------------------------------------------------------------------
  // * Move cub1 to dest
  
  // move to above destination
  sendGoal(dest_abv[0], 3);

  client->waitForResult(ros::Duration(0.0)); // infinite timeout
  if(client->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    ROS_INFO("Successfully moved to above destination.");
  }
  else
  {
    ROS_WARN("Cannot move to above destination");
    while(1);
  }

  usleep(1*1000*1000);

  // move to destination
  sendGoal(dest_abv[2], 2);

  client->waitForResult(ros::Duration(0.0)); // infinite timeout
  if(client->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    ROS_INFO("Successfully moved to destination.");
  }
  else
  {
    ROS_WARN("Cannot move to above destination");
  }

  usleep(1*1000*1000);

  // open gripper
  openGripper();

  usleep(1*1000*1000);

  // move to above destination
  sendGoal(dest_abv[0], 2);

  client->waitForResult(ros::Duration(0.0)); // infinite timeout
  if(client->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    ROS_INFO("Successfully moved to above destination.");
  }
  else
  {
    ROS_WARN("Cannot move to above destination");
  }

  usleep(1*1000*1000);

  // closeGripper();

  /*
  * ------------------------------------------------------------------
  * Move to pick up cub2
  */

  // make sure gripper is open first
  openGripper();

  // ROS_INFO("Target: %f %f %f", cub1_abv[0].pose.position.x, cub1_abv[0].pose.position.y, cub1_abv[0].pose.position.z);
  // ROS_INFO("Target: %f %f %f %f", cub1_abv[0].pose.orientation.x, cub1_abv[0].pose.orientation.y,
  //   cub1_abv[0].pose.orientation.z, cub1_abv[0].pose.orientation.w);

  usleep(1*1000*1000);

  // move to above cube 2
  sendGoal(cub2_abv[0], 1);

  client->waitForResult(ros::Duration(0.0)); // infinite timeout
  if(client->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    ROS_INFO("Successfully moved to above cube 1.");
  }
  else
  {
    ROS_WARN("Cannot move to above cube 1");
    while(1);
  }

  usleep(1*1000*1000);

  // move downwards
  sendGoal(cub2_abv[1],2);

  client->waitForResult(ros::Duration(0.0)); // infinite timeout
  if(client->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    ROS_INFO("Successfully moved to cube 1.");
  }
  else
  {
    ROS_WARN("Cannot move to cube 1");
    while(1);
  }

  usleep(1*1000*1000);

  closeGripper();

  usleep(1*1000*1000);

  // move to above cube 2
  sendGoal(cub2_abv[0], 2);

  client->waitForResult(ros::Duration(0.0)); // infinite timeout
  if(client->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    ROS_INFO("Successfully moved to above cube 1.");
  }
  else
  {
    ROS_WARN("Cannot move to above cube 1");
    while(1);
  }

  usleep(1*1000*1000);

  // * ------------------------------------------------------------------
  // * Move cub2 to dest
  
  // move to above destination
  sendGoal(dest_abv[4], 3);

  client->waitForResult(ros::Duration(0.0)); // infinite timeout
  if(client->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    ROS_INFO("Successfully moved to above destination.");
  }
  else
  {
    ROS_WARN("Cannot move to above destination");
    while(1);
  }

  usleep(1*1000*1000);

  // move to destination
  sendGoal(dest_abv[3], 2);

  client->waitForResult(ros::Duration(0.0)); // infinite timeout
  if(client->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    ROS_INFO("Successfully moved to destination.");
  }
  else
  {
    ROS_WARN("Cannot move to above destination");
  }

  usleep(1*1000*1000);

  // open gripper
  openGripper();

  usleep(1*1000*1000);

  // move to above destination
  sendGoal(dest_abv[4], 2);

  client->waitForResult(ros::Duration(0.0)); // infinite timeout
  if(client->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    ROS_INFO("Successfully moved to above destination.");
  }
  else
  {
    ROS_WARN("Cannot move to above destination");
  }

  usleep(1*1000*1000);

  closeGripper();

  ROS_INFO("Task finished!");
  return 0;
}

// find the tf above the markers
// 
std::vector<geometry_msgs::PoseStamped> central::findAbove(geometry_msgs::Pose pose)
{
  // move up 0.28m and 0.15m from base, rotate 180 deg in y-axis, rotate 45 deg in x-axis
  tf::Matrix3x3 abv_rot_y(0, 0, 1, 0, 1, 0, -1, 0, 0);
  tf::Matrix3x3 abv_rot_x(1, 0, 0, 0, 0.707, -0.707, 0, 0.707, 0.707);
  tf::Vector3   abv_trans_1(0, 0, 0.30);  // before go down
  tf::Vector3   abv_trans_2(0, 0, 0.20);  // pick up cube 1 and 2
  tf::Vector3   abv_trans_3(0, 0, 0.24);  // place cube 1
  tf::Vector3   abv_trans_4(0, 0, 0.30);  // place cube 2
  tf::Vector3   abv_trans_5(0, 0, 0.36);  // cube 2 dest above

  tf::Transform abv_tf_1(abv_rot_y*abv_rot_x, abv_trans_1);
  tf::Transform abv_tf_2(abv_rot_y*abv_rot_x, abv_trans_2);
  tf::Transform abv_tf_3(abv_rot_y*abv_rot_x, abv_trans_3);
  tf::Transform abv_tf_4(abv_rot_y*abv_rot_x, abv_trans_4);
  tf::Transform abv_tf_5(abv_rot_y*abv_rot_x, abv_trans_5);

  // marker
  tf::Quaternion mar_quat(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
  tf::Vector3    mar_trans(pose.position.x, pose.position.y, pose.position.z);
  tf::Transform  mar_tf(mar_quat, mar_trans);

  geometry_msgs::Pose msg1;
  geometry_msgs::Pose msg2;
  geometry_msgs::Pose msg3;
  geometry_msgs::Pose msg4;
  geometry_msgs::Pose msg5;

  tf::poseTFToMsg(mar_tf*abv_tf_1, msg1);
  tf::poseTFToMsg(mar_tf*abv_tf_2, msg2);
  tf::poseTFToMsg(mar_tf*abv_tf_3, msg3);
  tf::poseTFToMsg(mar_tf*abv_tf_4, msg4);
  tf::poseTFToMsg(mar_tf*abv_tf_5, msg5);

  msg1.position.x -= 0.03;
  msg2.position.x -= 0.03;
  msg3.position.x -= 0.03;
  msg4.position.x -= 0.03;
  msg5.position.x -= 0.03;

  // msg1.position.y -= 0.01;
  // msg2.position.y -= 0.01;
  // msg3.position.y -= 0.01;
  // msg4.position.y -= 0.01;
  // msg5.position.y -= 0.01;

  geometry_msgs::PoseStamped msg1_stamped;
  geometry_msgs::PoseStamped msg2_stamped;
  geometry_msgs::PoseStamped msg3_stamped;
  geometry_msgs::PoseStamped msg4_stamped;
  geometry_msgs::PoseStamped msg5_stamped;

  msg1_stamped.header.frame_id = "base_link";
  msg1_stamped.pose = msg1;
  msg2_stamped.header.frame_id = "base_link";
  msg2_stamped.pose = msg2;
  msg3_stamped.header.frame_id = "base_link";
  msg3_stamped.pose = msg3;
  msg4_stamped.header.frame_id = "base_link";
  msg4_stamped.pose = msg4;
  msg5_stamped.header.frame_id = "base_link";
  msg5_stamped.pose = msg5;

  std::vector<geometry_msgs::PoseStamped> output;
  output.push_back(msg1_stamped);
  output.push_back(msg2_stamped);
  output.push_back(msg3_stamped);
  output.push_back(msg4_stamped);
  output.push_back(msg5_stamped);

  return output;
}

void central::openGripper()
{
  std_msgs::String init_gripper;
  init_gripper.data = "set_tool_voltage(24)";
  gripper_pub_.publish(init_gripper);

  usleep(0.5*1000*1000);

  std_msgs::String s1;
  s1.data = "set_digital_out(8,False)";
  gripper_pub_.publish(s1);
  
  usleep(0.5*1000*1000);
}

void central::closeGripper()
{
  std_msgs::String init_gripper;
  init_gripper.data = "set_tool_voltage(24)";
  gripper_pub_.publish(init_gripper);

  usleep(0.5*1000*1000);

  std_msgs::String s1;
  s1.data = "set_digital_out(8,True)";
  gripper_pub_.publish(s1);
  
  usleep(0.5*1000*1000);
}

void central::sendGoal(geometry_msgs::PoseStamped pose, int type)
{
  central_launch::centralGoal goal;
  goal.pose = pose;
  goal.type = type;

  goal.pose.header.stamp = ros::Time::now();
  client->sendGoal(goal);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "central_action");

  central central_node;

  return central_node.run();
}