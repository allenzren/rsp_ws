/* modified from moveit kinematics model tutorial */

/* Author: Sachin Chitta */
/* Zhiyi Ren, 04/06/2018 */

/* http://docs.ros.org/kinetic/api/moveit_tutorials/html/
doc/pr2_tutorials/kinematics/src/doc/kinematic_model_tutorial.html */

#include <ros/ros.h>

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

// #include <moveit/move_group_interface/move_group_interface.h>
// #include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

// #include <moveit_msgs/AttachedCollisionObject.h>
// #include <moveit_msgs/CollisionObject.h>

// #include <moveit_visual_tools/moveit_visual_tools.h>

// FollowJointTrajectory action
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

#include <sensor_msgs/JointState.h>

// #include <trajectory_msgs/JointTrajectoryPoint.h>

typedef actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> TrajClient;

// namespace rvt = rviz_visual_tools;

class ur5_kinematics
{
  public:
    // const Eigen::Affine3d &end_effector_state;
    // Eigen::MatrixXd jacobian;

    // std::vector<double> get_joint_values();
    // void forward_kinematics();
    // bool inverse_kinematics();
    // void get_jacobian();

    // bool move_joints();

    // void update_traj();

    // bool move_end_effector();
    // void move_joints();

    // void update_rviz();

    // int traj_run();
    int run();

    ur5_kinematics();
    virtual ~ur5_kinematics();

  private:
    ros::NodeHandle n_;
    // ros::AsyncSpinner spinner;
    // ros::Rate r;


    robot_model_loader::RobotModelLoader robot_model_loader;
    robot_model::RobotModelPtr kinematic_model;
    robot_state::RobotStatePtr kinematic_state;

    std::vector<std::string> joint_names;
    std::vector<double> joint_limits;

    // bool set_joint_limits();  // set additional joint limits for UR5 if needed

    // ros::Subscriber hand_pose_sub_;
    ros::Subscriber cur_joints_sub_;  // ask for current joints each time
    ros::Publisher  tar_pose_pub_;  // publish target pose to planning node

    geometry_msgs::PoseStamped target_pose;
    std::vector<double> target_joints;
    std::vector<double> cur_joints;

    // TrajClient* traj_client_;
    // control_msgs::FollowJointTrajectoryGoal goal;

    const std::string robot_name;
    const robot_state::JointModelGroup *joint_model_group;
    // moveit::planning_interface::MoveGroupInterface move_group;
    // moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    // rviz
    // moveit_visual_tools::MoveItVisualTools visual_tools;
    // Eigen::Affine3d text_pose;

};

// initialization
ur5_kinematics::ur5_kinematics():
n_(),
// r(20),
robot_model_loader("robot_description"),
kinematic_model(robot_model_loader.getModel()),
kinematic_state(new robot_state::RobotState(kinematic_model)),
robot_name("manipulator"),
// move_group(robot_name),
// visual_tools("odom_combined"),
joint_limits()
{
  // spinner.start();

  // construct a RobotState that maintains the configurations of UR5
  kinematic_state->setToDefaultValues();
  joint_model_group = kinematic_model->getJointModelGroup(robot_name);
  joint_names = joint_model_group->getVariableNames();

  // listen to pose topic from hand tracking
  // hand_pose_sub_


  // rviz visualization
  // visual_tools.deleteAllMarkers();
  // visual_tools.loadRemoteControl();

  // text_pose = Eigen::Affine3d::Identity ();
  // text_pose.translation().z() = 1.75; // above head of UR5 ?
  // visual_tools.publishText(text_pose, "UR5", rvt::WHITE, rvt::XLARGE);

  // traj_client_ = new TrajClient("follow_joint_trajectory", true);

  // while(!traj_client_->waitForServer(ros::Duration(5.0))){
    // ROS_INFO("Waiting for the joint_trajectory_action server");
  // }

  // initialize trajectory
  // goal.trajectory.joint_names = {"shoulder_pan_joint", "shoulder_lift_//joint",
    // "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};  
  // goal.trajectory.joint_names = joint_names;
  // goal.trajectory.points.resize(2);
  // goal.trajectory.points[0].positions.resize(6);
  // goal.trajectory.points[1].positions.resize(6);
  // goal.trajectory.points[0].velocities.resize(6);
  // goal.trajectory.points[1].velocities.resize(6);
  // for (size_t i = 0; i < 2; ++i)
  // {
  //   for (size_t j = 0; j < 6; ++j)
  //   {
  //     goal.trajectory.points[i].velocities[j] = 0.0;
  //   }
  // }
  // // To be reached 0.1 second after starting along the trajectory
  // goal.trajectory.points[0].time_from_start = ros::Duration(0);
  // goal.trajectory.points[1].time_from_start = ros::Duration(5);
}

ur5_kinematics::~ur5_kinematics()
{
  // delete traj_client_;
}

int ur5_kinematics::run()
{

  tar_pose_pub_ = n_.advertise<geometry_msgs::PoseStamped>("target_pose", 1);

  // // usleep(15 * 1000 * 1000);
  // target_pose.header.frame_id = "base_link";  // relative to the base
  // target_pose.pose.orientation.w = 0.397;
  // target_pose.pose.orientation.x = -0.562;
  // target_pose.pose.orientation.y = 0.281;
  // target_pose.pose.orientation.z = 0.669;
  
  // // target_pose.pose.position.x = -0.146;
  // target_pose.pose.position.x = -0.156;
  // target_pose.pose.position.y = 0.456;
  // target_pose.pose.position.z = 0.508;
  // double poses[10][3];

  // for(size_t i = 1; i <= 10; ++i)
  // {
  //   poses[i-1][0] = -0.146+0.01*i;
  //   poses[i-1][1] = 0.456-0.01*i;
  //   poses[i-1][2] = 0.508;
  // }

  // change kinematics_ready parameter to true
  ros::param::set("kinematics_ready", true);
  ROS_INFO("Kinematics node is ready!");


  bool start = false;
  while(!(ros::param::get("start", start) && start));


  ROS_INFO("Kinematics node starts!");
  // for(size_t i = 0; i < 10; ++i)
  // {
  //   target_pose.pose.position.x = poses[i][0];
  //   target_pose.pose.position.y = poses[i][1];
  //   target_pose.pose.position.z = poses[i][2];
  //   // tar_pose_pub_.publish(target_pose);

  //   // simulate 10 hz
  //   usleep(1 * 1000 * 1000);
  // //   // wait from planning node, if movement is finished
  // }
  // for(size_t i = 0; i < 10; ++i)
  // {
  //   // usleep(0.5*1000*1000);
  //   tar_pose_pub_.publish(target_pose);
  //   // usleep(10*1000*1000);
  // // }

  

  return 0;
}

// bool ur5_kinematics::move_end_effector()
// {
//   move_group.setPoseTarget(target_pose);

//   bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
//   if(!success){
//     ROS_INFO("Fail to plan!");
//     return false;
//   }

//   move_group.move();
//   return true;
// }

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ur5_kinematics");
  // ros::AsyncSpinner spinner(4);
  // spinner.start();

  ur5_kinematics kinematics_node;
  return kinematics_node.run();
}

// std::vector<double> ur5_kinematics::get_joint_values()
// {
//   kinematic_state->copyJointGroupPositions(joint_model_group, cur_joints);
//   for (std::size_t i = 0; i < joint_names.size(); ++i)
//   {
//     ROS_INFO("Joint Current Value %s: %f", joint_names[i].c_str(), cur_joints[i]);
//   }
// }

// ur5_kinematics::get_jacobian()
// {
//   Eigen::Vector3d reference_point_position(0.0, 0.0, 0.0);
//   kinematic_state->getJacobian(joint_model_group,
//     kinematic_state->getLinkModel(joint_model_group->getLinkModelNames().back()),
//     reference_point_position, jacobian);
//   ROS_INFO_STREAM("Jacobian: " << jacobian);
// }

// bool ur5_kinematics::set_joint_limits(int num)
// { 
//   joint_values[0] = 1.57;
//   kinematic_state->setJointGroupPositions(joint_model_group, joint_values);
//   ROS_INFO_STREAM("Current state is " << (kinematic_state->satisfiesBounds() ? "valid" : "not valid"));
//   kinematic_state->enforceBounds();
//   ROS_INFO_STREAM("Current state is " << (kinematic_state->satisfiesBounds() ? "valid" : "not valid"));
// }

// ur5_kinematics::forward_kinematics()
// {
//   kinematic_state->setToRandomPositions(joint_model_group);
//   &end_effector_state = kinematic_state->getGlobalLinkTransform("r_wrist_roll_link");
//   ROS_INFO_STREAM("Translation: " << end_effector_state.translation());
//   ROS_INFO_STREAM("Rotation: " << end_effector_state.rotation());
// }

// void ur5_kinematics::update_rviz()
// {
//   ROS_INFO_NAMED("ur5_kinematics", "Visualizing plan as trajectory line");

//   visual_tools.publishAxisLabeled(target_pose, "pose");
//   visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
//   visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
//   visual_tools.trigger();
//   visual_tools.prompt("next step");
// }

// void ur5_kinematics::cur_joints_callback(const sensor_msgs::JointState::ConstPtr& state)
// {
//   cur_joints[0] = state->position[0];
//   cur_joints[1] = state->position[1];
//   cur_joints[2] = state->position[2];
//   cur_joints[3] = state->position[3];
//   cur_joints[4] = state->position[4];
//   cur_joints[5] = state->position[5];
// }

// int ur5_kinematics::traj_run()
// {
//   // listen to joint states
//   // cur_joints_sub_ = n_.subscribe("joint_states", 100, &ur5_kinematics::cur_joints_callback, this);

//   target_pose.orientation.w = 0.600;
//   target_pose.orientation.x = -0.500;
//   target_pose.orientation.y = 0.530;
//   target_pose.orientation.z = 0.330;
//   target_pose.position.x = 0.082;
//   target_pose.position.y = 0.520;
//   target_pose.position.z = 0.465;

//   inverse_kinematics();

//   // goal.trajectory.points[1].positions[0] = target_joints[0];
//   // goal.trajectory.points[1].positions[1] = target_joints[1];
//   // goal.trajectory.points[1].positions[2] = target_joints[2];
//   // goal.trajectory.points[1].positions[3] = target_joints[3];
//   // goal.trajectory.points[1].positions[4] = target_joints[4];
//   // goal.trajectory.points[1].positions[5] = target_joints[5];

//     goal.trajectory.points[1].positions[0] = 1.4;
//   goal.trajectory.points[1].positions[1] = -1.8;
//   goal.trajectory.points[1].positions[2] = 1.6;
//   goal.trajectory.points[1].positions[3] = -1.6;
//   goal.trajectory.points[1].positions[4] = -1.7;
//   goal.trajectory.points[1].positions[5] = 4;

//   // get_joint_values();


//   sensor_msgs::JointState::ConstPtr msg = ros::topic::waitForMessage<sensor_msgs::JointState>("/joint_states");


//   goal.trajectory.points[0].positions[0] = msg->position[0];
//   goal.trajectory.points[0].positions[1] = msg->position[1];
//   goal.trajectory.points[0].positions[2] = msg->position[2];
//   goal.trajectory.points[0].positions[3] = msg->position[3];
//   goal.trajectory.points[0].positions[4] = msg->position[4];
//   goal.trajectory.points[0].positions[5] = msg->position[5];

//   move_joints();
//   // move_end_effector();

//   return 0;
// }


// void ur5_kinematics::update_traj()
// {
    // Positions
  // goal.trajectory.points[0].positions[0] = cur_joints[0];
  // goal.trajectory.points[0].positions[1] = cur_joints[1];
  // goal.trajectory.points[0].positions[2] = cur_joints[2];
  // goal.trajectory.points[0].positions[3] = cur_joints[3];
  // goal.trajectory.points[0].positions[4] = cur_joints[4];
  // goal.trajectory.points[0].positions[5] = cur_joints[5];
  // Positions
  // goal.trajectory.points[1].positions[0] = target_joints[0];
  // goal.trajectory.points[1].positions[1] = target_joints[1];
  // goal.trajectory.points[1].positions[2] = target_joints[2];
  // goal.trajectory.points[1].positions[3] = target_joints[3];
  // goal.trajectory.points[1].positions[4] = target_joints[4];
  // goal.trajectory.points[1].positions[5] = target_joints[5];
// }

// void ur5_kinematics::move_joints()
// {
//   // start the trajectory right now
//   goal.trajectory.header.stamp = ros::Time::now();
//   traj_client_->sendGoal(goal);
// }

// bool ur5_kinematics::inverse_kinematics()
// {
//   // The number of attempts to be made at solving IK: 10
//   // The timeout for each attempt: 0.1 s
//   bool found_ik = kinematic_state->setFromIK(joint_model_group, target_pose, 10, 0.1);
//   if (found_ik)
//   {
//     kinematic_state->copyJointGroupPositions(joint_model_group, target_joints);
//     for (std::size_t i = 0; i < joint_names.size(); ++i)
//     {
//       ROS_INFO("Joint %s: %f", joint_names[i].c_str(), target_joints[i]);
//     }
//   }
//   else
//   {
//     ROS_INFO("Did not find IK solution");
//   }
// }