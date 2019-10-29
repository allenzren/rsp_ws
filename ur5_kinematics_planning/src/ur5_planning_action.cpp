#include <ros/ros.h>
#include <pluginlib/class_loader.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <moveit_msgs/LinkPadding.h>

#include <boost/scoped_ptr.hpp>
#include <eigen_conversions/eigen_msg.h>

#include <central_launch/centralAction.h>
#include <actionlib/server/simple_action_server.h>
typedef actionlib::SimpleActionServer<central_launch::centralAction> Server;

#include <octomap/octomap.h>
#include <octomap_msgs/OctomapWithPose.h>

class ur5_planning
{
public:

  int run();

  ur5_planning();
  virtual ~ur5_planning();

private:
  // ros node basics
  ros::NodeHandle n_;
  ros::AsyncSpinner spinner;
  ros::Rate r;

  // receive move request
  ros::Subscriber tar_pose_sub_;
  Server *server;

  // moveit parameters
  robot_model_loader::RobotModelLoader robot_model_loader;
  robot_model::RobotModelPtr robot_model;
  planning_scene::PlanningScenePtr planning_scene;
  ros::Publisher planning_scene_diff_publisher;
  moveit_msgs::PlanningScene planning_scene_msg;

  boost::scoped_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager>> planner_plugin_loader;
  planning_interface::PlannerManagerPtr planner_instance;
  std::string planner_plugin_name;

  const std::string robot_name;
  moveit::planning_interface::MoveGroupInterface move_group;

  // moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  moveit_msgs::AttachedCollisionObject attached_table;
  moveit_msgs::AttachedCollisionObject attached_gripper;
  moveit_msgs::AttachedCollisionObject attached_mount;
  moveit_msgs::AttachedCollisionObject attached_cube;
  moveit_msgs::AttachedCollisionObject detached_cube;


  // octomap
  octomap_msgs::OctomapWithPose map;


  // helper functions
  void executeAction(const central_launch::centralGoalConstPtr &goal);
  void prm();

};

ur5_planning::ur5_planning():
spinner(1),
n_("~"),
r(20),
robot_model_loader("robot_description"),
robot_model(robot_model_loader.getModel()),
planning_scene(new planning_scene::PlanningScene(robot_model)),
robot_name("manipulator"),
move_group(robot_name)
{
  spinner.start();

  // initialize the server
  server = new Server(n_, "centralAction", boost::bind(&ur5_planning::executeAction, this, _1), false);
  server->start();

  // load moveit
  n_.getParam("planning_plugin", planner_plugin_name);  
  planner_plugin_loader.reset(new pluginlib::ClassLoader<planning_interface::PlannerManager>(
        "moveit_core", "planning_interface::PlannerManager"));
  planner_instance.reset(planner_plugin_loader->createUnmanagedInstance(planner_plugin_name));
  planner_instance->initialize(robot_model, n_.getNamespace());

  /* Sleep a little to allow time to startup rviz, etc. */
  ros::WallDuration sleep_time(5.0);
  sleep_time.sleep();

  /**
   * add mount and gripper to scene
   **/
  planning_scene_diff_publisher = n_.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
  while (planning_scene_diff_publisher.getNumSubscribers() < 1)
  {
    ros::WallDuration sleep_t(0.5);
    sleep_t.sleep();

  }

  // camera mount
  attached_mount.link_name = "ee_link";
  attached_mount.object.header.frame_id = "ee_link";
  attached_mount.object.id = "mount";
  geometry_msgs::Pose pose_mount;
  pose_mount.orientation.w = 0.9239;   // default pose
  pose_mount.orientation.x = 0.3827;
  pose_mount.position.x = 0.01;
  pose_mount.position.y = -0.0318;
  pose_mount.position.z = 0.0318;
  shape_msgs::SolidPrimitive primitive_mount;
  primitive_mount.type = primitive_mount.BOX;
  primitive_mount.dimensions.resize(3);
  primitive_mount.dimensions[0] = 0.02;
  primitive_mount.dimensions[1] = 0.14;
  primitive_mount.dimensions[2] = 0.15;
  attached_mount.object.primitives.push_back(primitive_mount);
  attached_mount.object.primitive_poses.push_back(pose_mount);
  attached_mount.object.operation = attached_mount.object.ADD;
  
  // gripper
  attached_gripper.link_name = "ee_link";
  attached_gripper.object.header.frame_id = "ee_link";
  attached_gripper.object.id = "gripper";
  geometry_msgs::Pose pose_gripper;
  pose_gripper.orientation.w = 0.9239;
  pose_gripper.orientation.x = 0.3827;
  pose_gripper.position.x = 0.09;
  // pose_gripper.position.z = 0.035;
  shape_msgs::SolidPrimitive primitive_gripper;
  primitive_gripper.type = primitive_gripper.BOX;
  primitive_gripper.dimensions.resize(3);
  primitive_gripper.dimensions[0] = 0.18;
  primitive_gripper.dimensions[1] = 0.12;
  primitive_gripper.dimensions[2] = 0.05;
  attached_gripper.object.primitives.push_back(primitive_gripper);
  attached_gripper.object.primitive_poses.push_back(pose_gripper);
  attached_gripper.object.operation = attached_gripper.object.ADD;

  // table
  attached_table.link_name = "base_link";
  attached_table.object.header.frame_id = "base_link";
  attached_table.object.id = "table";
  geometry_msgs::Pose pose_table;
  pose_table.orientation.w = 1.0;   // default pose
  pose_table.position.z = - 0.15;
  shape_msgs::SolidPrimitive primitive_table;
  primitive_table.type = primitive_table.BOX;
  primitive_table.dimensions.resize(3);
  primitive_table.dimensions[0] = 3;
  primitive_table.dimensions[1] = 3;
  primitive_table.dimensions[2] = 0.2;
  attached_table.object.primitives.push_back(primitive_table);
  attached_table.object.primitive_poses.push_back(pose_table);
  attached_table.object.operation = attached_table.object.ADD;

  // attach cube
  attached_cube.link_name = "ee_link";
  attached_cube.object.header.frame_id = "ee_link";
  attached_cube.object.id = "cube";
  geometry_msgs::Pose pose_cube;
  pose_cube.orientation.w = 0.9239;
  pose_cube.orientation.x = 0.3827;
  pose_cube.position.x = 0.205;
  shape_msgs::SolidPrimitive primitive_cube;
  primitive_cube.type = primitive_cube.BOX;
  primitive_cube.dimensions.resize(3);
  primitive_cube.dimensions[0] = 0.05;
  primitive_cube.dimensions[1] = 0.05;
  primitive_cube.dimensions[2] = 0.05;
  attached_cube.object.primitives.push_back(primitive_cube);
  attached_cube.object.primitive_poses.push_back(pose_cube);
  attached_cube.object.operation = attached_cube.object.ADD;

  // detach cube
  detached_cube.object.id = "cube";
  detached_cube.link_name = "ee_link";
  detached_cube.object.operation = attached_cube.object.REMOVE;

  // start without cube
  planning_scene_msg.world.collision_objects.push_back(attached_table.object);
  planning_scene_msg.robot_state.attached_collision_objects.push_back(attached_mount);
  planning_scene_msg.robot_state.attached_collision_objects.push_back(attached_gripper);
  planning_scene_msg.is_diff = true;
  planning_scene_diff_publisher.publish(planning_scene_msg);

  sleep_time.sleep();
}

ur5_planning::~ur5_planning()
{
  planner_instance.reset();
  planner_plugin_loader.reset();
}

void ur5_planning::executeAction(const central_launch::centralGoalConstPtr &goal)
{

  bool success = true;
  move_group.setMaxVelocityScalingFactor(0.5);
  move_group.setPlannerId("RRTConnectkConfigDefault");
  move_group.setNumPlanningAttempts(3);
  move_group.setPlanningTime(10.0);

  move_group.setGoalPositionTolerance(0.002);
  move_group.setGoalOrientationTolerance(0.01);
  move_group.setGoalJointTolerance(0.005);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  // check types
  if(goal->type == 1)  // plan without cube
  {
    // add objects
    planning_scene_msg.robot_state.attached_collision_objects.clear();
    planning_scene_msg.world.collision_objects.clear();
    planning_scene_msg.world.collision_objects.push_back(attached_table.object);
    planning_scene_msg.robot_state.attached_collision_objects.push_back(attached_mount);
    planning_scene_msg.robot_state.attached_collision_objects.push_back(attached_gripper);
    // planning_scnee_msg.robot_state.attached_collision_objects.erase(2);
    planning_scene_msg.robot_state.attached_collision_objects.push_back(detached_cube);
    // planning_scene_msg.is_diff = true;
    planning_scene_diff_publisher.publish(planning_scene_msg);

    // plan
    move_group.setPoseTarget(goal->pose);
    success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

    move_group.execute(my_plan);
  }

  else if(goal->type == 2)  // cartesian path
  {
    // initialize vector for waypoints down
    std::vector<geometry_msgs::Pose> waypoints;

    waypoints.push_back(goal->pose.pose);

    moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 1000; // not sure
    const double eef_step = 0.01;
    // false indicates no collision check
    double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory, false);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 4 (cartesian path) (%.2f%% achieved)", fraction * 100.0);

    my_plan.trajectory_ = trajectory;
    success = (move_group.execute(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS); 
  }

  if(goal->type == 3)  // plan wih cube
  {
    // add objects
    planning_scene_msg.robot_state.attached_collision_objects.clear();
    planning_scene_msg.world.collision_objects.clear();
    planning_scene_msg.world.collision_objects.push_back(attached_table.object);
    planning_scene_msg.robot_state.attached_collision_objects.push_back(attached_mount);
    planning_scene_msg.robot_state.attached_collision_objects.push_back(attached_gripper);
    // planning_scnee_msg.robot_state.attached_collision_objects.erase(2);
    planning_scene_msg.robot_state.attached_collision_objects.push_back(attached_cube);
    // planning_scene_msg.is_diff = true;
    planning_scene_diff_publisher.publish(planning_scene_msg);

    move_group.setPoseTarget(goal->pose);
    success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

    move_group.execute(my_plan);
  }

  if(success)
  {
    server->setSucceeded();
  }
  else
  {
    server->setPreempted();
  }
}

int ur5_planning::run()
{
  // initialize
  geometry_msgs::PoseStamped::ConstPtr msg;

  // change kinematics_ready parameter to true
  ros::param::set("planning_ready", true);
  ROS_INFO("Planning node is ready!");

  bool start = false;
  while(!(ros::param::get("start", start) && start));
  ROS_INFO("Planning node starts!");

  /* 
  * -----------------------------------------------------------------------------
  * mapping process
  */
  bool mapping_done = false;
  ros::Duration move_timeout(0.2);

  // move according to arm
  while(1)
  {
    ROS_INFO("Waiting for the pose!");
    msg = ros::topic::waitForMessage<geometry_msgs::PoseStamped>("/kinect/teleop/pose", move_timeout);

    if(msg != NULL)
    {
      move_group.setPlanningTime(3.0);
      moveit::planning_interface::MoveGroupInterface::Plan my_plan;
      move_group.setPoseTarget(*msg);
      bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
      ROS_INFO("Move with hand!");
      ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

      move_group.execute(my_plan);
    }

    ros::param::get("mapping_done", mapping_done);
    if(mapping_done) break;
  }

  // read octomap
  boost::shared_ptr<octomap_msgs::Octomap const> sharedPtrMap;
  ros::Duration map_timeout(3);
    
  sharedPtrMap = ros::topic::waitForMessage<octomap_msgs::Octomap>("/octomap_binary", map_timeout);

  if(sharedPtrMap == NULL)
  {
    ROS_INFO("No map received");
  } else{
    ROS_INFO("received map.");
    map.octomap = *sharedPtrMap;
  }
  // map.origin
  planning_scene_msg.world.octomap = map;
  planning_scene_diff_publisher.publish(planning_scene_msg);

  /* 
  * -----------------------------------------------------------------------------
  * Pick and Place
  */
  while(ros::ok())
  {
    // use action here
  }

  return 0;
}

void ur5_planning::prm()
{
  // listen to kinect_teleop_, once receive "set" msg, read and store currrent joint states
  // add action type 3, use prm to move
  // use graph, find weight of each vertex, djistra?
  // check collision between them, use kinematics helper functions in moveit
  // cartesian move?
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "ur5_planning_action");

  ur5_planning planning_node;
  return planning_node.run();
}

  // // planning_scene_msg.robot_model_name = robot_name;
  // moveit_msgs::LinkPadding l1;
  // moveit_msgs::LinkPadding l2;
  // moveit_msgs::LinkPadding l3;
  // moveit_msgs::LinkPadding l4;
  // moveit_msgs::LinkPadding l5;
  // moveit_msgs::LinkPadding l6;
  // moveit_msgs::LinkPadding l7;

  // l1.link_name = "base_link";
  // l2.link_name = "shoulder_link";
  // l3.link_name = "upper_arm_link";
  // l4.link_name = "forearm_link";
  // l5.link_name = "wrist_1_link";
  // l6.link_name = "wrist_2_link";
  // l7.link_name = "wrist_3_link";

  // l1.padding = 0.1;
  // l2.padding = 0.1;
  // l3.padding = 0.1;
  // l4.padding = 0.1;
  // l5.padding = 0.1;
  // l6.padding = 0.1;
  // l7.padding = 0.1;

  // // l1.padding = 0.5;
  // // l2.padding = 0.5;
  // // l3.padding = 0.5;
  // // l4.padding = 0.5;
  // // l5.padding = 0.5;
  // // l6.padding = 0.5;
  // // l7.padding = 0.5;

  // moveit_msgs::LinkScale l1s;
  // moveit_msgs::LinkScale l2s;
  // moveit_msgs::LinkScale l3s;
  // moveit_msgs::LinkScale l4s;
  // moveit_msgs::LinkScale l5s;
  // moveit_msgs::LinkScale l6s;
  // moveit_msgs::LinkScale l7s;

  // l1s.scale = 1.0;
  // l2s.scale = 1.0;
  // l3s.scale = 1.0;
  // l4s.scale = 1.0;
  // l5s.scale = 1.0;
  // l6s.scale = 1.0;
  // l7s.scale = 1.0;

  // planning_scene_msg.link_padding.push_back(l1);
  // planning_scene_msg.link_padding.push_back(l2);
  // planning_scene_msg.link_padding.push_back(l3);
  // planning_scene_msg.link_padding.push_back(l4);
  // planning_scene_msg.link_padding.push_back(l5);
  // planning_scene_msg.link_padding.push_back(l6);
  // planning_scene_msg.link_padding.push_back(l7);

  // planning_scene_msg.link_scale.push_back(l1s);
  // planning_scene_msg.link_scale.push_back(l2s);
  // planning_scene_msg.link_scale.push_back(l3s);
  // planning_scene_msg.link_scale.push_back(l4s);
  // planning_scene_msg.link_scale.push_back(l5s);
  // planning_scene_msg.link_scale.push_back(l6s);
  // planning_scene_msg.link_scale.push_back(l7s);