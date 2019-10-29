#include <ros/ros.h>
#include <pluginlib/class_loader.h>

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <boost/scoped_ptr.hpp>
#include <eigen_conversions/eigen_msg.h>

#include <moveit_visual_tools/moveit_visual_tools.h>


class ur5_planning
{
public:

  int run();

  ur5_planning();
  virtual ~ur5_planning();

private:
  ros::NodeHandle n_;
  ros::AsyncSpinner spinner;
  ros::Rate r;

  ros::Subscriber tar_pose_sub_;

  robot_model_loader::RobotModelLoader robot_model_loader;
  robot_model::RobotModelPtr robot_model;
  planning_scene::PlanningScenePtr planning_scene;
  ros::Publisher planning_scene_diff_publisher;


  boost::scoped_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager>> planner_plugin_loader;
  planning_interface::PlannerManagerPtr planner_instance;
  std::string planner_plugin_name;

  const std::string robot_name;
  moveit::planning_interface::MoveGroupInterface move_group;

  // moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // rviz
  ros::Publisher display_publisher;
  moveit_msgs::DisplayTrajectory display_trajectory;

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

  display_publisher = n_.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);

  // camera mount
  moveit_msgs::AttachedCollisionObject attached_mount;
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
  moveit_msgs::AttachedCollisionObject attached_gripper;
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
  moveit_msgs::AttachedCollisionObject attached_table;
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

  moveit_msgs::PlanningScene planning_scene;
  planning_scene.world.collision_objects.push_back(attached_table.object);
  planning_scene.robot_state.attached_collision_objects.push_back(attached_mount);
  planning_scene.robot_state.attached_collision_objects.push_back(attached_gripper);
  planning_scene.is_diff = true;
  planning_scene_diff_publisher.publish(planning_scene);


  sleep_time.sleep();


  // moveit_msgs::CollisionObject mount;
  // mount.header.frame_id = move_group.getPlanningFrame();
  // mount.id = "mount";
  // shape_msgs::SolidPrimitive primitive_mount;
  // primitive_mount.type = primitive_mount.BOX;
  // primitive_mount.dimensions.resize(3);
  // primitive_mount.dimensions[0] = 0.02;
  // primitive_mount.dimensions[1] = 0.14;
  // primitive_mount.dimensions[2] = 0.15;
  // geometry_msgs::Pose pose_mount;
  // pose_mount.header
  // pose_mount.orientation.w = 0.9239;
  // pose_mount.orientation.x = 0.3827;
  // pose_mount.position.y = -0.0318;
  // pose_mount.position.z = 0.0318; 


  /**
  *
  **/
}

ur5_planning::~ur5_planning()
{
  planner_instance.reset();
  planner_plugin_loader.reset();
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

  // while(1);

  bool mapping_done = false;
  while(1)
  {
    msg = ros::topic::waitForMessage<geometry_msgs::PoseStamped>("/target_pose");

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    move_group.setPoseTarget(*msg);
    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

    move_group.execute(my_plan);

    ros::param::get("mapping_done", mapping_done);
    if(mapping_done) break;
  }

  // pick and place
  // while(1)
  // {
    // move above to obstacle, keep orientation down

    // cartesian path downwards

    // grasp

    // cartesian path upwards

    // wait for first pose
    msg = ros::topic::waitForMessage<geometry_msgs::PoseStamped>("/target_pose");

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    move_group.setPoseTarget(*msg);
    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

    move_group.execute(my_plan);


 
  // }

  return 0;
}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "ur5_planning");
  // ros::AsyncSpinner spinner(4);
  // spinner.start();

  ur5_planning planning_node;
  return planning_node.run();
}

// bool ur_planning::move_end_effector()
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

 // camera mount
  // moveit_msgs::AttachedCollisionObject attached_mount;
  // attached_mount.link_name = "ee_link";
  // attached_mount.object.header.frame_id = "ee_link";
  // attached_mount.object.id = "mount";
  // geometry_msgs::Pose pose_mount;
  // pose_mount.orientation.w = 1.0;   // default pose
  // pose_mount.position.x = 0.09;
  // shape_msgs::SolidPrimitive primitive_mount;
  // primitive_mount.type = primitive_mount.BOX;
  // primitive_mount.dimensions.resize(3);
  // primitive_mount.dimensions[0] = 0.18;  // TODO
  // primitive_mount.dimensions[1] = 0.12;
  // primitive_mount.dimensions[2] = 0.05;
  // attached_mount.object.primitives.push_back(primitive_mount);
  // attached_mount.object.primitive_poses.push_back(pose_mount);
  // attached_mount.object.operation = attached_mount.object.ADD;
  
  // // gripper
  // moveit_msgs::AttachedCollisionObject attached_gripper;
  // attached_gripper.link_name = "ee_link";
  // attached_gripper.object.header.frame_id = "ee_link";
  // attached_gripper.object.id = "gripper";
  // geometry_msgs::Pose pose_gripper;
  // pose_gripper.orientation.w = 1.0;
  // pose_gripper.position.x = 0.01;
  // pose_gripper.position.z = 0.035;
  // shape_msgs::SolidPrimitive primitive_gripper;
  // primitive_gripper.type = primitive_gripper.BOX;
  // primitive_gripper.dimensions.resize(3);
  // primitive_gripper.dimensions[0] = 0.02;  // TODO
  // primitive_gripper.dimensions[1] = 0.07;
  // primitive_gripper.dimensions[2] = 0.15;
  // attached_gripper.object.primitives.push_back(primitive_gripper);
  // attached_gripper.object.primitive_poses.push_back(pose_gripper);
  // attached_gripper.object.operation = attached_gripper.object.ADD;

  // // table
  // moveit_msgs::AttachedCollisionObject attached_table;
  // attached_table.link_name = "base_link";
  // attached_table.object.header.frame_id = "base_link";
  // attached_table.object.id = "table";
  // geometry_msgs::Pose pose_table;
  // pose_table.orientation.w = 1.0;   // default pose
  // pose_table.position.z = - 0.1;
  // shape_msgs::SolidPrimitive primitive_table;
  // primitive_table.type = primitive_table.BOX;
  // primitive_table.dimensions.resize(3);
  // primitive_table.dimensions[0] = 3;  // TODO
  // primitive_table.dimensions[1] = 3;
  // primitive_table.dimensions[2] = 0.2;
  // attached_table.object.primitives.push_back(primitive_table);
  // attached_table.object.primitive_poses.push_back(pose_table);
  // attached_table.object.operation = attached_table.object.ADD;


  // ROS_INFO("Adding the camera mount, gripper, and world into the world.");
  // moveit_msgs::PlanningScene planning_scene;
  // planning_scene.world.collision_objects.push_back(attached_mount.object);
  // planning_scene.world.collision_objects.push_back(attached_gripper.object);
  // planning_scene.world.collision_objects.push_back(attached_table.object);
  // planning_scene.robot_state.attached_collision_objects.push_back(attached_mount);
  // planning_scene.robot_state.attached_collision_objects.push_back(attached_gripper);
  // planning_scene.is_diff = true;
  // planning_scene_diff_publisher.publish(planning_scene);
  // sleep_time.sleep();


    // Visualizing plans
  // ^^^^^^^^^^^^^^^^^
  // We can also visualize the plan as a line with markers in Rviz.

  //   namespace rvt = rviz_visual_tools;
  // moveit_visual_tools::MoveItVisualTools visual_tools("odom_combined");
  // visual_tools.deleteAllMarkers();

  // // Remote control is an introspection tool that allows users to step through a high level script
  // // via buttons and keyboard shortcuts in Rviz
  // visual_tools.loadRemoteControl();
  // ROS_INFO_NAMED("tutorial", "Visualizing plan 1 as trajectory line");
  // visual_tools.publishAxisLabeled(*msg, "pose1");
  // visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
  // // visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  // visual_tools.trigger();
  // visual_tools.prompt("next step");


// int ur5_planning::run()
// {
// // Get starting robot state
//   // robot_state::RobotState start_state = *group.getCurrentState();

//   std::vector<double> tolerance_pose(3, 0.01);
//   std::vector<double> tolerance_angle(3, 0.01);

//   planning_interface::MotionPlanRequest req;
//   planning_interface::MotionPlanResponse res;
//   req.group_name = (robot_name);

//   ROS_INFO("Planning ready to receive!");

//   geometry_msgs::PoseStamped::ConstPtr msg = ros::topic::waitForMessage<geometry_msgs::PoseStamped>("/target_pose");

//   ROS_INFO("X: %f", msg->pose.position.x);

//   ROS_INFO("Receive msg");

//   moveit_msgs::Constraints pose_goal = kinematic_constraints::constructGoalConstraints("ee_link", *msg, tolerance_pose, tolerance_angle);
//   req.goal_constraints.push_back(pose_goal);

//   planning_interface::PlanningContextPtr context =
//       planner_instance->getPlanningContext(planning_scene, req, res.error_code_);
//   context->solve(res);
//   if (res.error_code_.val != res.error_code_.SUCCESS)
//   {
//     ROS_ERROR("Could not compute plan successfully");
//     return 1;
//   }

//   // Visualize the result

//   ROS_INFO("Visualizing the trajectory");
//   moveit_msgs::MotionPlanResponse response;
//   res.getMessage(response);

//   display_trajectory.trajectory_start = response.trajectory_start;
//   display_trajectory.trajectory.push_back(response.trajectory);
//   display_publisher.publish(display_trajectory);

//   // full collision checking
//   // collision_result.clear();
//   // planning_scene.checkCollision(collision_request, collision_result, copied_state, acm);
//   // ROS_INFO_STREAM("Test 6: Current state is " << (collision_result.collision ? "in" : "not in") << " self collision");

//   // robot_state::RobotState& robot_state = planning_scene->getCurrentStateNonConst();
//   // planning_scene->setCurrentState(response.trajectory_start);
//   // const robot_state::JointModelGroup* joint_model_group = robot_state.getJointModelGroup("manipulator");
//   // robot_state.setJointGroupPositions(joint_model_group, response.trajectory.joint_trajectory.points.back().positions);

//   // sleep_time.sleep();
//   // move_group.asyncMove();
//   // move_group.execute();
//   usleep(10 * 1000 * 1000);

//   return 0;
// }