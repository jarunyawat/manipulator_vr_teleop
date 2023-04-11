#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <memory>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include "tf2_ros/static_transform_broadcaster.h"

using namespace std::chrono_literals;

void publishCommands();

rclcpp::Node::SharedPtr node;


int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;

    node_options.use_intra_process_comms(false);
    node_options.automatically_declare_parameters_from_overrides(true);
    node = std::make_shared<rclcpp::Node>("cmd_test", node_options);
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node);
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = node->get_clock()->now();
    t.header.frame_id = "tool0";
    t.child_frame_id = "occulus";

    t.transform.translation.x = 0.0;
    t.transform.translation.y = 0.0;
    t.transform.translation.z = 0.0;
    t.transform.rotation.x = 0.5;
    t.transform.rotation.y = 0.5;
    t.transform.rotation.z = 0.5;
    t.transform.rotation.w = -0.5;

    tf_static_broadcaster_->sendTransform(t);
    // Create a ROS logger
    auto const logger = rclcpp::get_logger("cmd_test");
    static const std::string PLANNING_GROUP = "ur_manipulator";
    // Create new movegroup interface
    using moveit::planning_interface::MoveGroupInterface;
    auto move_group_interface = MoveGroupInterface(node, PLANNING_GROUP);
    // Create collision table for the robot to avoid
    auto const collision_table = [frame_id =
                                    move_group_interface.getPlanningFrame()] {
      moveit_msgs::msg::CollisionObject collision_table;
      collision_table.header.frame_id = frame_id;
      collision_table.id = "table";
      shape_msgs::msg::SolidPrimitive primitive;

      // Define the size of the box in meters
      primitive.type = primitive.BOX;
      primitive.dimensions.resize(3);
      primitive.dimensions[primitive.BOX_X] = 0.3;
      primitive.dimensions[primitive.BOX_Y] = 0.3;
      primitive.dimensions[primitive.BOX_Z] = 0.5;

      // Define the pose of the box (relative to the frame_id)
      geometry_msgs::msg::Pose box_pose;
      box_pose.orientation.w = 1.0;
      box_pose.position.x = 0.0;
      box_pose.position.y = 0.0;
      box_pose.position.z = -0.25;

      collision_table.primitives.push_back(primitive);
      collision_table.primitive_poses.push_back(box_pose);
      collision_table.operation = collision_table.ADD;

      return collision_table;
    }();
    // Add the collision object to the scene
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    planning_scene_interface.applyCollisionObject(collision_table);

    // Create collision wall for the robot to avoid for operator safty
    auto const collision_wall = [frame_id =
                                    move_group_interface.getPlanningFrame()] {
      moveit_msgs::msg::CollisionObject collision_wall;
      collision_wall.header.frame_id = frame_id;
      collision_wall.id = "wall";
      shape_msgs::msg::SolidPrimitive primitive;

      // Define the size of the box in meters
      primitive.type = primitive.BOX;
      primitive.dimensions.resize(3);
      primitive.dimensions[primitive.BOX_X] = 0.1;
      primitive.dimensions[primitive.BOX_Y] = 2.0;
      primitive.dimensions[primitive.BOX_Z] = 0.7;

      // Define the pose of the box (relative to the frame_id)
      geometry_msgs::msg::Pose box_pose;
      box_pose.orientation.w = 1.0;
      box_pose.position.x = -0.4;
      box_pose.position.y = 0.0;
      box_pose.position.z = 0.35;

      collision_wall.primitives.push_back(primitive);
      collision_wall.primitive_poses.push_back(box_pose);
      collision_wall.operation = collision_wall.ADD;

      return collision_wall;
    }();

    // Add the collision object to the scene
    planning_scene_interface.applyCollisionObject(collision_wall);

    // Create joint command
    robot_model_loader::RobotModelLoader robot_model_loader(node);
    const moveit::core::RobotModelPtr& kinematic_model = robot_model_loader.getModel();
    moveit::core::RobotStatePtr kinematic_state(new moveit::core::RobotState(kinematic_model));
    kinematic_state->setToDefaultValues();
    const moveit::core::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup(PLANNING_GROUP);
    std::vector<double> joint_group_positions;
    kinematic_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
    joint_group_positions[0] = 0.0;
    joint_group_positions[1] = -1.39626;
    joint_group_positions[2] = 1.74533;
    joint_group_positions[3] = -0.401426;
    joint_group_positions[4] = 1.5708;
    joint_group_positions[5] = 0.0;
    move_group_interface.setJointValueTarget(joint_group_positions);
    move_group_interface.setMaxVelocityScalingFactor(0.05);
    move_group_interface.setMaxAccelerationScalingFactor(0.05);
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group_interface.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    RCLCPP_INFO(logger, "Visualizing plan 2 (joint space goal) %s", success ? "" : "FAILED");
    move_group_interface.move();
    RCLCPP_INFO(logger, "Execute finish");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}