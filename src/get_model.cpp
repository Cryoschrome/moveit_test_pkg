#include <rclcpp/rclcpp.hpp>

// MoveIt
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h>

#include <moveit/kinematic_constraints/utils.h>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("planning_scene_tutorial");

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  auto planning_scene_tutorial_node = rclcpp::Node::make_shared("planning_scene_tutorial", node_options);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(planning_scene_tutorial_node);
  std::thread([&executor]() { executor.spin(); }).detach();

  robot_model_loader::RobotModelLoader robot_model_loader(planning_scene_tutorial_node, "robot_description");
  const moveit::core::RobotModelPtr& kinematic_model = robot_model_loader.getModel();
  planning_scene::PlanningScene planning_scene(kinematic_model);

  collision_detection::CollisionRequest collision_request;
  collision_detection::CollisionResult collision_result;
  planning_scene.checkSelfCollision(collision_request, collision_result);
  RCLCPP_INFO_STREAM(LOGGER, "Test 1: Current state is " << (collision_result.collision ? "in" : "not in")
                                                         << " self collision");
}