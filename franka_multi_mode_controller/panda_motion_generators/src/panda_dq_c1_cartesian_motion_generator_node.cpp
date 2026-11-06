#include <rclcpp/rclcpp.hpp>

#include <panda_motion_generators/motion_generators/panda_dq_c1_cartesian_motion_generator.h>

using namespace panda_motion_generators;

int main(int argc, char** argv) {
  //, "panda_motion_generator_node"
  rclcpp::init(argc, argv);
  const auto args = rclcpp::remove_ros_arguments(argc, argv);
  if (args.size() != 6) {
    std::cerr << "Usage: " << args[0]
              << " action_name robot_state_srv_name controller_name"
                 " controllet_name desired_pose_name [--ros-args ...]\n";
    rclcpp::shutdown();
    return 2;
  }
  const std::string& action_name = args[1];
  const std::string& robot_state_srv_name = args[2];
  const std::string& controller_name = args[3];
  const std::string& controllet_name = args[4];
  const std::string& desired_pose_name = args[5];
  printf("Starting cartesian via motion generator with the following parameters:\n%s\n%s\n%s\n%s\n%s",
          action_name.c_str(),
          robot_state_srv_name.c_str(),
          controller_name.c_str(),
          controllet_name.c_str(),
          desired_pose_name.c_str());

  auto mg = PandaDqC1CartesianMotionGenerator(action_name, 
                                              robot_state_srv_name, 
                                              controller_name, 
                                              controllet_name,
                                              desired_pose_name);
  mg.spinROSNode();
  rclcpp::shutdown();
  return 0;
}
