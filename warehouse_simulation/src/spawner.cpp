#include <string>
#include <chrono>
#include <functional>
#include <memory>
#include <rcl_interfaces/msg/detail/parameter_descriptor__struct.hpp>


#include "gazebo_msgs/srv/spawn_entity.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  // Declare service client node
  std::shared_ptr<rclcpp::Node> node = \
  rclcpp::Node::make_shared("Client_for_spawnning");
  // Bind client to callback
  rclcpp::Client<gazebo_msgs::srv::SpawnEntity>::SharedPtr client =
    node->create_client<gazebo_msgs::srv::SpawnEntity>\
    ("/spawn_entity");
  // Get the request of count
  auto request = std::make_shared<gazebo_msgs::srv::SpawnEntity::Request>();


  request->xml = "<?xml version='1.0'?> <sdf version='1.6'> <model name='box_pallets_3'> <static>true</static> <link name='link'> <pose>0 0 0 0 0 0</pose> <collision name='collision'> <pose>0 0 1 0 0 0</pose> <geometry> <box> <size>2.2 2.8 2</size> </box> </geometry> </collision> <visual name='visual'> <pose>1.35 -1 0 0 0 -0.1</pose> <geometry> <mesh> <scale>0.02 0.02 0.02</scale> <uri>model://box_pallets_3/meshes/box_pallet.dae</uri> </mesh> </geometry> </visual> </link> </model> </sdf>";

  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), \
      "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), \
    "service not available, waiting again...");
  }
  // Send the request
  auto result = client->async_send_request(request);
}