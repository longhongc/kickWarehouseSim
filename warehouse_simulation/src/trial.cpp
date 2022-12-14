#include <string>
#include <chrono>
#include <functional>
#include <memory>
#include <rcl_interfaces/msg/detail/parameter_descriptor__struct.hpp>

#include "warehouse_simulation/srv/modify.hpp"
#include "gazebo_msgs/srv/spawn_entity.hpp"
#include "gazebo_msgs/srv/delete_entity.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;
using Modify = warehouse_simulation::srv::Modify;
using namespace std::placeholders;

class WareHouse : public rclcpp::Node
{
  public:
    WareHouse()
    : Node("warehouse")
    {
      spawn_client_ = this->create_client<gazebo_msgs::srv::SpawnEntity>\
      ("/spawn_entity");

      int tmp[6][7] = {{1,0,0,1,0,1,1},
                        {1,0,0,1,1,1,1},
                        {1,1,0,1,1,1,1},
                        {1,1,0,0,0,0,0},
                        {1,1,0,0,0,0,0},
                        {1,1,0,1,1,1,1}};
      for (int x=0 ; x < 6; x++)
      {
        for (int y=0 ; y < 7; y++)
        {
            world[x][y] = tmp[x][y];
        }
      }
      box =0;

    auto request = std::make_shared<gazebo_msgs::srv::SpawnEntity::Request>();
    auto model = "<?xml version='1.0'?> <sdf version='1.6'> <model name='box_pallets_3'> <static>true</static> <link name='link'> <pose>0 0 0 0 0 0</pose> <collision name='collision'> <pose>0 0 1 0 0 0</pose> <geometry> <box> <size>2.2 2.8 2</size> </box> </geometry> </collision> <visual name='visual'> <pose>1.35 -1 0 0 0 -0.1</pose> <geometry> <mesh> <scale>0.02 0.02 0.02</scale> <uri>model://box_pallets_3/meshes/box_pallet.dae</uri> </mesh> </geometry> </visual> </link> </model> </sdf>";
    for(int i =0; i<6 ; i++){
        for(int j =0; j<7 ;j++){
            if(world[i][j] == 1){
                request->name = "box_" + std::to_string(box);
                request->xml = model;
                request->initial_pose.position.x = 2.2*j;
                request->initial_pose.position.y = 0.5 - 2.8*i;
                while (!spawn_client_->wait_for_service(1s)) {
                if (!rclcpp::ok()) {
                    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), \
                    "Interrupted while waiting for the service. Exiting.");
                    break;
                }
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), \
                "service not available, waiting again...");
                }
                // Send the request

                auto result = spawn_client_->async_send_request(request);
                box++;
                }
                
            }
        }
    delete_client = this->create_client<gazebo_msgs::srv::DeleteEntity>\
    ("/delete_entity");

     std::string get_modify_service_name =
        "/" + std::string(this->get_name()) + "/" + "Count";
    get_modify_service_ = this->create_service<Modify>(
        get_modify_service_name,
        std::bind(&WareHouse::modify_callback, this, _1, _2));

    }

  private:
    void modify_callback(const std::shared_ptr<Modify::Request> request,
                          std::shared_ptr<Modify::Response> response) {
        auto d_request = std::make_shared<gazebo_msgs::srv::DeleteEntity::Request>();
        d_request->name = "box_" + std::to_string(16);
        while (!delete_client->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), \
            "Interrupted while waiting for the service. Exiting.");
            break;
            }
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), \
            "service not available, waiting again...");
        }
        // Send the request
        auto result_d = delete_client->async_send_request(d_request);
        response->b = request->a;
    }
    int world[6][7];
    rclcpp::Client<gazebo_msgs::srv::SpawnEntity>::SharedPtr spawn_client_;
    rclcpp::Client<gazebo_msgs::srv::DeleteEntity>::SharedPtr delete_client;
    rclcpp::Service<Modify>::SharedPtr get_modify_service_;
    int box;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WareHouse>());
  rclcpp::shutdown();
  return 0;
}