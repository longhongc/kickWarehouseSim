#include <cstdlib>
#include <string>
#include <chrono>
#include <functional>
#include <memory>
#include <rcl_interfaces/msg/detail/parameter_descriptor__struct.hpp>
#include <stdlib.h>


#include "gazebo_msgs/srv/spawn_entity.hpp"
#include "gazebo_msgs/srv/delete_entity.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class Box {
  public:
  int i;
  int j;
  std::vector<int> free;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

//  ToDo: Read this matrix from YAML
  int world[6][7] = {{1,0,0,1,0,1,1},
                      {1,0,0,1,1,1,1},
                       {1,1,0,1,1,1,1},
                        {1,1,0,0,0,0,0},
                         {1,1,0,0,0,0,0},
                          {1,1,0,1,1,1,1}};

  Box bo[42];
  int box = 0;

  
  // Declare service client node
  std::shared_ptr<rclcpp::Node> node = \
  rclcpp::Node::make_shared("Client_for_spawnning");
  // Bind client to callback
  rclcpp::Client<gazebo_msgs::srv::SpawnEntity>::SharedPtr client =
    node->create_client<gazebo_msgs::srv::SpawnEntity>\
    ("/spawn_entity");
  // Get the request of count
  auto request = std::make_shared<gazebo_msgs::srv::SpawnEntity::Request>();
//   auto result = std::make_shared<gazebo_msgs::srv::SpawnEntity::Response>();
  auto model = "<?xml version='1.0'?> <sdf version='1.6'> <model name='box_pallets_3'> <static>true</static> <link name='link'> <pose>0 0 0 0 0 0</pose> <collision name='collision'> <pose>0 0 1 0 0 0</pose> <geometry> <box> <size>2.2 2.8 2</size> </box> </geometry> </collision> <visual name='visual'> <pose>1.35 -1 0 0 0 -0.1</pose> <geometry> <mesh> <scale>0.02 0.02 0.02</scale> <uri>model://box_pallets_3/meshes/box_pallet.dae</uri> </mesh> </geometry> </visual> </link> </model> </sdf>";
  for(int i =0; i<6 ; i++){
    for(int j =0; j<7 ;j++){
        if(world[i][j] == 1){
            request->name = "box_" + std::to_string(box);
            request->xml = model;
            request->initial_pose.position.x = 2.2*j;
            request->initial_pose.position.y = 0.5 - 2.8*i;
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
            bo[box].i = j;
            bo[box].j = i;
            bo[box].free.clear();
            bo[box].free.emplace_back(4);

            if(i == 0){
              bo[box].free.emplace_back(0);
              if(world[i+1][j] == 0)
                  bo[box].free.emplace_back(2);

              if(j == 0){
                bo[box].free.emplace_back(3);
                if(world[i][j+1] == 0)
                  bo[box].free.emplace_back(1);
              }
              else if (j == 6) {
               bo[box].free.emplace_back(1); 
                if(world[i][j-1] == 0)
                  bo[box].free.emplace_back(3);

              }
            }

            if( i == 5){
              bo[box].free.emplace_back(2);
              if(world[i-1][j] == 0)
                  bo[box].free.emplace_back(0);
              if(j == 0){
                bo[box].free.emplace_back(3);
                if(world[i][j+1] == 0)
                  bo[box].free.emplace_back(1);
              }
              else if (j == 6) {
               bo[box].free.emplace_back(1); 
                if(world[i][j-1] == 0)
                  bo[box].free.emplace_back(3);

              }
              

            }

            if(j == 0 && i != 0 && i != 5){
              bo[box].free.emplace_back(3); 
              if(world[i][j+1] == 0)
                  bo[box].free.emplace_back(1);
              if(world[i-1][j] == 0)
                  bo[box].free.emplace_back(0);
              if(world[i+1][j] == 0)
                  bo[box].free.emplace_back(2);
            }

            if(j == 6 && i != 0 && i != 5){
              bo[box].free.emplace_back(1); 
              if(world[i][j-1] == 0)
                  bo[box].free.emplace_back(3);
              if(world[i-1][j] == 0)
                  bo[box].free.emplace_back(0);
              if(world[i+1][j] == 0)
                  bo[box].free.emplace_back(2);
            }

            if(i != 0 && j != 0 && i != 5 && j != 6){
              
              if(world[i][j+1] == 0)
                  bo[box].free.emplace_back(1);
              if(world[i][j-1] == 0)
                  bo[box].free.emplace_back(3);
              if(world[i-1][j] == 0)
                  bo[box].free.emplace_back(0);
              if(world[i+1][j] == 0)
                  bo[box].free.emplace_back(2);

            }

            box++;
            }
            
        }
    }
  



  rclcpp::Client<gazebo_msgs::srv::DeleteEntity>::SharedPtr delete_client =
  node->create_client<gazebo_msgs::srv::DeleteEntity>\
  ("/delete_entity");
  int modify[5] ;
  for (int i= 0; i<5;i++){
    modify[i] = rand() % box;
    std::cout<<modify[i]<<"\n";
  }
  
  
  auto d_request = std::make_shared<gazebo_msgs::srv::DeleteEntity::Request>();

  for(int i = 0; i<5 ; i++){
  d_request->name = "box_" + std::to_string(modify[i]);
  while (!delete_client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), \
      "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), \
    "service not available, waiting again...");
  }
  // Send the request
  auto result_d = delete_client->async_send_request(d_request);

  int dir = rand() % bo[modify[i]].free.size();
  std::cout<<"\n" <<modify[i] << " " <<dir << bo[modify[i]].i << bo[modify[i]].j;

  if(bo[modify[i]].free[dir] != 4)
  {
    request->name = "box_" + std::to_string(box);
    request->xml = model;

    if(bo[modify[i]].free[dir]  == 0){
    request->initial_pose.position.x = 2.2*bo[modify[i]].i;
    request->initial_pose.position.y = 0.5 - 2.8*bo[modify[i]].j + 0.3;
    }
    if(bo[modify[i]].free[dir]  == 1){
    request->initial_pose.position.x = 2.2*bo[modify[i]].i + 0.3;
    request->initial_pose.position.y = 0.5 - 2.8*bo[modify[i]].j ;
    }
    if(bo[modify[i]].free[dir]  == 2){
    request->initial_pose.position.x = 2.2*bo[modify[i]].i;
    request->initial_pose.position.y = 0.5 - 2.8*bo[modify[i]].j - 0.3;
    }
    if(bo[modify[i]].free[dir]  == 3){
    request->initial_pose.position.x = 2.2*bo[modify[i]].i - 0.3;
    request->initial_pose.position.y = 0.5 - 2.8*bo[modify[i]].j ;
    }



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
    box++;
    std::cout<<"\n Box "<< box;

    
  }
  



  

  }


}