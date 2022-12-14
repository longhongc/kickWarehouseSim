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

class Box {
  public:
  int id;
  int i;
  int j;
  std::vector<int> free;
};

class WareHouse : public rclcpp::Node
{
  public:
    WareHouse()
    : Node("warehouse")
    {
      spawn_client_ = this->create_client<gazebo_msgs::srv::SpawnEntity>\
      ("/spawn_entity");

      int tmp[6][7] = { {1,0,0,1,0,1,1},
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
                request->initial_pose.position.x = 2.2*j - 7;
                request->initial_pose.position.y = 0.2 - 2.8*i + 7;
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

                calc(i,j);
                box++;
                }
                
            }
        }
    delete_client = this->create_client<gazebo_msgs::srv::DeleteEntity>\
    ("/delete_entity");
    max = box;

     std::string get_modify_service_name =
        "/" + std::string(this->get_name()) + "/" + "Count";
    get_modify_service_ = this->create_service<Modify>(
        get_modify_service_name,
        std::bind(&WareHouse::modify_callback, this, _1, _2));

    }

  private:
  void calc(int i, int j){
    bo[box].id = box;
    bo[box].i = j;
    bo[box].j = i;
    bo[box].free.clear();
    // bo[box].free.emplace_back(4);

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
  }

void modify_callback(const std::shared_ptr<Modify::Request> request,
                          std::shared_ptr<Modify::Response> response) {

int modify[42] ;
  for (int i= 0; i<request->a;i++){
    modify[i] = rand() % max;
    std::cout<<modify[i]<<"\n";
  }
  
  
  auto d_request = std::make_shared<gazebo_msgs::srv::DeleteEntity::Request>();
  auto s_request = std::make_shared<gazebo_msgs::srv::SpawnEntity::Request>();
   auto model = "<?xml version='1.0'?> <sdf version='1.6'> <model name='box_pallets_3'> <static>true</static> <link name='link'> <pose>0 0 0 0 0 0</pose> <collision name='collision'> <pose>0 0 1 0 0 0</pose> <geometry> <box> <size>2.2 2.8 2</size> </box> </geometry> </collision> <visual name='visual'> <pose>1.35 -1 0 0 0 -0.1</pose> <geometry> <mesh> <scale>0.02 0.02 0.02</scale> <uri>model://box_pallets_3/meshes/box_pallet.dae</uri> </mesh> </geometry> </visual> </link> </model> </sdf>";

  for(int i = 0; i<request->a ; i++){
  d_request->name = "box_" + std::to_string(bo[modify[i]].id);
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

  int dir = rand() % bo[modify[i]].free.size();
  std::cout<<"\n" <<modify[i] << " " <<dir << bo[modify[i]].i << bo[modify[i]].j;

  if(bo[modify[i]].free[dir] != 4)
  {

    bo[modify[i]].id = box;
    s_request->name = "box_" + std::to_string(box);
    s_request->xml = model;
    
    if(bo[modify[i]].free[dir]  == 0){
    s_request->initial_pose.position.x = 2.2*bo[modify[i]].i - 7;
    s_request->initial_pose.position.y = 0.2 - 2.8*bo[modify[i]].j + 0.5 + 7;
    }
    if(bo[modify[i]].free[dir]  == 1){
    s_request->initial_pose.position.x = 2.2*bo[modify[i]].i + 0.5 - 7;
    s_request->initial_pose.position.y = 0.2 - 2.8*bo[modify[i]].j + 7;
    }
    if(bo[modify[i]].free[dir]  == 2){
    s_request->initial_pose.position.x = 2.2*bo[modify[i]].i - 7;
    s_request->initial_pose.position.y = 0.2 - 2.8*bo[modify[i]].j - 0.5 + 7;
    }
    if(bo[modify[i]].free[dir]  == 3){
    s_request->initial_pose.position.x = 2.2*bo[modify[i]].i - 0.5 - 7;
    s_request->initial_pose.position.y = 0.2 - 2.8*bo[modify[i]].j + 7;
    }



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

    auto result = spawn_client_->async_send_request(s_request);
    
    std::cout<<"\n Box "<< box;
    box++;
    
  }
  
  }
    response->b = request->a;
}
    int world[6][7];
    rclcpp::Client<gazebo_msgs::srv::SpawnEntity>::SharedPtr spawn_client_;
    rclcpp::Client<gazebo_msgs::srv::DeleteEntity>::SharedPtr delete_client;
    rclcpp::Service<Modify>::SharedPtr get_modify_service_;
    int box;
    Box bo[42];
    int max;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WareHouse>());
  rclcpp::shutdown();
  return 0;
}