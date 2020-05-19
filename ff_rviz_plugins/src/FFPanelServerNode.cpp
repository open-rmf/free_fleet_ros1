#include <mutex>
#include <thread>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

#include <free_fleet/Server.hpp>
#include <free_fleet/ServerConfig.hpp>
#include <free_fleet/messages/Location.hpp>
#include <free_fleet/messages/DestinationRequest.hpp>
#include <free_fleet/messages/RobotState.hpp>


int main(int argc, char** argv)
{
  ros::init(argc, argv, "testing_node");
  ros::NodeHandle n;
  ros::Rate loop_rate(10);

  free_fleet::ServerConfig default_server_conf;
  auto server_ptr = free_fleet::Server::make(default_server_conf);
  if (!server_ptr)
  {
    ROS_ERROR("server_ptr is a nullptr");
    return 1;
  }

  while (ros::ok())
  {
    // ROS_INFO("testing testing rate");
    std::vector<free_fleet::messages::RobotState> new_robot_states;
    if (server_ptr->read_robot_states(new_robot_states))
    {
      ROS_INFO("Got %u number of new states.", new_robot_states.size());
      for (size_t i = 0; i < new_robot_states.size(); ++i)
      {
        ROS_INFO("name: %s", new_robot_states[i].name.c_str());
      }
    }

    loop_rate.sleep();
  }

  return 0;
}
