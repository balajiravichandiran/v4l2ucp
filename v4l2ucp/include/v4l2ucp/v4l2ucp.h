#ifndef V4L2UCP_V4L2UCP_H
#define V4L2UCP_V4L2UCP_H
/*  v4l2ucp - A universal control panel for all V4L2 devices
    Copyright (C) 2005 Scott J. Bertin (scottbertin@yahoo.com)
    Copyright (C) 2009 Vasily Khoruzhick (anarsoul@gmail.com)

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program; if not, write to the Free Software
    Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

 */
#include <map>
#include <rcl_interfaces/msg/parameter_event.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/int32.hpp>
#include <string>
#include <v4l2ucp/msg/control.hpp>
#include <v4l2ucp/srv/set_control.hpp>
#include <v4l2ucp/v4l2controls.h>

#define ROS_INFO(msg, ...) printf(msg,  ##__VA_ARGS__)
#define ROS_ERROR(msg, ...) printf(msg,  ##__VA_ARGS__)
#define ROS_WARN(msg, ...) printf(msg,  ##__VA_ARGS__)
// #define ROS_DEBUG(msg, ...) printf(msg,  ##__VA_ARGS__)
#define ROS_DEBUG(msg, ...) // ##__VA_ARGS__
#define __FILENAME__ (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__)

#define PRINT(msg) std::hex << std::this_thread::get_id() << std::dec << " [" << __FILENAME__ << ":" << std::dec << __LINE__ << " " << __FUNCTION__ << "] " << msg << "\n"
#define INFO(msg) std::cout << "I " << PRINT(msg)
#define WARN(msg) std::cout << "W " << PRINT(msg)
#define ERROR(msg) std::cerr << "E " << PRINT(msg)
#define DEBUG(msg) std::cout << "D " << PRINT(msg)
#define ROS_ERROR_STREAM(msg) ERROR(msg)
#define ROS_ERROR_STREAM(msg) ERROR(msg)

// TODO(lucasw)
// namespace v4l2ucp
// {
class V4l2Ucp : public rclcpp::Node
{
public:
  V4l2Ucp();
  ~V4l2Ucp();

  bool init();
  void about();

private:
  int fd = -1;

  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr configured_pub_;

  // TODO(lucasw) could combine these
  rclcpp::Service<v4l2ucp::srv::SetControl>::SharedPtr set_control_;
  void setControl(const std::shared_ptr<v4l2ucp::srv::SetControl::Request> req,
      std::shared_ptr<v4l2ucp::srv::SetControl::Response> res);

  rclcpp::Service<v4l2ucp::srv::SetControl>::SharedPtr get_control_;
  void getControl(const std::shared_ptr<v4l2ucp::srv::SetControl::Request> req,
      std::shared_ptr<v4l2ucp::srv::SetControl::Response> res);

  void onParameterEvent(const rcl_interfaces::msg::ParameterEvent::SharedPtr event);

  void add_control(const struct v4l2_queryctrl &ctrl, int fd);

  rclcpp::AsyncParametersClient::SharedPtr parameters_client_;
  rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr param_sub_;

  std::map<std::string, std::shared_ptr<V4L2Control> > controls_;
};

#endif  // V4L2UCP_V4L2UCP_H
