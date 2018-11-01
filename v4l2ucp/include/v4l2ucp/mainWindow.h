#ifndef V4L2UCP_MAINWINDOW_H
#define V4L2UCP_MAINWINDOW_H
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
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/int32.hpp>
#include <string>
#include <v4l2ucp/v4l2controls.h>

#define ROS_INFO(msg, ...) printf(msg,  ##__VA_ARGS__)
#define ROS_ERROR(msg, ...) printf(msg,  ##__VA_ARGS__)
#define ROS_WARN(msg, ...) printf(msg,  ##__VA_ARGS__)
#define ROS_DEBUG(msg, ...) printf(msg,  ##__VA_ARGS__)
// #define ROS_DEBUG(msg, ...) // ##__VA_ARGS__
#define __FILENAME__ (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__)

#define INFO(msg) std::cout << "I [" << __FILENAME__ << ":" << __LINE__ << " " << __FUNCTION__ << "] " << msg << "\n"
#define WARN(msg) std::cout << "W [" << __FILENAME__ << ":" << __LINE__ << " " << __FUNCTION__ << "] " << msg << "\n"
#define ERROR(msg) std::cerr << "E [" << __FILENAME__ << ":" << __LINE__ << " " << __FUNCTION__ << "] " << msg << std::endl
#define DEBUG(msg) // std::cerr << "E [" << __FILENAME__ << ":" << __LINE__ << " " << __FUNCTION__ << "] " << msg << std::endl
#define ROS_ERROR_STREAM(msg) ERROR(msg)
#define ROS_ERROR_STREAM(msg) ERROR(msg)

class MainWindow : public rclcpp::Node
{
public:
  MainWindow();
  ~MainWindow();

  void about();

private:
  int fd = -1;

  // TODO(lucasw) instead of subscribers just have a get/set service
  std::map<std::string, rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr> sub_;
  std::map<std::string, rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr> pub_;

  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr configured_pub_;

  std::map<std::string, V4L2IntegerControl*> integer_controls_;
  std::map<std::string, V4L2BooleanControl*> bool_controls_;
  std::map<std::string, V4L2MenuControl*> menu_controls_;
  std::map<std::string, V4L2ButtonControl*> button_controls_;

  void add_control(const struct v4l2_queryctrl &ctrl, int fd);

  void integerControlCallback(const std_msgs::msg::Int32::SharedPtr msg, std::string name);
  void boolControlCallback(const std_msgs::msg::Int32::SharedPtr msg, std::string name);
  void menuControlCallback(const std_msgs::msg::Int32::SharedPtr msg, std::string name);
  void buttonControlCallback(const std_msgs::msg::Int32::SharedPtr msg, std::string name);
};

#endif  // V4L2UCP_MAINWINDOW_H
