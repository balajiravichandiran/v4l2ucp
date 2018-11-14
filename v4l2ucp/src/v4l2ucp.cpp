/*  v4l2ucp - A universal control panel for all V4L2 devices
    Copyright (C) 2005,2009 Scott J. Bertin (scottbertin@yahoo.com)
    Copyright (C) 2009-2010 Vasily Khoruzhick (anarsoul@gmail.com)

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

#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <functional>
#include <libv4l2.h>
#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/parameter_type.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/int32.hpp>
#include <string>
#include <sys/ioctl.h>

#include "v4l2ucp/v4l2ucp.h"
using std::placeholders::_1;
using std::placeholders::_2;

V4l2Ucp::V4l2Ucp() : Node("v4l2ucp")
{
}

bool V4l2Ucp::init()
{
  std::string device = "/dev/video0";
  get_parameter_or("device", device, device);
  RCLCPP_INFO(get_logger(), "opening device %s", device.c_str());
  fd = v4l2_open(device.c_str(), O_RDWR, 0);
  if (fd < 0)
  {
    ERROR("v4l2ucp: Unable to open file" << device << " "
        << strerror(errno));
    return false;
  }

  struct v4l2_capability cap;
  if (v4l2_ioctl(fd, VIDIOC_QUERYCAP, &cap) == -1)
  {
    ERROR("v4l2ucp: Not a V4L2 device" << device);
    return false;
  }

  INFO(cap.driver);
  INFO(cap.card);
  INFO(cap.bus_info);

  INFO((cap.version >> 16) << "." << ((cap.version >> 8) & 0xff) << "."
              << (cap.version & 0xff));

  INFO("0x" << std::hex << cap.capabilities);

  // configured_pub_ = create_publisher<std_msgs::msg::Empty>("configured");

  struct v4l2_queryctrl ctrl;
#ifdef V4L2_CTRL_FLAG_NEXT_CTRL
  INFO("Use the extended control API first");
  ctrl.id = V4L2_CTRL_FLAG_NEXT_CTRL;
  if (0 == v4l2_ioctl(fd, VIDIOC_QUERYCTRL, &ctrl))
  {
    do
    {
      add_control(ctrl, fd);
      ctrl.id |= V4L2_CTRL_FLAG_NEXT_CTRL;
    }
    while (0 == v4l2_ioctl(fd, VIDIOC_QUERYCTRL, &ctrl));
  }
  else
#endif
  {
    INFO("Fall back on the standard API");
    /* Check all the standard controls */
    for (int i = V4L2_CID_BASE; i < V4L2_CID_LASTP1; i++)
    {
      ctrl.id = i;
      if (v4l2_ioctl(fd, VIDIOC_QUERYCTRL, &ctrl) == 0)
      {
        add_control(ctrl, fd);
      }
    }

    /* Check any custom controls */
    for (int i = V4L2_CID_PRIVATE_BASE; ; i++)
    {
      ctrl.id = i;
      if (v4l2_ioctl(fd, VIDIOC_QUERYCTRL, &ctrl) == 0)
      {
        add_control(ctrl, fd);
      }
      else
      {
        break;
      }
    }
  }

  parameters_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this);
  param_sub_ = parameters_client_->on_parameter_event(
      std::bind(&V4l2Ucp::onParameterEvent, this, _1));

  set_control_ = create_service<v4l2ucp::srv::SetControl>("set_control",
      std::bind(&V4l2Ucp::setControl, this, _1, _2));
  get_control_ = create_service<v4l2ucp::srv::SetControl>("get_control",
      std::bind(&V4l2Ucp::getControl, this, _1, _2));
  //  list_controls_ = create_service<v4l2ucp::srv::SetControl>("list_controls",
  //    std::bind(&V4l2Ucp::listControl, this, _1, _2));
  // configured_pub_->publish(std_msgs::msg::Empty());
  return true;
}

V4l2Ucp::~V4l2Ucp()
{
  if (fd >= 0)
    v4l2_close(fd);
}

void V4l2Ucp::setControl(const std::shared_ptr<v4l2ucp::srv::SetControl::Request> req,
    std::shared_ptr<v4l2ucp::srv::SetControl::Response> res)
{
  const std::string name = req->control.name;
  DEBUG("get control " << name << " " << req->control.value);
  if (controls_.count(name) < 1)
  {
    // TODO(lucasw) maybe should rescan controls, or provide that function elsewhere
    res->message = "No " + name + " in controls";
    res->success = false;
    return;
  }
  controls_[name]->setValue(req->control.value);
  res->control = controls_[name]->msg_;
}

void V4l2Ucp::getControl(const std::shared_ptr<v4l2ucp::srv::SetControl::Request> req,
    std::shared_ptr<v4l2ucp::srv::SetControl::Response> res)
{
  const std::string name = req->control.name;
  DEBUG("get control " << name);
  if (controls_.count(name) < 1)
  {
    // TODO(lucasw) maybe should rescan controls, or provide that function elsewhere
    res->message = "No " + name + " in controls";
    res->success = false;
    return;
  }
  controls_[name]->updateValue();
  res->control = controls_[name]->msg_;
}

void V4l2Ucp::onParameterEvent(const rcl_interfaces::msg::ParameterEvent::SharedPtr event)
{
  // auto -> ParameterValue
  for (auto & changed_parameter : event->changed_parameters) {
    std::string name = changed_parameter.name.substr(std::string("controls/").size());
    if (controls_.count(name) < 1)
    {
      // TODO(lucasw) maybe should rescan controls, or provide that function elsewhere
      RCLCPP_WARN(get_logger(), "No %s in controls", name.c_str());
      continue;
    }
    if (changed_parameter.value.type != rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER) {
      RCLCPP_WARN(get_logger(), "Wrong type %s %d",
          name, changed_parameter.value.type);
      continue;
    }
    controls_[name]->setValue(changed_parameter.value.integer_value, false);
  }
}

#if 0
void V4l2Ucp::listControls(const std::shared_ptr<v4l2ucp::srv::SetControl::Request> req,
    std::shared_ptr<v4l2ucp::srv::SetControl::Response> res)
{

}
#endif

bool not_alnum(char s)
{
  return !std::isalnum(s);
}

std::string typeToString(const int type)
{
  switch (type)
  {
  case V4L2_CTRL_TYPE_INTEGER:
    return "integer";
  case V4L2_CTRL_TYPE_BOOLEAN:
    return "boolean";
  case V4L2_CTRL_TYPE_MENU:
    return "menu";
  case V4L2_CTRL_TYPE_BUTTON:
    return "button";
  case V4L2_CTRL_TYPE_INTEGER64:
    return "integer64";
  case V4L2_CTRL_TYPE_CTRL_CLASS:
    return "ctrl_class";
  default:
    return "unknown type not yet implemented " + std::to_string(type);
  }
  return "error";
}

void V4l2Ucp::add_control(const struct v4l2_queryctrl &ctrl, int fd)
{
  std::stringstream name_ss;
  name_ss << ctrl.name;
  std::string name = name_ss.str();
  // std::replace(name.begin(), name.end(), " ", "_");

  // http://stackoverflow.com/questions/6319872/how-to-strip-all-non-alphanumeric-characters-from-a-string-in-c
  // name.erase(std::remove_if(name.begin(), name.end(),
  // std::not1(std::ptr_fun(std::isalnum)), name.end()), name.end());
  name.erase(std::remove_if(name.begin(), name.end(),
      (int(*)(int))not_alnum), name.end());

  // TODO(lucasw) clear out all other params under controls first
  // a previous run would leave leftovers
  set_parameter_if_not_set("controls/" + name + "/name", name_ss.str());
  set_parameter_if_not_set("controls/" + name + "/topic", "controls/" + name);
  set_parameter_if_not_set("controls/" + name + "/min", ctrl.minimum);
  set_parameter_if_not_set("controls/" + name + "/max", ctrl.maximum);
  set_parameter_if_not_set("controls/" + name + "/default", ctrl.minimum);
  set_parameter_if_not_set("controls/" + name + "/type", typeToString(ctrl.type));

  if (ctrl.flags & V4L2_CTRL_FLAG_DISABLED)
  {
    INFO(name << " disabled");
    return;
  }
  switch (ctrl.type)
  {
  case V4L2_CTRL_TYPE_INTEGER:
    // TODO(lucasw) This doesn't work
    // controls_[name] = std::make_shared<V4L2IntegerControl>(
    //    V4L2IntegerControl(fd, ctrl, name, shared_from_this()));
    // Memory isn't alocated correctly
    controls_[name].reset( new
        V4L2IntegerControl(fd, ctrl, name, shared_from_this()));
    break;
  case V4L2_CTRL_TYPE_BOOLEAN:
    controls_[name].reset(new
        V4L2BooleanControl(fd, ctrl, name, shared_from_this()));
    break;
  case V4L2_CTRL_TYPE_MENU:
    controls_[name].reset(new
        V4L2MenuControl(fd, ctrl, name, shared_from_this()));
    break;
  case V4L2_CTRL_TYPE_BUTTON:
    controls_[name].reset(new
        V4L2ButtonControl(fd, ctrl, name, shared_from_this()));
    break;
  case V4L2_CTRL_TYPE_INTEGER64:
    WARN("integer64 type not yet implemented");
    break;
  case V4L2_CTRL_TYPE_CTRL_CLASS:
    WARN("ctrl type not yet implemented");
  default:
    WARN("unknown type not yet implemented " << ctrl.type);
    break;
  }

  #if 0
  if (!w)
  {
    if (ctrl.type == V4L2_CTRL_TYPE_CTRL_CLASS)
      layout->addWidget(new QLabel(parent));
    else
      layout->addWidget(new QLabel("Unknown control", parent));

    layout->addWidget(new QLabel(parent));
    layout->addWidget(new QLabel(parent));
    return;
  }
  #endif

  if (ctrl.flags & (V4L2_CTRL_FLAG_GRABBED | V4L2_CTRL_FLAG_READ_ONLY | V4L2_CTRL_FLAG_INACTIVE))
  {
    // w->setEnabled(false);
  }


  if (ctrl.type == V4L2_CTRL_TYPE_BUTTON)
  {
  }
  else
  {
  }

  // TODO(lucasw) have a ros timer that updates the values from hardware
}

#if 0
void V4l2Ucp::boolControlCallback(
    const std_msgs::msg::Int32::SharedPtr msg, std::string name)
{
  bool_controls_[name]->setValue(msg->data);
}

void V4l2Ucp::menuControlCallback(
    const std_msgs::msg::Int32::SharedPtr msg, std::string name)
{
  menu_controls_[name]->setValue(msg->data);
}

void V4l2Ucp::buttonControlCallback(
    const std_msgs::msg::Int32::SharedPtr msg, std::string name)
{
  // TODO(lucasw) this doesn't do anything
  button_controls_[name]->setValue(msg->data);
}
#endif

void V4l2Ucp::about()
{
  std::string about ="This application is a port of an original v4l2ucp to Qt4 library,\n"
                     "v4l2ucp is a universal control panel for all V4L2 devices. The\n"
                     "controls come directly from the driver. If they cause problems\n"
                     "with your hardware, please contact the maintainer of the driver.\n\n"
                     "Copyright (C) 2005 Scott J. Bertin\n"
                     "Copyright (C) 2009-2010 Vasily Khoruzhick\n\n"
                     "This program is free software; you can redistribute it and/or modify\n"
                     "it under the terms of the GNU General Public License as published by\n"
                     "the Free Software Foundation; either version 2 of the License, or\n"
                     "(at your option) any later version.\n\n"
                     "This program is distributed in the hope that it will be useful,\n"
                     "but WITHOUT ANY WARRANTY; without even the implied warranty of\n"
                     "MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the\n"
                     "GNU General Public License for more details.\n\n"
                     "You should have received a copy of the GNU General Public License\n"
                     "along with this program; if not, write to the Free Software\n"
                     "Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA\n";

  // INFO("v4l2ucp Version " << V4L2UCP_VERSION << about);
}
