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
#include <sys/ioctl.h>
#include <cerrno>
#include <cstring>
#include <libv4l2.h>
#include <std_msgs/msg/int32.hpp>

#include "v4l2ucp/v4l2controls.h"
#include "v4l2ucp/v4l2ucp.h"

using std::placeholders::_1;

int V4L2Control::exposure_auto = V4L2_EXPOSURE_MANUAL;
int V4L2Control::focus_auto = 0;
int V4L2Control::hue_auto = 0;
int V4L2Control::whitebalance_auto = 0;

V4L2Control::V4L2Control(const int fd, const struct v4l2_queryctrl &ctrl,
    const std::string name, std::shared_ptr<rclcpp::Node> node) :
  fd(fd),
  cid(ctrl.id),
  ctrl_(ctrl),
  default_value(ctrl.default_value)
  // name_(name)
{
  msg_.name = name;
  msg_.type = ctrl.type;
  msg_.value = default_value;
  msg_.min = ctrl.minimum;
  msg_.max = ctrl.maximum;
  INFO("'" << msg_.name << "' (from '" << msg_.name << "') " << std::dec << msg_.type << " "
      << msg_.min << " " << msg_.max);
  pub_ = node->create_publisher<std_msgs::msg::Int32>("feedback/" + name);
  // TODO(lucasw) this isn't working currently, no response at all
  sub_ = node->create_subscription<std_msgs::msg::Int32>("controls/" + name,
      std::bind(&V4L2Control::callback, this, _1));
}

void V4L2Control::callback(
    const std_msgs::msg::Int32::SharedPtr msg)
{
  // INFO(name_ << " callback " << msg->data);
  setValue(msg->data);
}

void V4L2Control::cacheValue(const struct v4l2_control &c)
{
  switch (c.id)
  {
  case V4L2_CID_EXPOSURE_AUTO:
    exposure_auto = c.value;
    break;
  case V4L2_CID_FOCUS_AUTO:
    focus_auto = c.value;
    break;
  case V4L2_CID_HUE_AUTO:
    hue_auto = c.value;
    break;
  case V4L2_CID_AUTO_WHITE_BALANCE:
    whitebalance_auto = c.value;
    break;
  }
}

// TODO(lucasw) need to review this
void V4L2Control::queryCleanup(struct v4l2_queryctrl *ctrl)
{
  switch (ctrl->id)
  {
  case V4L2_CID_EXPOSURE_ABSOLUTE:
    switch (exposure_auto)
    {
    case V4L2_EXPOSURE_AUTO:
    case V4L2_EXPOSURE_APERTURE_PRIORITY:
      ctrl->flags |= V4L2_CTRL_FLAG_GRABBED;
      break;
    }
    break;

  case V4L2_CID_IRIS_RELATIVE:
    ctrl->flags |= V4L2_CTRL_FLAG_WRITE_ONLY;
  /* Fall through */
  case V4L2_CID_IRIS_ABSOLUTE:
    switch (exposure_auto)
    {
    case V4L2_EXPOSURE_AUTO:
    case V4L2_EXPOSURE_SHUTTER_PRIORITY:
      ctrl->flags |= V4L2_CTRL_FLAG_GRABBED;
      break;
    }
    break;

  case V4L2_CID_FOCUS_RELATIVE:
    ctrl->flags |= V4L2_CTRL_FLAG_WRITE_ONLY;
  /* Fall through */
  case V4L2_CID_FOCUS_ABSOLUTE:
    if (focus_auto)
      ctrl->flags |= V4L2_CTRL_FLAG_GRABBED;
    break;

  case V4L2_CID_HUE:
    if (hue_auto)
      ctrl->flags |= V4L2_CTRL_FLAG_GRABBED;
    break;

  case V4L2_CID_WHITE_BALANCE_TEMPERATURE:
  case V4L2_CID_BLUE_BALANCE:
  case V4L2_CID_RED_BALANCE:
    if (whitebalance_auto)
      ctrl->flags |= V4L2_CTRL_FLAG_GRABBED;
    break;

  case V4L2_CID_EXPOSURE_AUTO:
  case V4L2_CID_FOCUS_AUTO:
  case V4L2_CID_HUE_AUTO:
  case V4L2_CID_AUTO_WHITE_BALANCE:
    ctrl->flags |= V4L2_CTRL_FLAG_UPDATE;
    break;

  case V4L2_CID_PAN_RELATIVE:
  case V4L2_CID_TILT_RELATIVE:
  case V4L2_CID_ZOOM_RELATIVE:
    ctrl->flags |= V4L2_CTRL_FLAG_WRITE_ONLY;
    break;
  }
}

void V4L2Control::setValue(int value)
{
  DEBUG(msg_.name << " new value " << value);
  struct v4l2_control c;
  c.id = cid;
  c.value = value;
  if (v4l2_ioctl(fd, VIDIOC_S_CTRL, &c) == -1)
  {
    ERROR(msg_.name << " Unable to set control " << strerror(errno));
    updateValue(false);
  }
  else
  {
    value_ = value;
    updateValue(true);
  }
}

void V4L2Control::updateValue(bool hwChanged)
{
  struct v4l2_queryctrl ctrl = { 0 };
  ctrl.id = cid;
  if (v4l2_ioctl(fd, VIDIOC_QUERYCTRL, &ctrl) == -1)
  {
    ERROR("Unable to get control status " << strerror(errno));
  }
  else
  {
    queryCleanup(&ctrl);
    // setEnabled(!(ctrl.flags & (V4L2_CTRL_FLAG_GRABBED | V4L2_CTRL_FLAG_READ_ONLY | V4L2_CTRL_FLAG_INACTIVE)));
  }

  // if (hwChanged && (ctrl.flags & V4L2_CTRL_FLAG_UPDATE))

#ifdef V4L2_CTRL_FLAG_WRITE_ONLY
  if (ctrl.flags & V4L2_CTRL_FLAG_WRITE_ONLY)
    return;
#endif

  if (ctrl.type == V4L2_CTRL_TYPE_BUTTON)
    return;

  struct v4l2_control c;
  c.id = cid;
  if (v4l2_ioctl(fd, VIDIOC_G_CTRL, &c) == -1)
  {
    ERROR(ctrl.name << " Unable to get control " << strerror(errno));
  }
  else
  {
    cacheValue(c);
    value_ = c.value;
    msg_.value = c.value;
    // TODO(lucasw) need to query hardware periodically to check on true values
    std_msgs::msg::Int32 int32_msg;
    int32_msg.data = msg_.value;
    pub_->publish(int32_msg);
    if (c.value != getValue())
    {
      // INFO(name_ << " setting value from cache " << getValue() << " to " << c.value);
    }
  }
}

void V4L2Control::resetToDefault()
{
  setValue(default_value);
}

/*
 * V4L2IntegerControl
 */
V4L2IntegerControl::V4L2IntegerControl
(int fd, const struct v4l2_queryctrl &ctrl,
    const std::string name, std::shared_ptr<rclcpp::Node> node) :
  V4L2Control(fd, ctrl, name, node),
  minimum(ctrl.minimum), maximum(ctrl.maximum), step(ctrl.step)
{
  updateValue();
}

#if 0
void V4L2IntegerControl::setValue(int val)
{
  INFO(name_ << " new value " << val);
  if (val < minimum)
    val = minimum;
  if (val > maximum)
    val = maximum;
  if (step > 1)
  {
    int mod = (val - minimum) % step;
    if (mod > step / 2)
    {
      val += step - mod;
    }
    else
    {
      val -= mod;
    }
  }
  V4L2Control::setValue(val);
}
#endif
/*
 * V4L2BooleanControl
 */
V4L2BooleanControl::V4L2BooleanControl
(int fd, const struct v4l2_queryctrl &ctrl,
    const std::string name, std::shared_ptr<rclcpp::Node> node) :
  V4L2Control(fd, ctrl, name, node)
{
  updateValue();
}

/*
 * V4L2MenuControl
 */
V4L2MenuControl::V4L2MenuControl
(int fd, const struct v4l2_queryctrl &ctrl,
    const std::string name, std::shared_ptr<rclcpp::Node> node) :
  V4L2Control(fd, ctrl, name, node)
{
  for (int i = ctrl.minimum; i <= ctrl.maximum; i++)
  {
    struct v4l2_querymenu qm;
    qm.id = ctrl.id;
    qm.index = i;
    if (v4l2_ioctl(fd, VIDIOC_QUERYMENU, &qm) == 0)
    {
      INFO(ctrl.name << " " << qm.name);
      // cb->insertItem(i, (const char *)qm.name);
      // TODO(lucasw) add menu item to ros params
    }
    else
    {
      ERROR(ctrl.name << " Unable to get menu item" << qm.index);
      // cb->insertItem(i, "Unknown");
    }
  }
  // cb->setCurrentIndex(default_value);
  updateValue();
}

/*
 * V4L2ButtonControl
 */
V4L2ButtonControl::V4L2ButtonControl
(int fd, const struct v4l2_queryctrl &ctrl,
    const std::string name, std::shared_ptr<rclcpp::Node> node) :
  V4L2Control(fd, ctrl, name, node)
{
  updateValue();
}

// TODO(lucasw) maybe button isn't supposed to reset to default?
// void V4L2ButtonControl::resetToDefault()
// {
// }
