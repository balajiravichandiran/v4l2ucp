#ifndef V4L2UCP_V4L2CONTROLS_H
#define V4L2UCP_V4L2CONTROLS_H
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
#include <sys/time.h>
#include <linux/types.h>          /* for videodev2.h */
#include <linux/videodev2.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <v4l2ucp/msg/control.hpp>

#ifndef V4L2_CID_IRIS_ABSOLUTE
#define V4L2_CID_IRIS_ABSOLUTE      (V4L2_CID_CAMERA_CLASS_BASE+17)
#define V4L2_CID_IRIS_RELATIVE      (V4L2_CID_CAMERA_CLASS_BASE+18)
#endif

class MainWindow;

class V4L2Control
{
public:
  V4L2Control(int fd, const struct v4l2_queryctrl &ctrl,
      const std::string name,
      std::shared_ptr<rclcpp::Node> node);
  // this reads the value from hardware
  virtual void updateValue(bool hwChanged = false);
  virtual void resetToDefault();
  // this sets it in hardward
  virtual void setValue(int value);

  v4l2ucp::msg::Control msg_;

  virtual int getValue() { return value_; }

protected:
  int fd;
  int cid;
  v4l2_queryctrl ctrl_;
  int default_value;
  const std::string name_;
  int value_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr sub_;
  // virtual
  void callback(const std_msgs::msg::Int32::SharedPtr msg);

private:

  /* Not pretty we use these to keep track of the value of some special
     ctrls which impact the writability of other ctrls for queryCleanup(). */
  static int exposure_auto;
  static int focus_auto;
  static int hue_auto;
  static int whitebalance_auto;
  void cacheValue(const struct v4l2_control &c);
  /* This function sets various flags for well known (UVC) controls, these
     flags should really be set by the driver, but for older driver versions
     this does not happen. */
  void queryCleanup(struct v4l2_queryctrl *ctrl);
};

class V4L2IntegerControl : public V4L2Control
{
public:
  V4L2IntegerControl(int fd, const struct v4l2_queryctrl &ctrl,
      const std::string name, std::shared_ptr<rclcpp::Node> node);

  // virtual void setValue(int value);

private:
  int minimum = 0;
  int maximum = 1;
  int step = 0;
};

class V4L2BooleanControl : public V4L2Control
{
public:
  V4L2BooleanControl(int fd, const struct v4l2_queryctrl &ctrl,
      const std::string name, std::shared_ptr<rclcpp::Node> node);
};

class V4L2MenuControl : public V4L2Control
{
public:
  V4L2MenuControl(int fd, const struct v4l2_queryctrl &ctrl,
      const std::string name, std::shared_ptr<rclcpp::Node> node);
};

class V4L2ButtonControl : public V4L2Control
{
public:
  // void resetToDefault();

  V4L2ButtonControl(int fd, const struct v4l2_queryctrl &ctrl,
      const std::string name, std::shared_ptr<rclcpp::Node> node);
};

#endif  // V4L2UCP_V4L2CONTROLS_H
