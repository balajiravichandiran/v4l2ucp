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
#include <ros/ros.h>

#ifndef V4L2_CID_IRIS_ABSOLUTE
#define V4L2_CID_IRIS_ABSOLUTE      (V4L2_CID_CAMERA_CLASS_BASE+17)
#define V4L2_CID_IRIS_RELATIVE      (V4L2_CID_CAMERA_CLASS_BASE+18)
#endif

class V4l2Ucp;

class V4L2Control
{
public:
  void updateHardware();
  virtual void updateStatus(bool hwChanged = false);
  virtual void resetToDefault();
  virtual void setValue(int val)
  {
    value_ = val;
    updateHardware();
  }

  virtual int getValue() { return value_; }

  int min() const
  {
    return minimum;
  }

  int max() const
  {
    return maximum;
  }


protected:
  V4L2Control(int fd, const struct v4l2_queryctrl &ctrl);
  int fd;
  int cid;
  int minimum;
  int maximum;
  int default_value;
  char name[32];

private:
  int value_;

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
  V4L2IntegerControl(int fd, const struct v4l2_queryctrl &ctrl);

  void setValue(int val);

private:
  int step;
};

class V4L2BooleanControl : public V4L2Control
{
public:
  V4L2BooleanControl(int fd, const struct v4l2_queryctrl &ctrl);
};

class V4L2MenuControl : public V4L2Control
{
public:
  V4L2MenuControl(int fd, const struct v4l2_queryctrl &ctrl);
};

class V4L2ButtonControl : public V4L2Control
{
public:
  // void resetToDefault();

  V4L2ButtonControl(int fd, const struct v4l2_queryctrl &ctrl);
};

#endif  // V4L2UCP_V4L2CONTROLS_H
