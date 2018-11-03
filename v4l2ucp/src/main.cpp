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
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <rclcpp/rclcpp.hpp>

#include "v4l2ucp/v4l2ucp.h"

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  // Force flush of the stdout buffer.
  // This ensures a correct sync of all prints
  // even when executed simultaneously within a launch file.
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  auto v4l2ucp = std::make_shared<V4l2Ucp>();
  if (!v4l2ucp->init())
    return -1;
  rclcpp::spin(v4l2ucp);
  INFO("node done");
  rclcpp::shutdown();
  return 0;
}
