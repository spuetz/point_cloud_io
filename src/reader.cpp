/*
 * UOS-ROS packages - Robot Operating System code by the University of Osnabrück
 * Copyright (C) 2017 University of Osnabrück
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 *
 * reader.cpp
 *
 * Author: Sebastian Pütz <spuetz@uos.de>,
 *
 */

#include <ros/ros.h>
#include "point_cloud_io/point_cloud_io.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "point_cloud_io_reader");
  ros::NodeHandle n;

  // publisher parameters -> topic, queue_size, latch
  // latch has to be true to keep this static point cloud data available  
  ros::Publisher cloud_pub = n.advertise<sensor_msgs::PointCloud2>("pointcloud", 1, true);

  std::string frame;
  std::string file;

  int skip = 0;

  XmlRpc::XmlRpcValue channels, fields;

  ros::NodeHandle private_node_handle_("~");
  private_node_handle_.getParam("channels", channels);
  private_node_handle_.getParam("frame", frame);
  private_node_handle_.getParam("file", file);
  private_node_handle_.getParam("skip", skip);
  private_node_handle_.getParam("fields", fields);

  std::vector<point_cloud_io::ChannelInfo*>channel_vector;

  for(int i=0; i<channels.size(); i++){
  	point_cloud_io::ChannelInfo* info = 
  		new point_cloud_io::ChannelInfo(
			channels[i]["row"],
			channels[i]["name"],
			channels[i]["type"],
			(float) ((double)channels[i]["factor"])
  		);
    channel_vector.push_back(info);
  }

  std::vector<sensor_msgs::PointCloud2Modifier::PointFieldInfo> fields_vector;

  for(int i=0; i<fields.size(); i++){
    XmlRpc::XmlRpcValue field = fields[i];

    std::string name = field["name"];
  	std::string type = field["type"];

	fields_vector.push_back(
		sensor_msgs::PointCloud2Modifier::PointFieldInfo(name, type));
  }

  sensor_msgs::PointCloud2 cloud;
  cloud.header.frame_id = frame;
  cloud.header.stamp = ros::Time::now();
  cloud.header.seq = 0;

  point_cloud_io::readPointCloudFromFile(file,
        channel_vector,
        fields_vector,
        cloud,
        skip);

  cloud.header.stamp = ros::Time::now();
  cloud_pub.publish(cloud);
  ROS_INFO_STREAM("Publish Cloud.");

  ros::spin();

  return EXIT_SUCCESS;
}