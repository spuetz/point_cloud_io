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
 * writer.cpp
 *
 * Author: Sebastian Pütz <spuetz@uos.de>,
 *
 */

#include "point_cloud_io/point_cloud_io.h"

#include <iostream>
#include <fstream>
#include <locale>
#include <sstream>
#include <set>
#include <sensor_msgs/PointField.h>

namespace point_cloud_io{


	struct tokens: std::ctype<char> 
	{
	    tokens(): std::ctype<char>(get_table()) {}

	    static std::ctype_base::mask const* get_table()
	    {
	        typedef std::ctype<char> cctype;
	        static const cctype::mask *const_rc= cctype::classic_table();

	        static cctype::mask rc[cctype::table_size];
	        std::memcpy(rc, const_rc, cctype::table_size * sizeof(cctype::mask));

	        rc[','] = std::ctype_base::space; 
	        rc[' '] = std::ctype_base::space; 
	        return &rc[0];
	    }
	};

	bool readPointCloudFromFile(
		const std::string& file,
		std::vector<ChannelInfo*>& channels,
		std::vector<sensor_msgs::PointCloud2Modifier::PointFieldInfo> fields_vector,
		sensor_msgs::PointCloud2& cloud,
		int skip)
	{	
	    int num_lines = 0;
	    std::string line;
	    std::ifstream point_file(file.c_str());

	    if(!point_file.good()){
	      ROS_ERROR_STREAM("The file \"" << file << "\" does not exist or is not reable!");
	      return false;
	    }

	    ROS_INFO_STREAM("Read PointCloud2 from file: \"" << file << "\".");

	    ROS_INFO_STREAM("Skip: " << skip);

	    while (std::getline(point_file, line))
	        ++num_lines;

	  	int size = num_lines / (skip+1);

	    ROS_INFO_STREAM("Number of lines in point cloud file: " << num_lines);
  	    ROS_INFO_STREAM("Number of points used: " << size << " -> skipping " << num_lines - size << " points");
	    ROS_INFO_STREAM("Expecting the same number of point data values per field!");

  	    ROS_INFO_STREAM("Count of point fields: " << fields_vector.size());
   	    ROS_INFO_STREAM("Count of channels: " << channels.size());

  	    sensor_msgs::PointCloud2Modifier pcd_modifier(cloud);
  	    pcd_modifier.addPointCloud2Fields(fields_vector);
  	    cloud.is_dense = true;

  	    ROS_INFO_STREAM("Count of point fields: " << cloud.fields.size());

		for(int i = 0; i != cloud.fields.size(); i++){
			ROS_INFO_STREAM("Field name: " << cloud.fields[i].name);
			ROS_INFO_STREAM("Field datatype: " << (int) cloud.fields[i].datatype);
			ROS_INFO_STREAM("Field offset: " << cloud.fields[i].offset);
			ROS_INFO_STREAM("Field count: " << cloud.fields[i].count);
		}

		for(int i = 0; i != channels.size(); i++){
			ChannelInfo* info = channels[i];
			ROS_INFO_STREAM("ChannelInfo name: " << info->name);
			ROS_INFO_STREAM("ChannelInfo type: " << info->datatype << "(" << info->datatype_id << ")");
			ROS_INFO_STREAM("ChannelInfo offset: " << info->offset);
			ROS_INFO_STREAM("ChannelInfo row: " << info->row);
		}

	    pcd_modifier.resize(size);

		ROS_INFO_STREAM("Cloud Info:");
		ROS_INFO_STREAM("height: " << cloud.height);
		ROS_INFO_STREAM("width:  " << cloud.width);
		ROS_INFO_STREAM("size:   " << cloud.width * cloud.height);
		ROS_INFO_STREAM("step:   " << cloud.point_step);
		ROS_INFO_STREAM("frame:  " << cloud.header.frame_id);

	    // reset stream to the beginning of the file
	    point_file.clear();
	    point_file.seekg(0, point_file.beg);

	    int skipper = 0;
	    int percentage_step = 5;
	    int marker_stp = size * percentage_step / 100;
	    int marker = marker_stp;
	    int percentage = percentage_step;
	    int counter = 0;

	    for (int i=0; std::getline(point_file, line); ){
	    	if(skipper != 0 && skip != 0){
	    		skipper = skipper == skip ? 0 : skipper + 1;
	    		continue;
	    	}
	    	if(counter == marker){
	    		ROS_INFO_STREAM("" << percentage << "% -> read process: " << marker << "points...");
	    		marker += marker_stp;
	    		percentage += percentage_step;
	    	}

			std::stringstream ss(line);
			ss.imbue(std::locale(std::locale(), new tokens()));
			std::istream_iterator<std::string> begin(ss);
			std::istream_iterator<std::string> end;
			std::vector<std::string> values(begin, end);

			for(std::vector<ChannelInfo*>::iterator iter = channels.begin();
				iter != channels.end(); ++iter){
				double value = atof(values[(*iter)->row].c_str());
				value *= (*iter)->factor; // multiply with factor
				sensor_msgs::writePointCloud2BufferValue<double>(
					&cloud.data[i + ((*iter)->offset)],  // ptr to position in the buffer
					(unsigned char)(*iter)->datatype_id, // datatype representation in the buffer
					value 								 // value to write in the buffer
				);
			}
			skipper++;
			i+= cloud.point_step;
			counter++;
		}

	    return true;
	}



} /* end namespace point_cloud_io */
