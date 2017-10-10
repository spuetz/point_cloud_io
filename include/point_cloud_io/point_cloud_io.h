/*
 * UOS-ROS packages - Robot Operating System code by the University of Osnabrück
 * Copyright (C) 2013 University of Osnabrück
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
 * point_cloud_io.h
 *
 * Author: Sebastian Pütz <spuetz@uos.de>,
 *
 */

#ifndef POINT_CLOUD_IO__POINT_CLOUD_IO_H
#define POINT_CLOUD_IO__POINT_CLOUD_IO_H


#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <string>

namespace point_cloud_io{

    static int current_offset_ = 0;

    struct ChannelInfo
    {

    public:

        ChannelInfo(const ChannelInfo& info)
        : row(info.row),
          name(info.name),
          datatype(info.datatype),
          factor(info.factor),
          datatype_id(sensor_msgs::getPointFieldTypeFromString(datatype)),
          offset(info.offset)
        { }

        ChannelInfo(
            int row,
            std::string name,
            float factor
        )
        : row(row),
          name(name),
          factor(factor)
        {}

        int row;
        std::string name;
        std::string datatype;
        float factor;
        unsigned char* ptr;
        int offset;
        int step;

        int datatype_id;

        // to sort the channels
        bool operator< (const ChannelInfo &other) const {
            return row < other.row;
        }

    private:

    };


    bool readPointCloudFromFile(
        const std::string& file,
        std::vector<ChannelInfo>& channels,
        std::vector<sensor_msgs::PointCloud2Modifier::PointFieldInfo>& fields_vector,
        sensor_msgs::PointCloud2& cloud,
        int skip);


} /* end namespace point_cloud_io */

#endif /* POINT_CLOUD_IO__POINT_CLOUD_IO_H */