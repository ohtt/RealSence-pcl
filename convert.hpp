//
// Created by Akifumi Ohata on 2019/10/20.
//

#ifndef LRS_PCL_CONVERT_HPP
#define LRS_PCL_CONVERT_HPP

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_types.h>

#include "librealsense2/rs.hpp"
#include "librealsense2/rs_frame.h"
#include "librealsense2/rs_types.h"

pcl::PointCloud<pcl::PointXYZ>::Ptr points_to_pcl(const rs2::points& points)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    auto sp = points.get_profile().as<rs2::video_stream_profile>();
    cloud->width = sp.width();
    cloud->height = sp.height();
    cloud->is_dense = false;
    cloud->points.resize(points.size());
    auto ptr = points.get_vertices();
    for (auto& p : cloud->points)
    {
        if (ptr-> z < 3 ) {
            p.x = ptr->z;
            p.y = -(ptr->x);
            p.z = -(ptr->y);
        }
        ptr++;
    }

    return cloud;
}

#endif //LRS_PCL_CONVERT_HPP
