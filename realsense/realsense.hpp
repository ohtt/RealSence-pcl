
#ifndef LRS_PCL_REALSENSE_HPP
#include <iostream>
#include "librealsense2/rs.hpp"

#define LRS_PCL_REALSENSE_HPP


class RealSense{
private:
    rs2::pipeline pipeline;
    rs2::pointcloud rs_pointcloud;
    rs2::points rs_points;
public:
    RealSense();
    ~RealSense();

    rs2::points get_point_frame();
};



#endif //LRS_PCL_REALSENSE_HPP
