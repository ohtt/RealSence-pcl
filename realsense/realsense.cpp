
#include "realsense.hpp"
using namespace std;


RealSense::RealSense(){

    pipeline.start();

};

RealSense::~RealSense() {

    pipeline.stop();

}

rs2::points RealSense::get_point_frame() {

    auto frames = pipeline.wait_for_frames();

    auto depth = frames.get_depth_frame();

    rs_points = rs_pointcloud.calculate(depth);
    std::cout << " Configure and start the pipeline\n " << std::endl;
    //cloud = points_to_pcl(rs_points);
    return rs_points;
}


