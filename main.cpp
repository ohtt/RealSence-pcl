#include <iostream>

#include "librealsense2/rs.hpp"
#include <algorithm>            // std::min, std::max
#include "librealsense2/rs_frame.h"

#include "librealsense2/rs_types.h"

#include <pcl/io/pcd_io.h>

#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/registration/ndt.h>
#include <pcl/filters/approximate_voxel_grid.h>

using pcl_ptr = pcl::PointCloud<pcl::PointXYZ>::Ptr;
using namespace std;
bool stop_flag = false;

pcl_ptr points_to_pcl(const rs2::points& points)
{
    pcl_ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

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

void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event,
                            void* viewer_void)
{
    pcl::visualization::PCLVisualizer *viewer = static_cast<pcl::visualization::PCLVisualizer *> (viewer_void);
    if (event.getKeySym () == "r" && event.keyDown ())
    {
        std::cout << "r was pressed => stop all" << std::endl;
        stop_flag = true;
    }

}
Eigen::Matrix4f ndt_matching(const pcl::PointCloud<pcl::PointXYZ> source_points , const pcl::PointCloud<pcl::PointXYZ> map_points ,Eigen::Matrix4f init_guess,pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud){

    // Loading first scan of room.
    pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud (new pcl::PointCloud<pcl::PointXYZ>);

    *target_cloud = map_points;

    if (map_points.width == 0)
    {
        PCL_ERROR ("Couldn't read file room_scan1.pcd \n");
    }

    // Loading second scan of room from new perspective.
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    *input_cloud = source_points;
    if (source_points.width == 0)
    {
        PCL_ERROR ("Couldn't read source scan  \n");
    }

    // Filtering input scan to roughly 10% of original size to increase speed of registration.
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ApproximateVoxelGrid<pcl::PointXYZ> approximate_voxel_filter;
    approximate_voxel_filter.setLeafSize (0.1, 0.1, 0.1);
    approximate_voxel_filter.setInputCloud (input_cloud);
    approximate_voxel_filter.filter (*filtered_cloud);


    // Initializing Normal Distributions Transform (NDT).
    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt;

    // Setting scale dependent NDT parameters
    // Setting minimum transformation difference for termination condition.
    ndt.setTransformationEpsilon (0.01);
    // Setting maximum step size for More-Thuente line search.
    ndt.setStepSize (0.04);
    //Setting Resolution of NDT grid structure (VoxelGridCovariance).
    ndt.setResolution (0.5);

    // Setting max number of registration iterations.
    ndt.setMaximumIterations (100);

    // Setting point cloud to be aligned.
    ndt.setInputSource (filtered_cloud);
    // Setting point cloud to be aligned to.
    ndt.setInputTarget (target_cloud);


    // Calculating required rigid transform to align the input cloud to the target cloud.
    //pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    ndt.align (*output_cloud, init_guess);

    std::cout << "Normal Distributions Transform has converged:" << ndt.hasConverged ()
              << " score: " << ndt.getFitnessScore () << std::endl;

    // Transforming unfiltered, input cloud using found transform.
    pcl::transformPointCloud (*input_cloud, *output_cloud, ndt.getFinalTransformation ());


    return ndt.getFinalTransformation ();
}

int main() {


    std::cout << " Start program" << std::endl;
    // Declare pointcloud object, for calculating pointclouds and texture mappings
    rs2::pointcloud pc;
    // We want the points object to be persistent so we can display the last cloud when a frame drops
    rs2::points points;

    // Create a Pipeline - this serves as a top-level API for streaming and processing frames
    rs2::pipeline p;

    // Configure and start the pipeline
    p.start();
    std::cout << " Configure and start the pipeline\n " << std::endl;
    int count = 0;


   // pcl visualizatiion common  config
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "scan cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 0.5, "map cloud");

    viewer->addCoordinateSystem (1.0,"reference");
    viewer->initCameraParameters ();
    viewer->registerKeyboardCallback (keyboardEventOccurred, (void*)viewer.get ());
    const string cood_name = "realsense";



    pcl_ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    boost::shared_ptr<pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ>> color_handler( new pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ>( "x" ) );

    pcl_ptr map_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(map_cloud, 255, 255, 255);


    // Create the filtering object
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ> ());

    pcl::VoxelGrid<pcl::PointXYZ> grid;
    grid.setLeafSize (0.05f, 0.05f, 0.05f);




    // Set initial alignment estimate found using robot odometry.
    Eigen::AngleAxisf init_rotation (0, Eigen::Vector3f::UnitZ ());
    Eigen::Translation3f init_translation (0, 0, 0);
    Eigen::Matrix4f init_guess = (init_translation * init_rotation).matrix ();



    // Loading first scan of room.
    if (pcl::io::loadPCDFile<pcl::PointXYZ> ("room_scan_org.pcd", *map_cloud) == -1)
    {
        PCL_ERROR ("Couldn't read file room_scan1.pcd \n");
        return (-1);
    }

    // Wait for the next set of frames from the camera
    auto frames = p.wait_for_frames();

    auto depth = frames.get_depth_frame();

    // Generate the pointcloud and texture mappings
    points = pc.calculate(depth);
    cloud = points_to_pcl(points);

    // voxel grid filter
    grid.setLeafSize (0.05f, 0.05f, 0.05f);
    grid.setInputCloud (cloud);
    grid.filter (*map_cloud);

    while (!stop_flag) {

        // Wait for the next set of frames from the camera
        auto frames = p.wait_for_frames();

        auto depth = frames.get_depth_frame();

        // Generate the pointcloud and texture mappings
        points = pc.calculate(depth);
        cloud = points_to_pcl(points);

        // voxel grid filter
        grid.setLeafSize (0.05f, 0.05f, 0.05f);
        grid.setInputCloud (cloud);
        grid.filter (*cloud_filtered);
        //ndt

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_trance (new pcl::PointCloud<pcl::PointXYZ> ());

        init_guess = ndt_matching(*cloud_filtered,*map_cloud,init_guess,cloud_trance);



        cout<< init_guess << endl;
        *map_cloud+= *cloud_trance;

        // voxel grid filter
        grid.setLeafSize (0.1f, 0.1f, 0.1f);
        grid.setInputCloud (map_cloud);
        grid.filter (*map_cloud);

        color_handler->setInputCloud(cloud_trance);
        if( !viewer->updatePointCloud( cloud_trance, *color_handler, "scan cloud" ) ){
            viewer->addPointCloud( cloud_trance, *color_handler, "scan cloud" );
        }

        if( !viewer->updatePointCloud( map_cloud, single_color, "map cloud" ) ){
            viewer->addPointCloud( map_cloud, single_color, "map cloud" );
        }


        Eigen::Affine3f ret;
        ret.matrix() = init_guess;

        viewer->removeCoordinateSystem(cood_name);

        viewer->addCoordinateSystem (0.2,ret,cood_name);

        count+=1;
        viewer->spinOnce ();

        if (stop_flag == true) {
            break;
        }
    }


    viewer->close();

    p.stop();

    pcl::io::savePCDFileBinary ("room_scan_org.pcd", *map_cloud);

}