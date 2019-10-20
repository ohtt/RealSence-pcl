//
// Created by Akifumi Ohata on 2019/10/20.
//

#include "viewer.hpp"

PointViewer::PointViewer(){
    *viewer = pcl::visualization::PCLVisualizer ("3D Viewer");

    viewer->setBackgroundColor (0, 0, 0);
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "scan cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 0.5, "map cloud");

    viewer->addCoordinateSystem (1.0,"reference");
    viewer->initCameraParameters ();
    //viewer->registerKeyboardCallback (keyboardEventOccurred, (void*)viewer.get ());

    *color_handler = pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZ>( "x" );
    *single_color = pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>(200, 200, 200);



}

PointViewer::~PointViewer() {


    viewer->close();

}
void PointViewer::Update(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_trance,pcl::PointCloud<pcl::PointXYZ>::Ptr map_cloud,Eigen::Affine3f ret) {


    color_handler->setInputCloud(cloud_trance);
    if( !viewer->updatePointCloud( cloud_trance, *color_handler, "scan cloud" ) ){
        viewer->addPointCloud( cloud_trance, *color_handler, "scan cloud" );
    }

    single_color->setInputCloud(map_cloud);
    if( !viewer->updatePointCloud( map_cloud, *single_color, "map cloud" ) ){
        viewer->addPointCloud( map_cloud, *single_color, "map cloud" );
    }

    viewer->removeCoordinateSystem(cood_name);

    viewer->addCoordinateSystem (0.2, ret, cood_name);

    viewer->spinOnce ();

}


///

//void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event,
//                            void* viewer_void)
//{
//    pcl::visualization::PCLVisualizer *viewer = static_cast<pcl::visualization::PCLVisualizer *> (viewer_void);
//    if (event.getKeySym () == "r" && event.keyDown ())
//    {
//        std::cout << "r was pressed => stop all" << std::endl;
//        stop_flag = true;
//    }
//
//}
