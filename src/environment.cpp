/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"



std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    Car egoCar( Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");
    Car car1( Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
    Car car2( Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2");	
    Car car3( Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3");
  
    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if(renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}

void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI>* pointProcessorI, const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud)
// void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display City Block     -----
    // ----------------------------------------------------
    /*
    // Single Frame
    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = pointProcessorI->loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");
    */
    pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud = pointProcessorI->FilterCloud(inputCloud, 0.25 , Eigen::Vector4f (-15, -6, -3, 1), Eigen::Vector4f (30, 6, 15, 1));
    
    //renderPointCloud(viewer,filterCloud,"filterCloud");
    int maxIterations = 100;
    float distanceThreshold = 0.2;
    // Get the results of segmentation (road and obstacles) as a pair object
    std::pair<typename pcl::PointCloud<pcl::PointXYZI>::Ptr, typename pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessorI->Ransac(filterCloud, maxIterations, distanceThreshold);
    // Render the obstacles in red
    //renderPointCloud(viewer,segmentCloud.first,"obstCloud",Color(1,0,0));
    // Render the road in green
    renderPointCloud(viewer,segmentCloud.second,"planeCloud",Color(0,1,0));

    std::vector<typename pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI->myClustering(segmentCloud.first, 0.4, 10, 300);
    
    // Create a vector of color objects
    std::vector<Color> colors;
    
    for(int i = 0; i<cloudClusters.size(); i++) {
        if (i%3 == 0){
            colors.push_back(Color(1,0,0)); // Red
        }
        else if (i%3 == 1){
            colors.push_back(Color(0,0,1)); // Blue
        }
        else {
            colors.push_back(Color(1,1,0)); // Yellow
        }
    }

    // Reset cluster id
    int clusterId = 0;

    for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)
    {
        std::cout << "cluster size ";
        pointProcessorI->numPoints(cluster);
        renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId]);
        // Find the bounding boxes and store it in a box object
        Box box = pointProcessorI->BoundingBox(cluster);
        renderBox(viewer,box,clusterId);
        ++clusterId;
    }
    
}

void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    std::string name = "pointCloud";
    Color color(1,1,1);
    // RENDER OPTIONS
    bool renderScene = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    // TODO:: Create lidar sensor 
    Lidar* lidar = new Lidar(cars, 0);
    pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud = lidar->scan();
    // renderRays(viewer, lidar->position,inputCloud);
    //renderPointCloud(viewer, inputCloud, name, color);

    // TODO:: Create point processor
    ProcessPointClouds<pcl::PointXYZ> pointProcessor;

    int maxIterations = 100;
    float distanceThreshold = 0.2;
    // Get the results of segmentation (road and obstacles) as a pair object
    std::pair<typename pcl::PointCloud<pcl::PointXYZ>::Ptr, typename pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = pointProcessor.SegmentPlane(inputCloud, maxIterations, distanceThreshold);
    // Render the obstacles in red
    // renderPointCloud(viewer,segmentCloud.first,"obstCloud",Color(1,0,0));
    // Render the 
    // renderPointCloud(viewer,segmentCloud.second,"planeCloud",Color(0,1,0));

    std::vector<typename pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = pointProcessor.Clustering(segmentCloud.first, 1.0, 3, 30);

    int clusterId = 0;

    // Create a vector of color objects
    std::vector<Color> colors = {Color(1,0,0), Color(0,1,0), Color(0,0,1)};

    for(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters)
    {
        std::cout << "cluster size ";
        pointProcessor.numPoints(cluster);
        renderPointCloud(viewer,cluster,"obstCloud"+std::to_string(clusterId),colors[clusterId]);
        // Find the bounding boxes and sti=ore it a box object
        Box box = pointProcessor.BoundingBox(cluster);
        renderBox(viewer,box,clusterId);
        ++clusterId;
    }
}


//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    viewer->setBackgroundColor (0, 0, 0);
    
    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;
    
    switch(setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);
}


int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    // Creating a viewer
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    
    // Create a Point Processor object pointer (XYZI) in the heap
    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    // Something like a flow from directory. This directory contains the PCD files
    std::vector<boost::filesystem::path> stream = pointProcessorI->streamPcd("../src/sensors/data/pcd/data_1");
    // (Notice the usage of auto for iterator) When function returns, you could use auto
    auto streamIterator = stream.begin();
    
    // Point cloud container containing Point cloud intensity points
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);
    //cityBlock(viewer);

    while (!viewer->wasStopped ())
    {
        
        // Clear viewer
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        // Load pcd and run obstacle detection process
        inputCloudI = pointProcessorI->loadPcd((*streamIterator).string());
        cityBlock(viewer, pointProcessorI, inputCloudI);

        streamIterator++;
        if(streamIterator == stream.end())
            streamIterator = stream.begin();

        // Controls the framerate. By default, it waits 1 time step
        
        viewer->spinOnce ();
    } 
}