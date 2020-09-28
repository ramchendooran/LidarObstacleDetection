/* \author Aaron Brown */
// Quiz on implementing simple RANSAC line fitting

#include "../../render/render.h"
#include <unordered_set>
#include "../../processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "../../processPointClouds.cpp"
#include <cmath>
#include <math.h>

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  	// Add inliers
  	float scatter = 0.6;
  	for(int i = -5; i < 5; i++)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = i+scatter*rx;
  		point.y = i+scatter*ry;
  		point.z = 0;

  		cloud->points.push_back(point);
  	}
  	// Add outliers
  	int numOutliers = 10;
  	while(numOutliers--)
  	{
  		double rx = 2*(((double) rand() / (RAND_MAX))-0.5);
  		double ry = 2*(((double) rand() / (RAND_MAX))-0.5);
  		pcl::PointXYZ point;
  		point.x = 5*rx;
  		point.y = 5*ry;
  		point.z = 0;

  		cloud->points.push_back(point);

  	}
  	cloud->width = cloud->points.size();
  	cloud->height = 1;

  	return cloud;

}

pcl::PointCloud<pcl::PointXYZ>::Ptr CreateData3D()
{
	ProcessPointClouds<pcl::PointXYZ> pointProcessor;
	return pointProcessor.loadPcd("../../../sensors/data/pcd/simpleHighway.pcd");
}


pcl::visualization::PCLVisualizer::Ptr initScene()
{
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("2D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
  	viewer->initCameraParameters();
  	viewer->setCameraPosition(0, 0, 15, 0, 1, 0);
  	viewer->addCoordinateSystem (1.0);
  	return viewer;
}

int generateRandom(int span){
	int ind = rand() % span; // Generate random number between 0 and span
	return ind;
}

std::unordered_set<int> Ransac2D(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// TODO: Fill in this function
	int maxInlierCount = 0;
	// For max iterations 
	for(int i=0; i<=maxIterations; i++){

		// Randomly sample subset and fit line

		// Get size of the input cloud
		int span = cloud->points.size() - 1;

		int ind1, ind2;

		// Generate 2 random indices between 0 and size -1
		ind1 = generateRandom(span);
		ind2 = generateRandom(span);

		// Extract 2 points using the indices
		pcl::PointXYZ point1 = cloud->points[ind1];
		pcl::PointXYZ point2 = cloud->points[ind2];

		// Coeffecients of model
		float a = point1.y - point2.y;
		float b = point2.x - point1.x;
		float c = (point1.x * point2.y) - (point2.x * point1.y);

		// Measure distance between every point and fitted line
		int inlierCount = 0;
		std::unordered_set<int> currentInliers ={};
		int j = 0;
		 
		for(auto point : cloud->points){
			
			// Distance formula
			float d = std::abs((a*point.x + b*point.y + c)) / (sqrt(a*a + b*b));
			
			// If distance is smaller than threshold count it as inlier and add index to currentInliers
			if(d<distanceTol){
				currentInliers.insert(j);
				inlierCount++;
			}
			j++;
		}
		// cout<<"Inlier Count: "<<inlierCount<<endl;
		// If max inlier count is reached, modify the inlierResult unordered set
		if(inlierCount>maxInlierCount){
			maxInlierCount = inlierCount;
			inliersResult = currentInliers;
		}

	}
	// Return indicies of inliers from fitted line with most inliers

	// Print the value of the indices
	/*
	for(auto& elm : inliersResult){
        std::cout<< elm << " ";
    }
	*/
	return inliersResult;

}

std::unordered_set<int> Ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int maxIterations, float distanceTol)
{
	std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// TODO: Fill in this function
	int maxInlierCount = 0;
	// For max iterations 
	for(int i=0; i<=maxIterations; i++){

		// Randomly sample subset and fit line

		// Get size of the input cloud
		int span = cloud->points.size();

		int ind1, ind2, ind3;

		// Generate 3 random indices between 0 and size
		ind1 = generateRandom(span);
		ind2 = generateRandom(span);
		ind3 = generateRandom(span);

		// Extract 3 points using the indices
		pcl::PointXYZ point1 = cloud->points[ind1];
		pcl::PointXYZ point2 = cloud->points[ind2];
		pcl::PointXYZ point3 = cloud->points[ind3];

		// Extract the components of the vector
		float x1 = point1.x;
		float y1 = point1.y;
		float z1 = point1.z;

		float x2 = point2.x;
		float y2 = point2.y;
		float z2 = point2.z;

		float x3 = point3.x;
		float y3 = point3.y;
		float z3 = point3.z;

		// Coeffecients of model
		float a = ((y2 -y1)*(z3 - z1) - (z2 - z1)*(y3 - y1));
		float b = ((z2 - z1)*(x3 - x1) - (x2 - x1)*(z3 - z1));
		float c = ((x2 - x1)*(y3 - y1) - (y2 - y1)*(x3 - x1));
		float d = -(a*x1 + b*y1 +c*z1);

		// Measure distance between every point and fitted line
		int inlierCount = 0;
		std::unordered_set<int> currentInliers ={};
		int j = 0;
		 
		for(auto point : cloud->points){
			
			// Distance formula
			float dist = fabs((a*point.x + b*point.y + c*point.z +d)) / (sqrt(a*a + b*b +c*c));
			// cout << "Distance " << j << " : " << dist <<endl;
			// If distance is smaller than threshold count it as inlier and add index to currentInliers
			if(dist<distanceTol){
				currentInliers.insert(j);
				inlierCount++;
			}
			j++;
		}
		// cout<<"Inlier Count: "<<inlierCount<<endl;
		// If max inlier count is reached, modify the inlierResult unordered set
		if(inlierCount>maxInlierCount){
			maxInlierCount = inlierCount;
			inliersResult = currentInliers;
		}

	}
	// Return indicies of inliers from fitted line with most inliers

	// Print the value of the indices
	/*
	for(auto& elm : inliersResult){
        std::cout<< elm << " ";
    }
	*/
	return inliersResult;

}

int main ()
{

	// Create viewer
	pcl::visualization::PCLVisualizer::Ptr viewer = initScene();

	// Create data
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = CreateData3D();
	

	// TODO: Change the max iteration and distance tolerance arguments for Ransac function
	std::unordered_set<int> inliers = Ransac(cloud, 50, 0.5);

	pcl::PointCloud<pcl::PointXYZ>::Ptr  cloudInliers(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOutliers(new pcl::PointCloud<pcl::PointXYZ>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZ point = cloud->points[index];
		if(inliers.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}


	// Render 2D point cloud with inliers and outliers
	if(inliers.size())
	{
		renderPointCloud(viewer,cloudInliers,"inliers",Color(0,1,0));
  		renderPointCloud(viewer,cloudOutliers,"outliers",Color(1,0,0));
	}
  	else
  	{
  		renderPointCloud(viewer,cloud,"data");
  	}
	
  	while (!viewer->wasStopped ())
  	{
  	  viewer->spinOnce ();
  	}
  	
}
