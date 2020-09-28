// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"
#include <unordered_set>

//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
    // Create the filtering object
    pcl::VoxelGrid<PointT> sor;
    sor.setInputCloud (cloud);
    sor.setLeafSize (filterRes, filterRes, filterRes);
    sor.filter (*cloud);
    pcl::CropBox<PointT> crop;
    crop.setInputCloud (cloud);
    crop.setMax(maxPoint);
    crop.setMin(minPoint);
    crop.filter(*cloud);
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloud;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr obstacles(new pcl::PointCloud<PointT>);
    typename pcl::PointCloud<PointT>::Ptr road(new pcl::PointCloud<PointT>);

    // Create a nextract object
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud (cloud);

    // Extract road points
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (*road);

    // Extract obstacle points
    extract.setNegative (true);
    extract.filter (*obstacles);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstacles, road);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
	

    // TODO:: Fill in this function to find inliers for the cloud.
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());

    // Create the segmentation object
    pcl::SACSegmentation<PointT> seg;

    seg.setOptimizeCoefficients(true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (maxIterations);
    seg.setDistanceThreshold (distanceThreshold);

    // Segment the largest planar component from the input cloud
    seg.setInputCloud (cloud);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
      std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
    }


    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles

    // Creating the KdTree object for the search method of the extraction
    typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
    tree->setInputCloud (cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance (clusterTolerance); // 2cm
    ec.setMinClusterSize (minSize);
    ec.setMaxClusterSize (maxSize);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud);
    ec.extract (cluster_indices);

    // Iterate through PointIndices and extract the individual clusters. The clusters are stored as a vector of pcl::PointCloud<PointT>
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
        typename pcl::PointCloud<PointT>::Ptr cloud_cluster (new pcl::PointCloud<PointT>);
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit){
            cloud_cluster->push_back ((*cloud)[*pit]); //*
        }
        cloud_cluster->width = cloud_cluster->size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        // Cloud_cluster represents one cluster. Push it into the vector of pcl::PointCloud
        clusters.push_back(cloud_cluster);

    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}


template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}


template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}


template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}

// Own RANSAC implementation
template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Ransac(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{
    auto startTime = std::chrono::steady_clock::now();

    // Two different point clouds for inliers and outliers
    typename pcl::PointCloud<PointT>::Ptr  cloudInliers(new pcl::PointCloud<PointT>());
	typename pcl::PointCloud<PointT>::Ptr cloudOutliers(new pcl::PointCloud<PointT>());

	std::unordered_set<int> inliersResult;
	
	// TODO: Fill in this function
	int maxInlierCount = 0;
	// For max iterations 
	for(int i=0; i<=maxIterations; i++){

		// Randomly sample subset and fit line

		// Get size of the input cloud
		int span = cloud->points.size();

		int ind1, ind2, ind3;

		// Generate 3 random indices between 0 and size
		ind1 = rand() % span;
		ind2 = rand() % span;
		ind3 = rand() % span;

		// Extract 3 points using the indices
		PointT point1 = cloud->points[ind1];
		PointT point2 = cloud->points[ind2];
		PointT point3 = cloud->points[ind3];

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

    // Loop through the points in cloud and seperate inliers from outliers
    for(int index = 0; index < cloud->points.size(); index++)
	{
		PointT point = cloud->points[index];
		if(inliersResult.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}
    
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(cloudOutliers, cloudInliers);
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;
    return segResult;

}


template <typename PointT>
void ProcessPointClouds<PointT>::Proximity(KdTree* tree, const std::vector<std::vector<float>>& points, std::vector<int>& cluster, std::vector<bool>& flag, int i, float distanceTol) {
	// If point has not been processed
	if (!flag[i]) {
		// Mark point as processed
		flag[i] = true;
		// Add point to cluster
		cluster.push_back(i);
		// Find nearby points
		std::vector<int> nearby = tree->search(points[i],distanceTol);
		// Iterate  through each nearby point
  		for(int index : nearby) {
			// Recursion
			Proximity(tree, points, cluster, flag, index, distanceTol);
		}
	}
}

template <typename PointT>
std::vector<std::vector<int>> ProcessPointClouds<PointT>::euclideanCluster(const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol)
{

	// TODO: Fill out this function to return list of indices for each cluster
	// Flag keeps track of whether point is procesed
	std::vector<bool> flag(points.size(), false);
	// clusters is a vector of each cluster (which is a vector of each index of type int)
	std::vector<std::vector<int>> clusters;
	for (int i=0;i<points.size();i++) {
		// If point is not processed
		if(!flag[i]) {
			// Create cluster
			std::vector<int> cluster;
			// Build the cluster
			Proximity(tree, points, cluster, flag, i, distanceTol);
			// Append cluster to clusters
			clusters.push_back(cluster);
		}
	}
	return clusters;
}

// own implementation of Clustering algorithm using KDTree
template <typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::myClustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{   
    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    // Vector of XYZ coordinates
    std::vector<std::vector<float>> points;
    std::vector<typename pcl::PointCloud<PointT>::Ptr> obstacleClusters;
    // Extract the points in cloud and store it in points
    for (auto cloudPoint : cloud->points)
    {
        std::vector<float> point;
        point.push_back(cloudPoint.x);
        point.push_back(cloudPoint.y);
        point.push_back(cloudPoint.z);
        points.push_back(point);
    }

    // Create a KDTree object in heap
    KdTree* tree = new KdTree;

    // Insert the points into the tree object
    for (int i=0; i<points.size(); i++) 
    	tree->insert(points[i],i);

    // Get the indices for each obstacle cloud cluster
    std::vector<std::vector<int>> clusters = euclideanCluster(points, tree, clusterTolerance);

    // Using the indices, extract each cloud into target datastructure
    for (auto cluster: clusters) {
        typename pcl::PointCloud<PointT>::Ptr obstacleCluster(new pcl::PointCloud<PointT>);

    // Filter out clusters less than minSize
        if(cluster.size() < minSize) {
            continue;
        }
        for (auto index: cluster) {
            obstacleCluster->points.push_back(cloud->points[index]);
        }
        obstacleClusters.push_back(obstacleCluster);
    }
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << obstacleClusters.size() << " clusters" << std::endl;
    
    return obstacleClusters;
}