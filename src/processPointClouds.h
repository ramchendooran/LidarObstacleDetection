// PCL lib Functions for processing point clouds 

#ifndef PROCESSPOINTCLOUDS_H_
#define PROCESSPOINTCLOUDS_H_

#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/transforms.h>
#include <iostream> 
#include <string>  
#include <vector>
#include <ctime>
#include <chrono>
#include "render/box.h"
#include <cmath>
#include <math.h>
using namespace std;

// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node* left;
	Node* right;
	Node(std::vector<float> arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

	void insertHelper(Node** node, int depth, std::vector<float> point, int id){

		// If there are no pints in *root, then create a fresh Node in stack
		if(*node == NULL) {
			*node = new Node(point, id);
		}
		else {

			// point[0] is x coordinate, point[1] is y coordinate and point[2] is z coordinate
			int ind = depth%3;

			// If less insert left
			if(point[ind] < (*node)->point[ind]) {
				insertHelper(&(*node)->left, depth + 1, point, id);
			}
			// Else insert right
			else {
				insertHelper(&(*node)->right, depth + 1, point, id);
			}

		}

	}

	void insert(std::vector<float> point, int id)
	{	
		int depth = 0;
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
		insertHelper(&root, depth, point, id);

	}
	void searchHelper(Node** node, int depth, std::vector<float> point, float distanceTol, std::vector<int>* ids) {
		//cout<<"Search Helper start: "<<depth<<endl;
		if(*node!= NULL) {
			//Extrema of target point, applying distance tolerance
			float xMax = point[0] + distanceTol;
			float xMin = point[0] - distanceTol;
			float yMax = point[1] + distanceTol;
			float yMin = point[1] - distanceTol;
			float zMax = point[2] + distanceTol;
			float zMin = point[2] - distanceTol;

			// If point is in the cube
			if (xMax > (*node)->point[0] && xMin < (*node)->point[0] && yMax > (*node)->point[1] && yMin < (*node)->point[1] && zMax > (*node)->point[2] && zMin < (*node)->point[2] ) {
				// Check whether point is in the sphere
				float radius = sqrt(pow((point[0] - (*node)->point[0]),2) + pow((point[1] - (*node)->point[1]),2) + pow((point[2] - (*node)->point[2]),2));
				// If point is in the sphere, add the index
				if (radius < distanceTol) {
					ids->push_back((*node)->id);
				}
			}
			
			// ind indicates whethere we have to use x coordinate or y coordinate
			int ind = depth % 3;

			// Extrema of target point in either x, y or z
			float pMax = point[ind] + distanceTol;
			float pMin = point[ind] - distanceTol;
			
			// If Extrema is on the left, search left
			if(pMax < (*node)->point[ind]) {
				searchHelper(&((*node)->left), depth + 1, point, distanceTol, &(*ids));
			}

			// If extrema is on the right, search right
			else if (pMin > (*node)->point[ind]) {
				searchHelper(&((*node)->right), depth + 1, point, distanceTol, &(*ids));
			}
			
			// If point within tolerance, search on both sides
			else {
				//cout<< "Point within tolerance"<< std::endl;
				searchHelper(&((*node)->left), depth +1, point, distanceTol, &(*ids));
				searchHelper(&((*node)->right), depth +1, point, distanceTol, &(*ids));
			}
		}
	}
	
	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		searchHelper(&root, 0, target, distanceTol, &ids);
		return ids;
	}
};

template<typename PointT>
class ProcessPointClouds {
public:

    //constructor
    ProcessPointClouds();
    //deconstructor
    ~ProcessPointClouds();

    void numPoints(typename pcl::PointCloud<PointT>::Ptr cloud);

    typename pcl::PointCloud<PointT>::Ptr FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold);

    std::vector<typename pcl::PointCloud<PointT>::Ptr> Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize);

    Box BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster);

    void savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file);

    typename pcl::PointCloud<PointT>::Ptr loadPcd(std::string file);

    std::vector<boost::filesystem::path> streamPcd(std::string dataPath);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> Ransac(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol);
  
	std::vector<typename pcl::PointCloud<PointT>::Ptr> myClustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize);

	std::vector<std::vector<int>> euclideanCluster(const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol);

	void Proximity(KdTree* tree, const std::vector<std::vector<float>>& points, std::vector<int>& cluster, std::vector<bool>& flag, int i, float distanceTol);
};


#endif /* PROCESSPOINTCLOUDS_H_ */