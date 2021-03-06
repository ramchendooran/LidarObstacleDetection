/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"
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

			// If depth is odd, index has to be 1(y value), else get has to be 0 (x value)
			int ind = depth%2;

			if(point[ind] < (*node)->point[ind]) {
				insertHelper(&(*node)->left, depth + 1, point, id);
			}
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
		cout<<"Search Helper start: "<<depth<<endl;
		if(*node!= NULL) {
			//Extrema of target point, applying distance tolerance
			float xMax = point[0] + distanceTol;
			float xMin = point[0] - distanceTol;
			float yMax = point[1] + distanceTol;
			float yMin = point[1] - distanceTol;

			// If point is in the box
			if (xMax > (*node)->point[0] && xMin < (*node)->point[0] && yMax > (*node)->point[1] && yMin < (*node)->point[1]) {
				// Check whether point is in the circle
				float radius = sqrt(pow((point[0] - (*node)->point[0]),2) + pow((point[1] - (*node)->point[1]),2));
				// If point is in the circle, add the index
				if (radius < distanceTol) {
					ids->push_back((*node)->id);
				}
			}
			
			// ind indicates whethere we have to use x coordinate or y coordinate
			int ind = depth % 2;

			// Extrema of target point in either x or y
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
				cout<< "Point within tolerance"<< std::endl;
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




