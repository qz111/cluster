/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"


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

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
		
		Node* nNode=new Node(point, id);
		int layer=0;
		Node** temp=&root;
		while(*temp!=NULL)
		{
			if(layer%2==0)
			{
				if((*temp)->point[0]>=nNode->point[0])
				{
					temp=&((*temp)->left);
				}
				else
				{
					temp=&((*temp)->right);
				}
			}
			else
			{
				if((*temp)->point[1]>=nNode->point[1])
				{
					temp=&((*temp)->left);
				}
				else
				{
					temp=&((*temp)->right);
				}
			}
			layer++;
		}
		*temp=nNode;
		
		
	}
	void search_helper(std::vector<int> &ids, Node* node_point, int layer,std::vector<float> target, float distanceTol)
	{
		int index=layer%2;
		if(node_point!=NULL)
		{
			if(node_point->point[index]>target[index]+distanceTol)
			{
				search_helper(ids,node_point->left,layer+1,target,distanceTol);
			}
			else if(node_point->point[index]<target[index]-distanceTol)
			{
				search_helper(ids,node_point->right,layer+1,target,distanceTol);
			}
			else
			{
				float dis=std::sqrt(std::pow(node_point->point[0]-target[0],2)+std::pow(node_point->point[1]-target[1],2));
				if(dis<=distanceTol)
				{
					ids.push_back(node_point->id);
				}
				
				search_helper(ids,node_point->left,layer+1,target,distanceTol);
				search_helper(ids,node_point->right,layer+1,target,distanceTol);
				
			}
		}
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		search_helper(ids,root,0,target,distanceTol);
		return ids;
	}
	

};




