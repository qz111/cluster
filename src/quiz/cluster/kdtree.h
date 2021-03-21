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
	void insert_helper(Node* nNode, Node** oNode, int layer)
	{
		int depth=layer%(nNode->point.size());
		if(*oNode!=NULL)
		{
			if((*oNode)->point[depth]>=nNode->point[depth])
			{
				insert_helper(nNode,&((*oNode)->left),layer+1);
			}
			else
			{
				insert_helper(nNode,&((*oNode)->right),layer+1);
			}
		}
		else
		{
			*oNode=nNode;
		}
	}
	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
		
		Node* nNode=new Node(point, id);
		
		insert_helper(nNode,&root,0);
		
		
	}
	void search_helper(std::vector<int> &ids, Node* node_point, int layer,std::vector<float> target, float distanceTol)
	{
		int index=layer%(target.size());
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
				float tmp=0;
				for(int i=0; i<target.size();i++)
				{
					tmp+=std::pow(node_point->point[i]-target[i],2);
				}
				float dis=std::sqrt(tmp);
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




