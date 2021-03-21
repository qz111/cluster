// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"


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
    pcl::VoxelGrid<PointT> sor;
    typename pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);
    sor.setInputCloud (cloud);
    sor.setLeafSize(filterRes,filterRes,filterRes);
    sor.filter(*cloud_filtered);
    pcl::CropBox<PointT> region(true);
    region.setMin(minPoint);
    region.setMax(maxPoint);
    region.setInputCloud(cloud_filtered);
    region.filter(*cloud_filtered);
    pcl::CropBox<PointT> region_roof;
    region_roof.setMin(Eigen::Vector4f(-1.5,-1.7,-1,1));
    region_roof.setMax(Eigen::Vector4f(2.6,1.7,-0.4,1));
    typename pcl::PointIndices::Ptr index_roof(new pcl::PointIndices);
    region_roof.setInputCloud(cloud_filtered);
    region_roof.filter(index_roof->indices);
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud_filtered);
    extract.setIndices(index_roof);
    extract.setNegative(true);
    extract.filter(*cloud_filtered);
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloud_filtered;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr cloud_obst(new pcl::PointCloud<PointT>), cloud_plane(new pcl::PointCloud<PointT>);
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(false);
    extract.filter(*cloud_plane);
    extract.setNegative(true);
    extract.filter(*cloud_obst);
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(cloud_obst, cloud_plane);
    return segResult;
}

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(const std::unordered_set<int> &inliers, typename pcl::PointCloud<PointT>::Ptr cloud)
{
    typename pcl::PointCloud<PointT>::Ptr cloud_obst(new pcl::PointCloud<PointT>), cloud_plane(new pcl::PointCloud<PointT>);
    for(int index=0;index<cloud->points.size();index++)
    {
        auto point=cloud->points[index];
        if(inliers.count(index)>0)
        {
            cloud_plane->points.push_back(point);
        }
        else
        {
            cloud_obst->points.push_back(point);
        }
    }
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(cloud_obst, cloud_plane);
    return segResult;
}

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
    /*
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    // TODO:: Fill in this function to find inliers for the cloud.
    pcl::ModelCoefficients::Ptr coeffi(new pcl::ModelCoefficients());
    pcl::SACSegmentation<PointT> seg;
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(maxIterations);
    seg.setDistanceThreshold(distanceThreshold);
    seg.setInputCloud(cloud);
    seg.segment(*inliers,*coeffi);
    */
    std::unordered_set<int> inliers;
    Ransac3D(inliers,cloud, maxIterations, distanceThreshold);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
}

template<typename PointT>
void ProcessPointClouds<PointT>::Ransac3D(std::unordered_set<int> &inliersResult, typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{
	
	srand(time(NULL));
	std::unordered_set<int> temp;
	for(int i=0;i<maxIterations;i++)
	{
		int ran=std::rand()%(cloud->points.size());
		temp.emplace(ran);
		auto p1=cloud->points[ran];
		ran=std::rand()%(cloud->points.size());
		temp.emplace(ran);
		auto p2=cloud->points[ran];
		ran=std::rand()%(cloud->points.size());
		temp.emplace(ran);
		auto p3=cloud->points[ran];
		Vect3 po1(p1.x,p1.y,p1.z);
		Vect3 po2(p2.x,p2.y,p2.z);
		Vect3 po3(p3.x,p3.y,p3.z);
		Vect3 po12=po2-po1;
		Vect3 po13=po3-po1;
		Vect3 product=po12*po13;
		float a=product.x;
		float b=product.y;
		float c=product.z;
		float d=-1*(a*po1.x+b*po1.y+c*po1.z);
		for(int n=0; n<cloud->points.size();n++)
		{
			auto point=cloud->points[n];
			if(temp.count(n)>0)
			{
				continue;
			}
			float dis=fabs(a*point.x+b*point.y+c*point.z+d)/std::sqrt(a*a+b*b+c*c);
			if(dis<=distanceTol)
			{
				temp.emplace(n);
			}
		}
		if(temp.size()>inliersResult.size())
		{
			inliersResult=temp;
		}
		temp.clear();
		
	}
	
}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
    std::vector<pcl::PointIndices> cluster_indices;
    typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(clusterTolerance);
    ec.setMinClusterSize(minSize);
    ec.setMaxClusterSize(maxSize);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);
    for(auto indice:cluster_indices)
    {
        typename pcl::PointCloud<PointT>::Ptr cluster_cloud(new pcl::PointCloud<PointT>);
        for(int in:indice.indices)
        {
            cluster_cloud->points.push_back(cloud->points[in]);
        }
        clusters.push_back(cluster_cloud);
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

template<typename PointT>
void ProcessPointClouds<PointT>::Clustering(std::vector<bool> &notpro, typename pcl::PointCloud<PointT>::Ptr cluster, KdTree* tree, float distanceTol, const std::vector<std::vector<float>>& t_points, const int &id, typename pcl::PointCloud<PointT>::Ptr points)
{
	cluster->points.push_back(points->points[id]);
	notpro[id]=false;
	std::vector<int> ids=tree->search(t_points[id], distanceTol);
	for(auto index: ids)
	{
		auto check=notpro[index];
		if(check==true)
		{
			Clustering(notpro,cluster, tree, distanceTol,t_points,index,points);
		}
	}
}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::euclideanCluster(typename pcl::PointCloud<PointT>::Ptr points, KdTree* tree, float distanceTol)
{

	// TODO: Fill out this function to return list of indices for each cluster

	std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
    std::vector<std::vector<float>> t_points;
    for(auto point:points->points)
    {
        std::vector<float> Point;
        Point.push_back(point.x);
        Point.push_back(point.y);
        Point.push_back(point.z);
        t_points.push_back(Point);
    }
	std::vector<bool> not_Processed(t_points.size(),true);
	for(int i=0;i<t_points.size();i++)
	{
		if(not_Processed[i]==true)
		{
			typename pcl::PointCloud<PointT>::Ptr cluster (new pcl::PointCloud<PointT>);
			Clustering(not_Processed,cluster,tree,distanceTol,t_points,i,points);
			clusters.push_back(cluster);
		}
	}
	return clusters;

}