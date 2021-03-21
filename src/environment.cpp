/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h" 
#include "render/render.h" 
#include "processPointClouds.h" 
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp" 
#include <memory>
static int ID=0;
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


void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    bool renderScene = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    // TODO:: Create lidar sensor 
    std::shared_ptr<Lidar> dyvelony=std::make_shared<Lidar>(cars,0);
    //renderPointCloud(viewer,dyvelony->scan(),"lidar",Color(1,0,0));
    //renderRays(viewer,dyvelony->position,dyvelony->scan());

    // TODO:: Create point processor
    ProcessPointClouds<pcl::PointXYZ> *Processor=new ProcessPointClouds<pcl::PointXYZ>;
    pcl::PointCloud<pcl::PointXYZ>::Ptr Scan_points=dyvelony->scan();
    auto p=Processor->SegmentPlane(Scan_points,100,0.2);
    //renderPointCloud(viewer,p.first,"obstacle",Color(1,0,0));
    //renderPointCloud(viewer,p.second,"plane",Color(0,1,0));
    //auto clusters=Processor->Clustering(p.first,1,3,30);
    KdTree* tree = new KdTree;
    //std::vector<std::vector<float>> Tree_Points;
    //int id_point=0;
    for (int i=0;i<(p.first)->points.size();i++) 
    {
        std::vector<float> Point;
        Point.push_back((p.first)->points[i].x);
        Point.push_back((p.first)->points[i].y);
        Point.push_back((p.first)->points[i].z);
        //Tree_Points.push_back(Point);
        tree->insert(Point,i);
        //id_point++;
    }
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters=Processor->euclideanCluster(p.first, tree, 1);

    //int ID=0;
    std::vector<Color> colors={Color(1,0,0),Color(0,1,0),Color(0,0,1)};
    for(auto cluster:clusters)
    {
        renderPointCloud(viewer,cluster,"cluster"+std::to_string(ID),colors[ID]);
        renderBox(viewer,Processor->BoundingBox(cluster),ID,colors[ID],1);
        ID++;
    }
}

void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI> *Processor, pcl::PointCloud<pcl::PointXYZI>::Ptr Cloud)
{
    //ProcessPointClouds<pcl::PointXYZI> *Processor=new ProcessPointClouds<pcl::PointXYZI>;
    //pcl::PointCloud<pcl::PointXYZI>::Ptr Cloud=Processor->loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");
    Cloud=Processor->FilterCloud(Cloud, 0.2,Eigen::Vector4f(-15,-5,-2.5,1),Eigen::Vector4f(30,8,1.5,1));
    auto p = Processor->SegmentPlane(Cloud,100,0.2);
    renderPointCloud(viewer,p.second,"plane",Color(0,1,0));
    //renderPointCloud(viewer,p.first,"obstacle",Color(1,0,0));
    //auto clusters=Processor->Clustering(p.first,0.3,10,800);
    KdTree* tree = new KdTree;
    //std::vector<std::vector<float>> Tree_Points;
    //int id_point=0;
    for (int i=0;i<(p.first)->points.size();i++) 
    {
        std::vector<float> Point;
        Point.push_back((p.first)->points[i].x);
        Point.push_back((p.first)->points[i].y);
        Point.push_back((p.first)->points[i].z);
        //Tree_Points.push_back(Point);
        tree->insert(Point,i);
        //id_point++;
    }
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clusters=Processor->euclideanCluster(p.first, tree, 0.3);

    std::vector<Color> colors={Color(1,0,0),Color(0,1,0),Color(0,0,1),Color(0,1,1)};
    for(auto cluster:clusters)
    {
        int id=ID%4;
        renderPointCloud(viewer,cluster,"cluster"+std::to_string(ID),colors[id]);
        renderBox(viewer,Processor->BoundingBox(cluster),ID,Color(1,0,0),1);
        ID++;
        
    }

    //renderPointCloud(viewer,Cloud,"input");
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
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 0, 0, 1); break;
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

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = FPS;
    initCamera(setAngle, viewer);
    //simpleHighway(viewer);
    ProcessPointClouds<pcl::PointXYZI> *Processor=new ProcessPointClouds<pcl::PointXYZI>;
    std::vector<boost::filesystem::path> paths=Processor->streamPcd("../src/sensors/data/pcd/data_1/");
    auto iterate=paths.begin();
    //cityBlock(viewer);
    while (!viewer->wasStopped ())
    {
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();
        pcl::PointCloud<pcl::PointXYZI>::Ptr Cloud=Processor->loadPcd((*iterate).string());
        cityBlock(viewer,Processor,Cloud);
        iterate++;
        if(iterate==paths.end())
        {
            iterate=paths.begin();
        }
        
        
        viewer->spinOnce ();
    } 
}