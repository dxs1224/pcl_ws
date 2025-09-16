#include <iostream>
#include <vector>
#include <ctime>
#include <pcl/point_cloud.h>
#include <pcl/octree/octree_search.h>
#include <pcl/visualization/cloud_viewer.h>

int main(int argc, char **argv)
{
    srand((unsigned int)time(NULL));

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // Generate pointcloud data
    cloud->width = 1000;
    cloud->height = 1;
    cloud->points.resize(cloud->width * cloud->height);

    for (size_t i = 0; i < cloud->points.size(); ++i)
    {
        cloud->points[i].x = 1024.0f * rand() / (RAND_MAX + 1.0f);
        cloud->points[i].y = 1024.0f * rand() / (RAND_MAX + 1.0f);
        cloud->points[i].z = 1024.0f * rand() / (RAND_MAX + 1.0f);
    }

    float resolution = 128.0f;
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree(resolution);
    octree.setInputCloud(cloud);
    octree.addPointsFromInputCloud();

    pcl::PointCloud<pcl::PointXYZ>::Ptr searchCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointXYZ searchPoint;
    searchPoint.x = 1024.0f * rand() / (RAND_MAX + 1.0f);
    searchPoint.y = 1024.0f * rand() / (RAND_MAX + 1.0f);
    searchPoint.z = 1024.0f * rand() / (RAND_MAX + 1.0f);
    searchCloud->push_back(searchPoint);

    int K = 20;
    std::vector<int> pointIdxNKNSearch;
    std::vector<float> pointNKNSquaredDistance;

    std::cout << "K nearest neighbor search at (" << searchPoint.x
              << " " << searchPoint.y
              << " " << searchPoint.z
              << ") with K=" << K << std::endl;
    pcl::visualization::PCLVisualizer viewer("PCL Viewer");
    // K近邻搜索并添加线段
    if (octree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
    {
        for (size_t i = 0; i < pointIdxNKNSearch.size(); ++i)
        {
            pcl::PointXYZ neighborPoint = cloud->points[pointIdxNKNSearch[i]];
            std::string lineId = "line_" + std::to_string(i);
            viewer.addLine(searchPoint, neighborPoint, lineId);
            viewer.setShapeRenderingProperties(
                pcl::visualization::PCL_VISUALIZER_COLOR,
                1, 1, 0.2784, // 黄色
                lineId);
            std::cout << "    " << neighborPoint.x << " " << neighborPoint.y << " " << neighborPoint.z
                      << " (squared distance: " << pointNKNSquaredDistance[i] << ")" << std::endl;
        }
    }

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> originColorHandler(cloud, 255, 255, 255);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> searchColorHandler(searchCloud, 0, 255, 0);

    // pcl::visualization::PCLVisualizer viewer("PCL Viewer");
    viewer.setBackgroundColor(0.1176, 0.1176, 0.2353);
    viewer.addPointCloud<pcl::PointXYZ>(cloud, originColorHandler, "cloud");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "cloud");
    viewer.addPointCloud<pcl::PointXYZ>(searchCloud, searchColorHandler, "search_cloud");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "search_cloud");
    viewer.addCoordinateSystem(200);

    while (!viewer.wasStopped())
    {
        viewer.spinOnce();
    }

    return 0;
}