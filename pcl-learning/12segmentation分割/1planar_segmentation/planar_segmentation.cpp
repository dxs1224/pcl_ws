/*
 * @Description: 用一组点云数据做简单的平面的分割：https://www.cnblogs.com/li-yao7758258/p/6496664.html
 * @Author: HCQ
 * @Company(School): UCAS
 * @Date: 2020-10-13 16:33:43
 * @LastEditors: HCQ
 * @LastEditTime: 2020-10-13 17:32:41
 */
#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h> //随机参数估计方法头文件
#include <pcl/sample_consensus/model_types.h>  //模型定义头文件
#include <pcl/segmentation/sac_segmentation.h> //基于采样一致性分割的类的头文件
#include <pcl/filters/extract_indices.h>       // 添加提取索引的头文件
#include <pcl/visualization/pcl_visualizer.h> // 可视化头文件

int main(int argc, char **argv)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // 打开点云文件
    if (pcl::io::loadPCDFile<pcl::PointXYZ>("/home/dxs/output.pcd", *cloud) == -1)
    {
        PCL_ERROR("Couldn't read file /home/dxs/output.pcd \n");
        return (-1);
    }
    // 填充点云
    // cloud->width = 15;
    // cloud->height = 1;
    // cloud->points.resize(cloud->width * cloud->height);

    // // 生成数据，采用随机数填充点云的x,y坐标，都处于z为1的平面上
    // for (size_t i = 0; i < cloud->points.size(); ++i)
    // {
    //     cloud->points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
    //     cloud->points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
    //     cloud->points[i].z = 1.0;
    // }

    // 设置几个局外点，即重新设置几个点的z值，使其偏离z为1的平面
    // cloud->points[0].z = 2.0;
    // cloud->points[3].z = -2.0;
    // cloud->points[6].z = 4.0;

    // std::cerr << "Point cloud data: " << cloud->points.size() << " points" << std::endl; // 打印
    // for (size_t i = 0; i < cloud->points.size(); ++i)
    //     std::cerr << "    " << cloud->points[i].x << " "
    //               << cloud->points[i].y << " "
    //               << cloud->points[i].z << std::endl;

    // 创建分割时所需要的模型系数对象， coefficients 及存储内点的点索引集合对象 inliers
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    // 创建分割对象
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // 可选择配置，设置模型系数需要优化
    seg.setOptimizeCoefficients(true);
    // 必要的配置，设置分割的模型类型，所用的随机参数估计方法，距离阀值，输入点云
    seg.setModelType(pcl::SACMODEL_PLANE); // 设置模型类型 平面
    seg.setMethodType(pcl::SAC_RANSAC);    // 设置随机采样一致性方法类型 RANSAC
    seg.setDistanceThreshold(0.2);        // 设定距离阀值，距离阀值决定了点被认为是局内点是必须满足的条件
                                           // 表示点到估计模型的距离最大值
                                           // 这个距离阈值并不依赖于某一个固定的坐标系，而是相对于RANSAC算法拟合出的最佳平面模型本身的垂直距离阈值​​。

    seg.setInputCloud(cloud);
    // 引发分割实现，存储分割结果到点几何inliers及存储平面模型的系数coefficients
    seg.segment(*inliers, *coefficients);

    if (inliers->indices.size() == 0)
    {
        PCL_ERROR("Could not estimate a planar model for the given dataset.");
        return (-1);
    }

    // 创建用于存储内点的点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>);

    // 从原始点云中提取内点
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(false); // 设置为false表示提取内点，true表示提取外点
    extract.filter(*cloud_plane);

    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->addPointCloud<pcl::PointXYZ>(cloud_plane, "cloud_plane");
    viewer->addPointCloud<pcl::PointXYZ>(cloud, "cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 255, 0, 0, "cloud_plane");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud_plane");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();
    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
    }
    // 保存内点点云
    // pcl::io::savePCDFileASCII("../output_inliers.pcd", *cloud_plane);

    // 打印出平面模型
    // std::cerr << "Model coefficients: " << coefficients->values[0] << " "
    //           << coefficients->values[1] << " "
    //           << coefficients->values[2] << " "
    //           << coefficients->values[3] << std::endl;

    // std::cerr << "Model inliers: " << inliers->indices.size() << std::endl;
    // for (size_t i = 0; i < inliers->indices.size(); ++i)
    //     std::cerr << inliers->indices[i] << "    " << cloud->points[inliers->indices[i]].x << " "
    //               << cloud->points[inliers->indices[i]].y << " "
    //               << cloud->points[inliers->indices[i]].z << std::endl;

    return (0);
}