/*
 * @Description: 使用参数化模型投影点云   :https://www.cnblogs.com/li-yao7758258/p/6464145.html
 * @Author: HCQ
 * @Company(School): UCAS
 * @Email: 1756260160@qq.com
 * @Date: 2020-10-20 14:44:50
 * @LastEditTime: 2020-10-20 14:53:17
 * @FilePath: /pcl-learning/09filters滤波/4使用参数化模型投影点云/project_inliers.cpp
 */
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>       //模型系数头文件
#include <pcl/filters/project_inliers.h> //投影滤波类头文件
#include <pcl/visualization/pcl_visualizer.h>

int main(int argc, char **argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected(new pcl::PointCloud<pcl::PointXYZ>);

  // 创建点云并打印出来
  pcl::PCDReader reader;
  reader.read("/home/dxs/output.pcd", *cloud);
  // cloud->width = 5;
  // cloud->height = 1;
  // cloud->points.resize(cloud->width * cloud->height);

  // for (size_t i = 0; i < cloud->points.size(); ++i)
  // {
  //   cloud->points[i].x = 1024 * rand() / (RAND_MAX + 1.0f);
  //   cloud->points[i].y = 1024 * rand() / (RAND_MAX + 1.0f);
  //   cloud->points[i].z = 1024 * rand() / (RAND_MAX + 1.0f);
  // }

  std::cerr << "Cloud before projection: " << std::endl;
  std::cerr << "Cloud size: " << cloud->points.size() << std::endl;
  // for (size_t i = 0; i < cloud->points.size(); ++i)
  //   std::cerr << "    " << cloud->points[i].x << " "
  //             << cloud->points[i].y << " "
  //             << cloud->points[i].z << std::endl;

  // 填充 ModelCoefficients 的值,使用ax+by+cz+d=0平面模型，其中 a=b=d=0,c=1 也就是X——Y平面
  // 定义模型系数对象，并填充对应的数据
  // 1. 创建一个模型系数对象
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
  // 2. 设置系数数组的大小。对于平面模型，我们需要4个系数 (a, b, c, d)
  coefficients->values.resize(4);
  // 3. 为系数赋值
  coefficients->values[0] = 0;   // a
  coefficients->values[1] = 0;   // b
  coefficients->values[2] = 1.0; // c
  coefficients->values[3] = 0;   // d

  // 创建ProjectInliers对象，使用 ModelCoefficients 作为投影对象的模型参数
  pcl::ProjectInliers<pcl::PointXYZ> proj; // 创建投影滤波对象
  proj.setModelType(pcl::SACMODEL_PLANE);  // 设置对象对应的投影模型
  proj.setInputCloud(cloud);               // 设置输入点云
  // 4. 将系数对象传递给投影滤波器
  proj.setModelCoefficients(coefficients); // 设置模型对应的系数
  proj.filter(*cloud_projected);           // 投影结果存储cloud_projected

  std::cerr << "Cloud after projection: " << std::endl;
  std::cerr << "Cloud size: " << cloud_projected->points.size() << std::endl;
  // for (size_t i = 0; i < cloud_projected->points.size(); ++i)
  //   std::cerr << "    " << cloud_projected->points[i].x << " "
  //             << cloud_projected->points[i].y << " "
  //             << cloud_projected->points[i].z << std::endl;

  pcl::visualization::PCLVisualizer viewer("3D Viewer");
  viewer.setBackgroundColor(0, 0, 0);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_color_handler(cloud_projected, 255, 0, 0);
  viewer.addPointCloud(cloud_projected, cloud_color_handler, "cloud points");
  viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "cloud points");
  viewer.addCoordinateSystem(1); // 添加坐标系

  while (!viewer.wasStopped())
  {
    viewer.spinOnce();
    pcl_sleep(0.01);
  }
  return (0);
}