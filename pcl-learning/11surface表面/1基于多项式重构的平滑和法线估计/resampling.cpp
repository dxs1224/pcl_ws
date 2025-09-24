/*
 * @Description: 基于多项式重构的平滑和法线估计¶
http://robot.czxy.com/docs/pcl/chapter04/resampling/#_2   推荐
https://www.cnblogs.com/li-yao7758258/p/6497446.html
 * @Author: HCQ
 * @Company(School): UCAS
 * @Email: 1756260160@qq.com
 * @Date: 2020-10-21 18:08:55
 * @LastEditTime: 2020-10-21 20:15:10
 * @FilePath: /pcl-learning/11surface表面 /1基于多项式重构的平滑和法线估计/resampling.cpp
 */
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/filter.h> // 添加过滤头文件

int main(int argc, char **argv)
{
    // Load input file into a PointCloud<T> with an appropriate type
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    // Load bun0.pcd -- should be available with the PCL archive in test
    // pcl::io::loadPCDFile(argv[1], *cloud);  // 获取pcd文件
    // pcl::io::loadPCDFile("../ism_train_cat.pcd", *cloud); // 获取pcd文件
    pcl::io::loadPCDFile("/home/dxs/output.pcd", *cloud); // 获取pcd文件

    // 修复点云：移除NaN值
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_clean(new pcl::PointCloud<pcl::PointXYZ>);
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud, *cloud_clean, indices);

    // Create a KD-Tree 创建KD树用于最近邻搜索
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);

    // Output has the PointNormal type in order to store the normals calculated by MLS
    pcl::PointCloud<pcl::PointNormal> mls_points;

    // Init object (second point type is for the normals, even if unused) 初始化MLS对象，移动最小二乘
    pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;

    mls.setComputeNormals(true); // 启用法线计算

    // Set parameters
    mls.setInputCloud(cloud_clean);
    mls.setPolynomialOrder(2); // 设置多项式阶数（2或3）
    mls.setSearchMethod(tree); // 设置搜索方法
    mls.setSearchRadius(0.03); // 设置搜索半径（3cm）

    // Reconstruct
    mls.process(mls_points); // void 	process (PointCloudOut &output) override // 输出包含坐标和法线的点云
    // Save output
    if (mls_points.size() > 0)
    {
        // pcl::io::savePCDFileASCII("../target-mls.pcd", mls_points);
        pcl::io::savePCDFileASCII("../output-mls.pcd", mls_points);
    }
    else
    {
        std::cout << "保存数据为空." << std::endl;
    }

    pcl::visualization::CloudViewer viewer("Cloud Viewer");

    viewer.showCloud(cloud_clean);
    while (!viewer.wasStopped())
    {
    }
}