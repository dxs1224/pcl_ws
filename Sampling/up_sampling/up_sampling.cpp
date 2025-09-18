#include <pcl/io/pcd_io.h>
#include <pcl/surface/mls.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/filter.h>

int main(int argc, char **argv)
{
    // 新建点云存储对象
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZ>);

    // 从文件读取点云图
    pcl::PCDReader reader;
    reader.read("/home/dxs/output.pcd", *cloud);

    // 移除 NaN 点
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);

    std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height
              << " data points (" << pcl::getFieldsList(*cloud) << ")." << std::endl;

    // 检查点云是否为空
    if (cloud->empty())
    {
        std::cerr << "Error: Cloud is empty after removing NaN points!" << std::endl;
        return -1;
    }

    // 滤波对象
    pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ> filter;
    filter.setInputCloud(cloud);

    // 建立搜索对象
    pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZ>);
    filter.setSearchMethod(kdtree);

    // 设置搜索邻域的半径为3cm
    filter.setSearchRadius(0.03);

    // Upsampling 采样的方法
    filter.setUpsamplingMethod(pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointXYZ>::SAMPLE_LOCAL_PLANE);

    // 采样的半径
    filter.setUpsamplingRadius(0.03);

    // 采样步数的大小
    filter.setUpsamplingStepSize(0.02);

    try
    {
        filter.process(*filteredCloud);
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error during MLS processing: " << e.what() << std::endl;
        return -1;
    }

    std::cerr << "PointCloud after filtering: " << filteredCloud->width * filteredCloud->height
              << " data points (" << pcl::getFieldsList(*filteredCloud) << ")." << std::endl;

    // 将结果输出到文件
    pcl::PCDWriter writer;
    writer.write("/home/dxs/output_upsampling.pcd", *filteredCloud);

    return 0;
}