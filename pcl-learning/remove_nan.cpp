#include <pcl/filters/filter.h> // 添加过滤头文件

int main(int argc, char **argv)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());

    pcl::io::loadPCDFile("/home/dxs/output.pcd", *cloud); // 获取pcd文件

    // 修复点云：移除NaN值
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_clean(new pcl::PointCloud<pcl::PointXYZ>);
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud, *cloud_clean, indices);
}