#include <loam_velodyne/common_ros.h>

#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/filter.h>

using namespace std;
using namespace pcl;

void loadCloudFromMsg(const sensor_msgs::PointCloud2ConstPtr& msg,
    pcl::PointCloud<PointType>::Ptr out_cloud,
    double &out_time) {
  out_time = msg->header.stamp.toSec();

  out_cloud->clear();
  pcl::fromROSMsg(*msg, *out_cloud);
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*out_cloud,*out_cloud, indices);
}
