#include <cstdlib>
#include <cstdio>

#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/filter.h>

#include <algorithm>

#include <loam_velodyne/common.h>

using namespace std;
using namespace pcl;

Eigen::Affine3f vecToTransform(const vector<float> &vec) {
  return pcl::getTransformation(vec[5], vec[3], vec[4], vec[2], vec[0], vec[1]);
}

vector<float> transformToVec(const Eigen::Affine3f &t) {
  vector<float> vec(6);
  pcl::getTranslationAndEulerAngles(t, vec[5], vec[3], vec[4], vec[2], vec[0], vec[1]);
  return vec;
}

void transformAssociateToMap(vector<float> beforeMapping, vector<float> afterMapping, vector<float> current, vector<float> &output) {
  output = transformToVec(
          vecToTransform(afterMapping) *
          vecToTransform(beforeMapping).inverse()*
          vecToTransform(current));
}

void loadCloudFromMsg(const sensor_msgs::PointCloud2ConstPtr& msg,
    pcl::PointCloud<PointType>::Ptr out_cloud,
    double &out_time) {
  out_time = msg->header.stamp.toSec();

  out_cloud->clear();
  pcl::fromROSMsg(*msg, *out_cloud);
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*out_cloud,*out_cloud, indices);
}
