#include <cstdlib>
#include <cstdio>

#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/filter.h>

#include <algorithm>

#include <loam_velodyne/common.h>
#include <loam_velodyne/build_transform.h>

using namespace std;
using namespace pcl;

Eigen::Affine3f vecToTransform(const vector<float> &vec) {
  return pcl::getTransformation(vec[5], vec[3], vec[4], vec[2], vec[0], vec[1]);
}

vector<float> transformToVec(const Eigen::Affine3f &t, vector<float> &vec) {
  pcl::getTranslationAndEulerAngles(t, vec[5], vec[3], vec[4], vec[2], vec[0], vec[1]);
  return vec;
}

void improveOdometryByMapping(const vector<float> &beforeMapping,
                             const vector<float> &afterMapping,
                             const vector<float> &current,
                             vector<float> &output) {
  transformToVec(vecToTransform(afterMapping) *
                 vecToTransform(beforeMapping).inverse()*
                 vecToTransform(current),
                 output);
}

/**
 * Line is given by points AB.
 * The result is the distance and the direction to closest point from the third point X.
 */
float getLinePointDistance(const Eigen::Vector3f &A, const Eigen::Vector3f &B,
    const Eigen::Vector3f &X, Eigen::Vector3f &unit_direction) {
  Eigen::Vector3f BXcrossAX = (X-B).cross(X-A);
  float BXcrossAXnorm = BXcrossAX.norm();
  float lengthAB = (A-B).norm();
  unit_direction = -BXcrossAX.cross(B-A) / (BXcrossAXnorm * lengthAB);
  return BXcrossAXnorm / lengthAB;
}

float getSurfacePointDistance(const Eigen::Vector3f &A, const Eigen::Vector3f &B, const Eigen::Vector3f &C,
    const Eigen::Vector3f &X, Eigen::Vector3f &surfNormal) {
  surfNormal = (B-A).cross(C-A);
  surfNormal.normalize();

  float normalDotA = -surfNormal.dot(A);
  float distance = surfNormal.dot(X) + normalDotA;
  return distance;
}
