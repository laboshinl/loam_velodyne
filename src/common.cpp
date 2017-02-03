#include <cstdlib>
#include <cstdio>

#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>

#include <algorithm>

using namespace std;
using namespace pcl;

vector<float> transformToVec(const Eigen::Affine3f &t) {
  vector<float> vec(6);
  pcl::getTranslationAndEulerAngles(t, vec[3], vec[4], vec[5], vec[0], vec[1], vec[2]);
  return vec;
}

Eigen::Affine3f vecToTransform(const vector<float> &vec) {
  return pcl::getTransformation(vec[3], vec[4], vec[5], vec[0], vec[1], vec[2]);
}

void swapVec(vector<float> &vec) {
  vector<float> tmpvec = vec;
  vec[0] = tmpvec[2];
  vec[1] = tmpvec[0];
  vec[2] = tmpvec[1];
  vec[3] = tmpvec[5];
  vec[4] = tmpvec[3];
  vec[5] = tmpvec[4];
}

void swapVecBack(vector<float> &vec) {
  vector<float> tmpvec = vec;
  vec[0] = tmpvec[1];
  vec[1] = tmpvec[2];
  vec[2] = tmpvec[0];
  vec[3] = tmpvec[4];
  vec[4] = tmpvec[5];
  vec[5] = tmpvec[3];
}

void transformAssociateToMap(vector<float> beforeMapping, vector<float> afterMapping, vector<float> current, vector<float> &output) {
  swapVec(beforeMapping);
  swapVec(afterMapping);
  swapVec(current);

  output = transformToVec(
          vecToTransform(afterMapping) *
          vecToTransform(beforeMapping).inverse()*
          vecToTransform(current));

  swapVecBack(output);
}
