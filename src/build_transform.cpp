#include <loam_velodyne/build_transform.h>

#include <pcl/common/eigen.h>

Eigen::Affine3f getTransformationTRzRxRy(float tx, float ty, float tz, float rx, float ry, float rz) {
  float srx = sin(rx);
  float crx = cos(rx);
  float sry = sin(ry);
  float cry = cos(ry);
  float srz = sin(rz);
  float crz = cos(rz);

  float crycrz = cry*crz;
  float crycrzsrx = crycrz*srx;
  float srysrz = sry*srz;
  float srxsrysrz = srx*sry*srz;
  float srxsrycrz = srx*sry*crz;
  float crxsrz = crx*srz;
  float crysrz = cry*srz;
  float crxsry = crx*sry;
  float crxcrz = crx*crz;
  float crxcry = crx*cry;
  float crzsry = crz*sry;

  float crysrxsrz = cry*srx*srz;

  Eigen::Affine3f t;
  t (0, 0) = crycrz + srxsrysrz;   t (0, 1) = srxsrycrz - crysrz;  t (0, 2) = crxsry;  t (0, 3) = tx*(crycrz + srxsrysrz) - ty*(crysrz - srxsrycrz) + crxsry*tz;
  t (1, 0) = crxsrz;               t (1, 1) = crxcrz;              t (1, 2) = -srx;    t (1, 3) = crxcrz*ty - srx*tz + crxsrz*tx;
  t (2, 0) = crysrxsrz - crzsry;   t (2, 1) = srysrz + crycrzsrx;  t (2, 2) = crxcry;  t (2, 3) = ty*(srysrz + crycrzsrx) - tx*(crzsry - crysrxsrz) + crxcry*tz;
  t (3, 0) = 0;                    t (3, 1) = 0;                   t (3, 2) = 0;       t (3, 3) = 1;
  return t;
}

Eigen::Affine3f getTransformationTRzRxRy(const float *dof, float scale) {
  return getTransformationTRzRxRy(dof[3]*scale, dof[4]*scale, dof[5]*scale, dof[0]*scale, dof[1]*scale, dof[2]*scale);
}

Eigen::Affine3f getTransformationTRzRxRy(const std::vector<float> &dof, float scale) {
  return getTransformationTRzRxRy(dof[3]*scale, dof[4]*scale, dof[5]*scale, dof[0]*scale, dof[1]*scale, dof[2]*scale);
}

Eigen::Affine3f getTransformationRyRxRzT(float tx, float ty, float tz, float rx, float ry, float rz) {
  float srx = sin(rx);
  float crx = cos(rx);
  float sry = sin(ry);
  float cry = cos(ry);
  float srz = sin(rz);
  float crz = cos(rz);

  float crycrz = cry*crz;
  float crysrz = cry*srz;
  float srxsry = srx*sry;

  Eigen::Affine3f t;
  t (0, 0) = crycrz - srxsry*srz;  t (0, 1) = -crx*srz;  t (0, 2) = crz*sry + crysrz*srx;  t (0, 3) = tx;
  t (1, 0) = crysrz + crz*srxsry;  t (1, 1) = crx*crz;   t (1, 2) = sry*srz - crycrz*srx;  t (1, 3) = ty;
  t (2, 0) = -crx*sry;             t (2, 1) = srx;       t (2, 2) = crx*cry;               t (2, 3) = tz;
  t (3, 0) = 0;                    t (3, 1) = 0;         t (3, 2) = 0;                     t (3, 3) = 1;
  return t;
}

Eigen::Affine3f getTransformationRyRxRzT(const float *dof) {
  return getTransformationRyRxRzT(dof[3], dof[4], dof[5], dof[0], dof[1], dof[2]);
}

Eigen::Affine3f getTransformationRyRxRzT(const std::vector<float> &dof) {
  return getTransformationRyRxRzT(dof[3], dof[4], dof[5], dof[0], dof[1], dof[2]);
}

Eigen::Affine3f getTransformationRzRxRyT(float tx, float ty, float tz, float rx, float ry, float rz) {
  float srx = sin(rx);
  float crx = cos(rx);
  float sry = sin(ry);
  float cry = cos(ry);
  float srz = sin(rz);
  float crz = cos(rz);

  float crycrz = cry*crz;
  float crysrz = cry*srz;
  float srxsry = srx*sry;

  Eigen::Affine3f t;
  t (0, 0) = crycrz + srxsry*srz;   t (0, 1) = crz*srxsry - crysrz;   t (0, 2) = crx*sry;  t (0, 3) = tx;
  t (1, 0) = crx*srz;               t (1, 1) = crx*crz;               t (1, 2) = -srx;     t (1, 3) = ty;
  t (2, 0) = crysrz*srx - crz*sry;  t (2, 1) = sry*srz + crycrz*srx;  t (2, 2) = crx*cry;  t (2, 3) = tz;
  t (3, 0) = 0;                     t (3, 1) = 0;                     t (3, 2) = 0;        t (3, 3) = 1;

  return t;
}

Eigen::Affine3f getTransformationRzRxRyT(const std::vector<float> &dof) {
  return getTransformationRzRxRyT(dof[3], dof[4], dof[5], dof[0], dof[1], dof[2]);
}

Eigen::Affine3f getTransformationTRyRxRz(float tx, float ty, float tz, float rx, float ry, float rz) {
  float srx = sin(rx);
  float crx = cos(rx);
  float sry = sin(ry);
  float cry = cos(ry);
  float srz = sin(rz);
  float crz = cos(rz);

  float crycrz = cry*crz;
  float srysrz = sry*srz;
  float srxsrysrz = srx*sry*srz;
  float srxsrycrz = srx*sry*crz;
  float crxsrz = crx*srz;
  float crysrz = cry*srz;
  float crxsry = crx*sry;
  float crxcrz = crx*crz;
  float crxcry = crx*cry;
  float crzsry = crz*sry;
  float crysrxsrz = cry*srx*srz;
  float crycrzsrx = cry*crz*srx;

  Eigen::Affine3f t;
  t (0, 0) = crycrz - srxsrysrz;  t (0, 1) = -crxsrz;  t (0, 2) = crzsry + crysrxsrz;  t (0, 3) = tx*(crycrz - srxsrysrz) + tz*(crzsry + crysrxsrz) - crxsrz*ty;
  t (1, 0) = crysrz + srxsrycrz;  t (1, 1) = crxcrz;   t (1, 2) = srysrz - crycrzsrx;  t (1, 3) = tx*(crysrz + srxsrycrz) + tz*(srysrz - crycrzsrx) + crxcrz*ty;
  t (2, 0) = -crxsry;             t (2, 1) = srx;      t (2, 2) = crxcry;              t (2, 3) = srx*ty + crxcry*tz - crxsry*tx;
  t (3, 0) = 0;                   t (3, 1) = 0;        t (3, 2) = 0;                   t (3, 3) = 1;

  return t;
}

Eigen::Affine3f getTransformationTRyRxRz(const std::vector<float> &dof, float scale) {
  return getTransformationTRyRxRz(dof[3]*scale, dof[4]*scale, dof[5]*scale,
                                  dof[0]*scale, dof[1]*scale, dof[2]*scale);
}
