#include <math.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <opencv/cv.h>
#include <opencv2/highgui/highgui.hpp>

#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>

const double PI = 3.1415926;
const double rad2deg = 180 / PI;
const double deg2rad = PI / 180;

bool systemInited = false;

double initTime;
double timeLasted;
double timeLastedRec;
double startTimeCur;
double startTimeLast;

double timeLaserCloudExtreCur = 0;
double timeLaserCloudLast = 0;

bool newLaserCloudExtreCur = false;
bool newLaserCloudLast = false;

pcl::PointCloud<pcl::PointXYZHSV>::Ptr laserCloudExtreCur(new pcl::PointCloud<pcl::PointXYZHSV>());
pcl::PointCloud<pcl::PointXYZHSV>::Ptr laserCloudExtreLast(new pcl::PointCloud<pcl::PointXYZHSV>());
pcl::PointCloud<pcl::PointXYZHSV>::Ptr laserCloudExtreOri(new pcl::PointCloud<pcl::PointXYZHSV>());
//pcl::PointCloud<pcl::PointXYZHSV>::Ptr laserCloudExtreSel(new pcl::PointCloud<pcl::PointXYZHSV>());
//pcl::PointCloud<pcl::PointXYZHSV>::Ptr laserCloudExtreUnsel(new pcl::PointCloud<pcl::PointXYZHSV>());
//pcl::PointCloud<pcl::PointXYZHSV>::Ptr laserCloudExtreProj(new pcl::PointCloud<pcl::PointXYZHSV>());
pcl::PointCloud<pcl::PointXYZHSV>::Ptr laserCloudLast(new pcl::PointCloud<pcl::PointXYZHSV>());
pcl::PointCloud<pcl::PointXYZHSV>::Ptr laserCloudCornerLast(new pcl::PointCloud<pcl::PointXYZHSV>());
pcl::PointCloud<pcl::PointXYZHSV>::Ptr laserCloudSurfLast(new pcl::PointCloud<pcl::PointXYZHSV>());
pcl::PointCloud<pcl::PointXYZHSV>::Ptr laserCloudCornerLLast(new pcl::PointCloud<pcl::PointXYZHSV>());
pcl::PointCloud<pcl::PointXYZHSV>::Ptr laserCloudSurfLLast(new pcl::PointCloud<pcl::PointXYZHSV>());
//pcl::PointCloud<pcl::PointXYZHSV>::Ptr laserCloudSel(new pcl::PointCloud<pcl::PointXYZHSV>());
pcl::PointCloud<pcl::PointXYZHSV>::Ptr coeffSel(new pcl::PointCloud<pcl::PointXYZHSV>());
pcl::KdTreeFLANN<pcl::PointXYZHSV>::Ptr kdtreeCornerLast(new pcl::KdTreeFLANN<pcl::PointXYZHSV>());
pcl::KdTreeFLANN<pcl::PointXYZHSV>::Ptr kdtreeSurfLast(new pcl::KdTreeFLANN<pcl::PointXYZHSV>());
pcl::KdTreeFLANN<pcl::PointXYZHSV>::Ptr kdtreeCornerLLast(new pcl::KdTreeFLANN<pcl::PointXYZHSV>());
pcl::KdTreeFLANN<pcl::PointXYZHSV>::Ptr kdtreeSurfLLast(new pcl::KdTreeFLANN<pcl::PointXYZHSV>());

float transform[6] = {0};
float transformRec[6] = {0};
float transformSum[6] = {0};

float imuRollStartCur = 0, imuPitchStartCur = 0, imuYawStartCur = 0;
float imuRollCur = 0, imuPitchCur = 0, imuYawCur = 0;
float imuShiftFromStartXCur = 0, imuShiftFromStartYCur = 0, imuShiftFromStartZCur = 0;
float imuVeloFromStartXCur = 0, imuVeloFromStartYCur = 0, imuVeloFromStartZCur = 0;

float imuRollStartLast = 0, imuPitchStartLast = 0, imuYawStartLast = 0;
float imuRollLast = 0, imuPitchLast = 0, imuYawLast = 0;
float imuShiftFromStartXLast = 0, imuShiftFromStartYLast = 0, imuShiftFromStartZLast = 0;
float imuVeloFromStartXLast = 0, imuVeloFromStartYLast = 0, imuVeloFromStartZLast = 0;

bool imuInited = false;

void TransformReset()
{
  for (int i = 0; i < 6; i++) {
    transformRec[i] = transform[i];
    transform[i] = 0;
  }

  transformRec[3] -= imuVeloFromStartXLast * (startTimeCur - startTimeLast);
  transformRec[4] -= imuVeloFromStartYLast * (startTimeCur - startTimeLast);
  transformRec[5] -= imuVeloFromStartZLast * (startTimeCur - startTimeLast);
}

void TransformToStart(pcl::PointXYZHSV *pi, pcl::PointXYZHSV *po, double startTime, double endTime)
{
  float s = (pi->h - startTime) / (endTime - startTime);

  float rx = s * transform[0];
  float ry = s * transform[1];
  float rz = s * transform[2];
  float tx = s * transform[3];
  float ty = s * transform[4];
  float tz = s * transform[5];

  float x1 = cos(rz) * (pi->x - tx) + sin(rz) * (pi->y - ty);
  float y1 = -sin(rz) * (pi->x - tx) + cos(rz) * (pi->y - ty);
  float z1 = (pi->z - tz);

  float x2 = x1;
  float y2 = cos(rx) * y1 + sin(rx) * z1;
  float z2 = -sin(rx) * y1 + cos(rx) * z1;

  po->x = cos(ry) * x2 - sin(ry) * z2;
  po->y = y2;
  po->z = sin(ry) * x2 + cos(ry) * z2;
  po->h = pi->h;
  po->s = pi->s;
  po->v = pi->v;
}

void TransformToEnd(pcl::PointXYZHSV *pi, pcl::PointXYZHSV *po, double startTime, double endTime)
{
  float s = (pi->h - startTime) / (endTime - startTime);

  float rx = s * transform[0];
  float ry = s * transform[1];
  float rz = s * transform[2];
  float tx = s * transform[3];
  float ty = s * transform[4];
  float tz = s * transform[5];

  float x1 = cos(rz) * (pi->x - tx) + sin(rz) * (pi->y - ty);
  float y1 = -sin(rz) * (pi->x - tx) + cos(rz) * (pi->y - ty);
  float z1 = (pi->z - tz);

  float x2 = x1;
  float y2 = cos(rx) * y1 + sin(rx) * z1;
  float z2 = -sin(rx) * y1 + cos(rx) * z1;

  float x3 = cos(ry) * x2 - sin(ry) * z2;
  float y3 = y2;
  float z3 = sin(ry) * x2 + cos(ry) * z2;

  rx = transform[0];
  ry = transform[1];
  rz = transform[2];
  tx = transform[3];
  ty = transform[4];
  tz = transform[5];

  float x4 = cos(ry) * x3 + sin(ry) * z3;
  float y4 = y3;
  float z4 = -sin(ry) * x3 + cos(ry) * z3;

  float x5 = x4;
  float y5 = cos(rx) * y4 - sin(rx) * z4;
  float z5 = sin(rx) * y4 + cos(rx) * z4;

  float x6 = cos(rz) * x5 - sin(rz) * y5 + tx;
  float y6 = sin(rz) * x5 + cos(rz) * y5 + ty;
  float z6 = z5 + tz;

  float x7 = cos(imuRollStartLast) * (x6 - imuShiftFromStartXLast) 
           - sin(imuRollStartLast) * (y6 - imuShiftFromStartYLast);
  float y7 = sin(imuRollStartLast) * (x6 - imuShiftFromStartXLast) 
           + cos(imuRollStartLast) * (y6 - imuShiftFromStartYLast);
  float z7 = z6 - imuShiftFromStartZLast;

  float x8 = x7;
  float y8 = cos(imuPitchStartLast) * y7 - sin(imuPitchStartLast) * z7;
  float z8 = sin(imuPitchStartLast) * y7 + cos(imuPitchStartLast) * z7;

  float x9 = cos(imuYawStartLast) * x8 + sin(imuYawStartLast) * z8;
  float y9 = y8;
  float z9 = -sin(imuYawStartLast) * x8 + cos(imuYawStartLast) * z8;

  float x10 = cos(imuYawLast) * x9 - sin(imuYawLast) * z9;
  float y10 = y9;
  float z10 = sin(imuYawLast) * x9 + cos(imuYawLast) * z9;

  float x11 = x10;
  float y11 = cos(imuPitchLast) * y10 + sin(imuPitchLast) * z10;
  float z11 = -sin(imuPitchLast) * y10 + cos(imuPitchLast) * z10;

  po->x = cos(imuRollLast) * x11 + sin(imuRollLast) * y11;
  po->y = -sin(imuRollLast) * x11 + cos(imuRollLast) * y11;
  po->z = z11;
  po->h = pi->h;
  po->s = pi->s;
  po->v = pi->v;
}

void AccumulateRotation(float cx, float cy, float cz, float lx, float ly, float lz, 
                        float &ox, float &oy, float &oz)
{
  float srx = cos(lx)*cos(cx)*sin(ly)*sin(cz) - cos(cx)*cos(cz)*sin(lx) - cos(lx)*cos(ly)*sin(cx);
  ox = -asin(srx);

  float srycrx = sin(lx)*(cos(cy)*sin(cz) - cos(cz)*sin(cx)*sin(cy)) + cos(lx)*sin(ly)*(cos(cy)*cos(cz) 
               + sin(cx)*sin(cy)*sin(cz)) + cos(lx)*cos(ly)*cos(cx)*sin(cy);
  float crycrx = cos(lx)*cos(ly)*cos(cx)*cos(cy) - cos(lx)*sin(ly)*(cos(cz)*sin(cy) 
               - cos(cy)*sin(cx)*sin(cz)) - sin(lx)*(sin(cy)*sin(cz) + cos(cy)*cos(cz)*sin(cx));
  oy = atan2(srycrx / cos(ox), crycrx / cos(ox));

  float srzcrx = sin(cx)*(cos(lz)*sin(ly) - cos(ly)*sin(lx)*sin(lz)) + cos(cx)*sin(cz)*(cos(ly)*cos(lz) 
               + sin(lx)*sin(ly)*sin(lz)) + cos(lx)*cos(cx)*cos(cz)*sin(lz);
  float crzcrx = cos(lx)*cos(lz)*cos(cx)*cos(cz) - cos(cx)*sin(cz)*(cos(ly)*sin(lz) 
               - cos(lz)*sin(lx)*sin(ly)) - sin(cx)*(sin(ly)*sin(lz) + cos(ly)*cos(lz)*sin(lx));
  oz = atan2(srzcrx / cos(ox), crzcrx / cos(ox));
}

void PluginIMURotation(float bcx, float bcy, float bcz, float blx, float bly, float blz, 
                       float alx, float aly, float alz, float &acx, float &acy, float &acz)
{
  float sbcx = sin(bcx);
  float cbcx = cos(bcx);
  float sbcy = sin(bcy);
  float cbcy = cos(bcy);
  float sbcz = sin(bcz);
  float cbcz = cos(bcz);

  float sblx = sin(blx);
  float cblx = cos(blx);
  float sbly = sin(bly);
  float cbly = cos(bly);
  float sblz = sin(blz);
  float cblz = cos(blz);

  float salx = sin(alx);
  float calx = cos(alx);
  float saly = sin(aly);
  float caly = cos(aly);
  float salz = sin(alz);
  float calz = cos(alz);

  float srx = -sbcx*(salx*sblx + calx*caly*cblx*cbly + calx*cblx*saly*sbly) 
            - cbcx*cbcz*(calx*saly*(cbly*sblz - cblz*sblx*sbly) 
            - calx*caly*(sbly*sblz + cbly*cblz*sblx) + cblx*cblz*salx) 
            - cbcx*sbcz*(calx*caly*(cblz*sbly - cbly*sblx*sblz) 
            - calx*saly*(cbly*cblz + sblx*sbly*sblz) + cblx*salx*sblz);
  acx = -asin(srx);

  float srycrx = (cbcy*sbcz - cbcz*sbcx*sbcy)*(calx*saly*(cbly*sblz - cblz*sblx*sbly) 
               - calx*caly*(sbly*sblz + cbly*cblz*sblx) + cblx*cblz*salx) 
               - (cbcy*cbcz + sbcx*sbcy*sbcz)*(calx*caly*(cblz*sbly - cbly*sblx*sblz) 
               - calx*saly*(cbly*cblz + sblx*sbly*sblz) + cblx*salx*sblz) 
               + cbcx*sbcy*(salx*sblx + calx*caly*cblx*cbly + calx*cblx*saly*sbly);
  float crycrx = (cbcz*sbcy - cbcy*sbcx*sbcz)*(calx*caly*(cblz*sbly - cbly*sblx*sblz) 
               - calx*saly*(cbly*cblz + sblx*sbly*sblz) + cblx*salx*sblz) 
               - (sbcy*sbcz + cbcy*cbcz*sbcx)*(calx*saly*(cbly*sblz - cblz*sblx*sbly) 
               - calx*caly*(sbly*sblz + cbly*cblz*sblx) + cblx*cblz*salx) 
               + cbcx*cbcy*(salx*sblx + calx*caly*cblx*cbly + calx*cblx*saly*sbly);
  acy = atan2(srycrx / cos(acx), crycrx / cos(acx));
  
  float srzcrx = sbcx*(cblx*cbly*(calz*saly - caly*salx*salz) 
               - cblx*sbly*(caly*calz + salx*saly*salz) + calx*salz*sblx) 
               - cbcx*cbcz*((caly*calz + salx*saly*salz)*(cbly*sblz - cblz*sblx*sbly) 
               + (calz*saly - caly*salx*salz)*(sbly*sblz + cbly*cblz*sblx) 
               - calx*cblx*cblz*salz) + cbcx*sbcz*((caly*calz + salx*saly*salz)*(cbly*cblz 
               + sblx*sbly*sblz) + (calz*saly - caly*salx*salz)*(cblz*sbly - cbly*sblx*sblz) 
               + calx*cblx*salz*sblz);
  float crzcrx = sbcx*(cblx*sbly*(caly*salz - calz*salx*saly) 
               - cblx*cbly*(saly*salz + caly*calz*salx) + calx*calz*sblx) 
               + cbcx*cbcz*((saly*salz + caly*calz*salx)*(sbly*sblz + cbly*cblz*sblx) 
               + (caly*salz - calz*salx*saly)*(cbly*sblz - cblz*sblx*sbly) 
               + calx*calz*cblx*cblz) - cbcx*sbcz*((saly*salz + caly*calz*salx)*(cblz*sbly 
               - cbly*sblx*sblz) + (caly*salz - calz*salx*saly)*(cbly*cblz + sblx*sbly*sblz) 
               - calx*calz*cblx*sblz);
  acz = atan2(srzcrx / cos(acx), crzcrx / cos(acx));
}

void laserCloudExtreCurHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudExtreCur2)
{
  if (!systemInited) {
    initTime = laserCloudExtreCur2->header.stamp.toSec();
    systemInited = true;
  }
  timeLaserCloudExtreCur = laserCloudExtreCur2->header.stamp.toSec();
  timeLasted = timeLaserCloudExtreCur - initTime;

  pcl::PointCloud<pcl::PointXYZHSV>::Ptr laserCloudExtreCur3(new pcl::PointCloud<pcl::PointXYZHSV>());
  pcl::fromROSMsg(*laserCloudExtreCur2, *laserCloudExtreCur3);
  int laserCloudExtreCur3Size = laserCloudExtreCur3->points.size();

  laserCloudExtreCur->clear();
  for (int i = 0; i < laserCloudExtreCur3Size; i++) {
    if (fabs(laserCloudExtreCur3->points[i].v - 10) < 0.005) {
      imuPitchStartCur = laserCloudExtreCur3->points[i].x;
      imuYawStartCur = laserCloudExtreCur3->points[i].y;
      imuRollStartCur = laserCloudExtreCur3->points[i].z;
    } else if (fabs(laserCloudExtreCur3->points[i].v - 11) < 0.005) {
      imuPitchCur = laserCloudExtreCur3->points[i].x;
      imuYawCur = laserCloudExtreCur3->points[i].y;
      imuRollCur = laserCloudExtreCur3->points[i].z;
    } else if (fabs(laserCloudExtreCur3->points[i].v - 12) < 0.005) {
      imuShiftFromStartXCur = laserCloudExtreCur3->points[i].x;
      imuShiftFromStartYCur = laserCloudExtreCur3->points[i].y;
      imuShiftFromStartZCur = laserCloudExtreCur3->points[i].z;
    } else if (fabs(laserCloudExtreCur3->points[i].v - 13) < 0.005) {
      imuVeloFromStartXCur = laserCloudExtreCur3->points[i].x;
      imuVeloFromStartYCur = laserCloudExtreCur3->points[i].y;
      imuVeloFromStartZCur = laserCloudExtreCur3->points[i].z;
    } else {
      laserCloudExtreCur->push_back(laserCloudExtreCur3->points[i]);
    }
  }
  laserCloudExtreCur3->clear();

  if (!imuInited) {
    transformSum[0] += imuPitchStartCur;
    //transformSum[1] += imuYawStartCur;
    transformSum[2] += imuRollStartCur;

    imuInited = true;
  }

  if (timeLasted > 4.0) {
    newLaserCloudExtreCur = true;
  }
}

void laserCloudLastHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudLast2)
{
  if (laserCloudLast2->header.stamp.toSec() > timeLaserCloudLast + 0.005) {
    timeLaserCloudLast = laserCloudLast2->header.stamp.toSec();
    startTimeLast = startTimeCur;
    startTimeCur = timeLaserCloudLast - initTime;

    pcl::PointCloud<pcl::PointXYZHSV>::Ptr laserCloudPointer = laserCloudCornerLLast;
    laserCloudCornerLLast = laserCloudCornerLast;
    laserCloudCornerLast = laserCloudPointer;

    laserCloudPointer = laserCloudSurfLLast;
    laserCloudSurfLLast = laserCloudSurfLast;
    laserCloudSurfLast = laserCloudPointer;

    laserCloudLast->clear();
    pcl::fromROSMsg(*laserCloudLast2, *laserCloudLast);
    int laserCloudLastSize = laserCloudLast->points.size();

    laserCloudExtreLast->clear();
    laserCloudCornerLast->clear();
    laserCloudSurfLast->clear();
    for (int i = 0; i < laserCloudLastSize; i++) {
      if (fabs(laserCloudLast->points[i].v - 2) < 0.005 || fabs(laserCloudLast->points[i].v + 1) < 0.005) {
        laserCloudExtreLast->push_back(laserCloudLast->points[i]);
      } 
      if (fabs(laserCloudLast->points[i].v - 2) < 0.005 || fabs(laserCloudLast->points[i].v - 1) < 0.005) {
        laserCloudCornerLast->push_back(laserCloudLast->points[i]);
      } 
      if (fabs(laserCloudLast->points[i].v) < 0.005 || fabs(laserCloudLast->points[i].v + 1) < 0.005) {
        laserCloudSurfLast->push_back(laserCloudLast->points[i]);
      }
      if (fabs(laserCloudLast->points[i].v - 10) < 0.005) {
        imuPitchStartLast = laserCloudLast->points[i].x;
        imuYawStartLast = laserCloudLast->points[i].y;
        imuRollStartLast = laserCloudLast->points[i].z;
      }
      if (fabs(laserCloudLast->points[i].v - 11) < 0.005) {
        imuPitchLast = laserCloudLast->points[i].x;
        imuYawLast = laserCloudLast->points[i].y;
        imuRollLast = laserCloudLast->points[i].z;
      }
      if (fabs(laserCloudLast->points[i].v - 12) < 0.005) {
        imuShiftFromStartXLast = laserCloudLast->points[i].x;
        imuShiftFromStartYLast = laserCloudLast->points[i].y;
        imuShiftFromStartZLast = laserCloudLast->points[i].z;
      }
      if (fabs(laserCloudLast->points[i].v - 13) < 0.005) {
        imuVeloFromStartXLast = laserCloudLast->points[i].x;
        imuVeloFromStartYLast = laserCloudLast->points[i].y;
        imuVeloFromStartZLast = laserCloudLast->points[i].z;
      }
    }

    pcl::KdTreeFLANN<pcl::PointXYZHSV>::Ptr kdtreePointer = kdtreeCornerLLast;
    kdtreeCornerLLast = kdtreeCornerLast;
    kdtreeCornerLast = kdtreePointer;
    kdtreeCornerLast->setInputCloud(laserCloudCornerLast);

    kdtreePointer = kdtreeSurfLLast;
    kdtreeSurfLLast = kdtreeSurfLast;
    kdtreeSurfLast = kdtreePointer;
    kdtreeSurfLast->setInputCloud(laserCloudSurfLast);

    if (timeLasted > 4.0) {
      newLaserCloudLast = true;
    }
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "laserOdometry");
  ros::NodeHandle nh;

  ros::Subscriber subLaserCloudExtreCur = nh.subscribe<sensor_msgs::PointCloud2> 
                                          ("/laser_cloud_extre_cur", 2, laserCloudExtreCurHandler);

  ros::Subscriber subLaserCloudLast = nh.subscribe<sensor_msgs::PointCloud2> 
                                      ("/laser_cloud_last", 2, laserCloudLastHandler);

  ros::Publisher pubLaserCloudLast2 = nh.advertise<sensor_msgs::PointCloud2> ("/laser_cloud_last_2", 2);

  //ros::Publisher pub1 = nh.advertise<sensor_msgs::PointCloud2> ("/pc1", 1);

  //ros::Publisher pub2 = nh.advertise<sensor_msgs::PointCloud2> ("/pc2", 1);

  //ros::Publisher pub3 = nh.advertise<sensor_msgs::PointCloud2> ("/pc3", 1);

  //ros::Publisher pub4 = nh.advertise<sensor_msgs::PointCloud2> ("/pc4", 1);

  //ros::Publisher pub5 = nh.advertise<sensor_msgs::PointCloud2> ("/pc5", 1);

  //ros::Publisher pub6 = nh.advertise<sensor_msgs::PointCloud2> ("/pc6", 1);

  ros::Publisher pubLaserOdometry = nh.advertise<nav_msgs::Odometry> ("/cam_to_init", 5);
  nav_msgs::Odometry laserOdometry;
  laserOdometry.header.frame_id = "/camera_init";
  laserOdometry.child_frame_id = "/camera";

  tf::TransformBroadcaster tfBroadcaster;
  tf::StampedTransform laserOdometryTrans;
  laserOdometryTrans.frame_id_ = "/camera_init";
  laserOdometryTrans.child_frame_id_ = "/camera";

  std::vector<int> pointSearchInd;
  std::vector<float> pointSearchSqDis;
  std::vector<int> pointSelInd;

  pcl::PointXYZHSV extreOri, extreSel, extreProj, tripod1, tripod2, tripod3, coeff;

  bool status = ros::ok();
  while (status) {
    ros::spinOnce();

    bool sweepEnd = false;
    bool newLaserPoints = false;
    bool sufficientPoints = false;
    double startTime, endTime;
    pcl::PointCloud<pcl::PointXYZHSV>::Ptr extrePointsPtr, laserCloudCornerPtr, laserCloudSurfPtr;
    pcl::KdTreeFLANN<pcl::PointXYZHSV>::Ptr kdtreeCornerPtr, kdtreeSurfPtr;
    if (newLaserCloudExtreCur && newLaserCloudLast) {

      startTime = startTimeLast;
      endTime = startTimeCur;

      extrePointsPtr = laserCloudExtreLast;
      laserCloudCornerPtr = laserCloudCornerLLast;
      laserCloudSurfPtr = laserCloudSurfLLast;
      kdtreeCornerPtr = kdtreeCornerLLast;
      kdtreeSurfPtr = kdtreeSurfLLast;

      laserOdometry.header.stamp = ros::Time().fromSec(timeLaserCloudLast);
      laserOdometryTrans.stamp_ = ros::Time().fromSec(timeLaserCloudLast);

      sweepEnd = true;
      newLaserPoints = true;

      if (laserCloudSurfLLast->points.size() >= 100) {
      	sufficientPoints = true;
      }

    } else if (newLaserCloudExtreCur) {

      startTime = startTimeCur;
      endTime = timeLasted;

      extrePointsPtr = laserCloudExtreCur;
      laserCloudCornerPtr = laserCloudCornerLast;
      laserCloudSurfPtr = laserCloudSurfLast;
      kdtreeCornerPtr = kdtreeCornerLast;
      kdtreeSurfPtr = kdtreeSurfLast;

      laserOdometry.header.stamp = ros::Time().fromSec(timeLaserCloudExtreCur);
      laserOdometryTrans.stamp_ = ros::Time().fromSec(timeLaserCloudExtreCur);

      float s = (timeLasted - timeLastedRec) / (startTimeCur - startTimeLast);
      for (int i = 0; i < 6; i++) {
        transform[i] += s * transformRec[i];        
      }
      timeLastedRec = timeLasted;

      newLaserPoints = true;

      if (laserCloudSurfLast->points.size() >= 100) {
        sufficientPoints = true;
      }
    }

    if (newLaserPoints && sufficientPoints) {
      newLaserCloudExtreCur = false;
      newLaserCloudLast = false;

      int extrePointNum = extrePointsPtr->points.size();
      int laserCloudCornerNum = laserCloudCornerPtr->points.size();
      int laserCloudSurfNum = laserCloudSurfPtr->points.size();

      float st = 1;
      if (!sweepEnd) {
        st = (timeLasted - startTime) / (startTimeCur - startTimeLast);
      }
      int iterNum = st * 50;

      int pointSelSkipNum = 2;
      for (int iterCount = 0; iterCount < iterNum; iterCount++) {

        laserCloudExtreOri->clear();
        //laserCloudExtreSel->clear();
        //laserCloudExtreUnsel->clear();
        //laserCloudExtreProj->clear();
        //laserCloudSel->clear();
        coeffSel->clear();

        bool isPointSel = false;
        if (iterCount % (pointSelSkipNum + 1) == 0) {
          isPointSel = true;
          pointSelInd.clear();
        }

        for (int i = 0; i < extrePointNum; i++) {
          extreOri = extrePointsPtr->points[i];
          TransformToStart(&extreOri, &extreSel, startTime, endTime);

          if (isPointSel) {
            pointSelInd.push_back(-1);
            pointSelInd.push_back(-1);
            pointSelInd.push_back(-1);
          }

          if (fabs(extreOri.v + 1) < 0.05) {

            int closestPointInd = -1, minPointInd2 = -1, minPointInd3 = -1;
            if (isPointSel) {
              kdtreeSurfPtr->nearestKSearch(extreSel, 1, pointSearchInd, pointSearchSqDis);
              if (pointSearchSqDis[0] > 1.0) {
                continue;
              }

              closestPointInd = pointSearchInd[0];
              float closestPointTime = laserCloudSurfPtr->points[closestPointInd].h;

              float pointSqDis, minPointSqDis2 = 1, minPointSqDis3 = 1;
              for (int j = closestPointInd + 1; j < laserCloudSurfNum; j++) {
                if (laserCloudSurfPtr->points[j].h > closestPointTime + 0.07) {
                  break;
                }

                pointSqDis = (laserCloudSurfPtr->points[j].x - extreSel.x) * 
                             (laserCloudSurfPtr->points[j].x - extreSel.x) + 
                             (laserCloudSurfPtr->points[j].y - extreSel.y) * 
                             (laserCloudSurfPtr->points[j].y - extreSel.y) + 
                             (laserCloudSurfPtr->points[j].z - extreSel.z) * 
                             (laserCloudSurfPtr->points[j].z - extreSel.z);

                if (laserCloudSurfPtr->points[j].h < closestPointTime + 0.005) {
                   if (pointSqDis < minPointSqDis2) {
                     minPointSqDis2 = pointSqDis;
                     minPointInd2 = j;
                   }
                } else {
                   if (pointSqDis < minPointSqDis3) {
                     minPointSqDis3 = pointSqDis;
                     minPointInd3 = j;
                   }
                }
              }
              for (int j = closestPointInd - 1; j >= 0; j--) {
                if (laserCloudSurfPtr->points[j].h < closestPointTime - 0.07) {
                  break;
                }

                pointSqDis = (laserCloudSurfPtr->points[j].x - extreSel.x) * 
                             (laserCloudSurfPtr->points[j].x - extreSel.x) + 
                             (laserCloudSurfPtr->points[j].y - extreSel.y) * 
                             (laserCloudSurfPtr->points[j].y - extreSel.y) + 
                             (laserCloudSurfPtr->points[j].z - extreSel.z) * 
                             (laserCloudSurfPtr->points[j].z - extreSel.z);

                if (laserCloudSurfPtr->points[j].h > closestPointTime - 0.005) {
                   if (pointSqDis < minPointSqDis2) {
                     minPointSqDis2 = pointSqDis;
                     minPointInd2 = j;
                   }
                } else {
                   if (pointSqDis < minPointSqDis3) {
                     minPointSqDis3 = pointSqDis;
                     minPointInd3 = j;
                   }
                }
              }
            } else {
              if (pointSelInd[3 * i] >= 0) {
                closestPointInd = pointSelInd[3 * i];
                minPointInd2 = pointSelInd[3 * i + 1];
                minPointInd3 = pointSelInd[3 * i + 2];

                float distX = extreSel.x - laserCloudSurfPtr->points[closestPointInd].x;
                float distY = extreSel.y - laserCloudSurfPtr->points[closestPointInd].y;
                float distZ = extreSel.z - laserCloudSurfPtr->points[closestPointInd].z;
                if (distX * distX + distY * distY + distZ * distZ > 1.0) {
                  continue;
                }
              } else {
                continue;
              }
            }

            if (minPointInd2 >= 0 && minPointInd3 >= 0) {
              tripod1 = laserCloudSurfPtr->points[closestPointInd];
              tripod2 = laserCloudSurfPtr->points[minPointInd2];
              tripod3 = laserCloudSurfPtr->points[minPointInd3];

              float pa = (tripod2.y - tripod1.y) * (tripod3.z - tripod1.z) 
                       - (tripod3.y - tripod1.y) * (tripod2.z - tripod1.z);
              float pb = (tripod2.z - tripod1.z) * (tripod3.x - tripod1.x) 
                       - (tripod3.z - tripod1.z) * (tripod2.x - tripod1.x);
              float pc = (tripod2.x - tripod1.x) * (tripod3.y - tripod1.y) 
                       - (tripod3.x - tripod1.x) * (tripod2.y - tripod1.y);
              float pd = -(pa * tripod1.x + pb * tripod1.y + pc * tripod1.z);

              float ps = sqrt(pa * pa + pb * pb + pc * pc);
              pa /= ps;
              pb /= ps;
              pc /= ps;
              pd /= ps;

              float pd2 = pa * extreSel.x + pb * extreSel.y + pc * extreSel.z + pd;

              extreProj = extreSel;
              extreProj.x -= pa * pd2;
              extreProj.y -= pb * pd2;
              extreProj.z -= pc * pd2;

              float s = 1;
              if (iterCount >= 30) {
                s = 1 - 8 * fabs(pd2) / sqrt(sqrt(extreSel.x * extreSel.x
                  + extreSel.y * extreSel.y + extreSel.z * extreSel.z));
              }

              coeff.x = s * pa;
              coeff.y = s * pb;
              coeff.z = s * pc;
              coeff.h = s * pd2;

              if (s > 0.2 || iterNum < 30) {
                laserCloudExtreOri->push_back(extreOri);
                //laserCloudExtreSel->push_back(extreSel);
                //laserCloudExtreProj->push_back(extreProj);
                //laserCloudSel->push_back(tripod1);
                //laserCloudSel->push_back(tripod2);
                //laserCloudSel->push_back(tripod3);
                coeffSel->push_back(coeff);

                if (isPointSel) {
                  pointSelInd[3 * i] = closestPointInd;
                  pointSelInd[3 * i + 1] = minPointInd2;
                  pointSelInd[3 * i + 2] = minPointInd3;
                }
              } else {
                //laserCloudExtreUnsel->push_back(extreSel);
              }
            }
          } else if (fabs(extreOri.v - 2) < 0.05) {

            int closestPointInd = -1, minPointInd2 = -1;
            if (isPointSel) {
              kdtreeCornerPtr->nearestKSearch(extreSel, 1, pointSearchInd, pointSearchSqDis);
              if (pointSearchSqDis[0] > 1.0) {
                continue;
              }

              closestPointInd = pointSearchInd[0];
              float closestPointTime = laserCloudCornerPtr->points[closestPointInd].h;

              float pointSqDis, minPointSqDis2 = 1;
              for (int j = closestPointInd + 1; j < laserCloudCornerNum; j++) {
                if (laserCloudCornerPtr->points[j].h > closestPointTime + 0.07) {
                  break;
                }

                pointSqDis = (laserCloudCornerPtr->points[j].x - extreSel.x) * 
                             (laserCloudCornerPtr->points[j].x - extreSel.x) + 
                             (laserCloudCornerPtr->points[j].y - extreSel.y) * 
                             (laserCloudCornerPtr->points[j].y - extreSel.y) + 
                             (laserCloudCornerPtr->points[j].z - extreSel.z) * 
                             (laserCloudCornerPtr->points[j].z - extreSel.z);

                if (laserCloudCornerPtr->points[j].h > closestPointTime + 0.005) {
                   if (pointSqDis < minPointSqDis2) {
                     minPointSqDis2 = pointSqDis;
                     minPointInd2 = j;
                   }
                }
              }
              for (int j = closestPointInd - 1; j >= 0; j--) {
                if (laserCloudCornerPtr->points[j].h < closestPointTime - 0.07) {
                  break;
                }

                pointSqDis = (laserCloudCornerPtr->points[j].x - extreSel.x) * 
                             (laserCloudCornerPtr->points[j].x - extreSel.x) + 
                             (laserCloudCornerPtr->points[j].y - extreSel.y) * 
                             (laserCloudCornerPtr->points[j].y - extreSel.y) + 
                             (laserCloudCornerPtr->points[j].z - extreSel.z) * 
                             (laserCloudCornerPtr->points[j].z - extreSel.z);

                if (laserCloudCornerPtr->points[j].h < closestPointTime - 0.005) {
                   if (pointSqDis < minPointSqDis2) {
                     minPointSqDis2 = pointSqDis;
                     minPointInd2 = j;
                   }
                }
              }
            } else {
              if (pointSelInd[3 * i] >= 0) {
                closestPointInd = pointSelInd[3 * i];
                minPointInd2 = pointSelInd[3 * i + 1];

                float distX = extreSel.x - laserCloudCornerPtr->points[closestPointInd].x;
                float distY = extreSel.y - laserCloudCornerPtr->points[closestPointInd].y;
                float distZ = extreSel.z - laserCloudCornerPtr->points[closestPointInd].z;
                if (distX * distX + distY * distY + distZ * distZ > 1.0) {
                  continue;
                }
              } else {
                continue;
              }
            }

            if (minPointInd2 >= 0) {
              tripod1 = laserCloudCornerPtr->points[closestPointInd];
              tripod2 = laserCloudCornerPtr->points[minPointInd2];

              float x0 = extreSel.x;
              float y0 = extreSel.y;
              float z0 = extreSel.z;
              float x1 = tripod1.x;
              float y1 = tripod1.y;
              float z1 = tripod1.z;
              float x2 = tripod2.x;
              float y2 = tripod2.y;
              float z2 = tripod2.z;

              float a012 = sqrt(((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1))
                        * ((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1)) 
                        + ((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1))
                        * ((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1)) 
                        + ((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1))
                        * ((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1)));

              float l12 = sqrt((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2) + (z1 - z2)*(z1 - z2));

              float la = ((y1 - y2)*((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1)) 
                       + (z1 - z2)*((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1))) / a012 / l12;

              float lb = -((x1 - x2)*((x0 - x1)*(y0 - y2) - (x0 - x2)*(y0 - y1)) 
                       - (z1 - z2)*((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1))) / a012 / l12;

              float lc = -((x1 - x2)*((x0 - x1)*(z0 - z2) - (x0 - x2)*(z0 - z1)) 
                       + (y1 - y2)*((y0 - y1)*(z0 - z2) - (y0 - y2)*(z0 - z1))) / a012 / l12;

              float ld2 = a012 / l12;

              extreProj = extreSel;
              extreProj.x -= la * ld2;
              extreProj.y -= lb * ld2;
              extreProj.z -= lc * ld2;

              float s = 2 * (1 - 8 * fabs(ld2));

              coeff.x = s * la;
              coeff.y = s * lb;
              coeff.z = s * lc;
              coeff.h = s * ld2;

              if (s > 0.4) {
                laserCloudExtreOri->push_back(extreOri);
                //laserCloudExtreSel->push_back(extreSel);
                //laserCloudExtreProj->push_back(extreProj);
                //laserCloudSel->push_back(tripod1);
                //laserCloudSel->push_back(tripod2);
                coeffSel->push_back(coeff);

                if (isPointSel) {
                  pointSelInd[3 * i] = closestPointInd;
                  pointSelInd[3 * i + 1] = minPointInd2;
                }
              } else {
                //laserCloudExtreUnsel->push_back(extreSel);
              }
            }
          }
        }
        int extrePointSelNum = laserCloudExtreOri->points.size();

        if (extrePointSelNum < 10) {
          continue;
        }

        cv::Mat matA(extrePointSelNum, 6, CV_32F, cv::Scalar::all(0));
        cv::Mat matAt(6, extrePointSelNum, CV_32F, cv::Scalar::all(0));
        cv::Mat matAtA(6, 6, CV_32F, cv::Scalar::all(0));
        cv::Mat matB(extrePointSelNum, 1, CV_32F, cv::Scalar::all(0));
        cv::Mat matAtB(6, 1, CV_32F, cv::Scalar::all(0));
        cv::Mat matX(6, 1, CV_32F, cv::Scalar::all(0));
        for (int i = 0; i < extrePointSelNum; i++) {
          extreOri = laserCloudExtreOri->points[i];
          coeff = coeffSel->points[i];

          float s = (extreOri.h - startTime) / (endTime - startTime);

          float srx = sin(s * transform[0]);
          float crx = cos(s * transform[0]);
          float sry = sin(s * transform[1]);
          float cry = cos(s * transform[1]);
          float srz = sin(s * transform[2]);
          float crz = cos(s * transform[2]);
          float tx = s * transform[3];
          float ty = s * transform[4];
          float tz = s * transform[5];

          float arx = (-s*crx*sry*srz*extreOri.x + s*crx*crz*sry*extreOri.y + s*srx*sry*extreOri.z 
                    + s*tx*crx*sry*srz - s*ty*crx*crz*sry - s*tz*srx*sry) * coeff.x
                    + (s*srx*srz*extreOri.x - s*crz*srx*extreOri.y + s*crx*extreOri.z
                    + s*ty*crz*srx - s*tz*crx - s*tx*srx*srz) * coeff.y
                    + (s*crx*cry*srz*extreOri.x - s*crx*cry*crz*extreOri.y - s*cry*srx*extreOri.z
                    + s*tz*cry*srx + s*ty*crx*cry*crz - s*tx*crx*cry*srz) * coeff.z;

          float ary = ((-s*crz*sry - s*cry*srx*srz)*extreOri.x 
                    + (s*cry*crz*srx - s*sry*srz)*extreOri.y - s*crx*cry*extreOri.z 
                    + tx*(s*crz*sry + s*cry*srx*srz) + ty*(s*sry*srz - s*cry*crz*srx) 
                    + s*tz*crx*cry) * coeff.x
                    + ((s*cry*crz - s*srx*sry*srz)*extreOri.x 
                    + (s*cry*srz + s*crz*srx*sry)*extreOri.y - s*crx*sry*extreOri.z
                    + s*tz*crx*sry - ty*(s*cry*srz + s*crz*srx*sry) 
                    - tx*(s*cry*crz - s*srx*sry*srz)) * coeff.z;

          float arz = ((-s*cry*srz - s*crz*srx*sry)*extreOri.x + (s*cry*crz - s*srx*sry*srz)*extreOri.y
                    + tx*(s*cry*srz + s*crz*srx*sry) - ty*(s*cry*crz - s*srx*sry*srz)) * coeff.x
                    + (-s*crx*crz*extreOri.x - s*crx*srz*extreOri.y
                    + s*ty*crx*srz + s*tx*crx*crz) * coeff.y
                    + ((s*cry*crz*srx - s*sry*srz)*extreOri.x + (s*crz*sry + s*cry*srx*srz)*extreOri.y
                    + tx*(s*sry*srz - s*cry*crz*srx) - ty*(s*crz*sry + s*cry*srx*srz)) * coeff.z;

          float atx = -s*(cry*crz - srx*sry*srz) * coeff.x + s*crx*srz * coeff.y 
                    - s*(crz*sry + cry*srx*srz) * coeff.z;
  
          float aty = -s*(cry*srz + crz*srx*sry) * coeff.x - s*crx*crz * coeff.y 
                    - s*(sry*srz - cry*crz*srx) * coeff.z;
  
          float atz = s*crx*sry * coeff.x - s*srx * coeff.y - s*crx*cry * coeff.z;
  
          float d2 = coeff.h;

          matA.at<float>(i, 0) = arx;
          matA.at<float>(i, 1) = ary;
          matA.at<float>(i, 2) = arz;
          matA.at<float>(i, 3) = atx;
          matA.at<float>(i, 4) = aty;
          matA.at<float>(i, 5) = atz;
          matB.at<float>(i, 0) = -0.015 * st * d2;
        }
        cv::transpose(matA, matAt);
        matAtA = matAt * matA; //+ 0.1 * cv::Mat::eye(6, 6, CV_32F);
        matAtB = matAt * matB;
        cv::solve(matAtA, matAtB, matX, cv::DECOMP_QR);
        //cv::solve(matA, matB, matX, cv::DECOMP_SVD);

        if (fabs(matX.at<float>(0, 0)) < 0.005 &&
            fabs(matX.at<float>(1, 0)) < 0.005 &&
            fabs(matX.at<float>(2, 0)) < 0.005 &&
            fabs(matX.at<float>(3, 0)) < 0.01 &&
            fabs(matX.at<float>(4, 0)) < 0.01 &&
            fabs(matX.at<float>(5, 0)) < 0.01) {

          //transform[0] += 0.7 * matX.at<float>(0, 0);
          //transform[1] += 0.7 * matX.at<float>(1, 0);
          //transform[2] += 0.7 * matX.at<float>(2, 0);
          transform[0] += 0.1 * matX.at<float>(0, 0);
          transform[1] += 0.1 * matX.at<float>(1, 0);
          transform[2] += 0.1 * matX.at<float>(2, 0);
          transform[3] += matX.at<float>(3, 0);
          transform[4] += matX.at<float>(4, 0);
          transform[5] += matX.at<float>(5, 0);
        } else {
          //ROS_INFO ("Odometry update out of bound");
        }
      }

      /*sensor_msgs::PointCloud2 pc12;
      pcl::toROSMsg(*laserCloudCornerPtr, pc12);
      pc12.header.stamp = ros::Time().fromSec(timeLaserCloudExtreCur);
      pc12.header.frame_id = "/camera";
      pub1.publish(pc12);

      sensor_msgs::PointCloud2 pc22;
      pcl::toROSMsg(*laserCloudSurfPtr, pc22);
      pc22.header.stamp = ros::Time().fromSec(timeLaserCloudExtreCur);
      pc22.header.frame_id = "/camera";
      pub2.publish(pc22);

      sensor_msgs::PointCloud2 pc32;
      pcl::toROSMsg(*laserCloudExtreSel, pc32);
      pc32.header.stamp = ros::Time().fromSec(timeLaserCloudExtreCur);
      pc32.header.frame_id = "/camera";
      pub3.publish(pc32);

      sensor_msgs::PointCloud2 pc42;
      pcl::toROSMsg(*laserCloudExtreUnsel, pc42);
      pc42.header.stamp = ros::Time().fromSec(timeLaserCloudExtreCur);
      pc42.header.frame_id = "/camera";
      pub4.publish(pc42);

      sensor_msgs::PointCloud2 pc52;
      pcl::toROSMsg(*laserCloudExtreProj, pc52);
      pc52.header.stamp = ros::Time().fromSec(timeLaserCloudExtreCur);
      pc52.header.frame_id = "/camera";
      pub5.publish(pc52);

      sensor_msgs::PointCloud2 pc62;
      pcl::toROSMsg(*laserCloudSel, pc62);
      pc62.header.stamp = ros::Time().fromSec(timeLaserCloudExtreCur);
      pc62.header.frame_id = "/camera";
      pub6.publish(pc62);*/
    }

    if (newLaserPoints) {
      float rx, ry, rz, tx, ty, tz;
      if (sweepEnd) {
        AccumulateRotation(transformSum[0], transformSum[1], transformSum[2], 
                           -transform[0], -transform[1] * 1.05, -transform[2], rx, ry, rz);

        float x1 = cos(rz) * (transform[3] - imuShiftFromStartXLast) 
                 - sin(rz) * (transform[4] - imuShiftFromStartYLast);
        float y1 = sin(rz) * (transform[3] - imuShiftFromStartXLast) 
                 + cos(rz) * (transform[4] - imuShiftFromStartYLast);
        float z1 = transform[5] * 1.05 - imuShiftFromStartZLast;

        float x2 = x1;
        float y2 = cos(rx) * y1 - sin(rx) * z1;
        float z2 = sin(rx) * y1 + cos(rx) * z1;

        tx = transformSum[3] - (cos(ry) * x2 + sin(ry) * z2);
        ty = transformSum[4] - y2;
        tz = transformSum[5] - (-sin(ry) * x2 + cos(ry) * z2);

        PluginIMURotation(rx, ry, rz, imuPitchStartLast, imuYawStartLast, imuRollStartLast, 
                          imuPitchLast, imuYawLast, imuRollLast, rx, ry, rz);

        int laserCloudCornerLastNum = laserCloudCornerLast->points.size();
        for (int i = 0; i < laserCloudCornerLastNum; i++) {
          TransformToEnd(&laserCloudCornerLast->points[i], &laserCloudCornerLast->points[i], 
                         startTimeLast, startTimeCur);
        }

        int laserCloudSurfLastNum = laserCloudSurfLast->points.size();
        for (int i = 0; i < laserCloudSurfLastNum; i++) {
          TransformToEnd(&laserCloudSurfLast->points[i], &laserCloudSurfLast->points[i], 
                         startTimeLast, startTimeCur);
        }

        TransformReset();

        transformSum[0] = rx;
        transformSum[1] = ry;
        transformSum[2] = rz;
        transformSum[3] = tx;
        transformSum[4] = ty;
        transformSum[5] = tz;

        sensor_msgs::PointCloud2 laserCloudLast2;
        pcl::toROSMsg(*laserCloudCornerLast + *laserCloudSurfLast, laserCloudLast2);
        laserCloudLast2.header.stamp = ros::Time().fromSec(timeLaserCloudLast);
        laserCloudLast2.header.frame_id = "/camera";
        pubLaserCloudLast2.publish(laserCloudLast2);

      } else {
        AccumulateRotation(transformSum[0], transformSum[1], transformSum[2], 
                           -transform[0], -transform[1] * 1.05, -transform[2], rx, ry, rz);

        float x1 = cos(rz) * (transform[3] - imuShiftFromStartXCur) 
                 - sin(rz) * (transform[4] - imuShiftFromStartYCur);
        float y1 = sin(rz) * (transform[3] - imuShiftFromStartXCur) 
                 + cos(rz) * (transform[4] - imuShiftFromStartYCur);
        float z1 = transform[5] * 1.05 - imuShiftFromStartZCur;

        float x2 = x1;
        float y2 = cos(rx) * y1 - sin(rx) * z1;
        float z2 = sin(rx) * y1 + cos(rx) * z1;

        tx = transformSum[3] - (cos(ry) * x2 + sin(ry) * z2);
        ty = transformSum[4] - y2;
        tz = transformSum[5] - (-sin(ry) * x2 + cos(ry) * z2);

        PluginIMURotation(rx, ry, rz, imuPitchStartCur, imuYawStartCur, imuRollStartCur, 
                          imuPitchCur, imuYawCur, imuRollCur, rx, ry, rz);
      }

      geometry_msgs::Quaternion geoQuat = tf::createQuaternionMsgFromRollPitchYaw(rz, -rx, -ry);

      laserOdometry.pose.pose.orientation.x = -geoQuat.y;
      laserOdometry.pose.pose.orientation.y = -geoQuat.z;
      laserOdometry.pose.pose.orientation.z = geoQuat.x;
      laserOdometry.pose.pose.orientation.w = geoQuat.w;
      laserOdometry.pose.pose.position.x = tx;
      laserOdometry.pose.pose.position.y = ty;
      laserOdometry.pose.pose.position.z = tz;
      pubLaserOdometry.publish(laserOdometry);

      laserOdometryTrans.setRotation(tf::Quaternion(-geoQuat.y, -geoQuat.z, geoQuat.x, geoQuat.w));
      laserOdometryTrans.setOrigin(tf::Vector3(tx, ty, tz));
      tfBroadcaster.sendTransform(laserOdometryTrans);

      //ROS_INFO ("%f %f %f %f %f %f", transformSum[0], transformSum[1], transformSum[2], 
      //                               transformSum[3], transformSum[4], transformSum[5]);
    }

    status = ros::ok();
    cv::waitKey(10);
  }

  return 0;
}
