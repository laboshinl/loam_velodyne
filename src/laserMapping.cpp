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

double timeLaserCloudLast;
double timeLaserOdometry;

bool newLaserCloudLast = false;
bool newLaserOdometry = false;

const int laserCloudCenWidth = 5;
const int laserCloudCenHeight = 5;
const int laserCloudCenDepth = 5;
const int laserCloudWidth = 11;
const int laserCloudHeight = 11;
const int laserCloudDepth = 11;
const int laserCloudNum = laserCloudWidth * laserCloudHeight * laserCloudDepth;

int laserCloudValidInd[27];
int laserCloudSurroundInd[27];

pcl::PointCloud<pcl::PointXYZHSV>::Ptr laserCloudCorner(new pcl::PointCloud<pcl::PointXYZHSV>());
pcl::PointCloud<pcl::PointXYZHSV>::Ptr laserCloudSurf(new pcl::PointCloud<pcl::PointXYZHSV>());
pcl::PointCloud<pcl::PointXYZHSV>::Ptr laserCloudCorner2(new pcl::PointCloud<pcl::PointXYZHSV>());
pcl::PointCloud<pcl::PointXYZHSV>::Ptr laserCloudSurf2(new pcl::PointCloud<pcl::PointXYZHSV>());
pcl::PointCloud<pcl::PointXYZHSV>::Ptr laserCloudLast(new pcl::PointCloud<pcl::PointXYZHSV>());
pcl::PointCloud<pcl::PointXYZHSV>::Ptr laserCloudOri(new pcl::PointCloud<pcl::PointXYZHSV>());
//pcl::PointCloud<pcl::PointXYZHSV>::Ptr laserCloudSel(new pcl::PointCloud<pcl::PointXYZHSV>());
//pcl::PointCloud<pcl::PointXYZHSV>::Ptr laserCloudCorr(new pcl::PointCloud<pcl::PointXYZHSV>());
//pcl::PointCloud<pcl::PointXYZHSV>::Ptr laserCloudProj(new pcl::PointCloud<pcl::PointXYZHSV>());
pcl::PointCloud<pcl::PointXYZHSV>::Ptr coeffSel(new pcl::PointCloud<pcl::PointXYZHSV>());
pcl::PointCloud<pcl::PointXYZHSV>::Ptr laserCloudFromMap(new pcl::PointCloud<pcl::PointXYZHSV>());
pcl::PointCloud<pcl::PointXYZHSV>::Ptr laserCloudCornerFromMap(new pcl::PointCloud<pcl::PointXYZHSV>());
pcl::PointCloud<pcl::PointXYZHSV>::Ptr laserCloudSurfFromMap(new pcl::PointCloud<pcl::PointXYZHSV>());
pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudSurround(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZHSV>::Ptr laserCloudArray[laserCloudNum];

pcl::KdTreeFLANN<pcl::PointXYZHSV>::Ptr kdtreeCornerFromMap(new pcl::KdTreeFLANN<pcl::PointXYZHSV>());
pcl::KdTreeFLANN<pcl::PointXYZHSV>::Ptr kdtreeSurfFromMap(new pcl::KdTreeFLANN<pcl::PointXYZHSV>());

float transformSum[6] = {0};
float transformIncre[6] = {0};
float transformTobeMapped[6] = {0};
float transformBefMapped[6] = {0};
float transformAftMapped[6] = {0};

void transformAssociateToMap()
{
  float x1 = cos(transformSum[1]) * (transformBefMapped[3] - transformSum[3]) 
           - sin(transformSum[1]) * (transformBefMapped[5] - transformSum[5]);
  float y1 = transformBefMapped[4] - transformSum[4];
  float z1 = sin(transformSum[1]) * (transformBefMapped[3] - transformSum[3]) 
           + cos(transformSum[1]) * (transformBefMapped[5] - transformSum[5]);

  float x2 = x1;
  float y2 = cos(transformSum[0]) * y1 + sin(transformSum[0]) * z1;
  float z2 = -sin(transformSum[0]) * y1 + cos(transformSum[0]) * z1;

  transformIncre[3] = cos(transformSum[2]) * x2 + sin(transformSum[2]) * y2;
  transformIncre[4] = -sin(transformSum[2]) * x2 + cos(transformSum[2]) * y2;
  transformIncre[5] = z2;

  float sbcx = sin(transformSum[0]);
  float cbcx = cos(transformSum[0]);
  float sbcy = sin(transformSum[1]);
  float cbcy = cos(transformSum[1]);
  float sbcz = sin(transformSum[2]);
  float cbcz = cos(transformSum[2]);

  float sblx = sin(transformBefMapped[0]);
  float cblx = cos(transformBefMapped[0]);
  float sbly = sin(transformBefMapped[1]);
  float cbly = cos(transformBefMapped[1]);
  float sblz = sin(transformBefMapped[2]);
  float cblz = cos(transformBefMapped[2]);

  float salx = sin(transformAftMapped[0]);
  float calx = cos(transformAftMapped[0]);
  float saly = sin(transformAftMapped[1]);
  float caly = cos(transformAftMapped[1]);
  float salz = sin(transformAftMapped[2]);
  float calz = cos(transformAftMapped[2]);

  float srx = -sbcx*(salx*sblx + calx*caly*cblx*cbly + calx*cblx*saly*sbly) 
            - cbcx*cbcz*(calx*saly*(cbly*sblz - cblz*sblx*sbly) 
            - calx*caly*(sbly*sblz + cbly*cblz*sblx) + cblx*cblz*salx) 
            - cbcx*sbcz*(calx*caly*(cblz*sbly - cbly*sblx*sblz) 
            - calx*saly*(cbly*cblz + sblx*sbly*sblz) + cblx*salx*sblz);
  transformTobeMapped[0] = -asin(srx);

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
  transformTobeMapped[1] = atan2(srycrx / cos(transformTobeMapped[0]), 
                                 crycrx / cos(transformTobeMapped[0]));
  
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
  transformTobeMapped[2] = atan2(srzcrx / cos(transformTobeMapped[0]), 
                                 crzcrx / cos(transformTobeMapped[0]));

  x1 = cos(transformTobeMapped[2]) * transformIncre[3] - sin(transformTobeMapped[2]) * transformIncre[4];
  y1 = sin(transformTobeMapped[2]) * transformIncre[3] + cos(transformTobeMapped[2]) * transformIncre[4];
  z1 = transformIncre[5];

  x2 = x1;
  y2 = cos(transformTobeMapped[0]) * y1 - sin(transformTobeMapped[0]) * z1;
  z2 = sin(transformTobeMapped[0]) * y1 + cos(transformTobeMapped[0]) * z1;

  transformTobeMapped[3] = transformAftMapped[3] 
                         - (cos(transformTobeMapped[1]) * x2 + sin(transformTobeMapped[1]) * z2);
  transformTobeMapped[4] = transformAftMapped[4] - y2;
  transformTobeMapped[5] = transformAftMapped[5] 
                         - (-sin(transformTobeMapped[1]) * x2 + cos(transformTobeMapped[1]) * z2);
}

void transformUpdate()
{
  //for (int i = 0; i < 3; i++) {
  //  transformTobeMapped[i] = (1 - 0.1) * transformTobeMapped[i] + 0.1 * transformSum[i];
  //}

  for (int i = 0; i < 6; i++) {
    transformBefMapped[i] = transformSum[i];
    transformAftMapped[i] = transformTobeMapped[i];
  }
}

void pointAssociateToMap(pcl::PointXYZHSV *pi, pcl::PointXYZHSV *po)
{
  float x1 = cos(transformTobeMapped[2]) * pi->x
           - sin(transformTobeMapped[2]) * pi->y;
  float y1 = sin(transformTobeMapped[2]) * pi->x
           + cos(transformTobeMapped[2]) * pi->y;
  float z1 = pi->z;

  float x2 = x1;
  float y2 = cos(transformTobeMapped[0]) * y1 - sin(transformTobeMapped[0]) * z1;
  float z2 = sin(transformTobeMapped[0]) * y1 + cos(transformTobeMapped[0]) * z1;

  po->x = cos(transformTobeMapped[1]) * x2 + sin(transformTobeMapped[1]) * z2
        + transformTobeMapped[3];
  po->y = y2 + transformTobeMapped[4];
  po->z = -sin(transformTobeMapped[1]) * x2 + cos(transformTobeMapped[1]) * z2
        + transformTobeMapped[5];
  po->h = pi->h;
  po->s = pi->s;
  po->v = pi->v;
}

void laserCloudLastHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudLast2)
{
  timeLaserCloudLast = laserCloudLast2->header.stamp.toSec();

  laserCloudLast->clear();
  pcl::fromROSMsg(*laserCloudLast2, *laserCloudLast);

  newLaserCloudLast = true;
}

void laserOdometryHandler(const nav_msgs::Odometry::ConstPtr& laserOdometry)
{
  timeLaserOdometry = laserOdometry->header.stamp.toSec();

  double roll, pitch, yaw;
  geometry_msgs::Quaternion geoQuat = laserOdometry->pose.pose.orientation;
  tf::Matrix3x3(tf::Quaternion(geoQuat.z, -geoQuat.x, -geoQuat.y, geoQuat.w)).getRPY(roll, pitch, yaw);

  transformSum[0] = -pitch;
  transformSum[1] = -yaw;
  transformSum[2] = roll;

  transformSum[3] = laserOdometry->pose.pose.position.x;
  transformSum[4] = laserOdometry->pose.pose.position.y;
  transformSum[5] = laserOdometry->pose.pose.position.z;

  newLaserOdometry = true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "laserMapping");
  ros::NodeHandle nh;

  ros::Subscriber subLaserCloudLast2 = nh.subscribe<sensor_msgs::PointCloud2> 
                                       ("/laser_cloud_last_2", 2, laserCloudLastHandler);

  ros::Subscriber subLaserOdometry = nh.subscribe<nav_msgs::Odometry> 
                                     ("/cam_to_init", 5, laserOdometryHandler);

  ros::Publisher pubLaserCloudSurround = nh.advertise<sensor_msgs::PointCloud2> 
                                         ("/laser_cloud_surround", 1);

  //ros::Publisher pub1 = nh.advertise<sensor_msgs::PointCloud2> ("/pc1", 1);

  //ros::Publisher pub2 = nh.advertise<sensor_msgs::PointCloud2> ("/pc2", 1);

  //ros::Publisher pub3 = nh.advertise<sensor_msgs::PointCloud2> ("/pc3", 1);

  //ros::Publisher pub4 = nh.advertise<sensor_msgs::PointCloud2> ("/pc4", 1);

  ros::Publisher pubOdomBefMapped = nh.advertise<nav_msgs::Odometry> ("/bef_mapped_to_init_2", 5);
  nav_msgs::Odometry odomBefMapped;
  odomBefMapped.header.frame_id = "/camera_init_2";
  odomBefMapped.child_frame_id = "/bef_mapped";

  ros::Publisher pubOdomAftMapped = nh.advertise<nav_msgs::Odometry> ("/aft_mapped_to_init_2", 5);
  nav_msgs::Odometry odomAftMapped;
  odomAftMapped.header.frame_id = "/camera_init_2";
  odomAftMapped.child_frame_id = "/aft_mapped";

  std::vector<int> pointSearchInd;
  std::vector<float> pointSearchSqDis;

  pcl::PointXYZHSV pointOri, pointSel, pointProj, coeff;
  pcl::PointXYZI pointSurround;

  cv::Mat matA0(5, 3, CV_32F, cv::Scalar::all(0));
  cv::Mat matB0(5, 1, CV_32F, cv::Scalar::all(-1));
  cv::Mat matX0(3, 1, CV_32F, cv::Scalar::all(0));

  cv::Mat matA1(3, 3, CV_32F, cv::Scalar::all(0));
  cv::Mat matD1(1, 3, CV_32F, cv::Scalar::all(0));
  cv::Mat matV1(3, 3, CV_32F, cv::Scalar::all(0));

  for (int i = 0; i < laserCloudNum; i++) {
    laserCloudArray[i].reset(new pcl::PointCloud<pcl::PointXYZHSV>());
  }

  bool status = ros::ok();
  while (status) {
    ros::spinOnce();

    if (newLaserCloudLast && newLaserOdometry && fabs(timeLaserCloudLast - timeLaserOdometry) < 0.005) {
      newLaserCloudLast = false;
      newLaserOdometry = false;

      transformAssociateToMap();

      pcl::PointXYZHSV pointOnZAxis;
      pointOnZAxis.x = 0.0;
      pointOnZAxis.y = 0.0;
      pointOnZAxis.z = 10.0;
      pointAssociateToMap(&pointOnZAxis, &pointOnZAxis);        

      int centerCubeI = int((transformTobeMapped[3] + 10.0) / 20.0) + laserCloudCenWidth;
      int centerCubeJ = int((transformTobeMapped[4] + 10.0) / 20.0) + laserCloudCenHeight;
      int centerCubeK = int((transformTobeMapped[5] + 10.0) / 20.0) + laserCloudCenDepth;
      if (centerCubeI < 0 || centerCubeI >= laserCloudWidth || 
          centerCubeJ < 0 || centerCubeJ >= laserCloudHeight || 
          centerCubeK < 0 || centerCubeK >= laserCloudDepth) {

        transformUpdate();
        continue;
      }

      int laserCloudValidNum = 0;
      int laserCloudSurroundNum = 0;
      for (int i = centerCubeI - 1; i <= centerCubeI + 1; i++) {
        for (int j = centerCubeJ - 1; j <= centerCubeJ + 1; j++) {
          for (int k = centerCubeK - 1; k <= centerCubeK + 1; k++) {
            if (i >= 0 && i < laserCloudWidth && 
                j >= 0 && j < laserCloudHeight && 
                k >= 0 && k < laserCloudDepth) {
              
              float centerX = 20.0 * (i - laserCloudCenWidth);
              float centerY = 20.0 * (j - laserCloudCenHeight);
              float centerZ = 20.0 * (k - laserCloudCenDepth);

              bool isInLaserFOV = false;
              for (int ii = -1; ii <= 1; ii += 2) {
                for (int jj = -1; jj <= 1; jj += 2) {
                  for (int kk = -1; kk <= 1; kk += 2) {
                    float cornerX = centerX + 10.0 * ii;
                    float cornerY = centerY + 10.0 * jj;
                    float cornerZ = centerZ + 10.0 * kk;

                    float check = 100.0 
                                + (transformTobeMapped[3] - cornerX) * (transformTobeMapped[3] - cornerX) 
                                + (transformTobeMapped[4] - cornerY) * (transformTobeMapped[4] - cornerY)
                                + (transformTobeMapped[5] - cornerZ) * (transformTobeMapped[5] - cornerZ)
                                - (pointOnZAxis.x - cornerX) * (pointOnZAxis.x - cornerX) 
                                - (pointOnZAxis.y - cornerY) * (pointOnZAxis.y - cornerY)
                                - (pointOnZAxis.z - cornerZ) * (pointOnZAxis.z - cornerZ);

                    if (check > 0) {
                      isInLaserFOV = true;
                    }
                  }
                }
              }

              if (isInLaserFOV) {
                laserCloudValidInd[laserCloudValidNum] = i + laserCloudWidth * j 
                                                       + laserCloudWidth * laserCloudHeight * k;
                laserCloudValidNum++;
              }
              laserCloudSurroundInd[laserCloudSurroundNum] = i + laserCloudWidth * j 
                                                           + laserCloudWidth * laserCloudHeight * k;
              laserCloudSurroundNum++;
            }
          }
        }
      }

      laserCloudFromMap->clear();
      for (int i = 0; i < laserCloudValidNum; i++) {
        *laserCloudFromMap += *laserCloudArray[laserCloudValidInd[i]];
      }
      int laserCloudFromMapNum = laserCloudFromMap->points.size();

      laserCloudCornerFromMap->clear();
      laserCloudSurfFromMap->clear();
      for (int i = 0; i < laserCloudFromMapNum; i++) {
        if (fabs(laserCloudFromMap->points[i].v - 2) < 0.005 || 
            fabs(laserCloudFromMap->points[i].v - 1) < 0.005) {
          laserCloudCornerFromMap->push_back(laserCloudFromMap->points[i]);
        } else {
          laserCloudSurfFromMap->push_back(laserCloudFromMap->points[i]);
        }
      }

      laserCloudCorner->clear();
      laserCloudSurf->clear();
      int laserCloudLastNum = laserCloudLast->points.size();
      for (int i = 0; i < laserCloudLastNum; i++) {
        if (fabs(laserCloudLast->points[i].v - 2) < 0.005 || 
            fabs(laserCloudLast->points[i].v - 1) < 0.005) {
          laserCloudCorner->push_back(laserCloudLast->points[i]);
        } else {
          laserCloudSurf->push_back(laserCloudLast->points[i]);
        }
      }

      laserCloudCorner2->clear();
      pcl::VoxelGrid<pcl::PointXYZHSV> downSizeFilter;
      downSizeFilter.setInputCloud(laserCloudCorner);
      downSizeFilter.setLeafSize(0.05, 0.05, 0.05);
      downSizeFilter.filter(*laserCloudCorner2);

      laserCloudSurf2->clear();
      downSizeFilter.setInputCloud(laserCloudSurf);
      downSizeFilter.setLeafSize(0.1, 0.1, 0.1);
      downSizeFilter.filter(*laserCloudSurf2);

      laserCloudLast->clear();
      *laserCloudLast = *laserCloudCorner2 + *laserCloudSurf2;
      laserCloudLastNum = laserCloudLast->points.size();

      if (laserCloudSurfFromMap->points.size() > 500) {

        kdtreeCornerFromMap->setInputCloud(laserCloudCornerFromMap);
        kdtreeSurfFromMap->setInputCloud(laserCloudSurfFromMap);

        for (int iterCount = 0; iterCount < 10; iterCount++) {
          laserCloudOri->clear();
          //laserCloudSel->clear();
          //laserCloudCorr->clear();
          //laserCloudProj->clear();
          coeffSel->clear();

          for (int i = 0; i < laserCloudLastNum; i++) {
            if (fabs(laserCloudLast->points[i].x > 1.2) || fabs(laserCloudLast->points[i].y > 1.2) ||
                fabs(laserCloudLast->points[i].z > 1.2)) {

              pointOri = laserCloudLast->points[i];
              pointAssociateToMap(&pointOri, &pointSel); 
              if (fabs(pointOri.v) < 0.05 || fabs(pointOri.v + 1) < 0.05) {

                kdtreeSurfFromMap->nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);

                if (pointSearchSqDis[4] < 1.0) {
                  for (int j = 0; j < 5; j++) {
                    matA0.at<float>(j, 0) = laserCloudSurfFromMap->points[pointSearchInd[j]].x;
                    matA0.at<float>(j, 1) = laserCloudSurfFromMap->points[pointSearchInd[j]].y;
                    matA0.at<float>(j, 2) = laserCloudSurfFromMap->points[pointSearchInd[j]].z;
                  }
                  cv::solve(matA0, matB0, matX0, cv::DECOMP_QR);

                  float pa = matX0.at<float>(0, 0);
                  float pb = matX0.at<float>(1, 0);
                  float pc = matX0.at<float>(2, 0);
                  float pd = 1;
 
                  float ps = sqrt(pa * pa + pb * pb + pc * pc);
                  pa /= ps;
                  pb /= ps;
                  pc /= ps;
                  pd /= ps;

                  bool planeValid = true;
                  for (int j = 0; j < 5; j++) {
                    if (fabs(pa * laserCloudSurfFromMap->points[pointSearchInd[j]].x +
                        pb * laserCloudSurfFromMap->points[pointSearchInd[j]].y +
                        pc * laserCloudSurfFromMap->points[pointSearchInd[j]].z + pd) > 0.05) {
                      planeValid = false;
                      break;
                    }
                  }

                  if (planeValid) {
                    float pd2 = pa * pointSel.x + pb * pointSel.y + pc * pointSel.z + pd;

                    pointProj = pointSel;
                    pointProj.x -= pa * pd2;
                    pointProj.y -= pb * pd2;
                    pointProj.z -= pc * pd2;

                    float s = 1;
                    if (iterCount >= 6) {
                      s = 1 - 8 * fabs(pd2) / sqrt(sqrt(pointSel.x * pointSel.x
                        + pointSel.y * pointSel.y + pointSel.z * pointSel.z));
                    }

                    coeff.x = s * pa;
                    coeff.y = s * pb;
                    coeff.z = s * pc;
                    coeff.h = s * pd2;

                    if (s > 0.2) {
                      laserCloudOri->push_back(pointOri);
                      //laserCloudSel->push_back(pointSel);
                      //laserCloudProj->push_back(pointProj);
                      //laserCloudCorr->push_back(laserCloudSurfFromMap->points[pointSearchInd[0]]);
                      //laserCloudCorr->push_back(laserCloudSurfFromMap->points[pointSearchInd[1]]);
                      //laserCloudCorr->push_back(laserCloudSurfFromMap->points[pointSearchInd[2]]);
                      //laserCloudCorr->push_back(laserCloudSurfFromMap->points[pointSearchInd[3]]);
                      //laserCloudCorr->push_back(laserCloudSurfFromMap->points[pointSearchInd[4]]);
                      coeffSel->push_back(coeff);
                    }
                  }
                }
              } else {

                kdtreeCornerFromMap->nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);
                
                if (pointSearchSqDis[4] < 1.0) {
                  float cx = 0;
                  float cy = 0; 
                  float cz = 0;
                  for (int j = 0; j < 5; j++) {
                    cx += laserCloudCornerFromMap->points[pointSearchInd[j]].x;
                    cy += laserCloudCornerFromMap->points[pointSearchInd[j]].y;
                    cz += laserCloudCornerFromMap->points[pointSearchInd[j]].z;
                  }
                  cx /= 5;
                  cy /= 5; 
                  cz /= 5;

                  float a11 = 0;
                  float a12 = 0; 
                  float a13 = 0;
                  float a22 = 0;
                  float a23 = 0; 
                  float a33 = 0;
                  for (int j = 0; j < 5; j++) {
                    float ax = laserCloudCornerFromMap->points[pointSearchInd[j]].x - cx;
                    float ay = laserCloudCornerFromMap->points[pointSearchInd[j]].y - cy;
                    float az = laserCloudCornerFromMap->points[pointSearchInd[j]].z - cz;

                    a11 += ax * ax;
                    a12 += ax * ay;
                    a13 += ax * az;
                    a22 += ay * ay;
                    a23 += ay * az;
                    a33 += az * az;
                  }
                  a11 /= 5;
                  a12 /= 5; 
                  a13 /= 5;
                  a22 /= 5;
                  a23 /= 5; 
                  a33 /= 5;

                  matA1.at<float>(0, 0) = a11;
                  matA1.at<float>(0, 1) = a12;
                  matA1.at<float>(0, 2) = a13;
                  matA1.at<float>(1, 0) = a12;
                  matA1.at<float>(1, 1) = a22;
                  matA1.at<float>(1, 2) = a23;
                  matA1.at<float>(2, 0) = a13;
                  matA1.at<float>(2, 1) = a23;
                  matA1.at<float>(2, 2) = a33;

                  cv::eigen(matA1, matD1, matV1);

                  if (matD1.at<float>(0, 0) > 3 * matD1.at<float>(0, 1)) {

                    float x0 = pointSel.x;
                    float y0 = pointSel.y;
                    float z0 = pointSel.z;
                    float x1 = cx + 0.1 * matV1.at<float>(0, 0);
                    float y1 = cy + 0.1 * matV1.at<float>(0, 1);
                    float z1 = cz + 0.1 * matV1.at<float>(0, 2);
                    float x2 = cx - 0.1 * matV1.at<float>(0, 0);
                    float y2 = cy - 0.1 * matV1.at<float>(0, 1);
                    float z2 = cz - 0.1 * matV1.at<float>(0, 2);

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

                    pointProj = pointSel;
                    pointProj.x -= la * ld2;
                    pointProj.y -= lb * ld2;
                    pointProj.z -= lc * ld2;

                    float s = 2 * (1 - 8 * fabs(ld2));

                    coeff.x = s * la;
                    coeff.y = s * lb;
                    coeff.z = s * lc;
                    coeff.h = s * ld2;

                    if (s > 0.4) {
                      laserCloudOri->push_back(pointOri);
                      //laserCloudSel->push_back(pointSel);
                      //laserCloudProj->push_back(pointProj);
                      //laserCloudCorr->push_back(laserCloudCornerFromMap->points[pointSearchInd[0]]);
                      //laserCloudCorr->push_back(laserCloudCornerFromMap->points[pointSearchInd[1]]);
                      //laserCloudCorr->push_back(laserCloudCornerFromMap->points[pointSearchInd[2]]);
                      //laserCloudCorr->push_back(laserCloudCornerFromMap->points[pointSearchInd[3]]);
                      //laserCloudCorr->push_back(laserCloudCornerFromMap->points[pointSearchInd[4]]);
                      coeffSel->push_back(coeff);
                    }
                  }
                }
              }
            }
          }
          int laserCloudSelNum = laserCloudOri->points.size();

          float srx = sin(transformTobeMapped[0]);
          float crx = cos(transformTobeMapped[0]);
          float sry = sin(transformTobeMapped[1]);
          float cry = cos(transformTobeMapped[1]);
          float srz = sin(transformTobeMapped[2]);
          float crz = cos(transformTobeMapped[2]);

          if (laserCloudSelNum < 50) {
            continue;
          }

          cv::Mat matA(laserCloudSelNum, 6, CV_32F, cv::Scalar::all(0));
          cv::Mat matAt(6, laserCloudSelNum, CV_32F, cv::Scalar::all(0));
          cv::Mat matAtA(6, 6, CV_32F, cv::Scalar::all(0));
          cv::Mat matB(laserCloudSelNum, 1, CV_32F, cv::Scalar::all(0));
          cv::Mat matAtB(6, 1, CV_32F, cv::Scalar::all(0));
          cv::Mat matX(6, 1, CV_32F, cv::Scalar::all(0));
          for (int i = 0; i < laserCloudSelNum; i++) {
            pointOri = laserCloudOri->points[i];
            coeff = coeffSel->points[i];

            float arx = (crx*sry*srz*pointOri.x + crx*crz*sry*pointOri.y - srx*sry*pointOri.z) * coeff.x
                      + (-srx*srz*pointOri.x - crz*srx*pointOri.y - crx*pointOri.z) * coeff.y
                      + (crx*cry*srz*pointOri.x + crx*cry*crz*pointOri.y - cry*srx*pointOri.z) * coeff.z;

            float ary = ((cry*srx*srz - crz*sry)*pointOri.x 
                      + (sry*srz + cry*crz*srx)*pointOri.y + crx*cry*pointOri.z) * coeff.x
                      + ((-cry*crz - srx*sry*srz)*pointOri.x 
                      + (cry*srz - crz*srx*sry)*pointOri.y - crx*sry*pointOri.z) * coeff.z;

            float arz = ((crz*srx*sry - cry*srz)*pointOri.x + (-cry*crz - srx*sry*srz)*pointOri.y)*coeff.x
                      + (crx*crz*pointOri.x - crx*srz*pointOri.y) * coeff.y
                      + ((sry*srz + cry*crz*srx)*pointOri.x + (crz*sry - cry*srx*srz)*pointOri.y)*coeff.z;

            matA.at<float>(i, 0) = arx;
            matA.at<float>(i, 1) = ary;
            matA.at<float>(i, 2) = arz;
            matA.at<float>(i, 3) = coeff.x;
            matA.at<float>(i, 4) = coeff.y;
            matA.at<float>(i, 5) = coeff.z;
            matB.at<float>(i, 0) = -coeff.h;
          }
          cv::transpose(matA, matAt);
          matAtA = matAt * matA;
          matAtB = matAt * matB;
          cv::solve(matAtA, matAtB, matX, cv::DECOMP_QR);

          if (fabs(matX.at<float>(0, 0)) < 0.5 &&
              fabs(matX.at<float>(1, 0)) < 0.5 &&
              fabs(matX.at<float>(2, 0)) < 0.5 &&
              fabs(matX.at<float>(3, 0)) < 1 &&
              fabs(matX.at<float>(4, 0)) < 1 &&
              fabs(matX.at<float>(5, 0)) < 1) {

            transformTobeMapped[0] += matX.at<float>(0, 0);
            transformTobeMapped[1] += matX.at<float>(1, 0);
            transformTobeMapped[2] += matX.at<float>(2, 0);
            transformTobeMapped[3] += matX.at<float>(3, 0);
            transformTobeMapped[4] += matX.at<float>(4, 0);
            transformTobeMapped[5] += matX.at<float>(5, 0);
          } else {
            //ROS_INFO ("Mapping update out of bound");
          }
        }
      }

      transformUpdate();

      for (int i = 0; i < laserCloudLastNum; i++) {
        if (fabs(laserCloudLast->points[i].x) > 1.2 || fabs(laserCloudLast->points[i].y) > 1.2 ||
            fabs(laserCloudLast->points[i].z) > 1.2) {

          pointAssociateToMap(&laserCloudLast->points[i], &pointSel);

          int cubeI = int((pointSel.x + 10.0) / 20.0) + laserCloudCenWidth;
          int cubeJ = int((pointSel.y + 10.0) / 20.0) + laserCloudCenHeight;
          int cubeK = int((pointSel.z + 10.0) / 20.0) + laserCloudCenDepth;
          int cubeInd = cubeI + laserCloudWidth * cubeJ + laserCloudWidth * laserCloudHeight * cubeK;

          laserCloudArray[cubeInd]->push_back(pointSel);
        }
      }

      for (int i = 0; i < laserCloudValidNum; i++) {
        laserCloudCorner->clear();
        laserCloudSurf->clear();
        pcl::PointCloud<pcl::PointXYZHSV>::Ptr laserCloudCubePointer = 
                                               laserCloudArray[laserCloudValidInd[i]];
        int laserCloudCubeNum = laserCloudCubePointer->points.size();
        for (int j = 0; j < laserCloudCubeNum; j++) {
          if (fabs(laserCloudCubePointer->points[j].v - 2) < 0.005 || 
              fabs(laserCloudCubePointer->points[j].v - 1) < 0.005) {
            laserCloudCorner->push_back(laserCloudCubePointer->points[j]);
          } else {
            laserCloudSurf->push_back(laserCloudCubePointer->points[j]);
          }
        }

        laserCloudCorner2->clear();
        pcl::VoxelGrid<pcl::PointXYZHSV> downSizeFilter;
        downSizeFilter.setInputCloud(laserCloudCorner);
        downSizeFilter.setLeafSize(0.05, 0.05, 0.05);
        downSizeFilter.filter(*laserCloudCorner2);

        laserCloudSurf2->clear();
        downSizeFilter.setInputCloud(laserCloudSurf);
        downSizeFilter.setLeafSize(0.1, 0.1, 0.1);
        downSizeFilter.filter(*laserCloudSurf2);

        laserCloudCubePointer->clear();
        *laserCloudCubePointer = *laserCloudCorner2 + *laserCloudSurf2;
      }

      laserCloudSurround->clear();
      for (int i = 0; i < laserCloudSurroundNum; i++) {
         pcl::PointCloud<pcl::PointXYZHSV>::Ptr laserCloudCubePointer = 
                                                laserCloudArray[laserCloudSurroundInd[i]];
         int laserCloudCubeNum = laserCloudCubePointer->points.size();
         for (int j = 0; j < laserCloudCubeNum; j++) {
           pointSurround.x = laserCloudCubePointer->points[j].x;
           pointSurround.y = laserCloudCubePointer->points[j].y;
           pointSurround.z = laserCloudCubePointer->points[j].z;
           pointSurround.intensity = laserCloudCubePointer->points[j].h;
           laserCloudSurround->push_back(pointSurround);
         }
      }

      sensor_msgs::PointCloud2 laserCloudSurround2;
      pcl::toROSMsg(*laserCloudSurround, laserCloudSurround2);
      laserCloudSurround2.header.stamp = ros::Time().fromSec(timeLaserCloudLast);
      laserCloudSurround2.header.frame_id = "/camera_init_2";
      pubLaserCloudSurround.publish(laserCloudSurround2);

      geometry_msgs::Quaternion geoQuat = tf::createQuaternionMsgFromRollPitchYaw
                     (transformBefMapped[2], -transformBefMapped[0], -transformBefMapped[1]);

      odomBefMapped.header.stamp = ros::Time().fromSec(timeLaserCloudLast);
      odomBefMapped.pose.pose.orientation.x = -geoQuat.y;
      odomBefMapped.pose.pose.orientation.y = -geoQuat.z;
      odomBefMapped.pose.pose.orientation.z = geoQuat.x;
      odomBefMapped.pose.pose.orientation.w = geoQuat.w;
      odomBefMapped.pose.pose.position.x = transformBefMapped[3];
      odomBefMapped.pose.pose.position.y = transformBefMapped[4];
      odomBefMapped.pose.pose.position.z = transformBefMapped[5];
      pubOdomBefMapped.publish(odomBefMapped);

      geoQuat = tf::createQuaternionMsgFromRollPitchYaw
                (transformAftMapped[2], -transformAftMapped[0], -transformAftMapped[1]);

      odomAftMapped.header.stamp = ros::Time().fromSec(timeLaserCloudLast);
      odomAftMapped.pose.pose.orientation.x = -geoQuat.y;
      odomAftMapped.pose.pose.orientation.y = -geoQuat.z;
      odomAftMapped.pose.pose.orientation.z = geoQuat.x;
      odomAftMapped.pose.pose.orientation.w = geoQuat.w;
      odomAftMapped.pose.pose.position.x = transformAftMapped[3];
      odomAftMapped.pose.pose.position.y = transformAftMapped[4];
      odomAftMapped.pose.pose.position.z = transformAftMapped[5];
      pubOdomAftMapped.publish(odomAftMapped);

      /*sensor_msgs::PointCloud2 pc12;
      pcl::toROSMsg(*laserCloudCornerFromMap, pc12);
      pc12.header.stamp = ros::Time().fromSec(timeLaserCloudLast);
      pc12.header.frame_id = "/camera_init_2";
      pub1.publish(pc12);

      sensor_msgs::PointCloud2 pc22;
      pcl::toROSMsg(*laserCloudSel, pc22);
      pc22.header.stamp = ros::Time().fromSec(timeLaserCloudLast);
      pc22.header.frame_id = "/camera_init_2";
      pub2.publish(pc22);

      sensor_msgs::PointCloud2 pc32;
      pcl::toROSMsg(*laserCloudCorr, pc32);
      pc32.header.stamp = ros::Time().fromSec(timeLaserCloudLast);
      pc32.header.frame_id = "/camera_init_2";
      pub3.publish(pc32);

      sensor_msgs::PointCloud2 pc42;
      pcl::toROSMsg(*laserCloudProj, pc42);
      pc42.header.stamp = ros::Time().fromSec(timeLaserCloudLast);
      pc42.header.frame_id = "/camera_init_2";
      pub4.publish(pc42);*/
    }

    status = ros::ok();
    cv::waitKey(10);
  }

  return 0;
}
