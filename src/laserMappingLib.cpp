#include <loam_velodyne/laserMappingLib.h>

#include <loam_velodyne/build_transform.h>
#include <opencv/cv.h>
#include <opencv2/core/eigen.hpp>

void LaserMapping::run(const Inputs &inputs, Outputs &outputs, float timeLaserOdometry) {
  improveOdometryByMapping(transformBefMapped, transformAftMapped, inputs.t, transformTobeMapped);

  pcl::PointCloud<PointType> laserCloudCornerStack;
  pcl::PointCloud<PointType> laserCloudSurfStack;
  pcl::transformPointCloud(*inputs.corners, laserCloudCornerStack, getAssociationToMap());
  pcl::transformPointCloud(*inputs.surfels, laserCloudSurfStack, getAssociationToMap());

  PointType pointOnYAxis;
  pointOnYAxis.x = 0.0;
  pointOnYAxis.y = 10.0;
  pointOnYAxis.z = 0.0;
  pointAssociateToMap(pointOnYAxis, pointOnYAxis);

  int centerCubeI = int((transformTobeMapped[3] + 25.0) / 50.0) + laserCloudCenWidth;
  int centerCubeJ = int((transformTobeMapped[4] + 25.0) / 50.0) + laserCloudCenHeight;
  int centerCubeK = int((transformTobeMapped[5] + 25.0) / 50.0) + laserCloudCenDepth;

  if (transformTobeMapped[3] + 25.0 < 0) centerCubeI--;
  if (transformTobeMapped[4] + 25.0 < 0) centerCubeJ--;
  if (transformTobeMapped[5] + 25.0 < 0) centerCubeK--;

  int MIN_PADDING = 3;
  int newCenterCubeI = MIN(MAX(centerCubeI, MIN_PADDING), cloudGridWidth-MIN_PADDING-1);
  int newCenterCubeJ = MIN(MAX(centerCubeJ, MIN_PADDING), cloudGridHeight-MIN_PADDING-1);
  int newCenterCubeK = MIN(MAX(centerCubeK, MIN_PADDING), cloudGridDepth-MIN_PADDING-1);
  laserCloudCornerArray.shift(newCenterCubeI-centerCubeI, newCenterCubeJ-centerCubeJ, newCenterCubeK-centerCubeK);
  laserCloudSurfArray.shift(newCenterCubeI-centerCubeI, newCenterCubeJ-centerCubeJ, newCenterCubeK-centerCubeK);
  laserCloudCenWidth += newCenterCubeI-centerCubeI;
  laserCloudCenHeight += newCenterCubeJ-centerCubeJ;
  laserCloudCenDepth += newCenterCubeK-centerCubeK;
  centerCubeI = newCenterCubeI;
  centerCubeJ = newCenterCubeJ;
  centerCubeK = newCenterCubeK;

  int laserCloudValidNum = 0;
  laserCloudSurroundNum = 0;
  for (int i = centerCubeI - 2; i <= centerCubeI + 2; i++) {
    for (int j = centerCubeJ - 2; j <= centerCubeJ + 2; j++) {
      for (int k = centerCubeK - 2; k <= centerCubeK + 2; k++) {
        if (i >= 0 && i < cloudGridWidth &&
            j >= 0 && j < cloudGridHeight &&
            k >= 0 && k < cloudGridDepth) {

          float centerX = 50.0 * (i - laserCloudCenWidth);
          float centerY = 50.0 * (j - laserCloudCenHeight);
          float centerZ = 50.0 * (k - laserCloudCenDepth);

          bool isInLaserFOV = false;
          for (int ii = -1; ii <= 1; ii += 2) {
            for (int jj = -1; jj <= 1; jj += 2) {
              for (int kk = -1; kk <= 1; kk += 2) {
                float cornerX = centerX + 25.0 * ii;
                float cornerY = centerY + 25.0 * jj;
                float cornerZ = centerZ + 25.0 * kk;

                float squaredSide1 = (transformTobeMapped[3] - cornerX)
                                   * (transformTobeMapped[3] - cornerX)
                                   + (transformTobeMapped[4] - cornerY)
                                   * (transformTobeMapped[4] - cornerY)
                                   + (transformTobeMapped[5] - cornerZ)
                                   * (transformTobeMapped[5] - cornerZ);

                float squaredSide2 = (pointOnYAxis.x - cornerX) * (pointOnYAxis.x - cornerX)
                                   + (pointOnYAxis.y - cornerY) * (pointOnYAxis.y - cornerY)
                                   + (pointOnYAxis.z - cornerZ) * (pointOnYAxis.z - cornerZ);

                float check1 = 100.0 + squaredSide1 - squaredSide2
                             - 10.0 * sqrt(3.0) * sqrt(squaredSide1);

                float check2 = 100.0 + squaredSide1 - squaredSide2
                             + 10.0 * sqrt(3.0) * sqrt(squaredSide1);

                if (check1 < 0 && check2 > 0) {
                  isInLaserFOV = true;
                }
              }
            }
          }

          if (isInLaserFOV) {
            laserCloudValidInd[laserCloudValidNum] = i + cloudGridWidth * j
                                                 + cloudGridWidth * cloudGridHeight * k;
            laserCloudValidNum++;
          }
          laserCloudSurroundInd[laserCloudSurroundNum] = i + cloudGridWidth * j
                                                       + cloudGridWidth * cloudGridHeight * k;
          laserCloudSurroundNum++;
        }
      }
    }
  }

  pcl::PointCloud<PointType>::Ptr laserCloudCornerFromMap(new pcl::PointCloud<PointType>);
  pcl::PointCloud<PointType>::Ptr laserCloudSurfFromMap(new pcl::PointCloud<PointType>);
  for (int i = 0; i < laserCloudValidNum; i++) {
    *laserCloudCornerFromMap += *laserCloudCornerArray[laserCloudValidInd[i]];
    *laserCloudSurfFromMap += *laserCloudSurfArray[laserCloudValidInd[i]];
  }
  int laserCloudCornerFromMapNum = laserCloudCornerFromMap->size();
  int laserCloudSurfFromMapNum = laserCloudSurfFromMap->size();

  pcl::transformPointCloud(laserCloudCornerStack, laserCloudCornerStack, getAssociationToBeMapped());
  pcl::transformPointCloud(laserCloudSurfStack, laserCloudSurfStack, getAssociationToBeMapped());

  downSizeFilterCorner.setInputCloud(laserCloudCornerStack.makeShared());
  downSizeFilterCorner.filter(laserCloudCornerStack);
  int laserCloudCornerStackNum = laserCloudCornerStack.size();

  downSizeFilterSurf.setInputCloud(laserCloudSurfStack.makeShared());
  downSizeFilterSurf.filter(laserCloudSurfStack);
  int laserCloudSurfStackNum = laserCloudSurfStack.size();

  if (laserCloudCornerFromMapNum > 10 && laserCloudSurfFromMapNum > 100) {
    pcl::KdTreeFLANN<PointType> kdtreeCornerFromMap;
    pcl::KdTreeFLANN<PointType> kdtreeSurfFromMap;

    kdtreeCornerFromMap.setInputCloud(laserCloudCornerFromMap);
    kdtreeSurfFromMap.setInputCloud(laserCloudSurfFromMap);
    std::vector<int> pointSearchInd;
    std::vector<float> pointSearchSqDis;

    for (int iterCount = 0; iterCount < 10; iterCount++) {
      pcl::PointCloud<PointType> laserCloudOri;
      pcl::PointCloud<PointType> coeffSel;

      for (int i = 0; i < laserCloudCornerStackNum; i++) {
        const PointType &pointOri = laserCloudCornerStack[i];
        PointType pointSel;
        pointAssociateToMap(pointOri, pointSel);
        kdtreeCornerFromMap.nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);

        if (pointSearchSqDis[4] < 1.0) {

          Eigen::Vector4f centroid;
          Eigen::Matrix3f covariance;
          pcl::computeMeanAndCovarianceMatrix(*laserCloudCornerFromMap, pointSearchInd, covariance, centroid);

          cv::Mat covarianceMat(3, 3, CV_32F, cv::Scalar::all(0));
          cv::eigen2cv(covariance, covarianceMat);
          cv::Mat eigenvalues(1, 3, CV_32F, cv::Scalar::all(0));
          cv::Mat eigenvectors(3, 3, CV_32F, cv::Scalar::all(0));
          cv::eigen(covarianceMat, eigenvalues, eigenvectors);

          if (eigenvalues.at<float>(0) > 3 * eigenvalues.at<float>(1)) {
            const Eigen::Vector3f &point = pointSel.getVector3fMap();
            Eigen::Vector3f largestEigenVect;
            cv::cv2eigen(eigenvectors.row(0), largestEigenVect);
            Eigen::Vector3f centroidMinus = centroid.head(3) - largestEigenVect*0.1;
            Eigen::Vector3f centroidPlus = centroid.head(3) + largestEigenVect*0.1;

            Eigen::Vector3f l;
            float ld2 = getLinePointDistance(centroidMinus, centroidPlus, point, l);

            float s = 1 - 0.9*ld2;

            PointType coeff;
            coeff.getVector3fMap() = l*s;
            coeff.intensity = s * ld2;

            if (s > 0.1) {
              laserCloudOri.push_back(pointOri);
              coeffSel.push_back(coeff);
            }
          }
        }
      }

      for (int i = 0; i < laserCloudSurfStackNum; i++) {
        const PointType &pointOri = laserCloudSurfStack[i];
        PointType pointSel;
        pointAssociateToMap(pointOri, pointSel);
        kdtreeSurfFromMap.nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);

        if (pointSearchSqDis[4] < 1.0) {
          Eigen::Vector4f planeCoef;
          bool planeValid = findPlane(*laserCloudSurfFromMap, pointSearchInd, 0.2, planeCoef);

          if (planeValid) {
            float pd2 = planeCoef.head(3).dot(pointSel.getVector3fMap()) + planeCoef(3);

            float s = 1 - 0.9 * fabs(pd2) / sqrt(pointSel.getVector3fMap().norm());

            PointType coeff;
            coeff.getVector3fMap() = planeCoef.head(3) * s;
            coeff.intensity = s * pd2;

            if (s > 0.1) {
              laserCloudOri.push_back(pointOri);
              coeffSel.push_back(coeff);
            }
          }
        }
      }

      float srx = sin(transformTobeMapped[0]);
      float crx = cos(transformTobeMapped[0]);
      float sry = sin(transformTobeMapped[1]);
      float cry = cos(transformTobeMapped[1]);
      float srz = sin(transformTobeMapped[2]);
      float crz = cos(transformTobeMapped[2]);

      int laserCloudSelNum = laserCloudOri.size();
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
        const PointType &pointOri = laserCloudOri[i];
        const PointType &coeff = coeffSel[i];

        float arx = (crx*sry*srz*pointOri.x + crx*crz*sry*pointOri.y - srx*sry*pointOri.z) * coeff.x
                  + (-srx*srz*pointOri.x - crz*srx*pointOri.y - crx*pointOri.z) * coeff.y
                  + (crx*cry*srz*pointOri.x + crx*cry*crz*pointOri.y - cry*srx*pointOri.z) * coeff.z;

        float ary = ((cry*srx*srz - crz*sry)*pointOri.x
                  + (sry*srz + cry*crz*srx)*pointOri.y + crx*cry*pointOri.z) * coeff.x
                  + ((-cry*crz - srx*sry*srz)*pointOri.x
                  + (cry*srz - crz*srx*sry)*pointOri.y - crx*sry*pointOri.z) * coeff.z;

        float arz = ((crz*srx*sry - cry*srz)*pointOri.x + (-cry*crz-srx*sry*srz)*pointOri.y)*coeff.x
                  + (crx*crz*pointOri.x - crx*srz*pointOri.y) * coeff.y
                  + ((sry*srz + cry*crz*srx)*pointOri.x + (crz*sry-cry*srx*srz)*pointOri.y)*coeff.z;

        matA.at<float>(i, 0) = arx;
        matA.at<float>(i, 1) = ary;
        matA.at<float>(i, 2) = arz;
        matA.at<float>(i, 3) = coeff.x;
        matA.at<float>(i, 4) = coeff.y;
        matA.at<float>(i, 5) = coeff.z;
        matB.at<float>(i, 0) = -coeff.intensity;
      }
      cv::transpose(matA, matAt);
      matAtA = matAt * matA;
      matAtB = matAt * matB;
      cv::solve(matAtA, matAtB, matX, cv::DECOMP_QR);

      if (iterCount == 0) {
        cv::Mat matE(1, 6, CV_32F, cv::Scalar::all(0));
        cv::Mat matV(6, 6, CV_32F, cv::Scalar::all(0));
        cv::Mat matV2(6, 6, CV_32F, cv::Scalar::all(0));

        cv::eigen(matAtA, matE, matV);
        matV.copyTo(matV2);

        isDegenerate = false;
        float eignThre[6] = {100, 100, 100, 100, 100, 100};
        for (int i = 5; i >= 0; i--) {
          if (matE.at<float>(0, i) < eignThre[i]) {
            for (int j = 0; j < 6; j++) {
              matV2.at<float>(i, j) = 0;
            }
            isDegenerate = true;
          } else {
            break;
          }
        }
        matP = matV.inv() * matV2;
      }

      if (isDegenerate) {
        cv::Mat matX2(6, 1, CV_32F, cv::Scalar::all(0));
        matX.copyTo(matX2);
        matX = matP * matX2;
      }

      for(int i = 0; i < 6; i++){
        transformTobeMapped[i] += matX.at<float>(i, 0);
      }

      float deltaR = norm(matX.rowRange(0, 3));
      float deltaT = norm(matX.rowRange(3, 6)) * 100;

      if (deltaR < DEG2RAD(0.1) && deltaT < 0.1) {
        break;
      }
    }

    transformUpdate(timeLaserOdometry, inputs.t);
  }

  pcl::transformPointCloud(laserCloudCornerStack, laserCloudCornerStack, getAssociationToMap());
  for (int i = 0; i < laserCloudCornerStackNum; i++) {
    const PointType &pointSel = laserCloudCornerStack[i];

    int cubeI = int((pointSel.x + 25.0) / 50.0) + laserCloudCenWidth;
    int cubeJ = int((pointSel.y + 25.0) / 50.0) + laserCloudCenHeight;
    int cubeK = int((pointSel.z + 25.0) / 50.0) + laserCloudCenDepth;

    if (pointSel.x + 25.0 < 0) cubeI--;
    if (pointSel.y + 25.0 < 0) cubeJ--;
    if (pointSel.z + 25.0 < 0) cubeK--;

    if (cubeI >= 0 && cubeI < cloudGridWidth &&
        cubeJ >= 0 && cubeJ < cloudGridHeight &&
        cubeK >= 0 && cubeK < cloudGridDepth) {
      int cubeInd = cubeI + cloudGridWidth * cubeJ + cloudGridWidth * cloudGridHeight * cubeK;
      laserCloudCornerArray[cubeInd]->push_back(pointSel);
    }
  }

  pcl::transformPointCloud(laserCloudSurfStack, laserCloudSurfStack, getAssociationToMap());
  for (int i = 0; i < laserCloudSurfStackNum; i++) {
    const PointType &pointSel = laserCloudSurfStack[i];

    int cubeI = int((pointSel.x + 25.0) / 50.0) + laserCloudCenWidth;
    int cubeJ = int((pointSel.y + 25.0) / 50.0) + laserCloudCenHeight;
    int cubeK = int((pointSel.z + 25.0) / 50.0) + laserCloudCenDepth;

    if (pointSel.x + 25.0 < 0) cubeI--;
    if (pointSel.y + 25.0 < 0) cubeJ--;
    if (pointSel.z + 25.0 < 0) cubeK--;

    if (cubeI >= 0 && cubeI < cloudGridWidth &&
        cubeJ >= 0 && cubeJ < cloudGridHeight &&
        cubeK >= 0 && cubeK < cloudGridDepth) {
      int cubeInd = cubeI + cloudGridWidth * cubeJ + cloudGridWidth * cloudGridHeight * cubeK;
      laserCloudSurfArray[cubeInd]->push_back(pointSel);
    }
  }

  for (int i = 0; i < laserCloudValidNum; i++) {
    int ind = laserCloudValidInd[i];

    pcl::PointCloud<PointType>::Ptr tmpCloud(new pcl::PointCloud<PointType>);

    downSizeFilterCorner.setInputCloud(laserCloudCornerArray[ind]);
    downSizeFilterCorner.filter(*tmpCloud);
    laserCloudCornerArray[ind].swap(tmpCloud);

    tmpCloud->clear();

    downSizeFilterSurf.setInputCloud(laserCloudSurfArray[ind]);
    downSizeFilterSurf.filter(*tmpCloud);
    laserCloudSurfArray[ind].swap(tmpCloud);
  }

  std::copy(transformTobeMapped.begin(), transformTobeMapped.end(), outputs.transformToMap.begin());
  std::copy(transformBefMapped.begin(), transformBefMapped.end(), outputs.transformBeforeMapping.begin());
  std::copy(transformAftMapped.begin(), transformAftMapped.end(), outputs.transformAfterMapping.begin());
}

pcl::PointCloud<PointType>::Ptr LaserMapping::getSurroundingFeatures() {
  pcl::PointCloud<PointType>::Ptr laserCloudSurround(new pcl::PointCloud<PointType>());
  for (int i = 0; i < laserCloudSurroundNum; i++) {
    int ind = laserCloudSurroundInd[i];
    *laserCloudSurround += *laserCloudCornerArray[ind];
    *laserCloudSurround += *laserCloudSurfArray[ind];
  }

  downSizeFilterCorner.setInputCloud(laserCloudSurround);
  downSizeFilterCorner.filter(*laserCloudSurround);
  return laserCloudSurround;
}

void LaserMapping::imuUpdate(float time, float roll, float pitch) {
  imuPointerLast = (imuPointerLast + 1) % imuQueLength;

  imuTime[imuPointerLast] = time;
  imuRoll[imuPointerLast] = roll;
  imuPitch[imuPointerLast] = pitch;
}

void LaserMapping::transformUpdate(float timeLaserOdometry, const std::vector<float> &laserOdomTransform)
{
  if (imuPointerLast >= 0) {
    float imuRollLast = 0, imuPitchLast = 0;
    while (imuPointerFront != imuPointerLast) {
      if (timeLaserOdometry + scanPeriod < imuTime[imuPointerFront]) {
        break;
      }
      imuPointerFront = (imuPointerFront + 1) % imuQueLength;
    }

    if (timeLaserOdometry + scanPeriod > imuTime[imuPointerFront]) {
      imuRollLast = imuRoll[imuPointerFront];
      imuPitchLast = imuPitch[imuPointerFront];
    } else {
      int imuPointerBack = (imuPointerFront + imuQueLength - 1) % imuQueLength;
      float ratioFront = (timeLaserOdometry + scanPeriod - imuTime[imuPointerBack])
                       / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);
      float ratioBack = (imuTime[imuPointerFront] - timeLaserOdometry - scanPeriod)
                      / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);

      imuRollLast = imuRoll[imuPointerFront] * ratioFront + imuRoll[imuPointerBack] * ratioBack;
      imuPitchLast = imuPitch[imuPointerFront] * ratioFront + imuPitch[imuPointerBack] * ratioBack;
    }

    transformTobeMapped[0] = 0.998 * transformTobeMapped[0] + 0.002 * imuPitchLast;
    transformTobeMapped[2] = 0.998 * transformTobeMapped[2] + 0.002 * imuRollLast;
  }

  for (int i = 0; i < 6; i++) {
    transformBefMapped[i] = laserOdomTransform[i];
    transformAftMapped[i] = transformTobeMapped[i];
  }
}

Eigen::Affine3f LaserMapping::getAssociationToMap() {
  return getTransformationRzRxRyT(transformTobeMapped);
}

void LaserMapping::pointAssociateToMap(const PointType &pi, PointType &po)
{
  po.getVector4fMap() = getAssociationToMap() * pi.getVector4fMap();
  po.intensity = pi.intensity;
}

Eigen::Affine3f LaserMapping::getAssociationToBeMapped() {
  return getTransformationTRyRxRz(transformTobeMapped, -1.0);
}

bool LaserMapping::findPlane(const pcl::PointCloud<PointType> &cloud, const std::vector<int> &indices,
    float maxDistance, Eigen::Vector4f &coef) {
  cv::Mat matA0(5, 3, CV_32F);
  cv::Mat matB0(5, 1, CV_32F, cv::Scalar::all(-1));
  cv::Mat matX0(3, 1, CV_32F);
  for (int j = 0; j < indices.size(); j++) {
    matA0.at<float>(j, 0) = cloud[indices[j]].x;
    matA0.at<float>(j, 1) = cloud[indices[j]].y;
    matA0.at<float>(j, 2) = cloud[indices[j]].z;
  }
  cv::solve(matA0, matB0, matX0, cv::DECOMP_QR);

  coef(0) = matX0.at<float>(0, 0);
  coef(1) = matX0.at<float>(1, 0);
  coef(2) = matX0.at<float>(2, 0);
  coef(3) = 0;

  float norm = coef.norm();
  coef(3) = 1;
  coef *= norm;

  for (int j = 0; j < indices.size(); j++) {
    if (fabs(coef.dot(cloud[indices[j]].getVector4fMap()) + coef(3)) > maxDistance) {
      return false;
    }
  }
  return true;
}
