#include <loam_velodyne/laserMappingLib.h>

#include <loam_velodyne/build_transform.h>
#include <opencv/cv.h>

void LaserMapping::run(const Inputs &inputs, Outputs &outputs, float timeLaserOdometry) {
  transformAssociateToMap(transformBefMapped, transformAftMapped, inputs.t, transformTobeMapped);

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

          cv::Mat matA1(3, 3, CV_32F, cv::Scalar::all(0));
          cv::Mat matD1(1, 3, CV_32F, cv::Scalar::all(0));
          cv::Mat matV1(3, 3, CV_32F, cv::Scalar::all(0));

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

            PointType pointProj = pointSel;
            pointProj.x -= la * ld2;
            pointProj.y -= lb * ld2;
            pointProj.z -= lc * ld2;

            float s = 1 - 0.9 * fabs(ld2);

            PointType coeff;
            coeff.x = s * la;
            coeff.y = s * lb;
            coeff.z = s * lc;
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
          cv::Mat matA0(5, 3, CV_32F);
          cv::Mat matB0(5, 1, CV_32F, cv::Scalar::all(-1));
          cv::Mat matX0(3, 1, CV_32F);
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
                pc * laserCloudSurfFromMap->points[pointSearchInd[j]].z + pd) > 0.2) {
              planeValid = false;
              break;
            }
          }

          if (planeValid) {
            float pd2 = pa * pointSel.x + pb * pointSel.y + pc * pointSel.z + pd;

            PointType pointProj = pointSel;
            pointProj.x -= pa * pd2;
            pointProj.y -= pb * pd2;
            pointProj.z -= pc * pd2;

            float s = 1 - 0.9 * fabs(pd2) / sqrt(sqrt(pointSel.x * pointSel.x
                    + pointSel.y * pointSel.y + pointSel.z * pointSel.z));

            PointType coeff;
            coeff.x = s * pa;
            coeff.y = s * pb;
            coeff.z = s * pc;
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
