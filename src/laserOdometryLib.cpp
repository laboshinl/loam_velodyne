#include <loam_velodyne/laserOdometryLib.h>

#include <pcl/filters/filter.h>

void LaserOdometry::run(LaserOdometry::Inputs &inputs, LaserOdometry::Outputs &outputs) {
  std::vector<int> pointSearchInd;
  std::vector<float> pointSearchSqDis;
  cv::Mat matP(6, 6, CV_32F, cv::Scalar::all(0));

  if(laserCloudCornerLast->empty()) {
    inputs.cornerPointsLessSharp.swap(laserCloudCornerLast);
    inputs.surfPointsLessFlat.swap(laserCloudSurfLast);

    kdtreeCornerLast.setInputCloud(laserCloudCornerLast);
    kdtreeSurfLast.setInputCloud(laserCloudSurfLast);

    transformSum[0] += imuPitchStart;
    transformSum[2] += imuRollStart;

  } else {

    transformation[3] -= imuVeloFromStartX * scanPeriod;
    transformation[4] -= imuVeloFromStartY * scanPeriod;
    transformation[5] -= imuVeloFromStartZ * scanPeriod;

    // TODO this may cause another undesired skipping
    if (laserCloudCornerLastNum > 10 && laserCloudSurfLastNum > 100) {
      std::vector<int> indices;
      pcl::removeNaNFromPointCloud(*inputs.cornerPointsSharp,*inputs.cornerPointsSharp, indices);
      int cornerPointsSharpNum = inputs.cornerPointsSharp->points.size();
      int surfPointsFlatNum = inputs.surfPointsFlat->points.size();
      for (int iterCount = 0; iterCount < 25; iterCount++) {
        pcl::PointCloud<PointType> laserCloudOri;
        pcl::PointCloud<PointType> coeffSel;

        for (int i = 0; i < cornerPointsSharpNum; i++) {
          PointType pointSel;
          transformToStart(inputs.cornerPointsSharp->points[i], transformation, pointSel);

          if (iterCount % 5 == 0) {
            std::vector<int> indices;
            pcl::removeNaNFromPointCloud(*laserCloudCornerLast,*laserCloudCornerLast, indices);
            kdtreeCornerLast.nearestKSearch(pointSel, 1, pointSearchInd, pointSearchSqDis);
            int closestPointInd = -1, minPointInd2 = -1;
            if (pointSearchSqDis[0] < 25) {
              closestPointInd = pointSearchInd[0];
              int closestPointScan = int(laserCloudCornerLast->points[closestPointInd].intensity);

              float pointSqDis, minPointSqDis2 = 25;
              for (int j = closestPointInd + 1; j < cornerPointsSharpNum; j++) {
                if (int(laserCloudCornerLast->points[j].intensity) > closestPointScan + 2.5) {
                  break;
                }

                pointSqDis = pointsSqDistance(laserCloudCornerLast->points[j], pointSel);

                if (int(laserCloudCornerLast->points[j].intensity) > closestPointScan) {
                  if (pointSqDis < minPointSqDis2) {
                    minPointSqDis2 = pointSqDis;
                    minPointInd2 = j;
                  }
                }
              }
              for (int j = closestPointInd - 1; j >= 0; j--) {
                if (int(laserCloudCornerLast->points[j].intensity) < closestPointScan - 2.5) {
                  break;
                }

                pointSqDis = pointsSqDistance(laserCloudCornerLast->points[j], pointSel);

                if (int(laserCloudCornerLast->points[j].intensity) < closestPointScan) {
                  if (pointSqDis < minPointSqDis2) {
                    minPointSqDis2 = pointSqDis;
                    minPointInd2 = j;
                  }
                }
              }
            }

            pointSearchCornerInd1[i] = closestPointInd;
            pointSearchCornerInd2[i] = minPointInd2;
          }

          if (pointSearchCornerInd2[i] >= 0) {
            const PointType &A = laserCloudCornerLast->points[pointSearchCornerInd1[i]];
            const PointType &B = laserCloudCornerLast->points[pointSearchCornerInd2[i]];
            PointType coefficients;
            if(getCornerFeatureCoefficients(A, B, pointSel, iterCount, coefficients)) {
              laserCloudOri.push_back(inputs.cornerPointsSharp->points[i]);
              coeffSel.push_back(coefficients);
            }
          }
        }

        for (int i = 0; i < surfPointsFlatNum; i++) {
          PointType pointSel;
          transformToStart(inputs.surfPointsFlat->points[i], transformation, pointSel);

          if (iterCount % 5 == 0) {
            kdtreeSurfLast.nearestKSearch(pointSel, 1, pointSearchInd, pointSearchSqDis);
            int closestPointInd = -1, minPointInd2 = -1, minPointInd3 = -1;
            if (pointSearchSqDis[0] < 25) {
              closestPointInd = pointSearchInd[0];
              int closestPointScan = int(laserCloudSurfLast->points[closestPointInd].intensity);

              float minPointSqDis2 = 25, minPointSqDis3 = 25;
              for (int j = closestPointInd + 1; j < surfPointsFlatNum; j++) {
                if (int(laserCloudSurfLast->points[j].intensity) > closestPointScan + 2.5) {
                  break;
                }

                float pointSqDis = pointsSqDistance(laserCloudSurfLast->points[j], pointSel);

                if (int(laserCloudSurfLast->points[j].intensity) <= closestPointScan) {
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
                if (int(laserCloudSurfLast->points[j].intensity) < closestPointScan - 2.5) {
                  break;
                }

                float pointSqDis = pointsSqDistance(laserCloudSurfLast->points[j], pointSel);

                if (int(laserCloudSurfLast->points[j].intensity) >= closestPointScan) {
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
            }

            pointSearchSurfInd1[i] = closestPointInd;
            pointSearchSurfInd2[i] = minPointInd2;
            pointSearchSurfInd3[i] = minPointInd3;
          }

          if (pointSearchSurfInd2[i] >= 0 && pointSearchSurfInd3[i] >= 0) {
            const PointType &A = laserCloudSurfLast->points[pointSearchSurfInd1[i]];
            const PointType &B = laserCloudSurfLast->points[pointSearchSurfInd2[i]];
            const PointType &C = laserCloudSurfLast->points[pointSearchSurfInd3[i]];

            PointType coefficients;
            if(getSurfaceFeatureCoefficients(A, B, C, pointSel, iterCount, coefficients)) {
              laserCloudOri.push_back(inputs.surfPointsFlat->points[i]);
              coeffSel.push_back(coefficients);
            }
          }
        }

        int pointSelNum = laserCloudOri.size();
        if (pointSelNum < 10) {
          continue;
        }

        cv::Mat matA(pointSelNum, 6, CV_32F, cv::Scalar::all(0));
        cv::Mat matAt(6, pointSelNum, CV_32F, cv::Scalar::all(0));
        cv::Mat matAtA(6, 6, CV_32F, cv::Scalar::all(0));
        cv::Mat matB(pointSelNum, 1, CV_32F, cv::Scalar::all(0));
        cv::Mat matAtB(6, 1, CV_32F, cv::Scalar::all(0));
        cv::Mat matX(6, 1, CV_32F, cv::Scalar::all(0));
        for (int i = 0; i < pointSelNum; i++) {
          PointType &pointOri = laserCloudOri[i];
          PointType &coeff = coeffSel[i];

          float srx = sin(transformation[0]);
          float crx = cos(transformation[0]);
          float sry = sin(transformation[1]);
          float cry = cos(transformation[1]);
          float srz = sin(transformation[2]);
          float crz = cos(transformation[2]);
          float tx = transformation[3];
          float ty = transformation[4];
          float tz = transformation[5];

          float arx = (-crx*sry*srz*pointOri.x + crx*crz*sry*pointOri.y + srx*sry*pointOri.z
                    + tx*crx*sry*srz - ty*crx*crz*sry - tz*srx*sry) * coeff.x
                    + (srx*srz*pointOri.x - crz*srx*pointOri.y + crx*pointOri.z
                    + ty*crz*srx - tz*crx - tx*srx*srz) * coeff.y
                    + (crx*cry*srz*pointOri.x - crx*cry*crz*pointOri.y - cry*srx*pointOri.z
                    + tz*cry*srx + ty*crx*cry*crz - tx*crx*cry*srz) * coeff.z;

          float ary = ((-crz*sry - cry*srx*srz)*pointOri.x
                    + (cry*crz*srx - sry*srz)*pointOri.y - crx*cry*pointOri.z
                    + tx*(crz*sry + cry*srx*srz) + ty*(sry*srz - cry*crz*srx)
                    + tz*crx*cry) * coeff.x
                    + ((cry*crz - srx*sry*srz)*pointOri.x
                    + (cry*srz + crz*srx*sry)*pointOri.y - crx*sry*pointOri.z
                    + tz*crx*sry - ty*(cry*srz + crz*srx*sry)
                    - tx*(cry*crz - srx*sry*srz)) * coeff.z;

          float arz = ((-cry*srz - crz*srx*sry)*pointOri.x + (cry*crz - srx*sry*srz)*pointOri.y
                    + tx*(cry*srz + crz*srx*sry) - ty*(cry*crz - srx*sry*srz)) * coeff.x
                    + (-crx*crz*pointOri.x - crx*srz*pointOri.y
                    + ty*crx*srz + tx*crx*crz) * coeff.y
                    + ((cry*crz*srx - sry*srz)*pointOri.x + (crz*sry + cry*srx*srz)*pointOri.y
                    + tx*(sry*srz - cry*crz*srx) - ty*(crz*sry + cry*srx*srz)) * coeff.z;

          float atx = -(cry*crz - srx*sry*srz) * coeff.x + crx*srz * coeff.y
                    - (crz*sry + cry*srx*srz) * coeff.z;

          float aty = -(cry*srz + crz*srx*sry) * coeff.x - crx*crz * coeff.y
                    - (sry*srz - cry*crz*srx) * coeff.z;

          float atz = crx*sry * coeff.x - srx * coeff.y - crx*cry * coeff.z;

          float d2 = coeff.intensity;

          matA.at<float>(i, 0) = arx;
          matA.at<float>(i, 1) = ary;
          matA.at<float>(i, 2) = arz;
          matA.at<float>(i, 3) = atx;
          matA.at<float>(i, 4) = aty;
          matA.at<float>(i, 5) = atz;
          matB.at<float>(i, 0) = -0.05 * d2;
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
          float eignThre[6] = {10, 10, 10, 10, 10, 10};
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
          transformation[i] += matX.at<float>(i, 0);
          if(isnan(transformation[i])) {
            transformation[i] = 0;
          }
        }

        float deltaR = norm(matX.rowRange(0, 3));
        float deltaT = norm(matX.rowRange(3, 6)) * 100;

        if (deltaR < DEG2RAD(0.1) && deltaT < 0.1) {
          break;
        }
      }
    }

    accumulateTransformation(transformation, transformSum);

    transformToEnd(inputs.cornerPointsLessSharp, transformation);
    transformToEnd(inputs.surfPointsLessFlat, transformation);
    transformToEnd(inputs.laserCloudOut, transformation);

    inputs.cornerPointsLessSharp.swap(laserCloudCornerLast);
    inputs.surfPointsLessFlat.swap(laserCloudSurfLast);

    laserCloudCornerLastNum = laserCloudCornerLast->points.size();
    laserCloudSurfLastNum = laserCloudSurfLast->points.size();
    if (laserCloudCornerLastNum > 10 && laserCloudSurfLastNum > 100) {
      kdtreeCornerLast.setInputCloud(laserCloudCornerLast);
      kdtreeSurfLast.setInputCloud(laserCloudSurfLast);
    }
  }

  outputs.cloud = inputs.laserCloudOut;
  outputs.corners = laserCloudCornerLast;
  outputs.surfels = laserCloudSurfLast;
  std::copy(transformSum, &(transformSum[6]), outputs.t.begin());
}

void LaserOdometry::updateImu(const pcl::PointCloud<pcl::PointXYZ> &imuTrans) {
  imuPitchStart = imuTrans[0].x;
  imuYawStart = imuTrans[0].y;
  imuRollStart = imuTrans[0].z;

  imuPitchLast = imuTrans[1].x;
  imuYawLast = imuTrans[1].y;
  imuRollLast = imuTrans[1].z;

  imuShiftFromStartX = imuTrans[2].x;
  imuShiftFromStartY = imuTrans[2].y;
  imuShiftFromStartZ = imuTrans[2].z;

  imuVeloFromStartX = imuTrans[3].x;
  imuVeloFromStartY = imuTrans[3].y;
  imuVeloFromStartZ = imuTrans[3].z;
}


void LaserOdometry::transformToStart(const PointType &pi, float *curr_transform, PointType &po)
{
  float phase = 10 * (pi.intensity - int(pi.intensity));
  po.getVector4fMap() = getTransformationTRzRxRy(curr_transform, -phase)*pi.getVector4fMap();
  po.intensity = pi.intensity;
}

void LaserOdometry::transformToEnd(pcl::PointCloud<PointType>::Ptr points, float *curr_transform) {

  Eigen::Affine3f fromStartToEnd =
      getTransformationRyRxRzT(0, 0, 0, imuPitchLast, imuYawLast, imuRollLast) *
      getTransformationTRzRxRy(-imuShiftFromStartX, -imuShiftFromStartY, -imuShiftFromStartZ, imuPitchStart, imuYawStart, imuRollStart) *
      getTransformationRyRxRzT(curr_transform);

  for(pcl::PointCloud<PointType>::iterator p = points->begin(); p < points->end(); p++) {
    transformToStart(*p, curr_transform, *p);
    p->getVector4fMap() = fromStartToEnd*p->getVector4fMap();
  }
}

void LaserOdometry::pluginIMURotation(float bcx, float bcy, float bcz,
                       float blx, float bly, float blz,
                       float alx, float aly, float alz,
                       float &acx, float &acy, float &acz) {
  Eigen::Affine3f current = pcl::getTransformation(0, 0, 0, bcy, bcx, bcz);
  Eigen::Affine3f before = pcl::getTransformation(0, 0, 0, bly, blx, blz);
  Eigen::Affine3f after = pcl::getTransformation(0, 0, 0, aly, alx, alz);

  Eigen::Affine3f output = after * before.inverse() * current;
  pcl::getEulerAngles(output, acy, acx, acz);
}

void LaserOdometry::accumulateRotation(float cx, float cy, float cz, float lx, float ly, float lz,
                        float &ox, float &oy, float &oz) {
  Eigen::Affine3f current = pcl::getTransformation(0, 0, 0, cy, cx, cz);
  Eigen::Affine3f last = pcl::getTransformation(0, 0, 0, ly, lx, lz);

  pcl::getEulerAngles(last*current, oy, ox, oz);
}

void LaserOdometry::accumulateTransformation(const float *t_increment, float *t_sum) {
  float rx, ry, rz;
  accumulateRotation(t_sum[0], t_sum[1], t_sum[2],
                     -t_increment[0], -t_increment[1] * 1.05, -t_increment[2],
                     rx, ry, rz);

  Eigen::Vector4f pos;
  pos << t_increment[5] * 1.05 - imuShiftFromStartZ,
      t_increment[3] - imuShiftFromStartX,
      t_increment[4] - imuShiftFromStartY, 1;
  pos = pcl::getTransformation(0, 0, 0, rz, rx, ry).matrix() * pos;
  t_sum[3] -= pos(1);
  t_sum[4] -= pos(2);
  t_sum[5] -= pos(0);

  pluginIMURotation(rx, ry, rz,
                    imuPitchStart, imuYawStart, imuRollStart,
                    imuPitchLast, imuYawLast, imuRollLast,
                    t_sum[0], t_sum[1], t_sum[2]);
}

/**
 * Line is given by points AB.
 * The result is the distance and the direction to closest point from the third point X.
 */
float LaserOdometry::getLinePointDistance(const Eigen::Vector3f &A, const Eigen::Vector3f &B,
    const Eigen::Vector3f &X, Eigen::Vector3f &unit_direction) {
  Eigen::Vector3f BXcrossAX = (X-B).cross(X-A);
  float BXcrossAXnorm = BXcrossAX.norm();
  float lengthAB = (A-B).norm();
  unit_direction = -BXcrossAX.cross(B-A) / (BXcrossAXnorm * lengthAB);
  return BXcrossAXnorm / lengthAB;
}

bool LaserOdometry::getCornerFeatureCoefficients(const PointType &A, const PointType &B,
    const PointType &X, int iterration, PointType &coeff) {
  Eigen::Vector3f direction;
  float distance = getLinePointDistance(A.getVector3fMap(), B.getVector3fMap(), X.getVector3fMap(), direction);

  float weight = 1.0;
  if (iterration >= 5) {
    weight = 1 - 1.8 * fabs(distance);
  }

  coeff.getVector3fMap() = direction * weight;
  coeff.intensity = distance * weight;

  return (weight > 0.1 && distance != 0);
}

float LaserOdometry::getSurfacePointDistance(const Eigen::Vector3f &A, const Eigen::Vector3f &B, const Eigen::Vector3f &C,
    const Eigen::Vector3f &X, Eigen::Vector3f &surfNormal) {
  surfNormal = (B-A).cross(C-A);
  surfNormal.normalize();

  float normalDotA = -surfNormal.dot(A);
  float distance = surfNormal.dot(X) + normalDotA;
  return distance;
}

bool LaserOdometry::getSurfaceFeatureCoefficients(const PointType &A, const PointType &B, const PointType &C,
    const PointType &X, int iterration, PointType &coefficients) {
  Eigen::Vector3f surfNormal;
  float distance = getSurfacePointDistance(A.getVector3fMap(), B.getVector3fMap(), C.getVector3fMap(),
      X.getVector3fMap(), surfNormal);

  float weight = 1;
  if (iterration >= 5) {
    weight = 1 - 1.8 * fabs(distance) / sqrt(X.getVector3fMap().norm());
  }
  coefficients.getVector3fMap() = weight * surfNormal;
  coefficients.intensity = weight * distance;

  return (weight > 0.1 && distance != 0);
}
