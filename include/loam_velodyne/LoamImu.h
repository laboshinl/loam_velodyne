#ifndef __LOAM_IMU_H__
#define __LOAM_IMU_H__

#include <vector>
#include <cmath>
#include <pcl/point_types.h>

#include <loam_velodyne/common.h>

class LoamImuInput {

public:
  LoamImuInput(int scan_period) :
    scanPeriod(scan_period),

    imuPointerFront(0), imuPointerLast(-1),

    imuRollStart(0), imuPitchStart(0), imuYawStart(0),
    imuRollCur(0), imuPitchCur(0), imuYawCur(0),

    imuVeloXStart(0), imuVeloYStart(0), imuVeloZStart(0),
    imuShiftXStart(0), imuShiftYStart(0), imuShiftZStart(0),

    imuVeloXCur(0), imuVeloYCur(0), imuVeloZCur(0),
    imuShiftXCur(0), imuShiftYCur(0), imuShiftZCur(0),

    imuShiftFromStartXCur(0), imuShiftFromStartYCur(0), imuShiftFromStartZCur(0),
    imuVeloFromStartXCur(0), imuVeloFromStartYCur(0), imuVeloFromStartZCur(0),

    imuTime(imuQueLength, 0),
    imuRoll(imuQueLength, 0),
    imuPitch(imuQueLength, 0),
    imuYaw(imuQueLength, 0),

    imuAccX(imuQueLength, 0),
    imuAccY(imuQueLength, 0),
    imuAccZ(imuQueLength, 0),

    imuVeloX(imuQueLength, 0),
    imuVeloY(imuQueLength, 0),
    imuVeloZ(imuQueLength, 0),

    imuShiftX(imuQueLength, 0),
    imuShiftY(imuQueLength, 0),
    imuShiftZ(imuQueLength, 0) {
  }

  void handleInput(float roll, float pitch, float yaw,
      float accX, float accY, float accZ, float time) {
    accX += - sin(roll) * cos(pitch) * 9.81;
    accY += - cos(roll) * cos(pitch) * 9.81;
    accZ +=   sin(pitch) * 9.81;

    imuPointerLast = (imuPointerLast + 1) % imuQueLength;

    imuTime[imuPointerLast] = time;
    imuRoll[imuPointerLast] = roll;
    imuPitch[imuPointerLast] = pitch;
    imuYaw[imuPointerLast] = yaw;
    imuAccX[imuPointerLast] = accX;
    imuAccY[imuPointerLast] = accY;
    imuAccZ[imuPointerLast] = accZ;

    accumulateIMUShift();
  }

  bool isAvailable() {
    return (imuPointerLast >= 0);
  }

  void accumulateIMUShift()
  {
    float roll = imuRoll[imuPointerLast];
    float pitch = imuPitch[imuPointerLast];
    float yaw = imuYaw[imuPointerLast];
    float accX = imuAccX[imuPointerLast];
    float accY = imuAccY[imuPointerLast];
    float accZ = imuAccZ[imuPointerLast];

    float x1 = cos(roll) * accX - sin(roll) * accY;
    float y1 = sin(roll) * accX + cos(roll) * accY;
    float z1 = accZ;

    float x2 = x1;
    float y2 = cos(pitch) * y1 - sin(pitch) * z1;
    float z2 = sin(pitch) * y1 + cos(pitch) * z1;

    accX = cos(yaw) * x2 + sin(yaw) * z2;
    accY = y2;
    accZ = -sin(yaw) * x2 + cos(yaw) * z2;

    int imuPointerBack = (imuPointerLast + imuQueLength - 1) % imuQueLength;
    double timeDiff = imuTime[imuPointerLast] - imuTime[imuPointerBack];
    if (timeDiff < scanPeriod) {

      imuShiftX[imuPointerLast] = imuShiftX[imuPointerBack] + imuVeloX[imuPointerBack] * timeDiff
                                + accX * timeDiff * timeDiff / 2;
      imuShiftY[imuPointerLast] = imuShiftY[imuPointerBack] + imuVeloY[imuPointerBack] * timeDiff
                                + accY * timeDiff * timeDiff / 2;
      imuShiftZ[imuPointerLast] = imuShiftZ[imuPointerBack] + imuVeloZ[imuPointerBack] * timeDiff
                                + accZ * timeDiff * timeDiff / 2;

      imuVeloX[imuPointerLast] = imuVeloX[imuPointerBack] + accX * timeDiff;
      imuVeloY[imuPointerLast] = imuVeloY[imuPointerBack] + accY * timeDiff;
      imuVeloZ[imuPointerLast] = imuVeloZ[imuPointerBack] + accZ * timeDiff;
    }
  }

  void improvePointPossition(PointType &point, float current_scan_time, float point_rel_time, bool is_first) {
    float pointTime = point_rel_time * scanPeriod;
    while (imuPointerFront != imuPointerLast) {
      if (current_scan_time + pointTime < imuTime[imuPointerFront]) {
        break;
      }
      imuPointerFront = (imuPointerFront + 1) % imuQueLength;
    }

    if (current_scan_time + pointTime > imuTime[imuPointerFront]) {
      imuRollCur = imuRoll[imuPointerFront];
      imuPitchCur = imuPitch[imuPointerFront];
      imuYawCur = imuYaw[imuPointerFront];

      imuVeloXCur = imuVeloX[imuPointerFront];
      imuVeloYCur = imuVeloY[imuPointerFront];
      imuVeloZCur = imuVeloZ[imuPointerFront];

      imuShiftXCur = imuShiftX[imuPointerFront];
      imuShiftYCur = imuShiftY[imuPointerFront];
      imuShiftZCur = imuShiftZ[imuPointerFront];
    } else {
      int imuPointerBack = (imuPointerFront + imuQueLength - 1) % imuQueLength;
      float ratioFront = (current_scan_time + pointTime - imuTime[imuPointerBack])
                       / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);
      float ratioBack = (imuTime[imuPointerFront] - current_scan_time - pointTime)
                      / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);

      imuRollCur = imuRoll[imuPointerFront] * ratioFront + imuRoll[imuPointerBack] * ratioBack;
      imuPitchCur = imuPitch[imuPointerFront] * ratioFront + imuPitch[imuPointerBack] * ratioBack;
      if (imuYaw[imuPointerFront] - imuYaw[imuPointerBack] > M_PI) {
        imuYawCur = imuYaw[imuPointerFront] * ratioFront + (imuYaw[imuPointerBack] + 2 * M_PI) * ratioBack;
      } else if (imuYaw[imuPointerFront] - imuYaw[imuPointerBack] < -M_PI) {
        imuYawCur = imuYaw[imuPointerFront] * ratioFront + (imuYaw[imuPointerBack] - 2 * M_PI) * ratioBack;
      } else {
        imuYawCur = imuYaw[imuPointerFront] * ratioFront + imuYaw[imuPointerBack] * ratioBack;
      }

      imuVeloXCur = imuVeloX[imuPointerFront] * ratioFront + imuVeloX[imuPointerBack] * ratioBack;
      imuVeloYCur = imuVeloY[imuPointerFront] * ratioFront + imuVeloY[imuPointerBack] * ratioBack;
      imuVeloZCur = imuVeloZ[imuPointerFront] * ratioFront + imuVeloZ[imuPointerBack] * ratioBack;

      imuShiftXCur = imuShiftX[imuPointerFront] * ratioFront + imuShiftX[imuPointerBack] * ratioBack;
      imuShiftYCur = imuShiftY[imuPointerFront] * ratioFront + imuShiftY[imuPointerBack] * ratioBack;
      imuShiftZCur = imuShiftZ[imuPointerFront] * ratioFront + imuShiftZ[imuPointerBack] * ratioBack;
    }
    if (is_first) {
      imuRollStart = imuRollCur;
      imuPitchStart = imuPitchCur;
      imuYawStart = imuYawCur;

      imuVeloXStart = imuVeloXCur;
      imuVeloYStart = imuVeloYCur;
      imuVeloZStart = imuVeloZCur;

      imuShiftXStart = imuShiftXCur;
      imuShiftYStart = imuShiftYCur;
      imuShiftZStart = imuShiftZCur;
    } else {
      shiftToStartIMU(pointTime);
      veloToStartIMU();
      transformToStartIMU(&point);
    }
  }

  pcl::PointCloud<pcl::PointXYZ> to4Points() {
    pcl::PointCloud<pcl::PointXYZ> imuTrans(4, 1);

    imuTrans.points[0].x = imuPitchStart;
    imuTrans.points[0].y = imuYawStart;
    imuTrans.points[0].z = imuRollStart;

    imuTrans.points[1].x = imuPitchCur;
    imuTrans.points[1].y = imuYawCur;
    imuTrans.points[1].z = imuRollCur;

    imuTrans.points[2].x = imuShiftFromStartXCur;
    imuTrans.points[2].y = imuShiftFromStartYCur;
    imuTrans.points[2].z = imuShiftFromStartZCur;

    imuTrans.points[3].x = imuVeloFromStartXCur;
    imuTrans.points[3].y = imuVeloFromStartYCur;
    imuTrans.points[3].z = imuVeloFromStartZCur;

    return imuTrans;
  }

protected:
  void shiftToStartIMU(float pointTime)
  {
    imuShiftFromStartXCur = imuShiftXCur - imuShiftXStart - imuVeloXStart * pointTime;
    imuShiftFromStartYCur = imuShiftYCur - imuShiftYStart - imuVeloYStart * pointTime;
    imuShiftFromStartZCur = imuShiftZCur - imuShiftZStart - imuVeloZStart * pointTime;

    float x1 = cos(imuYawStart) * imuShiftFromStartXCur - sin(imuYawStart) * imuShiftFromStartZCur;
    float y1 = imuShiftFromStartYCur;
    float z1 = sin(imuYawStart) * imuShiftFromStartXCur + cos(imuYawStart) * imuShiftFromStartZCur;

    float x2 = x1;
    float y2 = cos(imuPitchStart) * y1 + sin(imuPitchStart) * z1;
    float z2 = -sin(imuPitchStart) * y1 + cos(imuPitchStart) * z1;

    imuShiftFromStartXCur = cos(imuRollStart) * x2 + sin(imuRollStart) * y2;
    imuShiftFromStartYCur = -sin(imuRollStart) * x2 + cos(imuRollStart) * y2;
    imuShiftFromStartZCur = z2;
  }

  void veloToStartIMU()
  {
    imuVeloFromStartXCur = imuVeloXCur - imuVeloXStart;
    imuVeloFromStartYCur = imuVeloYCur - imuVeloYStart;
    imuVeloFromStartZCur = imuVeloZCur - imuVeloZStart;

    float x1 = cos(imuYawStart) * imuVeloFromStartXCur - sin(imuYawStart) * imuVeloFromStartZCur;
    float y1 = imuVeloFromStartYCur;
    float z1 = sin(imuYawStart) * imuVeloFromStartXCur + cos(imuYawStart) * imuVeloFromStartZCur;

    float x2 = x1;
    float y2 = cos(imuPitchStart) * y1 + sin(imuPitchStart) * z1;
    float z2 = -sin(imuPitchStart) * y1 + cos(imuPitchStart) * z1;

    imuVeloFromStartXCur = cos(imuRollStart) * x2 + sin(imuRollStart) * y2;
    imuVeloFromStartYCur = -sin(imuRollStart) * x2 + cos(imuRollStart) * y2;
    imuVeloFromStartZCur = z2;
  }

  void transformToStartIMU(PointType *p)
  {
    float x1 = cos(imuRollCur) * p->x - sin(imuRollCur) * p->y;
    float y1 = sin(imuRollCur) * p->x + cos(imuRollCur) * p->y;
    float z1 = p->z;

    float x2 = x1;
    float y2 = cos(imuPitchCur) * y1 - sin(imuPitchCur) * z1;
    float z2 = sin(imuPitchCur) * y1 + cos(imuPitchCur) * z1;

    float x3 = cos(imuYawCur) * x2 + sin(imuYawCur) * z2;
    float y3 = y2;
    float z3 = -sin(imuYawCur) * x2 + cos(imuYawCur) * z2;

    float x4 = cos(imuYawStart) * x3 - sin(imuYawStart) * z3;
    float y4 = y3;
    float z4 = sin(imuYawStart) * x3 + cos(imuYawStart) * z3;

    float x5 = x4;
    float y5 = cos(imuPitchStart) * y4 + sin(imuPitchStart) * z4;
    float z5 = -sin(imuPitchStart) * y4 + cos(imuPitchStart) * z4;

    p->x = cos(imuRollStart) * x5 + sin(imuRollStart) * y5 + imuShiftFromStartXCur;
    p->y = -sin(imuRollStart) * x5 + cos(imuRollStart) * y5 + imuShiftFromStartYCur;
    p->z = z5 + imuShiftFromStartZCur;
  }

private:
  const int scanPeriod;

  int imuPointerFront;

  float imuRollStart, imuPitchStart, imuYawStart;
  float imuRollCur, imuPitchCur, imuYawCur;

  float imuVeloXStart, imuVeloYStart, imuVeloZStart;
  float imuShiftXStart, imuShiftYStart, imuShiftZStart;

  float imuVeloXCur, imuVeloYCur, imuVeloZCur;
  float imuShiftXCur, imuShiftYCur, imuShiftZCur;

  float imuShiftFromStartXCur, imuShiftFromStartYCur, imuShiftFromStartZCur;
  float imuVeloFromStartXCur, imuVeloFromStartYCur, imuVeloFromStartZCur;

  std::vector<float> imuVeloX, imuVeloY, imuVeloZ;
  std::vector<float> imuShiftX, imuShiftY, imuShiftZ;

protected:
  static const int imuQueLength = 2000;    // orig 200

  int imuPointerLast;

  std::vector<double> imuTime;
  std::vector<float> imuRoll, imuPitch, imuYaw;
  std::vector<float> imuAccX, imuAccY, imuAccZ;
};

#endif
