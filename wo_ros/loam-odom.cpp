#include <cmath>
#include <vector>

#include <loam_velodyne/common.h>
#include <loam_velodyne/LoamImu.h>
#include <loam_velodyne/scanRegistrationLib.h>
#include <loam_velodyne/laserOdometryLib.h>
#include <loam_velodyne/laserMappingLib.h>

#include <velodyne_pointcloud/point_types.h>
#include <but_velodyne/VelodynePointCloud.h>
#include <but_velodyne/KittiUtils.h>

#include <cv.h>
#include <boost/program_options.hpp>

using namespace std;
using namespace cv;
using namespace pcl;
using namespace velodyne_pointcloud;
using namespace but_velodyne;

namespace po = boost::program_options;

const float SCAN_PERIOD = 0.1;

bool parse_arguments(int argc, char **argv,
                     string &output_file,
                     vector<string> &clouds_to_process);

bool endsWith(const string &str, const string &suffix) {
  return suffix == str.substr(str.size() - suffix.size());
}

/**
 * ./collar-lines-odom $(ls *.bin | sort | xargs)
 */
int main(int argc, char** argv) {

    vector<string> clouds_to_process;
    string output_filename;
    if(!parse_arguments(argc, argv, output_filename, clouds_to_process)) {
      return EXIT_FAILURE;
    }
    ofstream output_file(output_filename.c_str());

    LoamImuInput imu(SCAN_PERIOD);
    ScanRegistration scanReg(imu, SCAN_PERIOD);
    LaserOdometry laserOdom(SCAN_PERIOD);
    LaserMapping mapping(SCAN_PERIOD);


    VelodynePointCloud cloud;
    float time = 0;
    for (int i = 0; i < clouds_to_process.size(); i++) {
        string filename = clouds_to_process[i];
        cerr << "KITTI file: " << filename << endl << flush;

        if (endsWith(filename, ".bin")) {
          but_velodyne::VelodynePointCloud::fromKitti(filename, cloud);
          pcl::transformPointCloud(cloud, cloud,
              cloud.getAxisCorrection().inverse().matrix());
        } else {
          pcl::io::loadPCDFile(filename, cloud);
        }

        LaserOdometry::Inputs odomInputs;
        scanReg.run(cloud, time, odomInputs);

        LaserMapping::Inputs mappingInputs;
        laserOdom.run(odomInputs, mappingInputs);

        LaserMapping::Outputs mappingOutputs;
        mapping.run(mappingInputs, mappingOutputs, time);

        Eigen::Affine3f transf = pcl::getTransformation(-mappingOutputs.transformToMap[3],
            -mappingOutputs.transformToMap[4], mappingOutputs.transformToMap[5], -mappingOutputs.transformToMap[0],
            -mappingOutputs.transformToMap[1], mappingOutputs.transformToMap[2]);
        but_velodyne::KittiUtils::printPose(output_file, transf.matrix());

        time += SCAN_PERIOD;
    }

    output_file.close();

    return EXIT_SUCCESS;
}

bool parse_arguments(int argc, char **argv,
                     string &output_file,
                     vector<string> &clouds_to_process) {
  bool use_kalman;
  int linear_estimator;
  string init_poses;

  po::options_description desc("LOAM odometry estimation\n"
      "======================================\n"
      " * Reference(s): Zhang and Singh. LOAM: Lidar Odometry and Mapping in Real-time. RSS 2014.\n"
      " * Allowed options");
  desc.add_options()
      ("help,h", "produce help message")
      ("output_file,o", po::value<string>(&output_file)->required(), "File where resulting poses will be stored")
   ;

  po::variables_map vm;
  po::parsed_options parsed = po::parse_command_line(argc, argv, desc);
  po::store(parsed, vm);
  clouds_to_process = po::collect_unrecognized(parsed.options, po::include_positional);

  if (vm.count("help") || clouds_to_process.size() < 1)
  {
      std::cerr << desc << std::endl;
      return false;
  }

  try
  {
      po::notify(vm);
  }
  catch(std::exception& e)
  {
      std::cerr << "Error: " << e.what() << std::endl << std::endl << desc << std::endl;
      return false;
  }

  return true;
}
