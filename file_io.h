#ifndef FILE_IO_H
#define FILE_IO_H

#include <list>
#include <map>
#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <iostream>
#include <fstream>
#include "types/sensor_data.h"
#include "transform/tranform.h"
using namespace std;

namespace io
{

	typedef pcl::PointXYZ Point3d;
	typedef pcl::PointXYZI PointI3d;
	typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
	typedef pcl::PointCloud<pcl::PointXYZI> PointCloudI;
	typedef pcl::PointCloud<pcl::PointXYZ>::Ptr PointCloudPtr;
	typedef pcl::PointCloud<pcl::PointXYZI>::Ptr PointCloudIPtr;
	
	bool readOdomTimes(string path, std::vector<double>& times);
	
	//In here, read calib file. 
	std::map<std::string, Transform> readCalibfromKitti(std::string path);
	//in here, read lidar data
	bool readlaserPointCloudFromKITTI( string path, int number, string suffix, io::PointCloudPtr& ppcd );
	
	
};


#endif // FILE_IO_H
