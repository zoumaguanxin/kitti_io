#include "io/file_io.h"
#include <boost/concept_check.hpp>
#include <ctime>

namespace io {

bool readOdomTimes(string path, std::vector<double>& times)
{
	times.clear();
	ifstream file;
	file.open(path.c_str(),std::ios::in);
	if(file.is_open())
	{
		while(file.peek()!=EOF)
		{
			double time;
			file>>time;
			times.emplace_back(time);
		}
	}
	else
	{
		return false;
	}
	return true;
}

bool readlaserPointCloudFromKITTI( string path, int number, string suffix,PointCloudPtr& ppcd )
{
	char fullname[1000];
	sprintf(fullname,"%s00%.4i%s",path.c_str(),number,suffix.c_str());
	std::cout<<"file name: "<<fullname<<std::endl;
	float data[4];
	ifstream filecplus;
	filecplus.open(fullname,ios::binary|ios::in);
	PointCloudPtr source_pcd(new PointCloud); 
	if(filecplus.good())
	{
		while(filecplus.peek()!=EOF)
		{
			//char is equal to one byte. so the streamsize _n means the number of the bytes that you want to read. 
			filecplus.read((char*)data,4*sizeof(float));
			if(!filecplus.fail())
			{
				pcl::PointXYZ tempPt;
				tempPt.x=data[0];
				tempPt.y=data[1];
				tempPt.z=data[2];
				//std::cout<<tempPt<<std::endl;
				source_pcd->push_back(tempPt);
			}
			else
			{
				throw std::runtime_error("file format error");
			}
		}
	}
	else
	{
		string tem="can not open the file: ";
		string filename_=fullname;
		tem+=filename_;
		std::cout<<tem<<std::endl;
		return false;
		//throw std::runtime_error("can not open file");
	}
	ppcd=source_pcd;
	return true;
}


std::map<std::string, Transform > readCalibfromKitti(std::string path)
{
	ifstream file;
	file.open(path, std::ios::in);
	std::map<std::string, Transform > calibs;
	
	if(file.is_open())
	{
		char line[1024];
		while(file.getline(line, sizeof(line)))
		{
			Eigen::Matrix<float,3,4> transform;
			stringstream s(line);
			std::string prefix;
			s>>prefix;
			for(int i=0;i<3;i++)
			{
				for(int j=0;j<4;j++)
				{
					s>>transform(i,j);
				}
			}
			
			calibs.insert(std::make_pair(prefix,Transform(transform)));
		}
	}
	else
	{
		throw std::runtime_error("can not open file");
	}
	return calibs;
}




}//namespace io
