#ifndef TRANSFORM_H
#define TRANSFORM_H

#include <Eigen/Geometry>
#include <Eigen/Dense>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <fstream>

class Transform
{
public:
	Transform();
	Transform(const Eigen::Matrix3f& rotation, const Eigen::Vector3f& translation);
	Transform(const Eigen::Matrix<float,3,4>& compactTransform);
	Eigen::Vector3f Translation()const {return _translation;}
	Eigen::Matrix3f Rotation()const {return _rotation;}
	Transform inverse();
	Transform operator*(const Transform& tf);
	Eigen::Matrix<float,3,4> matrix() const;
	bool good();
	void transformPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr ppcd_in, pcl::PointCloud<pcl::PointXYZ>::Ptr ppcd_out);
	void transformPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr ppcd_in, pcl::PointCloud<pcl::PointXYZ>& pcdout);
	void transformPointCloudI(pcl::PointCloud<pcl::PointXYZI>::Ptr ppcd_in, pcl::PointCloud<pcl::PointXYZI>::Ptr ppcd_out);
	friend std::ofstream& operator<<(std::ofstream & out, const Transform& rhs);
	friend std::ostream& operator<<(std::ostream& out, const Transform& rhs);
	friend std::ifstream& operator>>(std::ifstream& in, Transform& rhs);
private:
	Eigen::Matrix3f _rotation;
	//Eigen::Quaternionf _rotation;
	Eigen::Vector3f _translation;
};


struct poseStamped
{
	double stamp;
	Eigen::Vector3f translation;
	Eigen::Quaternionf q;
};

struct PoseStamped
{
	PoseStamped(){}
	PoseStamped(const double& stamp_,const Eigen::Matrix3f& rotation, const Eigen::Vector3f& translation);
	double stamp;
	Transform pose;
};

struct Veloc
{
	Veloc(){}
	Veloc(const Eigen::Vector3f&twist, const Eigen::Vector3f& velxyz);
	double vx();
	double vy();
	double vz();
	double wx();
	double wy();
	double wz();
	Eigen::Vector3f getRotationAxis();
	float getAngleVelocity();
	Eigen::Vector3f twist;
	Eigen::Vector3f velxyz;
};


PoseStamped predictPose(PoseStamped lastPose, Veloc v, double deltaTime);
Veloc parseVel(const Transform& tf, double deltaTime);

poseStamped PoseInterpolate( poseStamped p0,  poseStamped p1, double stamp);

PoseStamped InterpolatePose(PoseStamped p0, PoseStamped p1, double stamp);

#endif // TRANSFORM_H
