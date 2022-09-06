#include "transform.h"
#include <cmath>
#include <Eigen/Geometry>


Transform::Transform(const Eigen::Matrix3f& rotation, const Eigen::Vector3f& translation)
{
	Eigen::Affine3f affine3f;
	affine3f=rotation;
	affine3f.translation()=translation;
	_rotation=affine3f.rotation();
	_translation=affine3f.translation();
}

Transform::Transform ( const Eigen::Matrix< float, int ( 3 ), int ( 4 ) >& compactTransform )
{
	_rotation=compactTransform.block(0,0,3,3);
	_translation=compactTransform.block(0,3,3,1);
	Eigen::Affine3f affine3f;
	affine3f=_rotation;
	affine3f.translation()=_translation;
	_rotation=affine3f.rotation();
	_translation=affine3f.translation();

}


Transform::Transform()
{
	_rotation=Eigen::Matrix3f::Identity();
	_translation=Eigen::Vector3f::Zero();
}

Transform Transform::inverse()
{
	
 	Eigen::Matrix3f newRotation=_rotation.transpose();
	Eigen::Vector3f newTranslation=-newRotation*_translation;
	return Transform(newRotation,newTranslation);
}

Transform Transform::operator* ( const Transform& tf )
{
	Eigen::Matrix3f last_R=_rotation;
	Eigen::Matrix3f delta_R=tf.Rotation();
	Eigen::Matrix3f R=last_R*delta_R;
	Eigen::Vector3f delta_t=tf.Translation();
	Eigen::Vector3f t=last_R*delta_t+this->_translation;
	return Transform(R,t);
}

void Transform::transformPointCloud ( pcl::PointCloud< pcl::PointXYZ >::Ptr ppcd_in, pcl::PointCloud< pcl::PointXYZ >::Ptr ppcd_out )
{
	assert(!ppcd_in->empty());
	for(int j=0;j<ppcd_in->points.size();++j)
	{
		Eigen::Vector3f temV,v2;
		temV<<ppcd_in->points[j].x,ppcd_in->points[j].y,ppcd_in->points[j].z;
		v2=_rotation*temV;
		pcl::PointXYZ temPoint;
		temPoint.x=v2(0)+_translation(0);
		temPoint.y=v2(1)+_translation(1);
		temPoint.z=v2(2)+_translation(2);
		ppcd_out->points.push_back(temPoint);
	}
   assert(!ppcd_out->empty());
}


void Transform::transformPointCloud ( pcl::PointCloud< pcl::PointXYZ >::Ptr ppcd_in, pcl::PointCloud< pcl::PointXYZ >& pcdout )
{
	for(int j=0;j<ppcd_in->points.size();++j)
	{
		Eigen::Vector3f temV,v2;
		temV<<ppcd_in->points[j].x,ppcd_in->points[j].y,ppcd_in->points[j].z;
		v2=_rotation*temV;
		pcl::PointXYZ temPoint;
		temPoint.x=v2(0)+_translation(0);
		temPoint.y=v2(1)+_translation(1);
		temPoint.z=v2(2)+_translation(2);
		pcdout.points.push_back(temPoint);
	}
}


bool Transform::good()
{
	for(size_t i=0;i<3;++i)
	{
		for(size_t j=0;j<3;++j)
		{
			if(_rotation(i,j)>1.0+0.00001)
			{
				std::cout<<"rotation: "<<_rotation<<std::endl;
				std::cout<<"rotation: ("<<i<<", "<<j<<" )"<<_rotation(i,j)<<' '<<std::endl;
				return false;
			}
		}
	}
	return true;
}



void Transform::transformPointCloudI ( pcl::PointCloud< pcl::PointXYZI >::Ptr ppcd_in, pcl::PointCloud< pcl::PointXYZI >::Ptr ppcd_out )
{
	assert(!ppcd_in->empty());
	for(int j=0;j<ppcd_in->points.size();++j)
	{
		Eigen::Vector3f temV,v2;
		temV<<ppcd_in->points[j].x,ppcd_in->points[j].y,ppcd_in->points[j].z;
		v2=_rotation*temV;
		pcl::PointXYZI temPoint;
		temPoint.x=v2(0)+_translation(0);
		temPoint.y=v2(1)+_translation(1);
		temPoint.z=v2(2)+_translation(2);
		temPoint.intensity=ppcd_in->points[j].intensity;
		ppcd_out->points.push_back(temPoint);
	}
	assert(!ppcd_out->empty());
}

Eigen::Matrix< float, int ( 3 ), int ( 4 ) > Transform::matrix() const
{
	Eigen::Matrix<float,3,4> tf_matrix;
	tf_matrix.block(0,0,3,3)=_rotation;
	tf_matrix.block(0,3,3,1)=_translation;
	return tf_matrix;
}


std::ofstream& operator<<(std::ofstream & out, const Transform& rhs)
{
	Eigen::Matrix<float,3,4> m=rhs.matrix();
	for(size_t i=0;i<3;i++)
	{
		for(size_t j=0;j<4;j++)
		{
			if(j==3&&i==2)
			{
				out<<m(i,j);
			}
			else
			{
				out<<m(i,j)<<' ';
			}
		}
	}
	return out;
}


std::ostream& operator<<(std::ostream & out, const Transform& rhs)
{
	Eigen::Matrix<float,3,4> m=rhs.matrix();
	for(size_t i=0;i<3;i++)
	{
		for(size_t j=0;j<4;j++)
		{
			if(j==3&&i==2)
			{
				out<<m(i,j);
			}
			else
			{
				out<<m(i,j)<<' ';
			}
		}
	}
	return out;
}

std::ifstream& operator>> ( std::ifstream& in, Transform& rhs )
{
	Eigen::Matrix<float,3, 4> pose;
	for(size_t i=0;i<3;i++)
	{
		for(size_t j=0;j<4;j++)
		{
			in>>pose(i,j);
		}
	}
	rhs=Transform(pose);
}


PoseStamped::PoseStamped ( const double& stamp_, const Eigen::Matrix3f& rotation, const Eigen::Vector3f& translation )
{
	stamp=stamp_;
	pose=Transform(rotation, translation);
}


Veloc::Veloc ( const Eigen::Vector3f& twist_, const Eigen::Vector3f& velxyz_ )
{
	twist=twist_;
	velxyz=velxyz_;
}


double Veloc::vx()
{
	return velxyz(0);
}

double Veloc::vy()
{
	return velxyz(1);
}

double Veloc::vz()
{
	return velxyz(2);
}


double Veloc::wx()
{
	return twist(0);
}


double Veloc::wy()
{
	return twist(1);
}

double Veloc::wz()
{
	return twist(2);
}

float Veloc::getAngleVelocity()
{
	float w=std::sqrt(twist(0)*twist(0)+twist(1)*twist(1)+twist(2)*twist(2));
	return w;
}


Eigen::Vector3f Veloc::getRotationAxis()
{
	float w=getAngleVelocity();
	return Eigen::Vector3f(twist(0)/w,twist(1)/w,twist(2)/w);
}





poseStamped PoseInterpolate( poseStamped p0,  poseStamped p1, double stamp)
{
	if(p0.stamp>p1.stamp)
	{
		PoseInterpolate(p1,p0,stamp);
	}
	Eigen::Vector3f DeltaTranlation=p0.q.toRotationMatrix().transpose()*(p1.translation-p0.translation);
	double deltaTime=p1.stamp-p0.stamp;
	Eigen::Quaternionf q(p0.q.toRotationMatrix().transpose()*p1.q.toRotationMatrix());
	double deltahalfTheta;
	double sinHalftheta_positive=std::sqrt(q.x()*q.x()+q.y()*q.y()+q.z()*q.z());
	double cosHalfTheta=q.w();
	double halfTheta_possible0=std::acos(cosHalfTheta);
	double halfTheta_possible1=-std::acos(cosHalfTheta);
	if(sin(halfTheta_possible0)==sinHalftheta_positive||sin(halfTheta_possible0)==-sinHalftheta_positive)
	{
		deltahalfTheta=halfTheta_possible0;
	}
	else
	{
		deltahalfTheta=halfTheta_possible1;
	}
	float factor=sin(deltahalfTheta*(stamp-p0.stamp)/deltaTime)/sin(deltahalfTheta);
	float qx=q.x()*factor;
	float qy=q.y()*factor;
	float qz=q.z()*factor;
	float w=cos(deltahalfTheta*(stamp-p0.stamp)/deltaTime);
	Eigen::Vector3f middleTranslation=DeltaTranlation*(stamp-p0.stamp)/deltaTime;
	poseStamped middlePose;
	Eigen::Quaternionf detlaRotaton(w,qx,qy,qz);
	middlePose.q=p0.q.toRotationMatrix()*detlaRotaton;
	middlePose.stamp=stamp;
	Eigen::Vector3f translation=p0.q.toRotationMatrix()*middleTranslation+p0.translation;
	middlePose.translation=translation;
	return middlePose;
}


PoseStamped InterpolatePose ( PoseStamped p0, PoseStamped p1, double stamp )
{
	if(p0.stamp>p1.stamp)
	{
		InterpolatePose(p1,p0,stamp);
	}
	Eigen::Vector3f DeltaTranlation=p0.pose.Rotation().transpose()*(p1.pose.Translation()-p0.pose.Translation());
	double deltaTime=p1.stamp-p0.stamp;
	Eigen::Quaternionf q(p0.pose.Rotation().transpose()*p1.pose.Rotation());
	double deltahalfTheta;
	double sinHalftheta_positive=std::sqrt(q.x()*q.x()+q.y()*q.y()+q.z()*q.z());
	double cosHalfTheta=q.w();
	double halfTheta_possible0=std::acos(cosHalfTheta);
	double halfTheta_possible1=-std::acos(cosHalfTheta);
	if(sin(halfTheta_possible0)==sinHalftheta_positive||sin(halfTheta_possible0)==-sinHalftheta_positive)
	{
		deltahalfTheta=halfTheta_possible0;
	}
	else
	{
		deltahalfTheta=halfTheta_possible1;
	}
	float factor=sin(deltahalfTheta*(stamp-p0.stamp)/deltaTime)/sin(deltahalfTheta);
	float qx=q.x()*factor;
	float qy=q.y()*factor;
	float qz=q.z()*factor;
	float w=cos(deltahalfTheta*(stamp-p0.stamp)/deltaTime);
	Eigen::Vector3f middleTranslation=DeltaTranlation*(stamp-p0.stamp)/deltaTime;
	PoseStamped middlePose;
	Eigen::Quaternionf detlaRotaton(w,qx,qy,qz);
	middlePose.stamp=stamp;
	Eigen::Vector3f translation=p0.pose.Rotation()*middleTranslation+p0.pose.Translation();
	Transform pose(p0.pose.Rotation()*detlaRotaton,translation);
	middlePose.pose=pose;
	return middlePose;
}

PoseStamped predictPose(PoseStamped lastPose, Veloc v, double deltaTime){
	double AngleV=v.getAngleVelocity();
	Eigen::Vector3f axis=v.getRotationAxis();
	double halftheta=AngleV/2*deltaTime;
	float qx= axis(0)*sin(halftheta);
	float qy=axis(1)*sin(halftheta);
	float qz=axis(2)*sin(halftheta);
	float qw=cos(halftheta);
	Eigen::Quaternionf deltaQ(qw,qx,qy,qz);
	Transform deltT(deltaQ.toRotationMatrix(),v.velxyz*deltaTime);
	PoseStamped preditedPose;
	preditedPose.pose=lastPose.pose*deltT;
	return preditedPose;
}

Veloc parseVel ( const Transform& tf, double deltaTime )
{
	//Angle velocity
	Eigen::Vector3f Wxyz(0.f,0.f,0.f);
	Eigen::Quaternionf q(tf.Rotation());
	double deltahalfTheta;
	double sinHalftheta_positive=std::sqrt(q.x()*q.x()+q.y()*q.y()+q.z()*q.z());
	double cosHalfTheta=q.w();
	double halfTheta_possible0=std::acos(cosHalfTheta);
	double halfTheta_possible1=-std::acos(cosHalfTheta);
	if(sin(halfTheta_possible0)==sinHalftheta_positive||sin(halfTheta_possible0)==-sinHalftheta_positive)
	{
		deltahalfTheta=halfTheta_possible0;
	}
	else
	{
		deltahalfTheta=halfTheta_possible1;
	}
	double w=2*deltahalfTheta/deltaTime;
	double sinhalfTheta=sin(deltahalfTheta);
	if(sinhalfTheta!=0)
	{
		Wxyz(0)=q.x()/sinhalfTheta*w;
		Wxyz(1)=q.y()/sinhalfTheta*w;
		Wxyz(2)=q.z()/sinhalfTheta*w;
	}
	Eigen::Vector3f velxyz=tf.Translation()/deltaTime;
	return Veloc( Wxyz,velxyz);
}

