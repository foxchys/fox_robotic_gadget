#pragma once
#ifndef ROBOTBOX_H
#define ROBOTBOX_H
//this is used for all revolute robot
#include<vector>
#include<array>
#include<cmath>
#include"eigen-eigen\Eigen\Core"
#include"eigen-eigen\Eigen\Geometry"
#define PI 3.141592653589793

class robotbox
{
public:
	explicit robotbox(Eigen::MatrixXd DH);//initialize with DH d,a,alpha,offset,theta//default offset==0,theta==0
	~robotbox();

	std::vector<std::array<double, 5>> RoLink;//d,a,alpha,offset,theta

	//Link_homogeneous
	Eigen::MatrixXd Link_A(std::array<double, 5> Link);//DH every link2homogeneous  //m pi = 3.14...////T(theta+offset)T(d)T(a)T(alpha)

	Eigen::MatrixXd jacobe(std::vector<double> theta);//get the jacobe(based on the final coordinate ) matrix at pose of theta

	Eigen::MatrixXd Link_Base;//the pose of robot's base ,defaul is I
	void set_base(double x, double y, double z, double row, double pitch, double yaw);//set the pose of robot's base

	bool fkine(Eigen::MatrixXd &Forward,std::vector<double> theta);//Forward kinematics

	Eigen::MatrixXd r_homogeneous(Eigen::Matrix3d r);//r2homogeneous

	Eigen::Matrix3d rpy_r(double row, double pitch, double yaw);//rpy2r r=rotx(row)*roty(pitch)*rotz(yaw) x,y,z,pi=3.14........

	Eigen::MatrixXd EulerAngle_R(double x, double y, double z);//Euler Angle to R ...r=rotz(z)*roty(y)*rotx(x) x,y,z,pi=3.14........
	std::vector<double> R_EulerAngle(Eigen::MatrixXd R);//R to Euler Angle return x,y,z

	//ikine parameters:  qt:The results of ikine     tr:the robot end-effector pose   qo:initial joint configuration      ilimit: maximum number of iterations (default 500)
	          //rlimit: maximum number of consecutive step rejections(default 100)    tol::final error tolerance (default 1e-10)  lambda:initial value of lambda (default 0.1)
	          //lambdamin: minimum allowable value of lambda (default 0)
	bool ikine(std::vector<double> &qt, Eigen::MatrixXd tr, std::vector<double> qo, std::vector<double> mask, size_t ilimit = 500,
		size_t rlimit = 100, double tol = 1e-10, double lambda = 0.1, double lambdamin = 0.0);//The inverse kinematics....do the damped inverse Gauss-Newton with Levenberg-Marquadt

private:
	Eigen::MatrixXd tr2delta(Eigen::MatrixXd ta, Eigen::MatrixXd tb);//The difference between two relatively small changes in the two poses. ....return dx,dt,dz ,   drx,dry,drz

	Eigen::MatrixXd delta2tr(Eigen::MatrixXd d);//The vector d=(dx, dy, dz, dRx, dRy, dRz) Convert differential motion  to a homogeneous transform

	std::vector<double> vex(Eigen::MatrixXd S);//V = VEX(S) is the vector which has the corresponding skew-symmetric matrix S.

	template<typename _Matrix_Type_>
	_Matrix_Type_ pseudoInverse(const _Matrix_Type_ &a, double epsilon =
		std::numeric_limits<double>::epsilon());//svd_pinv(ps:if this function use"template<typename _Matrix_Type_>",we can not take it as "public" and call this function in another .cpp)
};

#endif // !ROBOTBOX_H
//.......written by FoxSu chy_s@outlook.com
