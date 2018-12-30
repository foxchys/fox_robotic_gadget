#include"robotbox.h"
#include<iostream>

Eigen::MatrixXd RODH(4, 4);
void setDH()//DH d,a,alpha,offset,theta
{
	RODH <<  0.08, 0.0, (PI / 2), 0.0,
		0.0, 0.3, (-PI / 2), 0.0,
		0.0, 0.0, (PI / 2), 0.0,
		0.3, 0.0, (PI / 2), 0.0;
	std::cout << "set DH d,a,alpha,offset:\n" << RODH << std::endl;
}

void main()
{
	setDH();
	robotbox robot(RODH);
	robot.RoLink[2][3] = PI / 2;//set Link3.offset=PI/2
	std::cout <<"Link3.A:\n"<< robot.Link_A(robot.RoLink[2])<<std::endl;//Link3.A
	std::cout<<"rpy_r(0.1, 0.2, 0.3):\n" << robot.r_homogeneous(robot.rpy_r(0.1, 0.2, 0.3)) << std::endl;//rpy2r
	Eigen::MatrixXd forwardf; std::vector<double> theta = { 0.0,PI/3,0.0,0.0};
	robot.set_base(0.0, 0.23, 0.0, PI / 2, 0.0, 0.0);//set the pose of robot's base
	if (robot.fkine(forwardf, theta))//Forward kinematics
		std::cout <<"Forward kinematics theta = { 0.0,PI/3,0.0,0.0}:\n"<< forwardf << std::endl;
	theta = { PI / 4,PI / 5,PI/3,PI/7};
	std::cout << "jacobe({ PI / 4,PI / 5,PI/3,PI/7}) matrix:\n" << robot.jacobe(theta) << std::endl;//get the jacobe matrix
	if (robot.fkine(forwardf, { 96 * PI / 180,-75 * PI / 180,120 * PI / 180,-60 * PI / 180 }))
	{
		std::vector<double> theikine;
		if (robot.ikine(theikine, forwardf, { 0.0,0.0,0.0,0.0 }, { 1.0,1.0,1.0,1.0,0.0,0.0 }))//The inverse kinematics
		{
			std::cout << "The inverse kinematics: ";
			for (size_t i=0; i < theikine.size(); i++)
				std::cout << theikine[i] << " ";
			std::cout << std::endl;
		}
	}
	getchar();
}