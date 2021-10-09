//  This file is part of fox_robotic_gadget/jacobe_ikine.
	
//  Copyright (C) 2021, by ChyS(foxchys)

//  This program is free software: you can redistribute it and/or modify
//  it under the terms of the GNU Affero General Public License as published
//  by the Free Software Foundation, either version 3 of the License, or
//  (at your option) any later version.

//  This program is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU Affero General Public License for more details.

//  You should have received a copy of the GNU Affero General Public License
//  along with this program.  If not, see <https://www.gnu.org/licenses/>.
	
//  ChyS(foxchys): https://github.com/foxchys

#include"robotbox.h"


robotbox::robotbox(Eigen::MatrixXd DH)
{
	int rows = DH.rows(), cols = DH.cols();
	std::array<double, 5> temp = { 0.0 };
	for (int i = 0; i < rows; i++)
	{
		for (int j = 0; j < cols; j++)
		{
			temp[j] = DH(i, j);
		}
		RoLink.push_back(temp);
		temp = { 0.0 };
	}
	Link_Base = Eigen::MatrixXd::Zero(4, 4);
	Link_Base(0, 0) = 1.0; Link_Base(1, 1) = 1.0; Link_Base(2, 2) = 1.0; Link_Base(3, 3) = 1.0;
}

robotbox::~robotbox()
{
}

Eigen::MatrixXd robotbox::Link_A(std::array<double, 5> Link)//m pi = 3.14...////T(theta+offset)T(d)T(a)T(alpha)
{
	double theta = Link[3]+Link[4],  d = Link[0],  a = Link[1],  alpha = Link[2];
	Eigen::MatrixXd temp_homogeneous = Eigen::MatrixXd::Zero(4, 4);
	temp_homogeneous(0, 0) = cos(theta); temp_homogeneous(0, 1) = -sin(theta)*cos(alpha);
	temp_homogeneous(0, 2) = sin(theta)*sin(alpha); temp_homogeneous(0, 3) = a*cos(theta);
	temp_homogeneous(1, 0) = sin(theta); temp_homogeneous(1, 1) = cos(theta)*cos(alpha);
	temp_homogeneous(1, 2) = -cos(theta)*sin(alpha); temp_homogeneous(1, 3) = a*sin(theta);
	temp_homogeneous(2, 0) = 0.0; temp_homogeneous(2, 1) = sin(alpha);
	temp_homogeneous(2, 2) = cos(alpha); temp_homogeneous(2, 3) = d;
	temp_homogeneous(3, 3) = 1.0;
	return temp_homogeneous;
}
//.......written by FoxSu chy_s@outlook.com
void robotbox::set_base(double x, double y, double z, double row, double pitch, double yaw)
{
	Link_Base = r_homogeneous(rpy_r(row, pitch, yaw));
	Link_Base(0, 3) = x; Link_Base(1, 3) = y; Link_Base(2, 3) = z;
}

bool robotbox::fkine(Eigen::MatrixXd &Forward,std::vector<double> theta)//Forward kinematics
{
	std::vector<std::array<double, 5>> templink = RoLink;
	int qnumber = theta.size();
	Eigen::MatrixXd temp = Eigen::MatrixXd::Zero(4, 4);
	temp(0, 0) = 1.0; temp(1, 1) = 1.0; temp(2, 2) = 1.0; temp(3, 3) = 1;
	do
	{
		if (qnumber!= templink.size())
		{
			break;
		}
		else
		{
			for (unsigned int i = 0; i < templink.size(); i++)
			{
				templink[i][4] = theta[i];
				temp = temp*Link_A(templink[i]);
			}
			Forward = Link_Base*temp;
			return true;
		}
	} while (false);
	return false;
}

Eigen::MatrixXd robotbox::r_homogeneous(Eigen::Matrix3d r)
{
	Eigen::MatrixXd homotem = Eigen::MatrixXd::Zero(4, 4);
	for (int i = 0; i <= 3; i++)
		for (int j = 0; j <= 3; j++)
		{
			if (i != 3 && j == 3)
			{
				homotem(i, j) = 0.0; continue;
			}
			else if (i == 3 && j == 3)
			{
				homotem(i, j) = 1.0; continue;
			}
			else if (i < 3 && j < 3)
			{
				homotem(i, j) = r(i, j); continue;
			}
			else
				continue;
		}
	return homotem;
}

Eigen::Matrix3d robotbox::rpy_r(double row, double pitch, double yaw)//x,y,z,pi=3.14...
{
	Eigen::AngleAxisd rotation_vector_row(row, Eigen::Vector3d(1, 0, 0));
	Eigen::AngleAxisd rotation_vector_pitch(pitch, Eigen::Vector3d(0, 1, 0));
	Eigen::AngleAxisd rotation_vector_yaw(yaw, Eigen::Vector3d(0, 0, 1));
	return ((rotation_vector_row.toRotationMatrix())*(rotation_vector_pitch.toRotationMatrix())
		*(rotation_vector_yaw.toRotationMatrix()));
}
//.......written by FoxSu chy_s@outlook.com
Eigen::MatrixXd robotbox::EulerAngle_R(double x, double y, double z)
{
	Eigen::AngleAxisd rotation_vector_row(x, Eigen::Vector3d(1, 0, 0));
	Eigen::AngleAxisd rotation_vector_pitch(y, Eigen::Vector3d(0, 1, 0));
	Eigen::AngleAxisd rotation_vector_yaw(z, Eigen::Vector3d(0, 0, 1));
	return ((rotation_vector_yaw.toRotationMatrix())*(rotation_vector_pitch.toRotationMatrix())
		*(rotation_vector_row.toRotationMatrix()));
}

std::vector<double> robotbox::R_EulerAngle(Eigen::MatrixXd R)
{
	std::vector<double> temp(3);
	double tempp;
	tempp = atan2(R(2, 1), R(2, 2));
	temp[0]=tempp;
	tempp = atan2(-R(2, 0), sqrt(R(2, 1)*R(2,1)+R(2,2)*R(2,2)));
	temp[1] = tempp;
	tempp = atan2(R(1, 0), R(0, 0));
	temp[2] = tempp;
	return temp;
}

Eigen::MatrixXd robotbox::jacobe(std::vector<double> theta)
{
	std::vector<std::array<double, 5>> temp = RoLink;
	Eigen::MatrixXd jacobematrix = Eigen::MatrixXd::Zero(6, temp.size());
	Eigen::MatrixXd Txn = Eigen::MatrixXd::Zero(4, 4);
	Eigen::MatrixXd TR = Eigen::MatrixXd::Zero(3, 3);
	Eigen::MatrixXd Px = Eigen::MatrixXd::Zero(3, 3);
	Txn(0, 0) = 1.0; Txn(1, 1) = 1.0; Txn(2, 2) = 1.0; Txn(3, 3) = 1.0;
	int num = theta.size();
	try
	{
		if (num == temp.size())
		{
			for (int i = 0; i < num; i++)
			{
				temp[(temp.size() - 1 - i)][4] = theta[(temp.size() - 1 - i)];
				Txn = Link_A(temp[(temp.size() -1-i)])*Txn;

				TR = Txn.topLeftCorner(3, 3);
				Px(0, 1) = -Txn(2, 3); Px(0, 2) = Txn(1, 3); Px(1, 2) = -Txn(0, 3);
				Px(1, 0) = Txn(2, 3); Px(2, 0) = -Txn(1, 3); Px(2, 1) = Txn(0, 3);
				jacobematrix(0, (temp.size() - 1 - i)) = (-TR.adjoint()*Px)(0, 2); 
				jacobematrix(1, (temp.size() - 1 - i)) = (-TR.adjoint()*Px)(1, 2);
				jacobematrix(2, (temp.size() - 1 - i)) = (-TR.adjoint()*Px)(2, 2);
				jacobematrix(3, (temp.size() - 1 - i)) = Txn(2, 0); 
				jacobematrix(4, (temp.size() - 1 - i)) = Txn(2, 1);
				jacobematrix(5, (temp.size() - 1 - i)) = Txn(2, 2);
			}
			return jacobematrix;
		}
		else throw num;
	}
	catch (int)
	{
		//std::cout << "this is not a " << i << " degrees of freedom robot!\n press enter to eixt!" << std::endl;
		//getchar();
		exit(1);
	}
}
//.......written by FoxSu chy_s@outlook.com
bool robotbox::ikine(std::vector<double> &qt, Eigen::MatrixXd tr, std::vector<double> qo, std::vector<double> mask, size_t ilimit,
	size_t rlimit, double tol, double lambda, double lambdamin)//The inverse kinematics....do the damped inverse Gauss-Newton with Levenberg-Marquadt
{
	size_t iterations = 0, rejcount = 0;
	std::vector<double> q = qo, qnew(qo.size());
	Eigen::MatrixXd W = Eigen::MatrixXd::Zero(mask.size(), mask.size());
	Eigen::MatrixXd e, forward, J, JtJ, dq, enew;
	for (size_t i = 0; i < mask.size(); i++)
		W(i, i) = mask[i];
	while (true)
	{
		fkine(forward, q);
		e = tr2delta(forward, tr);
		if ((W*e.adjoint()).norm() < tol)
			break;
		iterations = iterations + 1;
		if (iterations > ilimit)
			return false;
		J = jacobe(q);
		JtJ = J.adjoint()*W*J;
		Eigen::MatrixXd upltemp = Eigen::MatrixXd::Identity(JtJ.rows(), JtJ.cols());
		dq = ((JtJ + (lambda + lambdamin)*upltemp).inverse())*J.adjoint()*W*e.adjoint();
		for (size_t i = 0; i < q.size(); i++)
			qnew[i] = q[i] + dq(i, 0);
		fkine(forward, qnew);
		enew = tr2delta(forward, tr);
		if ((W*enew.adjoint()).norm() < (W*e.adjoint()).norm())
		{
			q = qnew; e = enew; lambda = lambda / 2; rejcount = 0;
		}
		else
		{
			lambda = lambda * 2; rejcount = rejcount + 1;
			if (rejcount > rlimit)
				return false;
		}
		for (size_t i = 0; i < q.size(); i++)
		{
			if (q[i] > PI)
				q[i] = q[i] - 2 * PI;
			else if(q[i]<-PI)
				q[i] = q[i] + 2 * PI;
		}
	}
	qt = q;
	return true;
}

Eigen::MatrixXd robotbox::tr2delta(Eigen::MatrixXd ta, Eigen::MatrixXd tb)
{
	Eigen::MatrixXd xyzrxyz = Eigen::MatrixXd::Zero(1, 6);;
	Eigen::MatrixXd ta_tb = (ta.inverse())*tb;
	xyzrxyz(0, 0) = ta_tb(0, 3); xyzrxyz(0, 1) = ta_tb(1, 3); xyzrxyz(0, 2) = ta_tb(2, 3);
	std::vector<double> rxyz = vex(ta_tb.topLeftCorner(3, 3)- Eigen::MatrixXd::Identity(3, 3));
	xyzrxyz(0, 3) = rxyz[0]; xyzrxyz(0, 4) = rxyz[1]; xyzrxyz(0, 5) = rxyz[2];
	return xyzrxyz;
}

std::vector<double> robotbox::vex(Eigen::MatrixXd S)
{
	std::vector<double> temp(3);
	temp[0] = 0.5*(S(2, 1) - S(1, 2));
	temp[1] = 0.5*(S(0, 2) - S(2, 0));
	temp[2] = 0.5*(S(1, 0) - S(0, 1));
	return temp;
}
//.......written by FoxSu chy_s@outlook.com
Eigen::MatrixXd robotbox::delta2tr(Eigen::MatrixXd d)
{
	Eigen::MatrixXd tr = Eigen::MatrixXd::Identity(4, 4);
	tr(0, 3) = d(0,0); tr(1, 3) = d(0,1); tr(2, 3) = d(0,2);
	tr(0, 1) = -d(0,5); tr(0, 2) = d(0,4); tr(1, 2) = -d(0,3);
	tr(1, 0) = d(0,5); tr(2, 0) = -d(0,4); tr(2, 1) = d(0,3);
	return tr;
}

template<typename _Matrix_Type_>
_Matrix_Type_ robotbox::pseudoInverse(const _Matrix_Type_ &a, double epsilon)
{
	Eigen::JacobiSVD< _Matrix_Type_ > svd(a, Eigen::ComputeThinU | Eigen::ComputeThinV);
	double tolerance = epsilon * std::max(a.cols(), a.rows()) *svd.singularValues().array().abs()(0);
	return svd.matrixV() *  (svd.singularValues().array().abs() > tolerance).select(svd.singularValues().array().inverse(),
		0).matrix().asDiagonal() * svd.matrixU().adjoint();
}

//.......written by FoxSu chy_s@outlook.com