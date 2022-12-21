//  This file is part of fox_robotic_gadget/manipulator_kine.
  
//  Copyright (C) 2021-2022, by ChyS(foxchys)

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

int main(int argc, char** argv)
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
  return 0;
}
