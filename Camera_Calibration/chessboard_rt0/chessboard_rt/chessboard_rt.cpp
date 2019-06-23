#include <iostream>  
#include <fstream>  
#include <conio.h>
#include "opencv2/core/core.hpp"  
#include "opencv2/imgproc/imgproc.hpp"  
#include "opencv2/calib3d/calib3d.hpp"  
#include "opencv2/highgui/highgui.hpp"  
#include"eigen-eigen\Eigen\Core"
#include"eigen-eigen\Eigen\Geometry"


cv::Mat image_analysis(int number);//read picture
bool find_point(cv::Mat &chessboard, std::vector<cv::Point2f> &image_piont_buf, cv::Size board_size);//find corner on the chessboard picture
void save_conrner(std::vector<cv::Point2f> image_piont_buf, int number);//save corner to .txt
std::vector<cv::Point2f> world_corner(cv::Size board_size, cv::Size2f realsize);//calculate corner point at world coordinate
bool get_r_t(std::vector<cv::Point2f> world_coordinate2, std::vector<cv::Point2f> image_piont,
	cv::Mat InstrinsicMatrix, cv::Mat distortion, cv::Mat &axis, cv::Mat &tra);

cv::Mat image_analysis(int number)//read picture
{
	std::string file_name = std::to_string(number);
	file_name = "chessboardpicture/" + file_name + ".jpg";
	return cv::imread(file_name);
}

bool find_point(cv::Mat &chessboard, std::vector<cv::Point2f> &image_piont_buf, cv::Size board_size)//find corner on the chessboard picture
{
	cv::Mat grayImage;
	grayImage.create(chessboard.size(), chessboard.type());
	cv::cvtColor(chessboard, grayImage, CV_BGR2GRAY);
	if (cv::findChessboardCorners(grayImage, board_size, image_piont_buf))
	{
		//cv::find4QuadCornerSubpix(grayImage, image_piont_buf, cv::Size(5, 5));
		//cv::drawChessboardCorners(frame, board_size, image_piont_buf, true);
		for (unsigned int point_count = 0; point_count < image_piont_buf.size(); point_count++)
		{
			std::string str = std::to_string(point_count);
			cv::putText(chessboard, str, image_piont_buf[point_count], cv::FONT_HERSHEY_DUPLEX, 0.5, cv::Scalar(0, 255, 0), 2);
		}
	}
	else return false;
	return true;
}

void save_conrner(std::vector<cv::Point2f> image_piont_buf, int number)//save corner to .txt
{
	std::string file_name = std::to_string(number);
	file_name = "chessboardpicture/" + file_name + ".txt";
	std::ofstream thepoint(file_name);
	thepoint << image_piont_buf << std::endl;
	thepoint.close();
}

std::vector<cv::Point2f> world_corner(cv::Size board_size, cv::Size2f realsize)//calculate corner point at world coordinate
{
	int width = board_size.width;
	int height = board_size.height;
	std::vector<cv::Point2f> world_corner;
	//cv::Point2f tempcorner = ((realsize.width)*(width-1), (realsize.height)*(height-1));
	cv::Point2f tempcorner;
	//tempcorner.x = (realsize.width)*(width - 1);
	//tempcorner.y = (realsize.height)*(height - 1);
	int j = 0, k = 0;
	for (int i = 0; i < (width*height); i++)
	{
		if (j == width)
		{
			j = 0;
			k++;
		}
		tempcorner.x = (realsize.width)*(width - 1);
		tempcorner.y = (realsize.height)*(height - 1);
		tempcorner.x -= ((realsize.width)*(float)j);
		tempcorner.y -= ((realsize.height)*(float)k);
		j++;
		world_corner.push_back(tempcorner);
	}
	return world_corner;
}

bool get_r_t(std::vector<cv::Point2f> world_coordinate2,std::vector<cv::Point2f> image_piont,
	cv::Mat InstrinsicMatrix,cv::Mat distortion,cv::Mat &axis,cv::Mat &tran)
{
	Eigen::MatrixXd R_T = Eigen::MatrixXd::Zero(4, 4);
	std::vector<cv::Point3f> world_coordinate3;
	for (unsigned int i = 0; i < world_coordinate2.size(); i++)
	{
		cv::Point3f temp;
		temp.z = 0; temp.x = world_coordinate2[i].y;
		temp.y = world_coordinate2[i].x;
		world_coordinate3.push_back(temp);
	}
	if (cv::solvePnPRansac(world_coordinate3, image_piont, InstrinsicMatrix, distortion, axis, tran))
		return true;
	return false;
}

int main()
{
	cv::Mat InstrinsicMatrix = (cv::Mat_<float>(3, 3) << 8.813793347015044e+02, 0.0, 3.032184630467796e+02,
		0, 8.795027373092486e+02, 2.852090395675019e+02, 0.0, 0.0, 1.0);
	cv::Mat distortion = (cv::Mat_<float>(1, 4) << -0.069282613226226, 0.362100790633439, 0.0, 0.0);

	cv::Size board_size = cv::Size(11, 8);//corner size
	cv::Size2f realsize(3.0f, 3.0f);//every grid real size
	std::vector<cv::Point2f> image_piont_buf;//corner on pictur
											 //std::vector<std::vector<cv::Point2f>> image_points_all;
	std::vector<cv::Point2f> world_coordinate;//world coordinate
	cv::Mat axis, tran, R;
	cv::Mat frame;
	world_coordinate = world_corner(board_size, realsize);

	cv::VideoCapture capture;
	capture.open(0);
	int ch;
	while (1)
	{
		capture >> frame;
		if (_kbhit())
		{
			ch = _getch();
			if (ch == 115)break;//press's' to exit
			if (find_point(frame, image_piont_buf, board_size))
			{
				if (get_r_t(world_coordinate, image_piont_buf, InstrinsicMatrix, distortion, axis, tran))
				{
					cv::Rodrigues(axis, R);
					std::cout << "R:\n" << R << std::endl << "tran:\n" << tran << "\n\n";
				}
			}
		}
		cv::imshow("chessboard", frame);
		cv::waitKey(10);
	}
	system("pause");
	return 0;
}