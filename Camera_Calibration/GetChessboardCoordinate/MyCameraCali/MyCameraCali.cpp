#include "opencv2/core/core.hpp"  
#include "opencv2/imgproc/imgproc.hpp"  
#include "opencv2/calib3d/calib3d.hpp"  
#include "opencv2/highgui/highgui.hpp"  
#include <iostream>  
#include <fstream>  

cv::Mat image_analysis(int number);//read picture
bool find_point(cv::Mat &chessboard, std::vector<cv::Point2f> &image_piont_buf, cv::Size board_size);//find corner on the chessboard picture
void save_conrner(std::vector<cv::Point2f> image_piont_buf, int number);//save corner to .txt
std::vector<cv::Point2f> world_corner(cv::Size board_size, cv::Size2f realsize);//calculate corner point at world coordinate

cv::Mat image_analysis(int number)//read picture
{
	std::string file_name = std::to_string(number);
	file_name = "ChessBoardPitcure/" + file_name + ".jpg";
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

void save_conrner(std::vector<cv::Point2f> image_piont_buf,int number)//save corner to .txt
{
	std::string file_name = std::to_string(number);
	file_name ="ChessBoardPitcureConner/" + file_name +".txt";
	std::ofstream thepoint(file_name);
	thepoint << image_piont_buf << std::endl;
	thepoint.close();
}

std::vector<cv::Point2f> world_corner(cv::Size board_size,cv::Size2f realsize)//calculate corner point at world coordinate
{
	int width = board_size.width;
	int height = board_size.height;
	std::vector<cv::Point2f> world_corner;
	//cv::Point2f tempcorner = ((realsize.width)*(width-1), (realsize.height)*(height-1));
	cv::Point2f tempcorner;
	//tempcorner.x = (realsize.width)*(width - 1);
	//tempcorner.y = (realsize.height)*(height - 1);
	int j = 0, k = 0;
	for (unsigned int i = 0; i < (width*height); i++)
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

int main()
{
	cv::Size board_size = cv::Size(11, 8);//corner size
	int image_all = 15;//picture count
	cv::Size2f realsize(3.0f, 3.0f);//every grid real size
	std::vector<cv::Point2f> image_piont_buf;//corner on pictur
	//std::vector<std::vector<cv::Point2f>> image_points_all;
	std::vector<cv::Point2f> world_coordinate;//world coordinate
	cv::Mat frame;
	cv::Mat grayImage;

	for (int image_number = 0; image_number <= image_all; image_number++)
	{
		frame = image_analysis(image_number);
		if (find_point(frame, image_piont_buf, board_size)) std::cout << "picture " << image_number << " find all corner!" << std::endl;
		else { std::cout << "can not find corner at number " << image_number<< " picture!" << std::endl; continue; }
		cv::imshow("thechess", frame);
		save_conrner(image_piont_buf, image_number);
		cv::waitKey(2000);
	}
	std::cout << "all picture is checked!" << std::endl;
	//image_points_all.push_back(image_piont_buf);
	world_coordinate = world_corner(board_size, realsize);
	save_conrner(world_coordinate, 777);
	return 0;
}