#include "opencv2/core/core.hpp"  
#include "opencv2/imgproc/imgproc.hpp"  
#include "opencv2/calib3d/calib3d.hpp"  
#include "opencv2/highgui/highgui.hpp"  
#include <iostream>  
#include <fstream>  

int main()
{
	cv::VideoCapture capture;
	capture.open(0);
	cv::Size image_size;
	cv::Size board_size = cv::Size(11, 8);
	std::vector<cv::Point2f> image_piont_buf;
	std::vector<std::vector<cv::Point2f>> image_points_all;
	int corner_count=0;
	while (1)
	{
		cv::Mat frame;
		cv::Mat grayImage;
		capture >> frame;
		grayImage.create(frame.size(), frame.type());
		cv::cvtColor(frame, grayImage, CV_BGR2GRAY);
		if (cv::findChessboardCorners(grayImage, board_size, image_piont_buf))
		{
			cv::find4QuadCornerSubpix(grayImage, image_piont_buf, cv::Size(5, 5));
			std::string path_name;
			path_name = std::to_string(corner_count);
			path_name += ".jpg";
			cv::imwrite(path_name, frame);
			cv::drawChessboardCorners(frame, board_size, image_piont_buf, true);
			corner_count++;
			cv::waitKey(5000);
		}
		imshow("thechess", frame);
		if (cv::waitKey(30) >= 0) break;
	}
}