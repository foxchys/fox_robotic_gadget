#include "opencv2/core/core.hpp"  
#include "opencv2/imgproc/imgproc.hpp"  
#include "opencv2/calib3d/calib3d.hpp"  
#include "opencv2/highgui/highgui.hpp"  
#include <iostream>  
#include <fstream>  

cv::Mat image_analysis(int number);//read picture
bool find_point(cv::Mat &chessboard, std::vector<cv::Point2f> &image_piont_buf, cv::Size board_size);//find corner on the chessboard picture
void save_conrner(std::vector<cv::Point2f> image_piont_buf, int number);//save corner to .txt
std::vector<cv::Point3f> world_corner(cv::Size board_size, cv::Size2f realsize);//calculate corner point at world coordinate

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

std::vector<cv::Point3f> world_corner(cv::Size board_size,cv::Size2f realsize)//calculate corner point at world coordinate
{
	int width = board_size.width;
	int height = board_size.height;
	std::vector<cv::Point3f> world_corner;
	//cv::Point2f tempcorner = ((realsize.width)*(width-1), (realsize.height)*(height-1));
	cv::Point3f tempcorner;
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
		tempcorner.z = 0.0f;
		world_corner.push_back(tempcorner);
	}
	return world_corner;
}

int main()////get corner point coordinate
{
	cv::Size board_size = cv::Size(11, 8);//corner size
	int image_all = 15;//picture count
	cv::Size2f realsize(3.0f, 3.0f);//every grid real size
	std::vector<cv::Point2f> image_piont_buf;//corner on pictur
	std::vector<std::vector<cv::Point2f>> image_points_all;
	std::vector<cv::Point3f> world_coordinate;//world coordinate
	cv::Mat frame;

	for (int image_number = 0; image_number <= image_all; image_number++)
	{
		frame = image_analysis(image_number);
		if (find_point(frame, image_piont_buf, board_size)) std::cout << "picture " << image_number << " find all corner!" << std::endl;
		else { std::cout << "can not find corner at number " << image_number<< " picture!" << std::endl; continue; }
		cv::imshow("thechess", frame);
		save_conrner(image_piont_buf, image_number);
		cv::waitKey(100);
		image_points_all.push_back(image_piont_buf);
	}
	std::cout << "all picture is checked!" << std::endl;
	//image_points_all.push_back(image_piont_buf);
	world_coordinate = world_corner(board_size, realsize);
	std::ofstream thepoint("ChessBoardPitcureConner/world_coordinate.txt");
	thepoint << world_coordinate << std::endl;
	thepoint.close();


	std::vector<std::vector<cv::Point3f>> world_coordinate_all;
	cv::Size image_size;
	cv::Mat cameraMatrix = cv::Mat(3, 3, CV_32FC1, cv::Scalar::all(0)); /* 摄像机内参数矩阵 */
	cv::Mat distCoeffs = cv::Mat(1, 5, CV_32FC1, cv::Scalar::all(0)); /* 摄像机的5个畸变系数：k1,k2,p1,p2,k3 */
	std::vector<cv::Mat> tvecsMat;  /* 每幅图像的旋转向量 */
	std::vector<cv::Mat> rvecsMat; /* 每幅图像的平移向量 */

	image_size.width = frame.cols;
	image_size.height = frame.rows;

	for (int i = 0; i <= image_all; i++)
	{
		world_coordinate_all.push_back(world_coordinate);
	}
	std::cout << "开始标定..............." << std::endl;
	cv::calibrateCamera(world_coordinate_all, image_points_all, image_size, cameraMatrix, distCoeffs, rvecsMat, tvecsMat, 0);
	std::cout << "相机内参数矩阵:" << std::endl;
	std::cout << cameraMatrix << std::endl;

	std::cout << "开始保存定标结果………………" << std::endl;
	std::ofstream fout("ChessBoardPitcureConner/caliberation_result.txt");  /* 保存标定结果的文件 */
	cv::Mat rotation_matrix = cv::Mat(3, 3, CV_32FC1, cv::Scalar::all(0)); /* 保存每幅图像的旋转矩阵 */
	fout << "相机内参数矩阵：" << std::endl;
	fout << cameraMatrix << std::endl << std::endl;
	fout << "畸变系数：\n";
	fout << distCoeffs << std::endl << std::endl << std::endl;
	for (int i = 0; i<=image_all; i++)
	{
		fout << "图: " << i << ".jpg 图像的旋转向量：" << std::endl;
		fout << tvecsMat[i] << std::endl;
		/* 将旋转向量转换为相对应的旋转矩阵 */
		Rodrigues(tvecsMat[i], rotation_matrix);
		fout << "图: " << i << ".jpg 图像的旋转矩阵：" << std::endl;
		fout << rotation_matrix << std::endl;
		fout << "图: " << i << ".jpg 图像的平移向量：" << std::endl;
		fout << rvecsMat[i] << std::endl << std::endl;
	}
	std::cout << "完成保存" << std::endl;
	fout << std::endl;
	fout.close();
	std::cin.get();
	return 0;
}
