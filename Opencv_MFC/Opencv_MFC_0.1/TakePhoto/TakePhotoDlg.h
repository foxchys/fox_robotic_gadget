// TakePhotoDlg.h : 头文件
//

#pragma once

//////opencv头文件//////////////// 

//#include "opencv\cv.h"
//#include "opencv\highgui.h"


#include"CvvImage.h"//结合mfc显示图像的类
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
/////////////////////////////////
//#include "opencv2/objdetect/objdetect.hpp"
//#include "opencv2/imgproc/imgproc.hpp"
//#include <ctime>//Windows系统世界操作

// CTakePhotoDlg 对话框
class CTakePhotoDlg : public CDialogEx
{
// 构造
public:
	CTakePhotoDlg(CWnd* pParent = NULL);	// 标准构造函数

// 对话框数据
#ifdef AFX_DESIGN_TIME
	enum { IDD = IDD_TAKEPHOTO_DIALOG };
#endif

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);	// DDX/DDV 支持


// 实现
protected:
	HICON m_hIcon;

	// 生成的消息映射函数
	virtual BOOL OnInitDialog();
	afx_msg void OnSysCommand(UINT nID, LPARAM lParam);
	afx_msg void OnPaint();
	afx_msg HCURSOR OnQueryDragIcon();
	DECLARE_MESSAGE_MAP()
public:
	//void DrawPicToHDC(IplImage *img, UINT ID);//摄像头显示到界面
	void DrawPicToHDC(cv::Mat img, UINT ID);//摄像头显示到界面
	afx_msg void OnBnClickedCamerasw();
	afx_msg void OnBnClickedExit();
	static UINT ShowCamera(void* param);
	bool ShowCaFlg;
};
