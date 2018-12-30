
// TakePhotoDlg.cpp : 实现文件
//

#include "stdafx.h"
#include "TakePhoto.h"
#include "TakePhotoDlg.h"
#include "afxdialogex.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#endif


// 用于应用程序“关于”菜单项的 CAboutDlg 对话框

class CAboutDlg : public CDialogEx
{
public:
	CAboutDlg();

// 对话框数据
#ifdef AFX_DESIGN_TIME
	enum { IDD = IDD_ABOUTBOX };
#endif

	protected:
	virtual void DoDataExchange(CDataExchange* pDX);    // DDX/DDV 支持

// 实现
protected:
	DECLARE_MESSAGE_MAP()
};

CAboutDlg::CAboutDlg() : CDialogEx(IDD_ABOUTBOX)
{
}

void CAboutDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
}

BEGIN_MESSAGE_MAP(CAboutDlg, CDialogEx)
END_MESSAGE_MAP()


// CTakePhotoDlg 对话框



CTakePhotoDlg::CTakePhotoDlg(CWnd* pParent /*=NULL*/)
	: CDialogEx(IDD_TAKEPHOTO_DIALOG, pParent)
{
	m_hIcon = AfxGetApp()->LoadIcon(IDR_MAINFRAME);
}

void CTakePhotoDlg::DoDataExchange(CDataExchange* pDX)
{
	CDialogEx::DoDataExchange(pDX);
}

BEGIN_MESSAGE_MAP(CTakePhotoDlg, CDialogEx)
	ON_WM_SYSCOMMAND()
	ON_WM_PAINT()
	ON_WM_QUERYDRAGICON()
	ON_BN_CLICKED(IDC_CameraSw, &CTakePhotoDlg::OnBnClickedCamerasw)
	ON_BN_CLICKED(IDC_Exit, &CTakePhotoDlg::OnBnClickedExit)
END_MESSAGE_MAP()


// CTakePhotoDlg 消息处理程序

BOOL CTakePhotoDlg::OnInitDialog()
{
	CDialogEx::OnInitDialog();

	// 将“关于...”菜单项添加到系统菜单中。

	// IDM_ABOUTBOX 必须在系统命令范围内。
	ASSERT((IDM_ABOUTBOX & 0xFFF0) == IDM_ABOUTBOX);
	ASSERT(IDM_ABOUTBOX < 0xF000);

	CMenu* pSysMenu = GetSystemMenu(FALSE);
	if (pSysMenu != NULL)
	{
		BOOL bNameValid;
		CString strAboutMenu;
		bNameValid = strAboutMenu.LoadString(IDS_ABOUTBOX);
		ASSERT(bNameValid);
		if (!strAboutMenu.IsEmpty())
		{
			pSysMenu->AppendMenu(MF_SEPARATOR);
			pSysMenu->AppendMenu(MF_STRING, IDM_ABOUTBOX, strAboutMenu);
		}
	}

	// 设置此对话框的图标。  当应用程序主窗口不是对话框时，框架将自动
	//  执行此操作
	SetIcon(m_hIcon, TRUE);			// 设置大图标
	SetIcon(m_hIcon, FALSE);		// 设置小图标

	// TODO: 在此添加额外的初始化代码

	GetDlgItem(IDC_CameraSw)->SetWindowTextW(_T("打开摄像头"));
	ShowCaFlg = false;

	return TRUE;  // 除非将焦点设置到控件，否则返回 TRUE
}

void CTakePhotoDlg::OnSysCommand(UINT nID, LPARAM lParam)
{
	if ((nID & 0xFFF0) == IDM_ABOUTBOX)
	{
		CAboutDlg dlgAbout;
		dlgAbout.DoModal();
	}
	else
	{
		CDialogEx::OnSysCommand(nID, lParam);
	}
}

// 如果向对话框添加最小化按钮，则需要下面的代码
//  来绘制该图标。  对于使用文档/视图模型的 MFC 应用程序，
//  这将由框架自动完成。

void CTakePhotoDlg::OnPaint()
{
	if (IsIconic())
	{
		CPaintDC dc(this); // 用于绘制的设备上下文

		SendMessage(WM_ICONERASEBKGND, reinterpret_cast<WPARAM>(dc.GetSafeHdc()), 0);

		// 使图标在工作区矩形中居中
		int cxIcon = GetSystemMetrics(SM_CXICON);
		int cyIcon = GetSystemMetrics(SM_CYICON);
		CRect rect;
		GetClientRect(&rect);
		int x = (rect.Width() - cxIcon + 1) / 2;
		int y = (rect.Height() - cyIcon + 1) / 2;

		// 绘制图标
		dc.DrawIcon(x, y, m_hIcon);
	}
	else
	{
		CDialogEx::OnPaint();
	}
}

//当用户拖动最小化窗口时系统调用此函数取得光标
//显示。
HCURSOR CTakePhotoDlg::OnQueryDragIcon()
{
	return static_cast<HCURSOR>(m_hIcon);
}


//
//void CTakePhotoDlg::OnBnClickedCamerasw()
//{
//	// TODO: Add your control notification handler code here
//	IplImage *image = NULL; //原始图像
//	if (image) cvReleaseImage(&image);
//	image = cvLoadImage("D:\\demo.jpg",1); //显示图片
//	/*cv::VideoCapture capture;
//	capture.open(0);
//	cv::Mat frame;
//	capture >> frame;*/
//	//IplImage CamTem;
//	//CamTem = frame;
//	//image = cvCloneImage(&CamTem);
//	DrawPicToHDC(image, IDC_TheCamera);
//	/*DrawPicToHDC(frame, IDC_TheCamera);*/
//}
//
//void CTakePhotoDlg::DrawPicToHDC(IplImage *img, UINT ID)
//{
//	CDC *pDC = GetDlgItem(ID)->GetDC();
//	HDC hDC = pDC->GetSafeHdc();
//	CRect rect;
//	GetDlgItem(ID)->GetClientRect(&rect);
//	CvvImage cimg;
//	cimg.CopyOf(img); // 复制图片
//	cimg.DrawToHDC(hDC, &rect); // 将图片绘制到显示控件的指定区域内
//	ReleaseDC(pDC);
//}



void CTakePhotoDlg::DrawPicToHDC(cv::Mat img, UINT ID)
{
	IplImage *CamShTem;
	IplImage Temp;
	Temp = img;
	CamShTem = cvCloneImage(&Temp);

	CDC *pDC = GetDlgItem(ID)->GetDC();
	HDC hDC = pDC->GetSafeHdc();
	CRect rect;
	GetDlgItem(ID)->GetClientRect(&rect);
	CvvImage cimg;
	cimg.CopyOf(CamShTem, CamShTem->nChannels); // 复制图片
	cimg.DrawToHDC(hDC, &rect); // 将图片绘制到显示控件的指定区域内
	cimg.Destroy();
	ReleaseDC(pDC);
	cvReleaseImage(&CamShTem);
}


void CTakePhotoDlg::OnBnClickedCamerasw()
{
	CString temp;
	GetDlgItem(IDC_CameraSw)->GetWindowTextW(temp);
	if (temp == "打开摄像头")
	{
		ShowCaFlg = true;
		AfxBeginThread(ShowCamera, this, THREAD_PRIORITY_HIGHEST);
		GetDlgItem(IDC_CameraSw)->SetWindowTextW(_T("关闭摄像头"));
	}
	else
	{
		ShowCaFlg = false;
	}
}

UINT CTakePhotoDlg::ShowCamera(void* param)
{
	CTakePhotoDlg* dlg = (CTakePhotoDlg*)param;

	std::vector<cv::Rect> faces;
	cv::CascadeClassifier face_cascade;
	cv::String face_cascade_name = "haarcascade_frontalface_alt.xml";
	face_cascade.load(face_cascade_name);

	/*CTakePhotoDlg* dlg = (CTakePhotoDlg*)param;*/
	cv::VideoCapture capture;
	capture.open(0);
	cv::Mat frame;
	cv::Mat grayImage;
	while (dlg->ShowCaFlg)
	{
		capture >> frame;
		grayImage.create(frame.size(), frame.type());
		cv::cvtColor(frame, grayImage, CV_BGR2GRAY);
		face_cascade.detectMultiScale(grayImage, faces, 1.1, 3, 0 | CV_HAAR_DO_ROUGH_SEARCH, cv::Size(100, 100));
		for (size_t i = 0; i < faces.size(); i++)
		{
			//cv::Point center(faces[i].x + faces[i].width*0.5, faces[i].y + faces[i].height*0.5);//计算脸部圆心
			//ellipse(frame, center, cv::Size(faces[i].width*0.5, faces[i].height*0.5), 0, 0, 360, cv::Scalar(0, 255, 0), 4, 8, 0);//在脸部画圆

			cv::Rect select;//声明矩形  
			select.x = faces[i].x;
			select.y = faces[i].y;
			select.width = faces[i].width;
			select.height = faces[i].height;
			rectangle(frame, select, cv::Scalar(0, 255, 0), 2, 8, 0);//在脸部画矩形：：参数形式void rectangle(Mat& img, Rect rec, const Scalar& color, int thickness=1, int lineType=8, int shift=0 )
		}
		dlg->DrawPicToHDC(frame, IDC_TheCamera);
		cv::waitKey(30);
	}
	dlg->GetDlgItem(IDC_CameraSw)->SetWindowTextW(_T("打开摄像头"));
	return 0;
}


void CTakePhotoDlg::OnBnClickedExit()
{
	// TODO: Add your control notification handler code here
	CDialogEx::OnCancel();
}
