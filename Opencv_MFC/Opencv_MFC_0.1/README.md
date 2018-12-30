# Opencv_MFC_0.1
Opencv_MFC_0.1
Opencv_MFC 使用opencv获取摄像头图像，结合MFC显示。
本方法采用opencv已删除的CvvImage.h和CvvImage.cpp从而达到显示摄像头图像的目的。 
    对比https://github.com/NikofoxS/Opencv_MFC_0.0.git  和  https://github.com/NikofoxS/Opencv_MFC_0.2.git
opencv对于MFC中“cv::Mat”析构先后顺序会导致MFC误报内存泄漏。强迫症可采取//在工程上右键-》属性-》c/c++-》代码生成-》运行库 选择MTD //
右击项目->属性->配置属性->常规，然后在右边的“项目默认值”中的“MFC的使用”选项中选择“在静态库中使用MFC”避免报错
