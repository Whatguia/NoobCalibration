#include<iostream>
#include<opencv2/opencv.hpp>
#include"intrinsic.hpp"

cv::Mat image;
cv::Mat image_extend;
cv::Mat resize_image;

//event鼠标事件代号，x,y鼠标坐标，flags拖拽和键盘操作的代号
void on_mouse(int event,int x,int y,int flags,void *ustc)
{
    cv::resize(
            image_extend(cv::Range(y,y+100),cv::Range(x,x+100)),
            resize_image,
            cv::Size(1000,1000)
    );  //从扩展图片中获取选定区域作为放大图像

    //在放大图中绘制准星来指示选定的原图中的像素点
    cv::line(resize_image,cv::Point(0,505),cv::Point(500,505),cv::Scalar(0,0,255));
    cv::line(resize_image,cv::Point(510,505),cv::Point(1000,505),cv::Scalar(0,0,255));
    cv::line(resize_image,cv::Point(505,0),cv::Point(505,500),cv::Scalar(0,0,255));
    cv::line(resize_image,cv::Point(505,510),cv::Point(505,1000),cv::Scalar(0,0,255));
    cv::line(resize_image,cv::Point(205,205),cv::Point(405,405),cv::Scalar(0,255,0));
    cv::line(resize_image,cv::Point(605,605),cv::Point(805,805),cv::Scalar(0,255,0));
    cv::line(resize_image,cv::Point(205,805),cv::Point(405,605),cv::Scalar(0,255,0));
    cv::line(resize_image,cv::Point(605,405),cv::Point(805,205),cv::Scalar(0,255,0));
    cv::circle(resize_image,cv::Point(505,505),10,cv::Scalar(0,0,255));
    cv::circle(resize_image,cv::Point(505,505),250,cv::Scalar(0,0,255));
    cv::putText(resize_image,"["+std::to_string(x)+","+std::to_string(y)+"]",cv::Point(515,495),cv::FONT_HERSHEY_PLAIN,1,cv::Scalar(0,255,0));
    
    if(event==cv::EVENT_LBUTTONDOWN)
    {
        std::cout<<"["<<x<<","<<y<<"]"<<std::endl;
        cv::circle(image_extend,cv::Point(x+50,y+50),0,cv::Scalar(0,0,255));    //单击鼠标左键后在扩展图像中标记位置
    }
    cv::imshow(" ",resize_image);
}

int main(int argc,char** argv)
{
    if(argc!=2&&argc!=3)
	{
	    std::cout<<"Usage: ./get_pixel <image_path> Optional:<intrinsic_json_path>\n"
				"example:\n"
				"\t./bin/get_pixel ./data/undistorted_image.jpg\n"
                "or:\n"
                "\t./bin/get_pixel ./data/image.jpg ./data/calibration.json"
                <<std::endl;
		return 0;
	}
    std::string image_path=argv[1];
    image=cv::imread(image_path);
    image_extend=cv::Mat::zeros(image.rows+100,image.cols+100,CV_8UC3);

    if(argc==3)
    {
        std::string intrinsic_json_path=argv[2];    //内参json文件路径
        cv::Mat intrinsic,distortion;   //相机内参、畸变系数
        cv::Size image_size; //相机内参对应的图像大小
        cv::Mat undistorted_image;
        loadIntrinsic(intrinsic_json_path,intrinsic,distortion,image_size,false);   //载入原始内参矩阵、畸变参数、图像大小
        cv::undistort(image,undistorted_image,intrinsic,distortion);
        image=undistorted_image;
    }

    image.copyTo(image_extend(cv::Rect(50,50,image.cols,image.rows)));
    
    cv::namedWindow(image_path);
    while(true)
    {
        cv::setMouseCallback(image_path,on_mouse,0);   //调用回调函数
        cv::imshow(image_path,image);
        if(cv::waitKey()==27)
        {
            break;
        }
    }
    return 0;
}