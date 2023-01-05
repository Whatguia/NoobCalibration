#include<iostream>
#include<opencv2/opencv.hpp>

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
    
    if(event==cv::EVENT_LBUTTONDOWN)
    {
        std::cout<<"["<<x<<","<<y<<"]"<<std::endl;
        cv::circle(image_extend,cv::Point(x+50,y+50),0,cv::Scalar(0,0,255));    //单击鼠标左键后在扩展图像中标记位置
    }
    cv::imshow(" ",resize_image);
}
int main(int argc,char** argv)
{
    if(argc!=2)
	{
	    std::cout<<"Usage: ./Get_Pixel <image_path>\n"
				"example:\n"
				"\t./bin/Get_Pixel ./data/test.png"
                <<std::endl;
		return 0;
	}
    std::string image_path=argv[1];
    image=cv::imread(image_path);
    image_extend=cv::Mat::zeros(image.rows+100,image.cols+100,CV_8UC3);
    image.copyTo(image_extend(cv::Rect(50,50,image.cols,image.rows)));

    cv::namedWindow("image");
    while(true)
    {
        cv::setMouseCallback("image",on_mouse,0);   //调用回调函数
        cv::imshow("image",image);
        if(cv::waitKey()==27)
        {
            break;
        }
    }
    return 0;
}