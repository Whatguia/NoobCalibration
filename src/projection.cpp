#include<iostream>
#include<opencv2/opencv.hpp>
#include"intrinsic.hpp"
#include"extrinsic.hpp"
#include"projection.hpp"

//激光点云3D点结构体
struct PointXYZI
{
	cv::Point3d xyz;	//激光点云3D点坐标
	double intensity;	//激光点云反射强度
};

//像素2D点结构体
struct PointXYI
{
	cv::Point2d xy;	//像素2D点坐标
	double intensity;	//像素点对应的激光点云反射强度
};

//按照指定规则切分字符串
std::vector<std::string> stringSplit(std::string str,std::string patten)
{
    std::vector<std::string> retString;
    while(true)
	{
        if(str.find(patten)==std::string::npos)
		{
			break;
		}
        int pos=str.find(patten);
        retString.push_back(str.substr(0,pos));
        str=str.substr(pos+patten.size(),str.size()-pos);
    }
    retString.push_back(str);
    return retString;
}

//以文本的方式读取激光点云PCD文件
std::vector<PointXYZI> loadPCD(const std::string &filename)
{
	std::vector<PointXYZI> pointcloud;

	std::ifstream is(filename);
	if(!is.is_open())
	{
		std::cout<<"Error opening file:"<<filename<<std::endl;
		return pointcloud;
	}
	int fields=3;
	int point_size;
	std::string temp;

	while(std::getline(is,temp))
	{
		std::vector<std::string> temps=stringSplit(temp," ");
		if(!(strcmp(temps[0].c_str(),"FIELDS")))
		{
			fields=temps.size()-1;
		}
		if(!(strcmp(temps[0].c_str(),"POINTS")))
		{
			point_size=std::stoi(temps[1]);
		}
		if(temps.size()==fields)
		{
			PointXYZI data;
			data.xyz.x=std::stod(temps[0]);
			data.xyz.y=std::stod(temps[1]);
			data.xyz.z=std::stod(temps[2]);
			data.intensity=std::stod(temps[3]);

			pointcloud.push_back(data);
		}
	}

	is.close();
	return pointcloud;
}

//按照点的反射强度返回对应的颜色以美化绘制效果
cv::Scalar fakeColor(double value)
{
	double posSlope=255/60.0;
	double negSlope=-255/60.0;
	value*=255;
	cv::Vec3d color;
	if(value<60)
	{
		color[0]=255;
		color[1]=posSlope*value+0;
		color[2]=0;
	}
	else if(value<120)
	{
		color[0]=negSlope*value+2*255;
		color[1]=255;
		color[2]=0;
	}
	else if(value<180)
	{
		color[0]=0;
		color[1]=255;
		color[2]=posSlope*value-2*255;
	}
	else if(value<240)
	{
		color[0]=0;
		color[1]=negSlope*value+4*255;
		color[2]=255;
	}
	else if(value<300)
	{
		color[0]=posSlope*value-4*255;
		color[1]=0;
		color[2]=255;
	}
	else
	{
		color[0]=255;
		color[1]=0;
		color[2]=negSlope*value+6*255;
	}

	return cv::Scalar(color[0],color[1],color[2]);
}

int main(int argc,char **argv)
{
	if(argc!=4&&argc!=5)
	{
		std::cout<<"Usage: ./projection <image_path> <pcd_path> <intrinsic_and_extrinsic_json_path> Optional:<undistorted>\n"
				"example:\n"
				"\t./bin/projection ./data/undistorted_image.jpg ./data/pointcloud.pcd ./data/calibration.json\n"
				"or:\n"
                "\t./bin/projection ./data/image.jpg ./data/pointcloud.pcd ./data/calibration.json false"
				<<std::endl;
		return 0;
	}
    std::string image_path=argv[1];
	std::string pcd_path=argv[2];
	std::string intrinsic_and_extrinsic_json_path=argv[3];

	cv::Mat image=cv::imread(image_path);

	cv::Mat intrinsic;  //相机内参
	cv::Mat distortion; //畸变系数
	cv::Size image_size;    //相机内参对应的图像大小
	cv::Mat extrinsic;   //相机到目标的外参

	if(argc==5)
	{
		//当可选参数<undistorted>为false或0时，则认为图像没有经过畸变矫正，因此读取原始内参与畸变系数进行去畸变处理
		if(!(strcmp(argv[4],"false")&&strcmp(argv[4],"0")))
		{
			loadIntrinsic(intrinsic_and_extrinsic_json_path,intrinsic,distortion,image_size,false);	//载入原始内参与畸变系数
			cv::Mat undistorted_image;
			cv::undistort(image,undistorted_image,intrinsic,distortion);	//对原始图像进行畸变矫正
			image=undistorted_image;
		}		
	}
	loadIntrinsic(intrinsic_and_extrinsic_json_path,intrinsic,distortion,image_size);	//载入去畸变后的图像再次标定的内参矩阵、畸变参数、图像大小
	loadExtrinsic(intrinsic_and_extrinsic_json_path,extrinsic);	//载入外参矩阵
	
	double maxIntensity=0;	//最大反射强度
	std::vector<PointXYI> pixel_points;	//待绘制的像素2D点容器

	std::vector<PointXYZI> pointcloud=loadPCD(pcd_path);	//载入激光点云3D点
	for(size_t i=0;i<pointcloud.size();i++)
	{
		cv::Point2d pixel_point=ProjectPoint(pointcloud[i].xyz,extrinsic.inv(),intrinsic);	//将激光点云3D点投影到像素坐标系

		if(pixel_point.x>0&&pixel_point.x<image_size.width&&pixel_point.y>0&&pixel_point.y<image_size.height)	//过滤图像之外的点
		{
			PointXYI data;
			data.xy=pixel_point;
			data.intensity=pointcloud[i].intensity;
			pixel_points.push_back(data);

			maxIntensity=std::max(maxIntensity,pointcloud[i].intensity);
		}
	}

	for(size_t i=0;i<pixel_points.size();i++)
	{
		cv::circle(image,pixel_points[i].xy,1,fakeColor(pixel_points[i].intensity/maxIntensity),-1);	//将投影的像素2D点绘制到图像中，并按照反射强度染色
	}

	cv::imshow("projection",image);
	if(cv::waitKey()==27)	//按下ESC保存图片
	{
		cv::imwrite(image_path+"_projection.jpg",image);
	}

	return 0;
}