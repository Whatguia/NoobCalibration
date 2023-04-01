#include<iostream>
#include<opencv2/opencv.hpp>
#include"intrinsic.hpp"
#include"extrinsic.hpp"
#include"projection.hpp"

std::vector<std::string> stringSplit(std::string str,std::string patten)
{
    std::vector<std::string> retString;
    while(true) {
        if(str.find(patten)==std::string::npos) break;
        int pos=str.find(patten);
        retString.push_back(str.substr(0,pos));
        str=str.substr(pos+patten.size(),str.size()-pos);
    }
    retString.push_back(str);
    return retString;
}

cv::Mat loadPCD(const std::string &filename)
{
	std::ifstream is(filename);
	if(!is.is_open())
	{
		std::cout<<"Error opening file:"<<filename<<std::endl;
		return cv::Mat::zeros(1,3,CV_64FC1);
	}
	int fields=3;
	int point_size;
	std::string temp;
	std::vector<std::string> temps;

	std::vector<double> pointcloud;

	while(std::getline(is,temp))
	{
		temps=stringSplit(temp," ");
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
			for(int i=0;i<3;i++)
			{
				double data=std::stod(temps[i]);
				pointcloud.push_back(data);
			}
		}
	}

	is.close();
	return cv::Mat(pointcloud).clone().reshape(1,point_size);
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
			std::vector<double> ori_dist;
			loadIntrinsic(intrinsic_and_extrinsic_json_path,intrinsic,distortion,image_size,false);
			cv::Mat undistorted_image;
			cv::undistort(image,undistorted_image,intrinsic,distortion);
			image=undistorted_image;
		}		
	}
	loadIntrinsic(intrinsic_and_extrinsic_json_path,intrinsic,distortion,image_size);
	loadExtrinsic(intrinsic_and_extrinsic_json_path,extrinsic);
	
	std::vector<cv::Point2d> pixel_points;

	cv::Mat pointcloud=loadPCD(pcd_path);
	for(int row=0;row<pointcloud.rows;row++)
	{
		cv::Point2d pixel_point=ProjectPoint(
			cv::Point3d(
				pointcloud.at<double>(row,0),
				pointcloud.at<double>(row,1),
				pointcloud.at<double>(row,2)
			),
			extrinsic.inv(),
			intrinsic
		);

		if(pixel_point.x>0&&pixel_point.x<image_size.width&&pixel_point.y>0&&pixel_point.y<image_size.height)
		{
			pixel_points.push_back(pixel_point);
		}
	}

	for(size_t i=0;i<pixel_points.size();i++)
	{
		cv::circle(image,pixel_points[i],1,cv::Scalar(0,255,0));
	}

	cv::imwrite("./temp.jpg",image);

	return 0;
}