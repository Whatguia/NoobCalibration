#pragma once

#include<iostream>
#include<opencv2/opencv.hpp>
#include"jsoncpp/json/json.h"

//读取内参矩阵、畸变参数、图像大小，默认读取的是去畸变后的图像再次标定的内参"undistort_intrinsic"以及畸变参数"undistort_distortion"，如果需要读取内参"intrinsic"以及畸变系数"distortion"，请将undistort参数设置为false
void loadIntrinsic(const std::string &filename,cv::Mat &intrinsic,cv::Mat &distortion,cv::Size &image_size,bool undistort=true)
{
	Json::Reader reader;
	Json::Value root;
	std::vector<float> intrinsic_vector,distortion_vector;
	const std::string intrinsic_key=undistort?"undistort_intrinsic":"intrinsic";
	const std::string distortion_key=undistort?"undistort_distortion":"distortion";

	std::ifstream is(filename,std::ios::binary);
	if(!is.is_open())
	{
		std::cout<<"Error opening file:"<<filename<<std::endl;
		return;
	}

	if(reader.parse(is,root))
	{
		//read intrinsic[9] or intrinsic[3][3]
		if(root[intrinsic_key].isNull()||root[intrinsic_key].type()!=Json::arrayValue)
		{
			std::cout<<"Error "<<intrinsic_key<<" type:"<<filename<<std::endl;
			is.close();
			return;
		}

		if(root[intrinsic_key].size()==3)
		{
			for(unsigned int i=0;i<root[intrinsic_key].size();i++)
			{
				if(root[intrinsic_key][i].isNull()||root[intrinsic_key][i].type()!=Json::arrayValue)
				{
					std::cout<<"Error "<<intrinsic_key<<" type:"<<filename<<":"<<i<<std::endl;
					is.close();
					return;
				}
				if(root[intrinsic_key][i].size()!=3)
				{
					std::cout<<"Error "<<intrinsic_key<<" size:"<<filename<<":"<<i<<std::endl;
					is.close();
					return;
				}
				
				for(unsigned int j=0;j<root[intrinsic_key][i].size();j++)
				{
					float data=root[intrinsic_key][i][j].asFloat();
					intrinsic_vector.push_back(data);
				}
			}
		}
		else if(root[intrinsic_key].size()==9)
		{
			for(unsigned int i=0;i<root[intrinsic_key].size();i++)
			{
				float data=root[intrinsic_key][i].asFloat();
				intrinsic_vector.push_back(data);
			}
		}
		else
		{
			std::cout<<"Error "<<intrinsic_key<<" size:"<<filename<<std::endl;
			is.close();
			return;
		}

		//read distortion[]
		if(root[distortion_key].isNull()||root[distortion_key].type()!=Json::arrayValue)
		{
			std::cout<<"Error "<<distortion_key<<" type:"<<filename<<std::endl;
			is.close();
			return;
		}
		
		for(unsigned int i=0;i<root[distortion_key].size();i++)
		{
			double data=root[distortion_key][i].asFloat();
			distortion_vector.push_back(data);
		}

		//read image_size[2]
		if(root["image_size"].isNull()||root["image_size"].type()!=Json::arrayValue)
		{
			std::cout<<"Error image_size type:"<<filename<<std::endl;
			is.close();
			return;
		}
		if(root["image_size"].size()!=2)
		{
			std::cout<<"Error image_size size:"<<filename<<std::endl;
			is.close();
			return;
		}
		
		image_size.width=root["image_size"][0].asInt();
		image_size.height=root["image_size"][1].asInt();
	}

	intrinsic=cv::Mat(intrinsic_vector).clone().reshape(1,3);
	distortion=cv::Mat(distortion_vector).clone().reshape(1,1);
	is.close();
	return;
}