#pragma once

#include<iostream>
#include<opencv2/opencv.hpp>
#include"jsoncpp/json/json.h"

//读取内参矩阵、畸变参数、图像大小
void loadIntrinsic(const std::string &filename,cv::Mat &intrinsic,cv::Mat &distortion,cv::Point2i &image_size)
{
	Json::Reader reader;
	Json::Value root;
	std::vector<float> intrinsic_vector,distortion_vector;

	std::ifstream is(filename,std::ios::binary);
	if(!is.is_open())
	{
		std::cout<<"Error opening file:"<<filename<<std::endl;
		return;
	}

	if(reader.parse(is,root))
	{
		//read intrinsic[9] or intrinsic[3][3]
		if(root["intrinsic"].isNull()||root["intrinsic"].type()!=Json::arrayValue)
		{
			std::cout<<"Error intrinsic type:"<<filename<<std::endl;
			is.close();
			return;
		}

		if(root["intrinsic"].size()==3)
		{
			for(unsigned int i=0;i<root["intrinsic"].size();i++)
			{
				if(root["intrinsic"][i].isNull()||root["intrinsic"][i].type()!=Json::arrayValue)
				{
					std::cout<<"Error intrinsic type:"<<filename<<":"<<i<<std::endl;
					is.close();
					return;
				}
				if(root["intrinsic"][i].size()!=3)
				{
					std::cout<<"Error intrinsic size:"<<filename<<":"<<i<<std::endl;
					is.close();
					return;
				}
				
				for(unsigned int j=0;j<root["intrinsic"][i].size();j++)
				{
					float data=root["intrinsic"][i][j].asFloat();
					intrinsic_vector.push_back(data);
				}

			}
		}
		else if(root["intrinsic"].size()==9)
		{
			for(unsigned int i=0;i<root["intrinsic"].size();i++)
			{
				float data=root["intrinsic"][i].asFloat();
				intrinsic_vector.push_back(data);
			}
		}
		else
		{
			std::cout<<"Error intrinsic size:"<<filename<<std::endl;
			is.close();
			return;
		}

		//read distortion[]
		if(root["distortion"].isNull()||root["distortion"].type()!=Json::arrayValue)
		{
			std::cout<<"Error distortion type:"<<filename<<std::endl;
			is.close();
			return;
		}
		
		for(unsigned int i=0;i<root["distortion"].size();i++)
		{
			double data=root["distortion"][i].asFloat();
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
		
		image_size.x=root["image_size"][0].asInt();
		image_size.y=root["image_size"][1].asInt();
	}

	intrinsic=cv::Mat(intrinsic_vector).clone().reshape(1,3);
	distortion=cv::Mat(distortion_vector).clone().reshape(1,1);
	is.close();
	return;
}

//保存去畸变后的内参
void saveIntrinsic(const std::string &filename,cv::Mat undistort_intrinsic)
{
	Json::Reader reader;
	Json::Value root;

	std::ifstream is(filename,std::ios::binary);
	if(!is.is_open())
	{
		std::cout<<"Error opening file:"<<filename<<std::endl;
		return;
	}

	if(reader.parse(is,root))
	{
		Json::Value undistort_intrinsic_obj;
		for(int i=0;i<9;i++)
		{
			undistort_intrinsic_obj.append(undistort_intrinsic.at<float>(i/3,i%3));
		}

		root["undistort_intrinsic"]=undistort_intrinsic_obj;
	}
	is.close();

	std::ofstream os;
	os.open(filename,std::ios::out);
	if (!os.is_open())
	{
		std::cout<<"Error opening file:"<<filename<<std::endl;
		return;
	}
	Json::StyledWriter sw;
	os<<sw.write(root);
	os.close();

	return;
}