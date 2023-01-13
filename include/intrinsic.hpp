#pragma once

#include<iostream>
#include<opencv2/opencv.hpp>
#include"jsoncpp/json/json.h"

//读取内参
void loadIntrinsic(const std::string &filename,cv::Mat &intrinsic,cv::Mat &distortion)
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
	}

	intrinsic=cv::Mat(intrinsic_vector).clone().reshape(1,3);
	distortion=cv::Mat(distortion_vector).clone().reshape(1,1);
	is.close();
	return;
}