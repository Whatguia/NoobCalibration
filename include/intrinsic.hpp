#pragma once

#include<iostream>
#include<opencv2/opencv.hpp>
#include"jsoncpp/json/json.h"

void LoadIntrinsic(const std::string &filename,cv::Mat &intrinsic,cv::Mat &distortion)
{
	Json::Reader reader;
	Json::Value root;
	std::vector<float> intrinsic_vector;
	std::vector<float> distortion_vector;

	std::ifstream is(filename,std::ios::binary);
	if(!is.is_open())
	{
		std::cout<<"Error opening file:"<<filename<<std::endl;
		return;
	}

	if(reader.parse(is,root))
	{
		//read intrinsic[9] or intrinsic[3][3]
		if(!root["intrinsic"].isNull()&&root["intrinsic"].type()==Json::arrayValue)
		{
			if(root["intrinsic"].size()==3)
			{
				for(unsigned int i=0;i<root["intrinsic"].size();i++)
				{
					if(!root["intrinsic"][i].isNull()&&root["intrinsic"][i].type()==Json::arrayValue)
					{
						if(root["intrinsic"][i].size()==3)
						{
							for(unsigned int j=0;j<root["intrinsic"][i].size();j++)
							{
								float data=root["intrinsic"][i][j].asFloat();
								intrinsic_vector.push_back(data);
							}
						}
						else
						{
							std::cout<<"Error intrinsic size:"<<filename<<":"<<i<<std::endl;
							is.close();
							return;
						}
					}
					else
					{
						std::cout<<"Error intrinsic type:"<<filename<<":"<<i<<std::endl;
						is.close();
						return;
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
		}
		else
		{
			std::cout<<"Error intrinsic type:"<<filename<<std::endl;
			is.close();
			return;
		}

		//read distortion[]
		if(!root["distortion"].isNull()&&root["distortion"].type()==Json::arrayValue)
		{
			for(unsigned int i=0;i<root["distortion"].size();i++)
			{
				double data=root["distortion"][i].asFloat();
				distortion_vector.push_back(data);
			}
		}
		else
		{
			std::cout<<"Error distortion type:"<<filename<<std::endl;
			is.close();
			return;
		}
	}

	intrinsic=cv::Mat(intrinsic_vector).clone().reshape(1,3);
	distortion=cv::Mat(distortion_vector).clone().reshape(1,1);
	is.close();
	return;
}