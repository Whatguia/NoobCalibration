#pragma once

#include<iostream>
#include"jsoncpp/json/json.h"

void LoadExtrinsic(const std::string &filename,cv::Mat &extrinsic)
{
	Json::Reader reader;
	Json::Value root;
	std::vector<float> extrinsic_vector;
	std::vector<float> rotation;
	std::vector<float> translation;

	std::ifstream is(filename,std::ios::binary);
	if(!is.is_open())
	{
		std::cout<<"Error opening file:"<<filename<<std::endl;
		return;
	}

	if(reader.parse(is,root))
	{
		//read rotation[9] or rotation[3][3]
		if(!root["rotation"].isNull()&&root["rotation"].type()==Json::arrayValue)
		{
			if(root["rotation"].size()==3)
			{
				for(unsigned int i=0;i<root["rotation"].size();i++)
				{
					if(!root["rotation"][i].isNull()&&root["rotation"][i].type()==Json::arrayValue)
					{
						if(root["rotation"][i].size()==3)
						{
							for(unsigned int j=0;j<root["rotation"][i].size();j++)
							{
								float data=root["rotation"][i][j].asFloat();
								rotation.push_back(data);
							}
						}
						else
						{
							std::cout<<"Error rotation size:"<<filename<<":"<<i<<std::endl;
							is.close();
							return;
						}
					}
					else
					{
						std::cout<<"Error rotation type:"<<filename<<":"<<i<<std::endl;
						is.close();
						return;
					}
				}
			}
			else if(root["rotation"].size()==9)
			{
				for(unsigned int i=0;i<root["rotation"].size();i++)
				{
					float data=root["rotation"][i].asFloat();
					rotation.push_back(data);
				}
			}
			else
			{
				std::cout<<"Error rotation size:"<<filename<<std::endl;
				is.close();
				return;
			}
		}
		else
		{
			std::cout<<"Error rotation type:"<<filename<<std::endl;
			is.close();
			return;
		}

		//read translation[3] or translation{x,y,z}
		if(!root["translation"].isNull()&&root["translation"].type()==Json::arrayValue)
		{
			for(unsigned int i=0;i<root["translation"].size();i++)
			{
				float data=root["translation"][i].asFloat();
				translation.push_back(data);
			}
		}
		else if(!root["translation"].isNull()&&root["translation"].type()==Json::objectValue)
		{
			float x=root["translation"]["x"].asFloat();
			float y=root["translation"]["y"].asFloat();
			float z=root["translation"]["z"].asFloat();
			translation.push_back(x);
			translation.push_back(y);
			translation.push_back(z);
		}
		else
		{
			std::cout<<"Error translation type:"<<filename<<std::endl;
			is.close();
			return;
		}

		for(int i=0;i<3;i++)
		{
			for(int j=0;j<3;j++)
			{
				extrinsic_vector.push_back(rotation[i*3+j]);
			}
			extrinsic_vector.push_back(translation[i]);
		}
		extrinsic_vector.push_back(0);
		extrinsic_vector.push_back(0);
		extrinsic_vector.push_back(0);
		extrinsic_vector.push_back(1);
	}

	extrinsic=cv::Mat(extrinsic_vector).clone().reshape(1,4);
	is.close();
	return;
}