#pragma once

#include<iostream>
#include<opencv2/opencv.hpp>
#include"jsoncpp/json/json.h"

//读取外参矩阵
void loadExtrinsic(const std::string &filename,cv::Mat &extrinsic)
{
	Json::Reader reader;
	Json::Value root;
	std::vector<double> extrinsic_vector,rotation,translation;

	std::ifstream is(filename,std::ios::binary);
	if(!is.is_open())
	{
		std::cout<<"Error opening file:"<<filename<<std::endl;
		return;
	}

	if(reader.parse(is,root))
	{
		//read rotation[9] or rotation[3][3]
		if(root["rotation"].isNull()||root["rotation"].type()!=Json::arrayValue)
		{
			std::cout<<"Error rotation type:"<<filename<<std::endl;
			is.close();
			return;
		}
		if(root["rotation"].size()==3)
		{
			for(unsigned int i=0;i<root["rotation"].size();i++)
			{
				if(root["rotation"][i].isNull()||root["rotation"][i].type()!=Json::arrayValue)
				{
					std::cout<<"Error rotation type:"<<filename<<":"<<i<<std::endl;
					is.close();
					return;
				}
				if(root["rotation"][i].size()!=3)
				{
					std::cout<<"Error rotation size:"<<filename<<":"<<i<<std::endl;
					is.close();
					return;
				}

				for(unsigned int j=0;j<root["rotation"][i].size();j++)
				{
					double data=root["rotation"][i][j].asDouble();
					rotation.push_back(data);
				}
			}
		}
		else if(root["rotation"].size()==9)
		{
			for(unsigned int i=0;i<root["rotation"].size();i++)
			{
				double data=root["rotation"][i].asDouble();
				rotation.push_back(data);
			}
		}
		else
		{
			std::cout<<"Error rotation size:"<<filename<<std::endl;
			is.close();
			return;
		}

		//read translation[3] or translation{x,y,z}
		if(!root["translation"].isNull()&&root["translation"].type()==Json::arrayValue)
		{
			for(unsigned int i=0;i<root["translation"].size();i++)
			{
				double data=root["translation"][i].asDouble();
				translation.push_back(data);
			}
		}
		else if(!root["translation"].isNull()&&root["translation"].type()==Json::objectValue)
		{
			double x=root["translation"]["x"].asDouble();
			double y=root["translation"]["y"].asDouble();
			double z=root["translation"]["z"].asDouble();
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

//修改并保存外参，仅修改外参部分（平移和旋转）
bool saveExtrinsic(const std::string &filename,cv::Mat extrinsic)
{
	Json::Reader reader;
	Json::Value root;

	std::ifstream is(filename,std::ios::binary);
	if(!is.is_open())
	{
		std::cout<<"Error opening file:"<<filename<<std::endl;
		return false;
	}

	if(reader.parse(is,root))
	{
		Json::Value rotation_obj;
		for(int i=0;i<9;i++)
		{
			rotation_obj.append(extrinsic.at<double>(i/3,i%3));
		}

		Json::Value translation_obj;
		for(int i=0;i<3;i++)
		{
			translation_obj.append(extrinsic.at<double>(i,3));
		}

		root["rotation"]=rotation_obj;
		root["translation"]=translation_obj;
	}
	is.close();

	std::ofstream os;
	os.open(filename,std::ios::out);
	if (!os.is_open())
	{
		std::cout<<"Error opening file:"<<filename<<std::endl;
		return false;
	}
	Json::StyledWriter sw;
	os<<sw.write(root)<<std::flush;
	os.close();
	
	return true;
}