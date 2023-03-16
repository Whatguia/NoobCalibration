#pragma once

#include <opencv2/opencv.hpp>

//读取Bat3D标注文件中的目标框信息
void loadBat3D(const std::string &filename,std::vector<cv::Mat> &xyz_wlh_yaw_vector)
{
    Json::Reader reader;
	Json::Value root;
    xyz_wlh_yaw_vector.clear();

	std::ifstream is(filename,std::ios::binary);
	if(!is.is_open())
	{
		std::cout<<"Error opening file:"<<filename<<std::endl;
		return;
	}

	if(reader.parse(is,root))
	{
		if(root["labels"].isNull()||root["labels"].type()!=Json::arrayValue)
		{
            std::cout<<"Error labels type:"<<filename<<std::endl;
			is.close();
			return;
        }
        
        for(unsigned int i=0;i<root["labels"].size();i++)
        {
            cv::Mat xyz_wlh_yaw(9,1,CV_32FC1);
            
            if(root["labels"][i]["box3d"].isNull()||root["labels"][i]["box3d"].type()!=Json::objectValue)
            {
                std::cout<<"Error box3d type:"<<filename<<":"<<i<<std::endl;
                is.close();
                return;
            }

            xyz_wlh_yaw.at<float>(0,0)=root["labels"][i]["box3d"]["location"]["x"].asFloat();//x
            xyz_wlh_yaw.at<float>(0,1)=root["labels"][i]["box3d"]["location"]["y"].asFloat();//y
            xyz_wlh_yaw.at<float>(0,2)=root["labels"][i]["box3d"]["location"]["z"].asFloat();//z
            xyz_wlh_yaw.at<float>(0,3)=root["labels"][i]["box3d"]["dimension"]["width"].asFloat();//w
            xyz_wlh_yaw.at<float>(0,4)=root["labels"][i]["box3d"]["dimension"]["length"].asFloat();//l
            xyz_wlh_yaw.at<float>(0,5)=root["labels"][i]["box3d"]["dimension"]["height"].asFloat();//h
            xyz_wlh_yaw.at<float>(0,6)=root["labels"][i]["box3d"]["orientation"]["rotationRoll"].asFloat();//roll
            xyz_wlh_yaw.at<float>(0,7)=root["labels"][i]["box3d"]["orientation"]["rotationPitch"].asFloat();//pitch
            xyz_wlh_yaw.at<float>(0,8)=root["labels"][i]["box3d"]["orientation"]["rotationYaw"].asFloat();//yaw

            xyz_wlh_yaw_vector.push_back(xyz_wlh_yaw);
        }
    }

	is.close();
	return;
}

//根据目标框的xyz、wlh、roll、pitch、yaw信息，获取目标框的角点（bat3D版本）
cv::Mat getBoxBat3D(cv::Mat xyz_wlh_yaw)
{
    float* point_data=xyz_wlh_yaw.ptr<float>(0);

    cv::Mat corners(3,8,CV_32FC1);

    float r11=cos(-point_data[8]);
    float r12=-sin(-point_data[8]);
    float r21=sin(-point_data[8]);
    float r22=cos(-point_data[8]);

    float w11=-point_data[3]/2;
    float l11=-point_data[4]/2;
    float w12=point_data[3]/2;
    float l12=-point_data[4]/2;
    float w21=point_data[3]/2;
    float l21=point_data[4]/2;
    float w22=-point_data[3]/2;
    float l22=point_data[4]/2;

    float w11_new=r11*w11+r21*l11;
    float l11_new=r12*w11+r22*l11;
    float w12_new=r11*w12+r21*l12;
    float l12_new=r12*w12+r22*l12;
    float w21_new=r11*w21+r21*l21;
    float l21_new=r12*w21+r22*l21;
    float w22_new=r11*w22+r21*l22;
    float l22_new=r12*w22+r22*l22;

    corners.at<float>(0,0)=point_data[0]+w11_new;
    corners.at<float>(1,0)=point_data[1]+l11_new;
    corners.at<float>(2,0)=point_data[2]+point_data[5]/2;

    corners.at<float>(0,1)=point_data[0]+w12_new;
    corners.at<float>(1,1)=point_data[1]+l12_new;
    corners.at<float>(2,1)=point_data[2]+point_data[5]/2;

    corners.at<float>(0,2)=point_data[0]+w21_new;
    corners.at<float>(1,2)=point_data[1]+l21_new;
    corners.at<float>(2,2)=point_data[2]+point_data[5]/2;

    corners.at<float>(0,3)=point_data[0]+w22_new;
    corners.at<float>(1,3)=point_data[1]+l22_new;
    corners.at<float>(2,3)=point_data[2]+point_data[5]/2;

    corners.at<float>(0,4)=point_data[0]+w11_new;
    corners.at<float>(1,4)=point_data[1]+l11_new;
    corners.at<float>(2,4)=point_data[2]-point_data[5]/2;

    corners.at<float>(0,5)=point_data[0]+w12_new;
    corners.at<float>(1,5)=point_data[1]+l12_new;
    corners.at<float>(2,5)=point_data[2]-point_data[5]/2;

    corners.at<float>(0,6)=point_data[0]+w21_new;
    corners.at<float>(1,6)=point_data[1]+l21_new;
    corners.at<float>(2,6)=point_data[2]-point_data[5]/2;

    corners.at<float>(0,7)=point_data[0]+w22_new;
    corners.at<float>(1,7)=point_data[1]+l22_new;
    corners.at<float>(2,7)=point_data[2]-point_data[5]/2;
    
    return corners;
}