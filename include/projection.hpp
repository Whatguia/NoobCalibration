#pragma once

#include <opencv2/opencv.hpp>

//将欧拉角转换成旋转矩阵
cv::Mat eular_to_matrix(float roll,float pitch,float yaw)
{
    std::vector<float> x;
    x.push_back(1.0);x.push_back(0.0);x.push_back(0.0);
    x.push_back(0.0);x.push_back(cos(roll));x.push_back(-sin(roll));
    x.push_back(0.0);x.push_back(sin(roll));x.push_back(cos(roll));
    cv::Mat rotX(cv::Size(3,3),CV_32FC1,x.data());

    std::vector<float> y;
    y.push_back(cos(pitch));y.push_back(0.0);y.push_back(sin(pitch));
    y.push_back(0.0);y.push_back(1.0);y.push_back(0.0);
    y.push_back(-sin(pitch));y.push_back(0.0);y.push_back(cos(pitch));
    cv::Mat rotY(cv::Size(3,3),CV_32FC1,y.data());
 
    std::vector<float> z;
    z.push_back(cos(yaw));z.push_back(-sin(yaw));z.push_back(0.0);
    z.push_back(sin(yaw));z.push_back(cos(yaw));z.push_back(0.0);
    z.push_back(0.0);z.push_back(0.0);z.push_back(1.0);
    cv::Mat rotZ(cv::Size(3,3),CV_32FC1,z.data());

    cv::Mat R=rotZ*rotY*rotX;

    return R;
}

//根据内参以及外参矩阵，计算投影矩阵，默认从输入的是从相机到目标的外参，因此会使用外参矩阵的逆矩阵，如果外参矩阵是从目标到相机，请将inverse_extrinsic参数设置为false
cv::Mat getProjectionMatrix(cv::Mat intrinsic,cv::Mat extrinsic,bool inverse_extrinsic=true)
{
    cv::Mat intrinsic_extend=cv::Mat::zeros(3,4,CV_32FC1);  //扩展内参矩阵到3*4
    intrinsic.copyTo(intrinsic_extend(cv::Rect(0,0,intrinsic.rows,intrinsic.cols)));    //将内参矩阵拷贝到扩展内参矩阵
    cv::Mat projection_matrix=intrinsic_extend*(inverse_extrinsic?extrinsic.inv():extrinsic);   //投影矩阵
    
    return projection_matrix;
}

//使用投影矩阵，将3D空间坐标系中的点转换到2D图像坐标系
cv::Point2f ProjectPoint(cv::Point3f origin_xyz,cv::Mat projection_matrix)
{
    cv::Mat origin_point=cv::Mat::zeros(4,1,CV_32FC1);
    origin_point.at<float>(0,0)=origin_xyz.x;
    origin_point.at<float>(1,0)=origin_xyz.y;
    origin_point.at<float>(2,0)=origin_xyz.z;
    origin_point.at<float>(3,0)=1;
    
    cv::Mat result_point=projection_matrix*origin_point;

    float result_x,result_y;
    result_x=result_point.at<float>(0,0)/result_point.at<float>(2,0);
    result_y=result_point.at<float>(1,0)/result_point.at<float>(2,0);
    return cv::Point2f(result_x,result_y);
}

//根据目标框的xyz、wlh、roll、pitch、yaw信息，获取目标框的角点
cv::Mat getBox(cv::Mat xyz_wlh_yaw)
{
    float* point_data=xyz_wlh_yaw.ptr<float>(0);

    cv::Mat corners(3,8,CV_32FC1);
    
    //根据目标框的长宽高生成目标框的8个顶点
    float* x=corners.ptr<float>(0);
    x[0]=x[1]=x[2]=x[3]=point_data[4]/2;
    x[4]=x[5]=x[6]=x[7]=-point_data[4]/2;

    float* y=corners.ptr<float>(1);
    y[0]=y[3]=y[4]=y[7]=point_data[3]/2;
    y[1]=y[2]=y[5]=y[6]=-point_data[3]/2;

    float* z=corners.ptr<float>(2);
    z[0]=z[1]=z[4]=z[5]=point_data[5]/2;
    z[2]=z[3]=z[6]=z[7]=-point_data[5]/2;

    //根据目标框的旋转角旋转顶点
    corners=eular_to_matrix(point_data[6],point_data[7],point_data[8])*corners;

    //根据目标框的位置平移顶点
    for(int row=0;row<corners.rows;row++)
    {
        float* row_data=corners.ptr<float>(row);
        for(int col=0;col<corners.cols;col++)
        {
            row_data[col]+=point_data[row];
        }
    }

    return corners;
}

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

//使用投影矩阵，将目标坐标系下的目标框角点按框的顺序连线绘制在图像中
void drawBoxCorners(cv::Mat data,cv::Mat corners,cv::Mat projection_matrix,cv::Scalar colors)
{
    //将目标框的顶点转换到图像坐标系
    std::vector<cv::Point2f> corners_xy;
    for(int col=0;col<corners.cols;col++)
    {
        corners_xy.push_back(
            ProjectPoint(
                cv::Point3f(
                    corners.at<float>(0,col),
                    corners.at<float>(1,col),
                    corners.at<float>(2,col)
                ),
                projection_matrix
            )
        );
    }

    //在图像中绘制目标框的边
    int err=0;
    for(int i=0;i<8;i++)
    {
        if(corners_xy[i].x>=data.cols||corners_xy[i].x<=0||corners_xy[i].y>=data.rows||corners_xy[i].y<=0)err++;
    }
    if(err>=7)return;
    for(int i=0;i<4;i++)
    {
        cv::line(data,cv::Point(corners_xy[i].x,corners_xy[i].y),cv::Point(corners_xy[i+4].x,corners_xy[i+4].y),colors,1,8);
        cv::line(data,cv::Point(corners_xy[2*i].x,corners_xy[2*i].y),cv::Point(corners_xy[2*i+1].x,corners_xy[2*i+1].y),colors,1,8);
        if(i<2)
        {
            cv::line(data,cv::Point(corners_xy[i].x,corners_xy[i].y),cv::Point(corners_xy[3-i].x,corners_xy[3-i].y),colors,1,8);
        }
        else
        {
            cv::line(data,cv::Point(corners_xy[i+2].x,corners_xy[i+2].y),cv::Point(corners_xy[9-i].x,corners_xy[9-i].y),colors,1,8);
        }
    }

    return;
}

//将目标坐标系下的目标框绘制在bev图像中
void drawBoxBev(cv::Mat data,cv::Mat corners,cv::Scalar colors)
{
    float* x_corners_bev=corners.ptr<float>(1);
    float* y_corners_bev=corners.ptr<float>(0);

    for(int i=0;i<4;i++)
    {
        cv::line(data,cv::Point(500-x_corners_bev[i]*10,500-y_corners_bev[i]*10),cv::Point(500-x_corners_bev[i+4]*10,500-y_corners_bev[i+4]*10),colors,1,8);
        cv::line(data,cv::Point(500-x_corners_bev[2*i]*10,500-y_corners_bev[2*i]*10),cv::Point(500-x_corners_bev[2*i+1]*10,500-y_corners_bev[2*i+1]*10),colors,1,8);
        if(i<2)
        {
            cv::line(data,cv::Point(500-x_corners_bev[i]*10,500-y_corners_bev[i]*10),cv::Point(500-x_corners_bev[3-i]*10,500-y_corners_bev[3-i]*10),colors,1,8);
        }
        else
        {
            cv::line(data,cv::Point(500-x_corners_bev[i+2]*10,500-y_corners_bev[i+2]*10),cv::Point(500-x_corners_bev[9-i]*10,500-y_corners_bev[9-i]*10),colors,1,8);
        }
    }

    return;
}