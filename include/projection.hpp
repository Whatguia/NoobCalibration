#pragma once

#include <opencv2/opencv.hpp>

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

cv::Mat get_projection_matrix(cv::Mat intrinsic,cv::Mat extrinsic,bool inverse_extrinsic=true)
{
    cv::Mat intrinsic_extend=cv::Mat::zeros(3,4,CV_32FC1);  //扩展内参矩阵到3*4
    intrinsic.copyTo(intrinsic_extend(cv::Rect(0,0,intrinsic.rows,intrinsic.cols)));    //将内参矩阵拷贝到扩展内参矩阵
    if(inverse_extrinsic)
    {
        extrinsic=extrinsic.inv();  //外参逆矩阵
    }
    cv::Mat projection_matrix=intrinsic_extend*extrinsic;   //投影矩阵
    return projection_matrix;

}

//使用投影矩阵，将3D空间坐标系中的点转换到2D图像坐标系
cv::Point2f project_point(cv::Point3f origin_xyz,cv::Mat projection_matrix)
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

cv::Mat get_box(cv::Mat xyz_wlh_yaw)
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

void draw_box_corners(cv::Mat data,cv::Mat corners,cv::Mat projection_matrix,cv::Scalar colors)
{
    //将目标框的顶点转换到图像坐标系
    std::vector<cv::Point2f> corners_xy;
    for(int col=0;col<corners.cols;col++)
    {
        corners_xy.push_back(
            project_point(
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
}

void draw_box_bev(cv::Mat data,cv::Mat corners,cv::Scalar colors)
{
    //在bev图像中绘制目标框的边
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
}