#include<iostream>
#include<opencv2/opencv.hpp>
#include"intrinsic.hpp"
#include"extrinsic.hpp"
#include"projection.hpp"
#include"PnP.hpp"

int main(int argc,char** argv)
{
    if(argc!=3)
	{
		std::cout<<"Usage: ./PnP <config_json_path> <intrinsic_and_extrinsic_json_path>\n"
            "example:\n"
            "\t./bin/PnP ./data/config.json ./data/calibration.json\n"
            <<std::flush;
		return 1;
	}
    std::string config_json_path=argv[1];   //像素坐标系以及目标坐标系下的点对json文件路径
	std::string intrinsic_and_extrinsic_json_path=argv[2];  //内参与外参json文件路径

    std::vector<cv::Point2d> pixel_points;  //像素坐标系下的点
    std::vector<cv::Point3d> target_points; //目标坐标系下的点
    bool useExtrinsicGuess; //用于SOLVEPNP_ITERATIVE的参数，如果为true，则函数使用所提供的rvec和tvec值分别作为旋转向量和平移向量的初始近似值，并进一步优化它们
    int iterationsCount;    //RANSAC算法的迭代次数
    float reprojectionError;    //RANSAC过程使用的观测点投影和计算点投影之间允许的最大距离，以将其视为内投影
    double confidence;  //算法产生有用结果的概率
    int flags;  //解决PnP问题的方法
    if(!loadConfig(config_json_path,pixel_points,target_points,useExtrinsicGuess,iterationsCount,reprojectionError,confidence,flags))
    {
        std::cout<<"The configuration file format should be as follows:\n"
            "{\n"
            "   \"useExtrinsicGuess\":false,\n"
            "   \"iterationsCount\":100,\n"
            "   \"reprojectionError\":8.0,\n"
            "   \"confidence\":0.9899999999999999911,\n"
            "   \"flags\":1,\n"
            "   \"points\":{\n"
            "       \"000000\":[\n"
            "           [1093,387,3.612351,-1.260774,-0.337581],\n"
            "           [1191,286,3.601554,-1.509676,-0.087125],\n"
            "           [948,390,3.616652,-0.890895,-0.348027],\n"
            "           [786,350,3.643460,-0.500715,-0.247620],\n"
            "           [339,350,3.713760,0.639352,-0.240699],\n"
            "           [297,332,3.682724,0.738677,-0.194302],\n"
            "           [18,385,3.637017,1.374771,-0.316020]\n"
            "       ],\n"
            "       \"000150\":[\n"
            "           [157,338,5.531450,1.640031,-0.139118],\n"
            "           [188,404,5.469248,1.520065,-0.403334],\n"
            "           [288,405,5.429010,1.134052,-0.403679],\n"
            "           [124,344,5.640391,1.826593,-0.163570]\n"
            "       ]\n"
            "   }\n"
            "}\n"
            <<std::flush;
        return 1;
    }

    cv::Mat intrinsic;  //相机内参
    cv::Mat distortion; //畸变系数
    cv::Size image_size;    //相机内参对应的图像大小
    loadIntrinsic(intrinsic_and_extrinsic_json_path,intrinsic,distortion,image_size);   //载入去畸变后的图像再次标定的内参矩阵、畸变参数、图像大小

    cv::Mat extrinsic=cv::Mat::eye(4,4,CV_64FC1);   //相机到目标的外参
    cv::Mat rvec=cv::Mat::zeros(3,1,CV_64FC1);  //创建旋转向量
    cv::Mat tvec=cv::Mat::zeros(3,1,CV_64FC1);  //创建平移向量
    cv::Mat rotate_Matrix=cv::Mat::eye(3,3,CV_64FC1);   //创建旋转矩阵
    if(useExtrinsicGuess)
    {
        //当useExtrinsicGuess启用时，读取外参文件中的外参信息，并作为外参的初值送入PnP进行计算
        loadExtrinsic(intrinsic_and_extrinsic_json_path,extrinsic);   //载入外参矩阵
        extrinsic=extrinsic.inv();  //默认从文件读取的是从相机到目标的外参，因此会使用外参矩阵的逆矩阵

        extrinsic(cv::Rect(0,0,3,3)).copyTo(rotate_Matrix); //取出外参矩阵中的旋转矩阵

        cv::Rodrigues(rotate_Matrix,rvec);  //将旋转矩阵变换成旋转向量
        for(int i=0;i<3;i++)
        {
            tvec.at<double>(i,0)=extrinsic.at<double>(i,3);  //取出外参矩阵中的平移向量
        }
    }

    /*
    bool cv::solvePnPRansac(
        cv::InputArray objectPoints,    Array of object points in the object coordinate space, Nx3 1-channel or 1xN/Nx1 3-channel, where N is the number of points. vector<Point3d> can be also passed here.
        cv::InputArray imagePoints,     Array of corresponding image points, Nx2 1-channel or 1xN/Nx1 2-channel, where N is the number of points. vector<Point2d> can be also passed here.
        cv::InputArray cameraMatrix,    Input camera matrix
        cv::InputArray distCoeffs,      Input vector of distortion coefficients
        cv::OutputArray rvec,           Output rotation vector (see Rodrigues ) that, together with tvec, brings points from the model coordinate system to the camera coordinate system.
        cv::OutputArray tvec,           Output translation vector.
        bool useExtrinsicGuess = false, Parameter used for SOLVEPNP_ITERATIVE. If true (1), the function uses the provided rvec and tvec values as initial approximations of the rotation and translation vectors, respectively, and further optimizes them.
        int iterationsCount = 100,      Number of iterations.
        float reprojectionError = (8.0F),   Inlier threshold value used by the RANSAC procedure. The parameter value is the maximum allowed distance between the observed and computed point projections to consider it an inlier.
        double confidence = (0.9899999999999999911),    The probability that the algorithm produces a useful result.
        cv::OutputArray inliers = noArray(),    Output vector that contains indices of inliers in objectPoints and imagePoints .
        int flags = 0   Method for solving a PnP problem (see solvePnP ). The function estimates an object pose given a set of object points, their corresponding image projections, as well as the camera matrix and the distortion coefficients. This function finds such a pose that minimizes reprojection error, that is, the sum of squared distances between the observed projections imagePoints and the projected (using projectPoints ) objectPoints. The use of RANSAC makes the function resistant to outliers.
    )
    切记，虽然Opencv的参数偏爱float类型，但是solvepnp中除了相机内参和畸变参数矩阵是用float类型外，其余的矩阵都是double类型，不然出出现计算结果不正确的情况。
    */
    //求解pnp
    cv::solvePnPRansac(target_points,pixel_points,intrinsic,distortion,rvec,tvec,useExtrinsicGuess,iterationsCount,reprojectionError,confidence,cv::noArray(),flags);
    
    cv::Rodrigues(rvec,rotate_Matrix);  //将旋转向量变换成旋转矩阵

    for(int row=0;row<extrinsic.rows-1;row++)
    {
        for(int col=0;col<extrinsic.cols-1;col++)
        {
            extrinsic.at<double>(row,col)=rotate_Matrix.at<double>(row,col);  //将旋转矩阵填入外参矩阵
        }
        extrinsic.at<double>(row,extrinsic.cols-1)=tvec.at<double>(row,0);    //将平移向量填入外参矩阵
    }

    //根据相机内参、畸变系数以及相机内参对应的图像大小，将像素坐标系下的点转换到去畸变后图像中像素坐标系下的点，以计算投影误差
    cv::Mat pixel_points_mat=cv::Mat(1,pixel_points.size(),CV_64FC2);   //像素坐标系下的点矩阵
    for(size_t i=0;i<pixel_points.size();i++)
    {
        pixel_points_mat.at<cv::Vec2d>(i)=pixel_points[i];  //将像素坐标系下的点填入像素坐标系下的点矩阵
    }

    cv::Mat undistorted_pixel_points_mat;   //去畸变后的像素坐标系下的点矩阵
    cv::Mat undistorted_intrinsic=cv::getOptimalNewCameraMatrix(intrinsic,distortion,image_size,0.0,image_size);    //去畸变后的相机内参矩阵
    cv::undistortPoints(pixel_points_mat,undistorted_pixel_points_mat,intrinsic,distortion,cv::noArray(),undistorted_intrinsic);    //对像素坐标系下的点去畸变，获取去畸变后的像素坐标系下的点
    
    std::vector<cv::Point2d> undistorted_pixel_point;   //去畸变后的像素坐标系下的点
    for(int row=0;row<undistorted_pixel_points_mat.rows;row++)
    {
        cv::Point2d data;
        data.x=undistorted_pixel_points_mat.at<double>(row,0);
        data.y=undistorted_pixel_points_mat.at<double>(row,1);
        undistorted_pixel_point.push_back(data);
    }

    cv::Mat projection_matrix=getProjectionMatrix(undistorted_intrinsic,extrinsic,false);   //根据内参与外参计算投影矩阵，注意外参的目标，PnP计算的外参是从相机到目标的坐标系，因此将inverse_extrinsic参数设置为false
    std::vector<double> projection_error=getProjectionError(undistorted_pixel_point,target_points,projection_matrix);   //获取投影误差

    std::cout<<"Intrinsic:"<<std::endl;
    std::cout<<intrinsic<<std::endl;
    std::cout<<"\nDistortion:"<<std::endl;
    std::cout<<distortion<<std::endl;
    std::cout<<"\nExtrinsic(target in the camera coordinate system):"<<std::endl;
    std::cout<<extrinsic<<std::endl;
    std::cout<<"\nExtrinsic(camera in the target coordinate system):"<<std::endl;
    std::cout<<extrinsic.inv()<<std::endl;
    std::cout<<"\nProjection matrix:"<<std::endl;
    std::cout<<projection_matrix<<std::endl;

    double average_projection_error=0;
    std::cout<<"\nProjection error:"<<std::endl;
    for(size_t i=0;i<projection_error.size();i++)
    {
        average_projection_error+=projection_error[i];
        std::cout<<(i+1)<<": "<<projection_error[i]<<"\t";
        if(!((i+1)%5))
        {
            std::cout<<std::endl;
        }
    }
    std::cout<<"\nAverage projection error:"<<std::endl;
    std::cout<<average_projection_error/projection_error.size()<<std::endl;

    if(!saveExtrinsic(intrinsic_and_extrinsic_json_path,extrinsic.inv()))
    {
        std::cout<<"Failed to save result"<<std::endl;
    }

    return 0;
}