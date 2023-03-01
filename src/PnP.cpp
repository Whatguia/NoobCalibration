#include<iostream>
#include<opencv2/opencv.hpp>
#include"intrinsic.hpp"
#include"extrinsic.hpp"
#include"projection.hpp"
#include"PnP.hpp"

int main(int argc,char** argv)
{
    if(argc!=3&&argc!=4)
	{
		std::cout<<"Usage: ./PnP <points_json_path> <intrinsic_json_path> Optional:<extrinsic_json_path>\n"
				"example:\n"
				"\t./bin/PnP ./data/points.json ./data/test.json\n"
                "or:\n"
                "\t./bin/PnP ./data/points.json ./data/test.json ./data/test.json"
                <<std::endl;
		return 0;
	}
    std::string point_json_path=argv[1];    //像素坐标系以及目标坐标系下的点对json文件路径
	std::string intrinsic_json_path=argv[2];    //内参json文件路径
    std::string extrinsic_json_path;    //可选参数，外参json文件路径

    std::vector<cv::Point2f> pixel_points;  //像素坐标系下的点
    std::vector<cv::Point3f> target_points; //目标坐标系下的点
    loadPoints(point_json_path,pixel_points,target_points); //载入像素坐标系以及目标坐标系中点的信息

    cv::Mat intrinsic,distortion;   //相机内参、畸变系数
    cv::Point2i image_size; //相机内参对应的图像大小
    loadIntrinsic(intrinsic_json_path,intrinsic,distortion,image_size);   //载入内参矩阵、畸变参数、图像大小
    cv::Mat undistort_intrinsic=cv::getOptimalNewCameraMatrix(intrinsic,distortion,cv::Size(image_size.x,image_size.y),0.0,cv::Size(image_size.x,image_size.y));    //根据内参与畸变系数计算去畸变后的内参

    cv::Mat extrinsic=cv::Mat::eye(4,4,CV_32FC1);   //相机到目标的外参
    cv::Mat rvec=cv::Mat::zeros(3,1,CV_64FC1);  //创建旋转矩阵
    cv::Mat tvec=cv::Mat::zeros(3,1,CV_64FC1);  //创建平移矩阵
    cv::Mat rotate_Matrix=cv::Mat::eye(3,3,CV_64FC1);
    if(argc==4)
    {
        //当可选参数<extrinsic_json_path>启用时，读取外参文件中的外参信息，并作为外参的初值送入PnP进行计算
        extrinsic_json_path=argv[3];    //外参json文件路径
        loadExtrinsic(extrinsic_json_path,extrinsic);   //载入外参矩阵
        extrinsic=extrinsic.inv();  //默认从文件读取的是从相机到目标的外参，因此会使用外参矩阵的逆矩阵

        extrinsic(cv::Rect(0,0,3,3)).copyTo(rotate_Matrix); //取出外参矩阵中的旋转矩阵

        cv::Rodrigues(rotate_Matrix,rvec);  //将旋转矩阵变换成旋转向量
        for(int i=0;i<3;i++)
        {
            tvec.at<double>(i,0)=(double)extrinsic.at<float>(i,3);  //取出外参矩阵中的平移向量
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
    cv::solvePnPRansac(target_points,pixel_points,intrinsic,distortion,rvec,tvec,argc==4,100,8.0,0.9899999999999999911,cv::noArray(),1);
    
    cv::Rodrigues(rvec,rotate_Matrix);  //将旋转向量变换成旋转矩阵

    for(int row=0;row<extrinsic.rows-1;row++)
    {
        for(int col=0;col<extrinsic.cols-1;col++)
        {
            extrinsic.at<float>(row,col)=(float)rotate_Matrix.at<double>(row,col);  //将旋转矩阵填入外参矩阵
        }
        extrinsic.at<float>(row,extrinsic.cols-1)=(float)tvec.at<double>(row,0);    //将平移向量填入外参矩阵
    }

    cv::Mat projection_matrix=getProjectionMatrix(intrinsic,extrinsic,false);   //根据内参与外参计算投影矩阵，注意外参的目标，PnP计算的外参是从相机到目标的坐标系，因此将inverse_extrinsic参数设置为false
    std::vector<float> projection_error=getProjectionError(pixel_points,target_points,projection_matrix);   //获取重投影误差

    std::cout<<"\n内参:"<<std::endl;
    std::cout<<intrinsic<<std::endl;
    std::cout<<"\n畸变:"<<std::endl;
    std::cout<<distortion<<std::endl;
    std::cout<<"\n去畸变后的内参:"<<std::endl;
    std::cout<<undistort_intrinsic<<std::endl;
    std::cout<<"\n外参(目标到相机):"<<std::endl;
    std::cout<<extrinsic<<std::endl;
    std::cout<<"\n外参(相机到目标):"<<std::endl;
    std::cout<<extrinsic.inv()<<std::endl;
    std::cout<<"\n投影矩阵:"<<std::endl;
    std::cout<<projection_matrix<<std::endl;

    float average_projection_error=0;
    std::cout<<"\n投影误差:"<<std::endl;
    for(size_t i=0;i<projection_error.size();i++)
    {
        average_projection_error+=projection_error[i];
        std::cout<<(i+1)<<": "<<projection_error[i]<<"\t";
        if(!((i+1)%5))
        {
            std::cout<<std::endl;
        }
    }
    std::cout<<"\n平均投影误差:"<<std::endl;
    std::cout<<average_projection_error/projection_error.size()<<std::endl;

    if(argc==4)
    {
        //当可选参数<extrinsic_json_path>启用时，更新外参文件中的信息
        saveExtrinsic(extrinsic_json_path,extrinsic.inv());
        saveIntrinsic(extrinsic_json_path,undistort_intrinsic);
    }

    // int x,y,z;
    // std::cout<<"输入世界坐标:"<<std::endl;
    // while (std::cin>>x>>y>>z)
    // {
    //     std::cout<<x<<" "<<y<<" "<<z<<" "<<std::endl;
    //     cv::Point2f pix=project_point(cv::Point3f(x,y,z),projection_matrix);
        
    //     std::cout<<"像素坐标:"<< pix <<std::endl<<pix.x<<" "<<pix.y<<std::endl;
    // }
    return 0;
}