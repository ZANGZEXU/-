#include<iostream>
#include<opencv2/opencv.hpp>
#include<cmath>
#include<vector>

using namespace std;
using namespace cv;


void SolvePnp(vector<Point2i> m.points){
      //世界坐标 图上的点坐标 摄像头系数 畸变系数 输出坐标
      Matx<Point3f,4,1>objectPoints{
          {-115,63.5,0},
          {115,63.5,0},
          {115,-63.5,0},
          {-115,-63.5,0}
      };

      //相机内参
      Matx<float,3,3> camera_matrix ({
           5.9763827661155904e+02, 0., 4.1575511901601089e+02, 
          0.,5.9922205940008985e+02, 2.6769310598084320e+02, 
          0., 0., 1.});
      //畸变系数
      Mat distortion_coefficients({5.9365728086275861e-02, 6.3271114889939875e-02, 
      5.5006940318826766e-03, -3.5032524991503678e-03, 0.});
          << 5.9365728086275861e-02, 6.3271114889939875e-02,
          5.5006940318826766e-03, -3.5032524991503678e-03, 0.);
      Mat Rvec,Rmat;
      Mat Tvec;
      Mat raux, taux;
      solvePnP(m_markerCorners3d,m.points,camera_matrix,
            distortion_coefficients, raux, taux, false, CV_P3P);
      //罗德里格斯变换
      Rodrigues(raux,Rmat);
      Rmat.convertTo(Rmat,CV_64FC1);
      taux.convertTo(Tvec,CV_64FC1);
      
      Mat P_oc;
      P_oc= -Rmat.inv() * Tvec;
      cout<<"distance"<<P_oc<<"mm"<<endl;//计算出来的单位是mm

}
