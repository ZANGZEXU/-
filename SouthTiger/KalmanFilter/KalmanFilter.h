#pragma once
#include<opencv2/opencv.hpp>
#include<vector>
#include<iostream>
#include <Eigen/Dense>
using namespace cv;
using namespace std;

class  KalmanFilter  
{  
public:      
    KalmanFilter();                                                                           //构造默认KalmanFilter对象  
	KalmanFilter(Mat & video);
    void init();
    //卡尔曼增益的更迭
    float Kalman_Gain_Calculation(double E_est, double E_mea);
    //预测
    void Kalman_X_K_Calculation();
    //
    void Generating_Random_Measurement();

private:
   const double delta_t=0.1;//周期
   const double acc=10;//加速度大小
   




    MatrixXd A(2,2);
	A(0,0) = 1;
	A(1,0) = 0;
	A(0,1) = delta_t;
	A(1,1) = 1;
 
	MatrixXd B(2,1);
	B(0,0) = pow(delta_t,2)/2;
	B(1,0) = delta_t;
 
	MatrixXd H(1,2);//测量的是小车的位移，速度为0
	H(0,0) = 1;
	H(0,1) = 0;
	
	MatrixXd Q(2,2);//过程激励噪声协方差，假设系统的噪声向量只存在速度分量上，且速度噪声的方差是一个常量0.01，位移分量上的系统噪声为0
	Q(0,0) = 0;
	Q(1,0) = 0;
	Q(0,1) = 0;
	Q(1,1) = 0.01;
 
	MatrixXd R(1,1);//观测噪声协方差，测量值只有位移，它的协方差矩阵大小是1*1，就是测量噪声的方差本身。
	R(0,0) = 10;

     //变量定义，包括状态预测值，状态估计值，测量值，预测状态与真实状态的协方差矩阵，估计状态和真实状态的协方差矩阵，初始值均为零
	X_evlt = MatrixXd::Constant(2,1,0);
    X_pdct = MatrixXd::Constant(2,1,0),
    Z_meas = MatrixXd::Constant(1,1,0), 
	Pk = MatrixXd::Constant(2,2,0),
    Pk_p = MatrixXd::Constant(2,2,0), 
    K = MatrixXd::Constant(2,1,0);
	vector<MatrixXd> x_evlt, x_pdct, z_meas, pk, pk_p, k;
	x_evlt.push_back(X_evlt);
	x_pdct.push_back(X_pdct);
	z_meas.push_back(Z_meas);
	pk.push_back(Pk);
	pk_p.push_back(Pk_p);
	k.push_back(K);

};  

KalmanFilter KF;