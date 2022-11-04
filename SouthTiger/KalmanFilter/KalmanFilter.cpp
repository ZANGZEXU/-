#include"KalmanFilter.h"
#include<iostream>
#include<opencv2/opencv.hpp>
#include<cmath>

using namespace std;
using namespace cv;
using namespace KalmanFilter;

KalmanFilter::KalmanFilter(vecotr<Point2f>Xpoint)
{
    for(int i = 0; ; ++i)
	{
		//先验估计
		KF.X_pdct = KF.A * x_evlt[i-1] + KF.B *KF.acc;
		KalmanFilter::x_pdct.push_back(X_pdct);

		//预测状态与真实状态的协方差矩阵，Pk'
		KF.Pk_p = KF.A * pk[i-1]*KF.A.transpose() + KF.Q;
		pk_p.push_back(Pk_p);

		//K:2x1
		MatrixXd::tmp(1,1);
		tmp = KF.H * pk_p[i] * H.transpose() + KF.R;
		KF.K = pk_p[i] * H.transpose() * tmp.inverse();
		k.push_back(K);

		//测量值z
        //使用计算机测量
		KF.Z_meas = Xpoints.clone();
		z_meas.push_back(Z_meas);
       
		//估计值
		KF.X_evlt = KalmanFilter::x_pdct[i] + KalmanFilter::k[i] * 
                               (KalmanFilter::z_meas[i] - KalmanFilter::H * KalmanFilter::x_pdct[i]);
		KalmanFilter::x_evlt.push_back(X_evlt);
		Predic(vector<MatrixXd> x_evlt,int i),vecotr<MatrixXd>Xpoint);
        
		//估计状态和真实状态的协方差矩阵，Pk
		KalmanFilter::Pk = (MatrixXd::Identity(2,2) - k[i] * H) * pk_p[i];
		KalmanFilter::pk.push_back(Pk);
	}
}
//状态变量 例如：x=AX0+Vot+(1/2)at^2

//观测变量 Zk=HXk+Vk

//噪声  1.过程噪声 2.观测噪声

    //方差与协方差的定义

//递归
KalmanFilter::Kalman_X_K_Calculation()
{
    char i;
    double Kalman_Gain =Kalman_Gain_Calculation();
    KalmanFilter::X_k[i]=KalmanFilter::X_k[i-1]+Kalman_Gain*(Z_k[i]-Z_k[i-1]);
    KalmanFilter::E_est[i]=((double)1-Kalman_Gain)*E_est[i-1];
}
//更新估计误差
KalmanFilter::Kalman_Gain_Refresh(double & E_east,double & K_n){
    E_est=(1-K_n)*E_east;
}
//卡尔曼增益
KalmanFilter::Kalman_Gain_Calculation(double & E_est,double & E_mea){
    double result;
    result=E_est/(E_est+E_mea);
    Kalman_Gain_Refresh(E_east,result);
}



KalmanFilter::Generating_Random_Measurement(){
    char i;
    for(i=0;i<50;i++){
        KalmanFilter::Z_k[i]=47+rand()%6;
    }
}
//数据融合