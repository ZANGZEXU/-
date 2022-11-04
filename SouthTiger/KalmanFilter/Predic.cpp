#include"KalmanFilter.h"
#include<iostream>
#include<opencv2/opencv.hpp>
#include<cmath>

using namespace std;
using namespace cv;
using namespace KalmanFilter;

void predict(vector<MatrixXd> x_evlt,int i,vecotr<MatrixXd>Xpoint){
if(i){

     MatrixXd deleta=x_evlt[i]-x_evlt[i-1];
     Xpoint=Xpoint+deleta;
     for(int j=0;j<4;j++){
      line(video, Xpoint[0], vertices[(1) % 4], Scalar(0, 255, 0));
     }
}
}