#include<iostream>
#include<opencv2/opencv.hpp>
#include<vector>
#include"ArmorParam.h"
#include"ArmorDescriptor.h"
#include"LightDescriptor.h"
using namespace std;
using namespace cv;

template<typename T>
float distance(const cv::Point_<T>& pt1, const cv::Point_<T>& pt2)
{
    return std::sqrt(std::pow((pt1.x - pt2.x), 2) + std::pow((pt1.y - pt2.y), 2));
}



class ArmorDetector {
public:
    //��ʼ�������������ҷ���ɫ
    void init(int selfColor) {
        if (selfColor == RED) {
            _enemy_color = BLUE;
            _self_color = RED;
        }
        cout << "hi";
    }

    void loadImg(Mat& img) {
        _srcImg = img;


        Rect imgBound = Rect(cv::Point(50, 50), Point(_srcImg.cols - 50, _srcImg.rows - 50));

        _roi = imgBound;
        _roiImg = _srcImg(_roi).clone();//ע��һ�£���_srcImg����roi�ü�֮��ԭ������Ҳ���ƶ����ü���ͼƬ�����Ͻ�

    }
    //ʶ��װ�װ��������
    int detect() {
        //��ɫ����
        _grayImg = separateColors();
        //namedWindow("_gray", WINDOW_FREERATIO);
        //imshow("_gray", _grayImg);
        //waitKey(0);


        int brightness_threshold = 120;//������ֵ,ȡ��������ع��
        Mat binBrightImg;
        //��ֵ��
        threshold(_grayImg, binBrightImg, brightness_threshold, 255, cv::THRESH_BINARY);
        //namedWindow("thresh", WINDOW_FREERATIO);
        //imshow("thresh", binBrightImg);
        //waitKey(0);

        //����
        Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3));
        dilate(binBrightImg, binBrightImg, element);
        //namedWindow("dilate", WINDOW_FREERATIO);
        //imshow("dilate", binBrightImg);
        //waitKey(0);

        //������
        vector<vector<Point> > lightContours;
        findContours(binBrightImg.clone(), lightContours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

        //////debug/////
        _debugImg = _roiImg.clone();
        for (size_t i = 0; i < lightContours.size(); i++) {
            drawContours(_debugImg, lightContours, i, Scalar(0, 0, 255), 1, 8);

        }
        //namedWindow("contours", WINDOW_FREERATIO);
        //imshow("contours", _debugImg);
        //waitKey(0);
        ////////////////


        //ɸѡ����
        vector<LightDescriptor> lightInfos;
        filterContours(lightContours, lightInfos);
        //û�ҵ������ͷ���û�ҵ�
        if (lightInfos.empty()) {
            return  -1;
        }

        //debug ���Ƶ�������
        drawLightInfo(lightInfos);
        //waitKey(0);

        //ƥ��װ�װ�
        _armors = matchArmor(lightInfos);
        if (_armors.empty()) {
            return  -1;
        }

        //����װ�װ�����
        for (size_t i = 0; i < _armors.size(); i++) {
            vector<Point2i> points;
            for (int j = 0; j < 4; j++) {
                points.push_back(Point(static_cast<int>(_armors[i].vertex[j].x), static_cast<int>(_armors[i].vertex[j].y)));
            }
           
            polylines(_debugImg, points, true, Scalar(0, 255, 0), 1, 8, 0);//�������������Ķ����
            //SolvePnp(points);
        }
        namedWindow("aromors", WINDOW_FREERATIO);
        imshow("aromors", _debugImg);
        //waitKey(100);
        return 0;
    }
    //����ɫ�ʣ���ȡ������Ҫ��Ҳ���ǵ��ˣ�����ɫ�����ػҶ�ͼ
    Mat separateColors() {
        vector<Mat> channels;
        // ��һ��3ͨ��ͼ��ת����3����ͨ��ͼ��
        split(_roiImg, channels);//����ɫ��ͨ��

        //imshow("split_B", channels[0]);
        //imshow("split_G", channels[1]);
        //imshow("split_R", channels[2]);
        Mat grayImg;

        //�޳����ǲ���Ҫ����ɫ
        //����ͼ���к�ɫ��������˵����rgb������r��ֵ���g��b�����������Ӧ����0��ͬ����ɫ�����b����Ӧ�����,������Ҫ����ɫ��ȥ��ʣ�µľ���������Ҫ����ɫ
        if (_enemy_color == RED) {
            grayImg = channels.at(2) - channels.at(0);//R-B
        }
        else {
            grayImg = channels.at(0) - channels.at(2);//B-R
        }
        return grayImg;
    }

    //ɸѡ��������������
    //����洢�����ľ��󣬷��ش洢������Ϣ�ľ���
    void filterContours(vector<vector<Point> >& lightContours, vector<LightDescriptor>& lightInfos) {
        for (const auto& contour : lightContours) {
            //�õ����
            float lightContourArea = contourArea(contour);
            //���̫С�Ĳ�Ҫ
            if (lightContourArea < _param.light_min_area) continue;
            //��Բ�������õ���Ӿ���
            RotatedRect lightRec = fitEllipse(contour);
            //���������ĽǶȣ�����Լ��Ϊ-45~45
            adjustRec(lightRec);
            //���߱ȡ�͹��ɸѡ����  ע��͹��=�������/��Ӿ������
            if (lightRec.size.width / lightRec.size.height > _param.light_max_ratio ||
                lightContourArea / lightRec.size.area() < _param.light_contour_min_solidity)
                continue;
            //�Ե�����Χ�ʵ�����
            lightRec.size.width *= _param.light_color_detect_extend_ratio;
            lightRec.size.height *= _param.light_color_detect_extend_ratio;

            //��Ϊ��ɫͨ������󼺷�����ֱ�ӹ��ˣ�����Ҫ�ж���ɫ��,����ֱ�ӽ���������
            lightInfos.push_back(LightDescriptor(lightRec));
        }
    }



    //������ת����
    void drawLightInfo(vector<LightDescriptor>& LD) {
        _debugImg = _roiImg.clone();

        vector<std::vector<cv::Point> > cons;
        int i = 0;
        for (auto& lightinfo : LD) {
            RotatedRect rotate = lightinfo.rec();
            auto vertices = new cv::Point2f[4];
            rotate.points(vertices);
            vector<Point> con;
            for (int i = 0; i < 4; i++) {
                con.push_back(vertices[i]);
            }
            cons.push_back(con);
            drawContours(_debugImg, cons, i, Scalar(0, 255, 255), 1, 8);
            //namedWindow("rotateRectangle", WINDOW_FREERATIO);
            //imshow("rotateRectangle", _debugImg);
            i++;
            //waitKey(0);
        }


    }

    //ƥ�������ɸѡ��װ�װ�
    vector<ArmorDescriptor> matchArmor(vector<LightDescriptor>& lightInfos) {
        vector<ArmorDescriptor> armors;
        //����������x��С��������
        sort(lightInfos.begin(), lightInfos.end(), [](const LightDescriptor& ld1, const LightDescriptor& ld2) {
            //Lambda����,��Ϊsort��cmp����
            return ld1.center.x < ld2.center.x;
            });
        for (size_t i = 0; i < lightInfos.size(); i++) {
            //�������е�������ƥ��
            for (size_t j = i + 1; (j < lightInfos.size()); j++) {
                const LightDescriptor& leftLight = lightInfos[i];
                const LightDescriptor& rightLight = lightInfos[j];

                //�ǲ�
                float angleDiff_ = abs(leftLight.angle - rightLight.angle);
                //���Ȳ����
                float LenDiff_ratio = abs(leftLight.length - rightLight.length) / max(leftLight.length, rightLight.length);
                //ɸѡ
                if (angleDiff_ > _param.light_max_angle_diff_ ||
                    LenDiff_ratio > _param.light_max_height_diff_ratio_) {

                    continue;
                }
                //���ҵ���������
                float dis = distance(leftLight.center, rightLight.center);
                //���ҵ������ȵ�ƽ��ֵ
                float meanLen = (leftLight.length + rightLight.length) / 2;
                //���ҵ������ĵ�y�Ĳ�ֵ
                float yDiff = abs(leftLight.center.y - rightLight.center.y);
                //y�����
                float yDiff_ratio = yDiff / meanLen;
                //���ҵ������ĵ�x�Ĳ�ֵ
                float xDiff = abs(leftLight.center.x - rightLight.center.x);
                //x�����
                float xDiff_ratio = xDiff / meanLen;
                //��������������ȱ�ֵ
                float ratio = dis / meanLen;
                //ɸѡ
                if (yDiff_ratio > _param.light_max_y_diff_ratio_ ||
                    xDiff_ratio < _param.light_min_x_diff_ratio_ ||
                    ratio > _param.armor_max_aspect_ratio_ ||
                    ratio < _param.armor_min_aspect_ratio_) {
                    continue;
                }

                //����ֵ��ȷ����Сװ��
                int armorType = ratio > _param.armor_big_armor_ratio ? BIG_ARMOR : SMALL_ARMOR;
                // ������ת�÷�
                float ratiOff = (armorType == BIG_ARMOR) ? max(_param.armor_big_armor_ratio - ratio, float(0)) : max(_param.armor_small_armor_ratio - ratio, float(0));
                float yOff = yDiff / meanLen;
                float rotationScore = -(ratiOff * ratiOff + yOff * yOff);
                //�õ�ƥ���װ�װ�
                ArmorDescriptor armor(leftLight, rightLight, armorType, _grayImg, rotationScore, _param);

                armors.emplace_back(armor);
                break;
            }
        }
        return armors;
    }

    void adjustRec(cv::RotatedRect& rec)
    {
        using std::swap;

        float& width = rec.size.width;
        float& height = rec.size.height;
        float& angle = rec.angle;



        while (angle >= 90.0) angle -= 180.0;
        while (angle < -90.0) angle += 180.0;


        if (angle >= 45.0)
        {
            swap(width, height);
            angle -= 90.0;
        }
        else if (angle < -45.0)
        {
            swap(width, height);
            angle += 90.0;
        }


    }
    cv::Mat _debugImg;
private:
    int _enemy_color;
    int _self_color;

    cv::Rect _roi; //ROI����

    cv::Mat _srcImg; //�����ͼƬ�����ڸó�Ա������
    cv::Mat _roiImg; //����һ֡��õ�ROI����
    cv::Mat _grayImg; //ROI����ĻҶ�ͼ
    vector<ArmorDescriptor> _armors;

    ArmorParam _param;
};


int main(int argc, char* argv[])
{
    string path = "/hard disk/opencv/1920_1.avi";
    VideoCapture cap(path);
    Mat img;
    while(true){
        
    cap >> img;
    ArmorDetector detector;
    detector.init(RED);
    detector.loadImg(img);
    detector.detect();


    waitKey(100);
    }
}