#ifndef CAMERACALIBRATION_H_
#define CAMERACALIBRATION_H_
#include <opencv2/opencv.hpp>

class cameracalibration
{
public:
	cameracalibration();
	cameracalibration(float fx, float xy, float cx, float cy);
	cameracalibration(float fx, float xy, float cx, float cy, float distortioncoefficient[5]);
	
    void getMatrix34(float cparam[3][4]) const;

    const cv::Matx33f& getIntrinsic() const;
    const cv::Mat_<float>& getDistorsion() const;

    float& fx();
    float& fy();

    float& cx();
    float& cy();

    float fx() const;
    float fy() const;

    float cx() const;
    float cy() const;
private:
    cv::Matx33f m_intrinsic;
    cv::Mat_<float> m_distortion;
};
#endif