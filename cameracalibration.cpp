#include "cameracalibration.h"


cameracalibration::cameracalibration()
{
}

cameracalibration::cameracalibration(float _fx, float _fy, float _cx, float _cy)
{
    m_intrinsic = cv::Matx33f::zeros();

    fx() = _fx;
    fy() = _fy;
    cx() = _cx;
    cy() = _cy;

    m_distortion.create(5,1);
    for (int i=0; i<5; i++)
        m_distortion(i) = 0;
}

cameracalibration::cameracalibration(float _fx, float _fy, float _cx, float _cy, float distorsionCoeff[5])
{
    m_intrinsic = cv::Matx33f::zeros();

    fx() = _fx;
    fy() = _fy;
    cx() = _cx;
    cy() = _cy;

    m_distortion.create(5,1);
    for (int i=0; i<5; i++)
        m_distortion(i) = distorsionCoeff[i];
}

const cv::Matx33f& cameracalibration::getIntrinsic() const
{
    return m_intrinsic;
}

const cv::Mat_<float>& cameracalibration::getDistorsion() const
{
    return m_distortion;
}

float& cameracalibration::fx()
{
    return m_intrinsic(1,1);
}

float& cameracalibration::fy()
{
    return m_intrinsic(0,0);
}

float& cameracalibration::cx()
{
    return m_intrinsic(0,2);
}

float& cameracalibration::cy()
{
    return m_intrinsic(1,2);
}

float cameracalibration::fx() const
{
    return m_intrinsic(1,1);
}

float cameracalibration::fy() const
{
    return m_intrinsic(0,0);
}

float cameracalibration::cx() const
{
    return m_intrinsic(0,2);
}

float cameracalibration::cy() const
{
    return m_intrinsic(1,2);
}