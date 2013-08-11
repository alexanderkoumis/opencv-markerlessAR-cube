#ifndef PATTERN_H_
#define PATTERN_H_

#include "geometrytypes.h"
#include "cameracalibration.h"

#include <opencv2/opencv.hpp>
#include <opencv2/nonfree/gpu.hpp>
using namespace cv;

struct pattern
{
	Size size;
	Mat frame;
	Mat grayImg;
	
	vector<KeyPoint> keypoints;
	Mat descriptors;

	vector<Point2f> points2d;
	vector<Point3f> points3d;
};

struct patterntrackinginfo
{
	Mat homography;
	vector<Point2f> points2d;
	transformation pose3d;

	void draw2dcontour(Mat& image, Scalar color) const;

	void computepose(const pattern& pattern, const cameracalibration& calibration);
};

#endif