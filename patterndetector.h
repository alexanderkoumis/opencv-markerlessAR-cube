#ifndef PATTERNDETECTOR_H_
#define PATTERNDETECTOR_H_
#include <opencv2/opencv.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include "pattern.h"


class patterndetector
{
public:
	patterndetector
		(
		cv::Ptr<cv::FeatureDetector> detector = new cv::ORB(1000),
		cv::Ptr<cv::DescriptorExtractor> extractor = new cv::FREAK(false, false),
		cv::Ptr<cv::DescriptorMatcher> matcher = new cv::BFMatcher(cv::NORM_HAMMING, true),
			bool enableRatioTest = false);

	void train(const pattern& pattern);
	void buildpatternfromimage(const Mat& image, pattern& pattern) const;

	bool findpattern(const Mat& image, patterntrackinginfo& info);
	bool enableratiotest;
	bool enablehomographyrefinement;
	float homographyreprojectionthreshold;


	vector<KeyPoint> mquerykeypoints; // used to be private
	vector<DMatch> mmatches; // used to be private

protected:
	bool extractfeatures(const Mat& image, vector<KeyPoint>& keypoints, Mat& descriptors) const;
	void getmatches(const Mat& querydescriptors, vector<DMatch>& matches);
	static void getgray(const Mat& image, Mat& gray);
	static bool refinematcheswithhomography(
		const vector<KeyPoint>& querykeypoints,
		const vector<KeyPoint>& trainkeypoints,
		float reprojectionthreshold,
		vector<DMatch>& matches,
		Mat& homography);

private:
	
	Mat mquerydescriptors;

	vector<vector<DMatch>> mknnmatches;

	Mat mgrayimg;	
	Mat mwarpedimg;
	Mat mroughhomography;
	Mat mrefinedhomography;
	
	pattern mpattern;
	cv::Ptr<cv::FeatureDetector> mdetector;
	cv::Ptr<cv::DescriptorExtractor> mextractor;
	cv::Ptr<cv::DescriptorMatcher> mmatcher;
};

#endif