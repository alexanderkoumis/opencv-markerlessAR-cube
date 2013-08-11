#ifndef DEBUGHELPERS_H_
#define DEBUGHELPERS_H_
#include <opencv2/opencv.hpp>

#include <string>
#include <sstream>

using namespace cv;

template <typename T>
std::string tostring(const T& value)
{
	std::ostringstream stream;
	stream << value;
	return stream.str();
}

namespace cv
{
	inline void showandsave(string name, const Mat& m)
	{
		imshow(name, m);
		imwrite(name + ".png", m);
	}
	inline Mat getmatchesimage(Mat query, Mat pattern, const vector<KeyPoint>& querykp, const vector<KeyPoint>& trainkp, vector<DMatch> matches, int maxmatchesdrawn)
	{
		Mat outimg;
		if (matches.size() > maxmatchesdrawn)
		{
			matches.resize(maxmatchesdrawn);
		}

		cv::drawMatches(query, querykp, pattern, trainkp, matches, outimg, Scalar(0,200,0,255), Scalar::all(-1), vector<char>(), DrawMatchesFlags::DEFAULT);
		return outimg;
	}
}

#endif