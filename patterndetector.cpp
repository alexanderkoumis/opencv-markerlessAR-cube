#include "patterndetector.h"


patterndetector::patterndetector(cv::Ptr<cv::FeatureDetector> detector,
		cv::Ptr<cv::DescriptorExtractor> extractor,
		cv::Ptr<cv::DescriptorMatcher> matcher,
			bool ratiotest) : mdetector(detector), mextractor(extractor), mmatcher(matcher), enableratiotest(ratiotest)
{
}

void patterndetector::train(const pattern& pattern)
{
	mpattern = pattern;
	mmatcher->clear();
	std::vector<cv::Mat> descriptors(1);
	descriptors[0] = pattern.descriptors.clone();
	mmatcher->add(descriptors);
}

void patterndetector::buildpatternfromimage(const Mat& image, pattern& pattern) const
{
	int numImages = 4;
	float step = sqrtf(2.0f);

	// Store original image in pattern structure
	pattern.size = cv::Size(image.cols, image.rows);
	pattern.frame = image.clone();
	getgray(image, pattern.grayImg);
    
	// Build 2d and 3d contours (3d contour lie in XY plane since it's planar)
	pattern.points2d.resize(4);
	pattern.points3d.resize(4);

	// Image dimensions
	const float w = image.cols;
	const float h = image.rows;

	// Normalized dimensions:
	const float maxSize = std::max(w,h);
	const float unitW = w / maxSize;
	const float unitH = h / maxSize;

	pattern.points2d[0] = cv::Point2f(0,0);
	pattern.points2d[1] = cv::Point2f(w,0);
	pattern.points2d[2] = cv::Point2f(w,h);
	pattern.points2d[3] = cv::Point2f(0,h);

	pattern.points3d[0] = cv::Point3f(-unitW, -unitH, 0);
	pattern.points3d[1] = cv::Point3f( unitW, -unitH, 0);
	pattern.points3d[2] = cv::Point3f( unitW, unitH, 0);
	pattern.points3d[3] = cv::Point3f(-unitW, unitH, 0);

	extractfeatures(pattern.grayImg, pattern.keypoints, pattern.descriptors);

}


bool patterndetector::findpattern(const cv::Mat& image, patterntrackinginfo& info) //problem
{
    // Convert input image to gray
    getgray(image, mgrayimg);
    
    // Extract feature points from input gray image
    extractfeatures(mgrayimg, mquerykeypoints, mquerydescriptors);

    // Get matches with current pattern
    getmatches(mquerydescriptors, mmatches); // problem

//	cv::Mat tmp = image.clone(); // for debugging purposes

    // Find homography transformation and detect good matches
    bool homographyfound = refinematcheswithhomography(
        mquerykeypoints,
        mpattern.keypoints,
        homographyreprojectionthreshold,
        mmatches,
        mroughhomography);

    if (homographyfound)
    {
        // If homography refinement enabled improve found transformation
        if (enablehomographyrefinement)
        {
            // Warp image using found homography
            cv::warpPerspective(mgrayimg, mwarpedimg, mroughhomography, mpattern.size, cv::WARP_INVERSE_MAP | cv::INTER_CUBIC);
		
			// Get refined matches:
            std::vector<cv::KeyPoint> warpedKeypoints;
            std::vector<cv::DMatch> refinedMatches;

            // Detect features on warped image
            extractfeatures(mwarpedimg, warpedKeypoints, mquerydescriptors);

            // Match with pattern
            getmatches(mquerydescriptors, refinedMatches);

            // Estimate new refinement homography
            homographyfound = refinematcheswithhomography(
                warpedKeypoints,
                mpattern.keypoints,
                homographyreprojectionthreshold,
                refinedMatches,
                mrefinedhomography);

			// Get a result homography as result of matrix product of refined and rough homographies:
            info.homography = mroughhomography * mrefinedhomography;

            // Transform contour with rough homography
			perspectiveTransform(mpattern.points2d, info.points2d, mroughhomography);
	//		info.draw2dcontour(tmp, CV_RGB(0,200,0));

			// Transform contour with precise homography
			perspectiveTransform(mpattern.points2d, info.points2d, info.homography);
//			info.draw2dcontour(tmp, CV_RGB(200,0,0));

        }
    
		else
        {
            info.homography = mroughhomography;

            // Transform contour with rough homography
            cv::perspectiveTransform(mpattern.points2d, info.points2d, mroughhomography);
        }
    }
    return homographyfound;
}

bool patterndetector::extractfeatures(const Mat& image, vector<KeyPoint>& keypoints, Mat& descriptors) const
{
	assert(!image.empty());
	assert(image.channels() == 1);

	mdetector->detect(image, keypoints);
	if (keypoints.empty())
        return false;

	mextractor->compute(image, keypoints, descriptors);
    if (keypoints.empty())
        return false;

    return true;
}

void patterndetector::getmatches(const Mat& querydescriptors, vector<DMatch>& matches)
{
	matches.clear();

	if(enableratiotest)
	{
		const float minratio = 1.f / 1.5f;
		mmatcher->knnMatch(querydescriptors, mknnmatches, 2);

		for(size_t i = 0; i < mknnmatches.size(); i++)
		{
			const cv::DMatch& bestmatch = mknnmatches[i][0];
			const cv::DMatch& bettermatch = mknnmatches[i][1];

			float distanceratio = bestmatch.distance / bettermatch.distance;

			if (distanceratio < minratio)
			{
				matches.push_back(bestmatch);
			}
		}
	}
	else
	{
		mmatcher->match(querydescriptors, matches);
	}
}

void patterndetector::getgray(const Mat& image, Mat& gray)
{
	if (image.channels() == 3)
	{
		cvtColor(image, gray, CV_BGR2GRAY);
	}
    else if (image.channels() == 4)
	{
		cvtColor(image, gray, CV_BGRA2GRAY);
	}
    else if (image.channels() == 1)
	{
		gray = image;
	}
}

bool patterndetector::refinematcheswithhomography
	(
	const std::vector<cv::KeyPoint>& queryKeypoints,
	const std::vector<cv::KeyPoint>& trainKeypoints,
	float reprojectionThreshold,
	std::vector<cv::DMatch>& matches,
	cv::Mat& homography
	)
{
	const int minNumberMatchesAllowed = 20;

	if (matches.size() < minNumberMatchesAllowed)
		return false;

    // Prepare data for cv::findHomography
	std::vector<cv::Point2f> srcPoints(matches.size());
	std::vector<cv::Point2f> dstPoints(matches.size());

	for (size_t i = 0; i < matches.size(); i++)
	{
		srcPoints[i] = trainKeypoints[matches[i].trainIdx].pt;
		dstPoints[i] = queryKeypoints[matches[i].queryIdx].pt;
	}

	// Find homography matrix and get inliers mask
	std::vector<unsigned char> inliersMask(srcPoints.size());
	homography = cv::findHomography(srcPoints,
                                    dstPoints,
                                    CV_FM_RANSAC,
                                    reprojectionThreshold,
                                    inliersMask);

	std::vector<cv::DMatch> inliers;
	for (size_t i=0; i<inliersMask.size(); i++)
	{
		if (inliersMask[i])
			inliers.push_back(matches[i]);
	}
	matches.swap(inliers);

	return matches.size() > minNumberMatchesAllowed;
}