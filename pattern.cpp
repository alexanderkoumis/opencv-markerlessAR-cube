#include "pattern.h"

void patterntrackinginfo::computepose(const pattern& pattern, const cameracalibration& calibration)
{
	Mat rvec;
	Mat_<float> tvec;
	Mat raux, taux;

	solvePnP(pattern.points3d, points2d, calibration.getIntrinsic(), calibration.getDistorsion(), raux, taux);
	raux.convertTo(rvec, CV_32F);
	taux.convertTo(tvec, CV_32F);

	Mat_<float> rotmat(3, 3);
	Rodrigues(rvec, rotmat); // wat the hell does this do

	for (int col = 0; col < 3; col++)
	{
		for (int row = 0; row < 3; row++)
		{
			pose3d.r().mat[row][col] = rotmat(row, col);
		}
		pose3d.t().data[col] = tvec(col);
	}
	pose3d = pose3d.getinverted();
}

void patterntrackinginfo::draw2dcontour(Mat& image, Scalar color) const
{
	for (size_t i = 0; i < points2d.size(); i++)
	{
		line(image, points2d[i], points2d[ (i+1) % points2d.size() ], color, 2, CV_AA);
	}
}