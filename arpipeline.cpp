#include "arpipeline.h"


arpipeline::arpipeline(const Mat patternimage, const cameracalibration& calibration) : mcalibration(calibration)
{
  mpatterndetector.buildpatternfromimage(patternimage, mpattern);
  mpatterndetector.train(mpattern);
}

bool arpipeline::processframe(const Mat& inputframe)
{
	bool patternfound = mpatterndetector.findpattern(inputframe, mpatterninfo);

	if (patternfound)
	{
		mpatterninfo.computepose(mpattern, mcalibration);
	}

	return patternfound;
}

const transformation& arpipeline::getpatternlocation() const
{
	return mpatterninfo.pose3d;
}