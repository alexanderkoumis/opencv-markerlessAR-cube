#ifndef ARPIPELINE_H_
#define ARPIPELINE_H_

#include "patterndetector.h"
#include "cameracalibration.h"
#include "geometrytypes.h"

class arpipeline
{
public:
	arpipeline(const Mat patternimage, const cameracalibration& calibration);
	bool processframe(const Mat& inputframe);
	const transformation& getpatternlocation() const;
	patterndetector mpatterndetector;
	pattern mpattern; // used to be private
private:
	cameracalibration mcalibration;
	patterntrackinginfo mpatterninfo;
};

#endif