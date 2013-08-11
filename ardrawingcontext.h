#ifndef ardrawingcontext_hpp
#define ardrawingcontext_hpp
#include "geometrytypes.h"
#include "cameracalibration.h"
#include <opencv2/opencv.hpp>
using namespace std;
using namespace cv;

void ardrawingcontextdrawcallback(void* param);

class ardrawingcontext
{
public:
	ardrawingcontext(string windowname, Size framesize, const cameracalibration& c);
	~ardrawingcontext();

	bool ispatternpresent;
	transformation patternpose;

	//Set the new frame for the background
	void updatebackground(const Mat& frame);
	void updatewindow();
private:
	friend void ardrawingcontextdrawcallback(void* param);
	void draw();
	void drawaugmentedscene();
	void drawcameraframe();
	void buildprojectionmatrix(const cameracalibration& calibration, int w, int h, matrix44& result);
	void drawcoordinateaxis();
	void drawcubemodel();
private:
	bool mistextureinitialized;
	unsigned int mbackgroundtextureid;
	cameracalibration mcalibration;
	cv::Mat mbackgroundimage;
	string mwindowname;
};
#endif
