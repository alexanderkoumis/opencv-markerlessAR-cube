#include <opencv2/opencv.hpp>
#define NOMINMAX
#include <Windows.h>

#include <gl/GL.h>
#include <gl/GLU.h>

#include "ardrawingcontext.h"
#include "arpipeline.h"
#include "debughelpers.h"

using namespace std;
using namespace cv;


void processvideo(const Mat& patternimage, cameracalibration& calibration, VideoCapture& capture);
bool processframe(const Mat& cameraframe, arpipeline& pipeline, ardrawingcontext& drawingctx);

int main()
{
	cameracalibration calibration(681.60368696469448f, 681.60368696469448f, 319.5f, 239.5f);
	Mat patternimage = imread("snapshot.jpg");
	VideoCapture cap(0);
	processvideo(patternimage, calibration, cap);
}

void processvideo(const cv::Mat& patternimage, cameracalibration& calibration, cv::VideoCapture& capture)
{
	Mat currentframe;
	capture >> currentframe;
	Size framesize(currentframe.cols, currentframe.rows);
	arpipeline pipeline(patternimage, calibration);
	ardrawingcontext drawingctx("Markerless AR", framesize, calibration);

    bool shouldQuit = false;
    do
    {
        capture >> currentframe;
        if (currentframe.empty())
        {
            shouldQuit = true;
            continue;
        }

        shouldQuit = processframe(currentframe, pipeline, drawingctx);

    } while (!shouldQuit);
}

bool processframe(const Mat& cameraframe, arpipeline& pipeline, ardrawingcontext& drawingctx)
{
	Mat img = cameraframe.clone();
	// Put all the homography refinement junk here later
	drawingctx.updatebackground(img); // copies img to camera frame
	drawingctx.ispatternpresent = pipeline.processframe(cameraframe); // finds the pattern and updates its detection status
	drawingctx.patternpose = pipeline.getpatternlocation(); // pushes pose3d info to drawingctx patternpose
	drawingctx.updatewindow();

	int keycode = waitKey(5);

	bool shouldquit = false;
	if (keycode == 27 || keycode == 'q')
	{
		shouldquit = true;
	}
	return shouldquit;
}