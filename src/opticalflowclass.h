#ifndef _OPTICALFLOWCLASS_H
#define _OPTICALFLOWCLASS_H

#ifndef M_PI
	#define M_PI 3.14159265358979323846
#endif

#define ROI_WIDTH 3
#define ROI_HEIGHT 3

#include <opencv2/opencv.hpp>
#include <opencv/cv.h>
#include <fstream>
#include <iostream>
#include <algorithm>

using namespace std;

using namespace cv;

class OpticalFlow
{
public:
	//variables and pointers
	Mat frame;
	Mat frame_prev;
	Mat frame_color;
	Mat frame_prev_color;
	Mat flow;
	Mat flow_direction;
	Mat flow_mag;
	Mat status_image;
	Mat err_image;
	
	double pyrScale;
	int levels;
	int winsize;
	int iterations;
	int polyN;
	double polySigma;
	int flags;
	Size Win_Size_LK;
	TermCriteria criteria;
	
	//parameters for corner detection
	int maxCorners;
	int maxTrackbar;
	double qualityLevel;
	double minDistance;
	int blockSize_corner;
	bool useHarrisDetector;
	double k_param;
	
	// defining the methods
	OpticalFlow();
	OpticalFlow(Mat& frame);
	~OpticalFlow();
	
	void Initialize(Mat& frame);
	void UpdateImages(Mat& current_frame);
	void ComputeAndDrawOpticalFlow(Mat& flow,Mat& fl_mag, Mat& fl_dir);
	vector<Point2f> ComputeAndDrawOpticalFlowLK(Mat& flow,Mat mask,Mat mask_prev, Mat& fl_mag, Mat& fl_dir, vector<Point2f> prevPts, vector<Point2f>& currentPts,bool flagDraw = true);
	vector<Point2f> ExtractCoordsFromImage(Mat& image, Mat mask = Mat(), Rect roi_region = Rect());
	vector<Point2f> DrawOpticalFlowOnImage(Mat& image,vector<Point2f> prevPts,vector<Point2f> currentPts,Scalar* colors);
	Point2f ComputeOpticalFlowLK(Mat& image, Mat mask, Mat mask_prev, Mat& fl_mag, Mat& fl_dir, vector<Point2f> prevPts,vector<Point2f>& currentPts, bool flagDraw = true);
	
	//function to write a float image onto a file
	void WriteFloatImage(const string& filename,Mat& float_image);

};

#endif
