#ifndef _BGMODEL_H
#define _BGMODEL_H

#include "opencv2/opencv.hpp"
#include <queue>
#include <algorithm>

using namespace cv;
using namespace std;

//declaring a class
class BGModel
{
private:
	Mat current_frame;
	Mat bg_frame;
	Mat prev_frame;
	queue<Mat> acc_frames; //set of frames from which to compute the background model
	
public:
	//constructor/destructor
	BGModel();
	~BGModel();
	void LoadAccumulator(Mat frame);
	Mat GetCurrentFrame();
	Mat GetPrevFrame();
	Mat GetBGFrame(); 
	void ComputeMedianBG();
	void ComputeMeanBG();
	
};

#endif
