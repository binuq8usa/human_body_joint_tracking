#include "bgmodel.h"

//constructor/destructor definition
BGModel::BGModel()
{

}

BGModel::~BGModel()
{

}

//Definition
void BGModel::LoadAccumulator(Mat frame)
{
	current_frame = frame;
	acc_frames.push(current_frame);
}

Mat BGModel::GetCurrentFrame()
{
	return current_frame;
}

Mat BGModel::GetPrevFrame()
{
	return prev_frame;
}

Mat BGModel::GetBGFrame()
{
	return bg_frame;
}

void BGModel::ComputeMedianBG()
{
	
}

void BGModel::ComputeMeanBG()
{
	Mat acc = Mat::zeros(current_frame.rows,current_frame.cols,CV_32FC3);
	int num_of_frames = (int)(acc_frames.size());
	
	// computing the mean of the pixel location from all the frames
	while(!acc_frames.empty())
	{
		Mat temp_uint = acc_frames.front();
		Mat temp_fl;
		temp_uint.convertTo(temp_fl,CV_32FC3);
		acc = acc + temp_fl;
		acc_frames.pop();
	}
	acc /= num_of_frames;
	
	acc.convertTo(bg_frame,CV_8UC3);
	
}

