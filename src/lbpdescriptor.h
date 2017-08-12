#ifndef _LBPDESCRIPTOR_H
#define _LBPDESCRIPTOR_H

#include <iostream>
#include <algorithm>
#include <vector>
#include <bitset>
#include <cmath>
#include <opencv2/opencv.hpp>
	
using namespace cv;
using namespace std;

#define PI 3.14159265

struct _mapping
{
	vector<int> table;
	int samples;
	int num;
	bool flag; //to check if mapping is present. If false, no mapping
};

enum _maptype
{
	n, //no mapping
	u2, //for uniform lbp
	ri, //for rotation-invariant lbp
	riu2 // for uniform rotation invariant lbp
};

enum _mode
{
	none,
	h, //for histogram computation
	nh //for normalized histogram computation
};

//class for LBP
class LBPDescriptor
{
	public:
		int radius;
		int neighbors;
		int bins;
		Mat spoints;
		_mapping mapping;
		_maptype maptype;
		_mode mode;
		vector<float> img_hist;

		double miny,maxy,minx,maxx;
		int ysize,xsize; //image size
		int bsizey,bsizex; //block size
		int origy,origx;
		int dx,dy;

		LBPDescriptor(_maptype m_type);
		~LBPDescriptor();
		void ComputeMapping(int samples);
		vector<float> ComputeLBPImage(Mat source_img,Mat& dest_img,Mat mask = Mat(),Mat w_image = Mat());
		vector<float> ComputeHistogram(Mat dest_img,Mat mask = Mat(), Mat w_image = Mat());

};

#endif
