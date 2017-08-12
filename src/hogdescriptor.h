#ifndef _HOGDESCRIPTOR_H
#define _HOGDESCRIPTOR_H

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <vector>
#include <cmath>
#define PI 3.14159265
using namespace std;

//defining the class
namespace visionlab
{

	class HOGDescriptor
	{
	public:
		int num_cells_x; //along columns
		int num_cells_y; //along rows
		int num_bins;
		CvRect roi; //rectangle to define the region of interest
		IplImage* image_gradient_mag;
		IplImage* image_orient;
		float* bins;
		vector<float> data; //this refers to the HOG descriptors computed for the image

		HOGDescriptor();
		~HOGDescriptor();
		vector<float> ComputeHOGDescriptor(IplImage* input); //declaring the method for computing the Hog descriptors
		vector<float> ComputeGlobalHOGDescriptor(IplImage* input);
		//vector<float> ComputeHOGDescriptorFromCorners(IplImage* input);
		void ClearOrientationBins(); //clearing the orietation bins
		void ComputeGradientAndOrientation(IplImage* input);
		//void ComputeCorners(IplImage* input);
	};

}

#endif
