#include "hogdescriptor.h"
//definition of the class methods

//constructor
visionlab::HOGDescriptor::HOGDescriptor()
{
	num_cells_x = 2;
	num_cells_y = 2;
	num_bins = 36;
	image_gradient_mag = NULL;
	image_orient = NULL;
	bins = new float[num_bins];
	data.clear();
}

//destructor
visionlab::HOGDescriptor::~HOGDescriptor()
{
	if(image_gradient_mag != NULL)
		cvReleaseImage(&image_gradient_mag);
	if(image_orient != NULL)
		cvReleaseImage(&image_orient);
	delete[] bins;
}

//methods
vector<float> visionlab::HOGDescriptor::ComputeHOGDescriptor(IplImage* input)
{
	//computing the gradient and orientation
	ComputeGradientAndOrientation(input);
	//cvNamedWindow("Gradient");
	IplImage* temp_grad = cvCreateImage(cvGetSize(image_gradient_mag),IPL_DEPTH_8U,1);
	cvConvert(image_gradient_mag,temp_grad);
//	cvShowImage("Gradient",temp_grad);
//	cvWaitKey(0);
//	cvSaveImage("GradImage.jpg",temp_grad);
	
	int step_x = floor(input->width/(num_cells_x+1));
	int step_y = floor(input->height/(num_cells_y+1));
	int bin_size = 180/num_bins;
	
	// Local Gradient
	for(int cell_row = 0 ; cell_row < num_cells_y ; cell_row++)
	{
		for(int cell_col = 0; cell_col < num_cells_x ; cell_col++)
		{
			//setting the region of interest
			roi = cvRect(cell_col*step_x,cell_row*step_y,2*step_x,2*step_y);
			cvSetImageROI(image_gradient_mag,roi);
			cvSetImageROI(image_orient,roi);
			IplImage* subimg_grad = cvCreateImage(cvSize(roi.width,roi.height),image_gradient_mag->depth,image_gradient_mag->nChannels);
			IplImage* subimg_orient = cvCreateImage(cvSize(roi.width,roi.height),image_orient->depth,image_orient->nChannels);
			cvResize(image_gradient_mag,subimg_grad);
			cvResize(image_orient,subimg_orient);
			
//				IplImage* temp = cvCreateImage(cvGetSize(subimg_grad),IPL_DEPTH_8U,1);
//				char local_grad_image[20];
//				sprintf(local_grad_image,"%s%d.jpg","LocalGradImage",num_cells_y*cell_row+cell_col);
//				cvConvert(subimg_grad,temp);
//				cvSaveImage(local_grad_image,temp);
//				cvNamedWindow("Test",0);
//				cvShowImage("Test",temp);
//				cvWaitKey(0);
			
			
			ClearOrientationBins();
			//computing the histogram corresponding to that orientation 
			for(int i = 0 ; i < 2* step_y ; i ++)
			{
				float* ptr_grad = (float*)(subimg_grad->imageData + i * subimg_grad->widthStep);
				float* ptr_orient = (float*)(subimg_orient->imageData + i * subimg_orient->widthStep);
				for(int j = 0; j < 2*step_x ; j ++)
				{
					int bin_num = 0;
					for(int k = bin_size; k <= 180 ;k+= bin_size)
					{
						if(ptr_orient[j] < 0)
							ptr_orient[j] += 180; // to make the angle positive
						if( (ptr_orient[j] < k) && (ptr_orient[j] > (k-bin_size) ) )
						bins[bin_num] += ptr_grad[j];
						bin_num++;
					}
				}
			}
			
			//normalizing the histogram and adding to the HOG vector
			float sumsq = 0.0;
			for(int bin_num = 0 ; bin_num < num_bins ; bin_num ++)
				sumsq += bins[bin_num] * bins[bin_num];
			for(int bin_num = 0 ; bin_num < num_bins ; bin_num ++)
			{
				bins[bin_num] /= sqrt(sumsq);
				//bins[bin_num] *= 255;
				data.push_back(bins[bin_num]);
			}
			
			cvReleaseImage(&subimg_grad);
			cvReleaseImage(&subimg_orient);
			
			//resetting the image roi
			cvResetImageROI(image_gradient_mag);
			cvResetImageROI(image_orient);
		}
	}
	
	return data;
// END OF FUNCTION
}

vector<float> visionlab::HOGDescriptor::ComputeGlobalHOGDescriptor(IplImage* input)
{
	//computing the gradient and orientation
	ComputeGradientAndOrientation(input);
	//cvNamedWindow("Gradient");
	IplImage* temp_grad = cvCreateImage(cvGetSize(image_gradient_mag),IPL_DEPTH_8U,1);
	cvConvert(image_gradient_mag,temp_grad);
//	cvShowImage("Gradient",temp_grad);
//	cvWaitKey(0);
//	cvSaveImage("GradImage.jpg",temp_grad);

	int step_x = floor(input->width);
	int step_y = floor(input->height);
	int bin_size = 180/num_bins;

	// Local Gradient
	for(int cell_row = 0 ; cell_row < 1 ; cell_row++)
	{
		for(int cell_col = 0; cell_col < 1; cell_col++)
		{
			//setting the region of interest
			roi = cvRect(cell_col*step_x,cell_row*step_y,step_x,step_y);
			cvSetImageROI(image_gradient_mag,roi);
			cvSetImageROI(image_orient,roi);
			IplImage* subimg_grad = cvCreateImage(cvSize(roi.width,roi.height),image_gradient_mag->depth,image_gradient_mag->nChannels);
			IplImage* subimg_orient = cvCreateImage(cvSize(roi.width,roi.height),image_orient->depth,image_orient->nChannels);
			cvResize(image_gradient_mag,subimg_grad);
			cvResize(image_orient,subimg_orient);

//				IplImage* temp = cvCreateImage(cvGetSize(subimg_grad),IPL_DEPTH_8U,1);
//				char local_grad_image[20];
//				sprintf(local_grad_image,"%s%d.jpg","LocalGradImage",num_cells_y*cell_row+cell_col);
//				cvConvert(subimg_grad,temp);
//				cvSaveImage(local_grad_image,temp);
//				cvNamedWindow("Test",0);
//				cvShowImage("Test",temp);
//				cvWaitKey(0);


			ClearOrientationBins();
			//computing the histogram corresponding to that orientation
			for(int i = 0 ; i < step_y ; i ++)
			{
				float* ptr_grad = (float*)(subimg_grad->imageData + i * subimg_grad->widthStep);
				float* ptr_orient = (float*)(subimg_orient->imageData + i * subimg_orient->widthStep);
				for(int j = 0; j < step_x ; j ++)
				{
					int bin_num = 0;
					for(int k = bin_size; k <= 180 ;k+= bin_size)
					{
						if(ptr_orient[j] < 0)
							ptr_orient[j] += 180; // to make the angle positive
						if( (ptr_orient[j] < k) && (ptr_orient[j] > (k-bin_size) ) )
						bins[bin_num] += ptr_grad[j];
						bin_num++;
					}
				}
			}

			//normalizing the histogram and adding to the HOG vector
			float sumsq = 0.0;
			for(int bin_num = 0 ; bin_num < num_bins ; bin_num ++)
				sumsq += bins[bin_num] * bins[bin_num];
			for(int bin_num = 0 ; bin_num < num_bins ; bin_num ++)
			{
				bins[bin_num] /= sqrt(sumsq);
				//bins[bin_num] *= 255;
				data.push_back(bins[bin_num]);
			}

			cvReleaseImage(&subimg_grad);
			cvReleaseImage(&subimg_orient);

			//resetting the image roi
			cvResetImageROI(image_gradient_mag);
			cvResetImageROI(image_orient);
		}
	}

	return data;
// END OF FUNCTION
}

void visionlab::HOGDescriptor::ComputeGradientAndOrientation(IplImage* input)
{
	IplImage* input_grad_x = cvCreateImage(cvGetSize(input),IPL_DEPTH_32F,1);
	IplImage* input_grad_y = cvCreateImage(cvGetSize(input),IPL_DEPTH_32F,1);
	IplImage* input_gray = cvCreateImage(cvGetSize(input),IPL_DEPTH_8U,1);
	
	//cvCvtColor(input,input_gray,CV_BGR2GRAY);
	cvCopy(input,input_gray);
	
	image_gradient_mag = cvCreateImage(cvGetSize(input),IPL_DEPTH_32F,1);
	image_orient = cvCreateImage(cvGetSize(input),IPL_DEPTH_32F,1);
	
	//taking the gradient in the x and y direction
	cvSobel(input_gray,input_grad_x,1,0,1);//grad_x
	cvSobel(input_gray,input_grad_y,0,1,1);//grad_y
	
	//Computing the gradient magnitude and direction - some problem here
	for(int i = 0 ;i < input->height; i++)
	{
		float* ptr_gradient = (float*)(image_gradient_mag->imageData + i * image_gradient_mag->widthStep);
		float* ptr_orient = (float*)(image_orient->imageData + i * image_orient->widthStep);
		float* ptr_grad_x = (float*)(input_grad_x ->imageData + i * input_grad_x ->widthStep);
		float* ptr_grad_y = (float*)(input_grad_y->imageData + i * input_grad_y->widthStep);
		for(int j = 0; j < input->width; j++)
		{
			ptr_gradient[j] = sqrt( (float)(ptr_grad_x[j]*ptr_grad_x[j] + ptr_grad_y[j] * ptr_grad_y[j]) );
			ptr_orient[j] = atan2((float)(ptr_grad_y[j]),(float)(ptr_grad_x[j])) *180 / PI; 
		}
	}
// 	cvNamedWindow("Orient");
// 	cvShowImage("Orient",image_orient);
// 	cvWaitKey(0);
	cvReleaseImage(&input_grad_x);
	cvReleaseImage(&input_grad_y);
	cvReleaseImage(&input_gray);
	
}
void visionlab::HOGDescriptor::ClearOrientationBins()
{
	for(int k = 0 ; k < num_bins ; k ++)
		bins[k] = 0;
}
