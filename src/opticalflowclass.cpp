#include "opticalflowclass.h"
#include <cmath>
//defining the methods

//constructor
OpticalFlow::OpticalFlow()
{
	//Initializing the variables for optical flow
	pyrScale = 0.5;
	levels = 3;
	winsize = 7;
	iterations = 10;
	polyN = 7;
	polySigma = 1.5;
	flags = OPTFLOW_FARNEBACK_GAUSSIAN;
	criteria= TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 30, 0.01);
	
	Win_Size_LK = Size(winsize,winsize);
	
	//Initializing the parameters for the corner detection
	maxCorners = 1000;
	maxTrackbar = 100;
	qualityLevel = 0.001;
	minDistance = 1;
	blockSize_corner = 3;
	useHarrisDetector = true;
	k_param = 0.04;
}

OpticalFlow::OpticalFlow(Mat& f)
{
	//Initializing the matrices and structures
	if(f.channels() == 3)
	{
		frame_color = f.clone();
		cvtColor(f,frame,CV_BGR2GRAY);
	}
	else
		frame = f.clone();
	frame_prev = Mat(frame.rows,frame.cols,CV_8UC1);
	frame_prev_color = Mat(frame_color.rows,frame_color.cols,CV_8UC3);
	flow = Mat(frame.rows,frame.cols,CV_32FC2);
	flow_direction = Mat(frame.rows,frame.cols,CV_32FC1);
	flow_mag = Mat(frame.rows,frame.cols,CV_32FC1);
	//Initializing the variables for optical flow - Farneback method
	pyrScale = 0.5;
	levels = 3;
	winsize = 10;
	iterations = 3;
	polyN = 7;
	polySigma = 1.5;
	flags = OPTFLOW_FARNEBACK_GAUSSIAN;
	Win_Size_LK = Size(winsize,winsize);
	
	//Initializing the parameters for the corner detection
	maxCorners = 50;
	maxTrackbar = 100;
	qualityLevel = 0.01;
	minDistance = 5;
	blockSize_corner = 3;
	useHarrisDetector = true;
	k_param = 0.04;
}

OpticalFlow::~OpticalFlow()
{
	frame.release();
	frame_prev.release();
	frame_color.release();
	frame_prev_color.release();
	flow.release();
	flow_direction.release();
	flow_mag.release();
}

//initializing with the first frame
void OpticalFlow::Initialize(Mat& f)
{
	//Initializing the matrices and structures
	if(f.channels() == 3)
	{
	    frame_color = f.clone();
		cvtColor(f,frame,CV_BGR2GRAY);
	}
	else
		frame = f.clone();
	frame_prev = Mat(frame.rows,frame.cols,CV_8UC1);
	frame_prev_color = Mat(frame_color.rows,frame.cols,CV_8UC3);
	flow = Mat(frame.rows,frame.cols,CV_32FC2);
}

//updating the images
void OpticalFlow::UpdateImages(Mat& current_f)
{
	frame_prev = frame.clone();
	frame_prev_color = frame_color.clone();
	frame.release();

	if(current_f.channels() == 3)
	{
		frame_color = current_f.clone();
		cvtColor(current_f,frame,CV_BGR2GRAY);
	}
	else
		frame = current_f.clone();
}

//computing optical flow 
void OpticalFlow::ComputeAndDrawOpticalFlow(Mat& frame_display,Mat& fl_mag, Mat& fl_dir)
{
	//extract corners from previous and current frame
	vector<Point2f> prevPts,currentPts;
	goodFeaturesToTrack(frame_prev,prevPts,maxCorners,qualityLevel,minDistance,Mat(),blockSize_corner,useHarrisDetector,k_param);
	goodFeaturesToTrack(frame,currentPts,maxCorners,qualityLevel,minDistance,Mat(),blockSize_corner,useHarrisDetector,k_param);
	
	//draw the corners - Blue
	for(int i = 0 ; i < prevPts.size();i++)
		circle(frame_display,prevPts[i],1,Scalar(255,0,0),-1,8,0);
	
	//image representing the location of points
	Mat points_prev = Mat::zeros(Size(frame_prev.cols,frame_prev.rows),CV_8UC1);
	Mat points_current = Mat::zeros(Size(frame_prev.cols,frame_prev.rows),CV_8UC1);
	for(int k = 0; k < prevPts.size(); k++)
		points_prev.at<unsigned char>((int)(prevPts[k].y),(int)(prevPts[k].x)) = frame_prev.at<unsigned char>((int)(prevPts[k].y),(int)(prevPts[k].x)); 
		
	for(int k = 0; k < currentPts.size(); k++)
		points_current.at<unsigned char>((int)(currentPts[k].y),(int)(currentPts[k].x)) = frame.at<unsigned char>((int)(currentPts[k].y),(int)(currentPts[k].x)); 
	
	
	calcOpticalFlowFarneback(points_prev,points_current,flow,pyrScale,levels,winsize,iterations,polyN,polySigma,flags);
	
	//Computing the hypotenuse and the angle from the optical flow vector for each point
	for(int i=0; i < frame.rows ; i ++)
	{
		float* ptr_flow = flow.ptr<float>(i); // getting the float pointer to the row
		float* ptr_mag = flow_mag.ptr<float>(i);
		float* ptr_dir = flow_direction.ptr<float>(i);
		for(int j = 0 ; j < frame.cols ; j ++)
		{
			Point st(j,i); // referring to the previous image
			Point en((int)roundf(j+ptr_flow[2*j]), (int)roundf(i+ptr_flow[2*j + 1])); //referring to the current image
			
			float hypotenuse = sqrt( (st.y - en.y)*(st.y - en.y) + (st.x - en.x)*(st.x - en.x));
			float angle = atan2(ptr_flow[2*j + 1],ptr_flow[2*j]);
			
			if( hypotenuse < 1)
			{
				ptr_mag[j] = 0;
				ptr_dir[j] = 0;
				continue;
			}
			else
			{
				//storing the magnitude and direction of optical from each pixel
				ptr_mag[j] = hypotenuse;
				ptr_dir[j] = angle * 180/M_PI;
			}
			
				
			//drawing the optical flow line 
			line(frame_display,st,en,Scalar(0,0,255));
			Point arr = en - Point(5 * cos(angle + M_PI/4), 5 * sin(angle + M_PI/4));
			line(frame_display,arr,en,Scalar(0,255,0));
			arr = en - Point(5 * cos(angle - M_PI/4), 5 * sin(angle - M_PI/4));
			line(frame_display,arr,en,Scalar(0,255,0));
		}
	}
	fl_mag = flow_mag.clone();
	fl_dir = flow_direction.clone();
}

//computing optical flow  using Lucas Kanade
vector<Point2f> OpticalFlow::ComputeAndDrawOpticalFlowLK(Mat& frame_display, Mat mask, Mat mask_prev, Mat& fl_mag, Mat& fl_dir, vector<Point2f> prevPts, vector<Point2f>& currentPts, bool flagDraw)
{
	Mat flow_direction_temp = Mat::zeros(mask.rows,mask.cols,CV_32FC1);
	Mat flow_mag_temp = Mat::zeros(mask.rows,mask.cols,CV_32FC1);
	vector<Point2f> velocity;
	//extract the suitable difference points using the mask
	if(prevPts.empty())
		prevPts = ExtractCoordsFromImage(frame_prev,mask_prev);
	if(currentPts.empty())
		currentPts = ExtractCoordsFromImage(frame,mask);
		
	// if points are empty even after foreground extraction, then return
	if(prevPts.empty())
		return velocity;
	if(currentPts.empty())
		return velocity;

	// Find the region of interest in the frame
	Rect b_rect = boundingRect(Mat(currentPts));

	//finding the centroid
	Moments shape_moments = moments(mask);
	Point centroid = Point((int)(shape_moments.m10/shape_moments.m00),(int)(shape_moments.m01/shape_moments.m00));
	int max_dist_width = max(abs(centroid.x-b_rect.x),abs(centroid.x-b_rect.x-b_rect.width));
	int max_dist_height = max(abs(centroid.y-b_rect.y),abs(centroid.y-b_rect.y-b_rect.height));

	//editing the region of interest
	max_dist_width = 45;
	max_dist_height = 80;

	b_rect.x = centroid.x - max_dist_width;
	b_rect.y = centroid.y - max_dist_height;
	b_rect.width = 2 * max_dist_width;
	b_rect.height = 2* max_dist_height;

	//cout << b_rect.width << " " <<b_rect.height <<endl;
	
	b_rect = b_rect - Point(b_rect.width/12,b_rect.height/12);
	b_rect = b_rect + Size(b_rect.width/6,b_rect.height/6);

	if(b_rect.x < 0)
		b_rect.x = 0;
	if(b_rect.x+b_rect.width > mask.cols)
		b_rect.x = mask.cols - b_rect.width;
	
	int size_diff = prevPts.size() - currentPts.size();
	if(size_diff > 0) // previous frame points are more
	{
		prevPts.clear();
		//computing optical flow from current frame to previous frame
		calcOpticalFlowPyrLK(frame,frame_prev,currentPts,prevPts,status_image,err_image,Win_Size_LK,levels,criteria,0.5);
	}
	else if(size_diff < 0) //current frame points are more
	{
		currentPts.clear();
		//computing optical flow from points on the previous frame to the current frame
		calcOpticalFlowPyrLK(frame_prev,frame,prevPts,currentPts,status_image,err_image,Win_Size_LK,levels,criteria,0.5);
	}
	else if(size_diff == 0) //consecutive have same number of points
		calcOpticalFlowPyrLK(frame_prev,frame,prevPts,currentPts,status_image,err_image,Win_Size_LK,levels,criteria,0.5,OPTFLOW_USE_INITIAL_FLOW);
	
	// Drawing the flow vectors

	for(int k = 0 ; k < currentPts.size(); k ++)
	{
		//circle(frame_display,prevPts[k],1,Scalar(255,0,0),-1,8,0);
		Point2f st = prevPts[k]; // initial position
		Point2f en = currentPts[k]; // final position

		float hypotenuse = sqrt( (en.y - st.y)*(en.y - st.y) + (en.x - st.x)*(en.x - st.x));
		float angle = atan2((en.y - st.y),(en.x - st.x));

		Point2f velxy(hypotenuse*cos(angle),hypotenuse*sin(angle));
		velocity.push_back(velxy);

		int j = (int)(st.x);
		int i = (int)(st.y);
		if(flagDraw)
		{
			if( hypotenuse < 3*winsize)
			{
				flow_mag_temp.at<float>(i,j) = hypotenuse;
				flow_direction_temp.at<float>(i,j) = angle * 180/M_PI;
				Point st_int = Point(j,i);
				j = (int)(en.x);
				i = (int)(en.y);
				Point en_int = Point(j,i);


				//drawing the optical flow line
				line(frame_display,st_int,en_int,Scalar(0,0,255));
	//			if(!reverse)
	//			{
					Point arr = en_int + Point(3 * cos(angle + M_PI/4), 3 * sin(angle + M_PI/4));
					line(frame_display,arr,en_int,Scalar(0,255,0),2);
					arr = en_int + Point(3 * cos(angle - M_PI/4), 3 * sin(angle - M_PI/4));
					line(frame_display,arr,en_int,Scalar(0,255,0),2);
	//			}
	//			else
	//			{
	//				Point arr = en_int - Point(3 * cos(angle + M_PI/4), 3 * sin(angle + M_PI/4));
	//				line(frame_display,arr,en_int,Scalar(0,255,0));
	//				arr = en_int - Point(3 * cos(angle - M_PI/4), 3 * sin(angle - M_PI/4));
	//				line(frame_display,arr,en_int,Scalar(0,255,0));
	//
	//			}
			}
		}
	}
	
	//setting the flow direction and magnitude
	flow_mag = flow_mag_temp.clone();
	flow_direction = flow_direction_temp.clone();
	//fl_mag = flow_mag(b_rect);
	//fl_dir = flow_direction(b_rect);
	fl_mag = flow_mag;
	fl_dir = flow_direction;

	return velocity;
}

Point2f OpticalFlow::ComputeOpticalFlowLK(Mat& frame_display, Mat mask, Mat mask_prev, Mat& fl_mag, Mat& fl_dir, vector<Point2f> prevPts, vector<Point2f>& currentPts, bool flagDraw)
{
	Mat flow_direction_temp = Mat::zeros(mask.rows,mask.cols,CV_32FC1);
	Mat flow_mag_temp = Mat::zeros(mask.rows,mask.cols,CV_32FC1);
	vector<Point2f> velocity;
	Point2f average_vel;
	vector<float> velX;
	vector<float> velY;

	// extract points from the masked part from previous frame
	vector<Point2f> ptsInterest = ExtractCoordsFromImage(frame_prev,mask_prev);

	// Compute the optical flow
	vector<Point2f> ptsTracked;
	calcOpticalFlowPyrLK(frame_prev,frame,ptsInterest,ptsTracked,status_image,err_image,Win_Size_LK,levels,criteria,0.5);

	for(int k = 0 ; k < ptsInterest.size(); k ++)
	{
		//circle(frame_display,prevPts[k],1,Scalar(255,0,0),-1,8,0);
		Point2f st = ptsInterest[k];
		Point2f en = ptsTracked[k];

		float hypotenuse = sqrt( (en.y - st.y)*(en.y - st.y) + (en.x - st.x)*(en.x - st.x));
		float angle = atan2((en.y - st.y),(en.x - st.x));

		Point2f velxy(hypotenuse*cos(angle),hypotenuse*sin(angle));
		velocity.push_back(velxy);
		velX.push_back(hypotenuse*cos(angle));
		velY.push_back(hypotenuse*sin(angle));

		int j = (int)(st.x);
		int i = (int)(st.y);
		if(flagDraw)
		{
			if( hypotenuse < 3*winsize)
			{
				flow_mag_temp.at<float>(i,j) = hypotenuse;
				flow_direction_temp.at<float>(i,j) = angle * 180/M_PI;
				Point st_int = Point(j,i);
				j = (int)(en.x);
				i = (int)(en.y);
				Point en_int = Point(j,i);

				//drawing the optical flow line
				line(frame_display,st_int,en_int,Scalar(0,0,255));
				Point arr = en_int + Point(3 * cos(angle + M_PI/4), 3 * sin(angle + M_PI/4));
				line(frame_display,arr,en_int,Scalar(0,255,0),2);
				arr = en_int + Point(3 * cos(angle - M_PI/4), 3 * sin(angle - M_PI/4));
				line(frame_display,arr,en_int,Scalar(0,255,0),2);

			}
		}
	}

	//setting the flow direction and magnitude
	flow_mag = flow_mag_temp.clone();
	flow_direction = flow_direction_temp.clone();
	//fl_mag = flow_mag(b_rect);
	//fl_dir = flow_direction(b_rect);
	fl_mag = flow_mag;
	fl_dir = flow_direction;

	// Computing median optical flow velocity
	std::sort(velX.begin(),velX.end());
	std::sort(velY.begin(),velY.end());

	average_vel.x = velX[velX.size()/2];
	average_vel.y = velY[velY.size()/2];

	// Computing the average optical flow at specified points in previous Pts
	if(prevPts.empty())
		return average_vel;
	else
	{
		// For each point, extract velocity and neighboring point
		for(int p = 0 ; p < prevPts.size(); p++)
		{
			velX.clear();
			velY.clear();
			velocity.clear();
			ptsInterest.clear();
			ptsTracked.clear();

			// Get each point in previous frame
			Point2f ptInt = prevPts[p];

			// create a region of interest around that point
			Rect roi_region(ptInt.x - ROI_WIDTH/2,ptInt.y - ROI_HEIGHT/2,ROI_WIDTH,ROI_HEIGHT);

			// TODO:Extract the coordinates: wont give the correct coordinates
			//ptsInterest = ExtractCoordsFromImage(frame_prev,mask_prev,roi_region);
			ptsInterest = ExtractCoordsFromImage(frame_prev,Mat(),roi_region);

			// compute optical flow for these points
			calcOpticalFlowPyrLK(frame_prev,frame,ptsInterest,ptsTracked,status_image,err_image,Win_Size_LK,levels,criteria,0.5);

			for(int k = 0 ; k < ptsInterest.size(); k ++)
			{
				//circle(frame_display,prevPts[k],1,Scalar(255,0,0),-1,8,0);
				Point2f st = ptsInterest[k];
				Point2f en = ptsTracked[k];

				float hypotenuse = sqrt( (en.y - st.y)*(en.y - st.y) + (en.x - st.x)*(en.x - st.x));
				float angle = atan2((en.y - st.y),(en.x - st.x));

				Point2f velxy(hypotenuse*cos(angle),hypotenuse*sin(angle));
				velocity.push_back(velxy);
				velX.push_back(hypotenuse*cos(angle));
				velY.push_back(hypotenuse*sin(angle));
			}

			// Find the median velocity
			// Computing median optical flow velocity
			std::sort(velX.begin(),velX.end());
			std::sort(velY.begin(),velY.end());

			Point2f local_vel;

			// median velocity
			local_vel.x = velX[velX.size()/2];
			local_vel.y = velY[velY.size()/2];

			//max velocity
			//local_vel.x = velX[velX.size()];
			//local_vel.y = velY[velY.size()];

			// getting new tracked point
			Point2f new_tracked_point;
			new_tracked_point.x = ptInt.x + local_vel.x;
			new_tracked_point.y = ptInt.y + local_vel.y;

			currentPts.push_back(new_tracked_point);

		}

		// Extract the neighbor coordinates of each point( make a mask and extract those points)

		// Search for those suitable coordinates  in the interest points


		return average_vel;
	}

}

//extracting the coordinates of non-zero pixels
vector<Point2f> OpticalFlow::ExtractCoordsFromImage(Mat& Image, Mat mask, Rect roi_region)
{
	vector<Point2f> coords;
	if(mask.empty())
	{
		mask = Mat::ones(Image.rows,Image.cols,CV_8UC1);
		mask = mask * 255;
	}
	// region of interest is empty
	if(roi_region.area() == 0)
	{
		roi_region.x = 0;
		roi_region.y = 0;
		roi_region.width = Image.cols;
		roi_region.height = Image.rows;
	}
	for(int i = roi_region.y ; i < roi_region.y + roi_region.height; i++)
	{
		unsigned char* ptr_image = Image.ptr<unsigned char>(i);
		unsigned char* ptr_mask = mask.ptr<unsigned char>(i);
		for(int j = roi_region.x ; j < roi_region.x + roi_region.width; j++)
		{
			if( ptr_image[j] > 0 && ptr_mask[j]==255 )
				coords.push_back(Point2f(j,i));
		}
	
	}	
	return coords;
}

//function definition
void OpticalFlow::WriteFloatImage(const string& filename, Mat& fl_image)
{
	ofstream outFile;
	outFile.open(filename.c_str());
	//cout << outFile.is_open() << endl;
	if(!outFile.is_open())
	{
		cerr << "File Open Error : " <<endl;
		exit(1);	
	}
	
	//writing the image file in the format below
	// FLOAT IMAGE TYPE
	// ROWS COLS 
	// pixel 1 pixel 2 .......
	
	outFile << "FLOAT IMAGE TYPE" << endl;
	outFile << fl_image.rows << " " << fl_image.cols << endl;
	
	for(int row = 0 ; row < fl_image.rows; row ++) 
	{
		float* ptr_image = fl_image.ptr<float>(row);
		for(int col = 0 ; col < fl_image.cols ; col++)
		{
			outFile << ptr_image[col] << " ";
		}
	}
	outFile << "\n";
	
	outFile.close();	
}

// Function for drawing the optical flow - returns the optical flow velocity
vector<Point2f> OpticalFlow::DrawOpticalFlowOnImage(Mat& image,vector<Point2f> prevPts, vector<Point2f> currentPts, Scalar* colors)
{
	vector<Point2f> velocity;
	for(int k = 0 ; k < currentPts.size(); k ++)
	{
		//circle(frame_display,prevPts[k],1,Scalar(255,0,0),-1,8,0);
		Point2f st = prevPts[k];
		Point2f en = currentPts[k];

		float hypotenuse = sqrt( (st.y - en.y)*(st.y - en.y) + (st.x - en.x)*(st.x - en.x));
		float angle = atan2((st.y - en.y),(st.x - en.x));
		Point2f velxy(hypotenuse*cos(angle),hypotenuse*sin(angle));
		velocity.push_back(velxy);

		int j = (int)(st.x);
		int i = (int)(st.y);

		if( hypotenuse < 3*winsize)
		{
			Point st_int = Point(j,i);
			j = (int)(en.x);
			i = (int)(en.y);
			Point en_int = Point(j,i);

			//drawing the optical flow line
			line(image,st_int,en_int,Scalar(0,255,0));
			Point arr = en_int + Point(2 * cos(angle + M_PI/4), 2 * sin(angle + M_PI/4));
			line(image,arr,en_int,colors[k],1);
			arr = en_int + Point(2 * cos(angle - M_PI/4), 2 * sin(angle - M_PI/4));
			line(image,arr,en_int,colors[k],1);
		}
	}

	return velocity;
}
