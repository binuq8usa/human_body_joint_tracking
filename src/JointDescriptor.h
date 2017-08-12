/*
 * JointDescriptor.h
 *
 *  Created on: Jul 17, 2013
 *      Author: bnair002
 */

#ifndef _JOINTDESCRIPTOR_H
#define _JOINTDESCRIPTOR_H

#include <vector>
#include <iostream>
#include <cmath>
#include <algorithm>
#include <opencv2/opencv.hpp>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <algorithm>
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/nonfree/nonfree.hpp>

#include "hogdescriptor.h"
#include "lbpdescriptor.h"
#include "cvplot.h"

using namespace std;
using namespace cv;

#define MATCH_THRESHOLD 0.25 // for window_size of 17, use 0.25
#define DEFAULT_WINSIZE 17

#define MATCH_THRESHOLD_SHOULDER 0.25
#define MATCH_THRESHOLD_ELBOW 0.25
#define MATCH_THRESHOLD_WRIST 0.25
#define MATCH_THRESHOLD_HIP 0.25
#define MATCH_THRESHOLD_KNEE 0.25
#define MATCH_THRESHOLD_ANKLE 0.25

#define ACCELERATION 0.1
//#define MATCH_THRESHOLD_SHOULDER 1.25
//#define MATCH_THRESHOLD_ELBOW 1.25
//#define MATCH_THRESHOLD_WRIST 1.25
//#define MATCH_THRESHOLD_HIP 1.25
//#define MATCH_THRESHOLD_KNEE 1.25
//#define MATCH_THRESHOLD_ANKLE 1.25

#define DEFAULT_WINSIZE_SHOULDER 17
#define DEFAULT_WINSIZE_ELBOW 17
#define DEFAULT_WINSIZE_WRIST 17
#define DEFAULT_WINSIZE_HIP 17
#define DEFAULT_WINSIZE_KNEE 17
#define DEFAULT_WINSIZE_ANKLE 17

#define DESCRIPTOR_NAME "SIFT"
//#define DESCRIPTOR_NAME "SURF"
//#define DESCRIPTOR_NAME "BRIEF"
//#define DESCRIPTOR_NAME "BRISK"
//#define DESCRIPTOR_NAME "ORB"
//#define DESCRIPTOR_NAME "FREAK"

#define DESCRIPTOR_MATCH_NAME "BruteForce"
//#define DESCRIPTOR_MATH_NAME "BruteForce-L1"
//#define DESCRIPTOR_MATH_NAME "BruteForce-Hamming"
//#define DESCRIPTOR_MATH_NAME "BruteForce-Hamming(2)"
//#define DESCRIPTOR_MATH_NAME "BruteForce-FlannBased"

#define ALPHA_P 2 // specifying the bounds for the ratio of the elliptical region size : Default is 5

class JointDescriptor
{
public:
		Point2f location; //retains the previous location
	    Point2f velocity; //retains the velocity between frame k and k+1
	    Point2f global_velocity;
	    Point2f local_velocity;
		int winSize;
		double matchDistance;
		float matchThreshold;
		float timeStep;
		float acceleration;
		bool flagFirst;
		visionlab::HOGDescriptor* desc_hog;
		LBPDescriptor* desc_lbp;

		string des_name;
		string des_match_name;

		// declaring a generic descriptor
		Ptr<DescriptorExtractor> generic_extractor;

		// declaring the descriptor matcher
		Ptr<DescriptorMatcher> generic_matcher;

		Scalar joint_color;
		RotatedRect search_region;
		RotatedRect search_region2; // for second matching

		//parameters for tracker
		KalmanFilter* tracker;
		int dynamParams;
		int measureParams;
		int controlParams;
		int type;
		bool firstTime; // flag to see if the tracker is initialized for the first time
//
		Mat state; // xhat_k - contains the previous location and the velocity vector
		Mat measurement; // z - contains the current location
		Mat img; // represents the region surrounding the joint
		Mat kGain; //Kalman Gain
		Mat errorCovPre;
		Mat errorCovPost;

		vector<float> feature_vector;

		float delta_x;
		float delta_y;
		float delta_z;

public:
		bool displayOn;
		//Constructors/Destructors
		JointDescriptor();
		JointDescriptor(Point2f loc,Mat img,bool display = true, string des_name = "SIFT", string des_match_name = "BruteForce");
		JointDescriptor(Point2f loc,Mat img, int win_size,bool display = true,string des_name = "SIFT", string des_match_name = "BruteForce");
		JointDescriptor(Point2f loc,Mat img, int win_size, float match_thresh,bool display = true,string des_name = "SIFT", string des_match_name = "BruteForce");
		JointDescriptor(Point2f loc,Mat img, int win_size, float match_thresh, Point2f vel, bool display = true,string des_name = "SIFT", string des_match_name = "BruteForce");
		~JointDescriptor();

		// Method to call the Kalman Filter Tracking algorithm
		Point2f track(Point2f loc,Mat img, bool flagMatch = true, bool flagPassThrough = false,Point2f detect_loc = Point2f(),bool flag_useDetectLoc = false, bool flagInitialize = false, bool firstStageCheck = true);

		// Method to call the region based tracker from frame to frame
		Point2f track_HOGBased(Point2f prevLoc, Mat prev_img, Mat current_img, Point2f detect_loc, Mat mask = Mat());

		// Method to call the region based tracker from frame to frame
		Point2f track_LBPBased(Point2f prevLoc, Mat prev_img, Mat current_img, Point2f detect_loc, Mat mask = Mat());

		// Method to call the region based tracker from frame to frame
		Point2f track_regionBased(Point2f prevLoc, Mat prev_img, Mat current_img, Point2f detect_loc, Mat mask = Mat());

		// Method to call the region based tracker from frame to frame with Kalman filtering
		Point2f track_regionBasedKalman(Point2f prevLoc, Mat prev_img, Mat current_img, Point2f detect_loc, Mat mask = Mat());

//		// Method to call the region based tracker from frame to frame with Kalman filtering
		Point2f track_HOGBasedKalman(Point2f prevLoc, Mat prev_img, Mat current_img, Point2f detect_loc, Mat mask = Mat());
//
//		// Method to call the region based tracker from frame to frame with Kalman filtering
		Point2f track_LBPBasedKalman(Point2f prevLoc, Mat prev_img, Mat current_img, Point2f detect_loc, Mat mask = Mat());

		// Method to call the proposed scheme tracker
		Point2f track_proposed(Point2f prevLoc, Mat prev_img, Mat current_img, Point2f detect_loc, Mat mask = Mat());

		// Method to call the proposed scheme tracker
		Point2f track_proposed_withHOG(Point2f prevLoc, Mat prev_img, Mat current_img, Point2f detect_loc, Mat mask = Mat());

		// Method to call the proposed scheme tracker
		Point2f track_proposed_withLBP(Point2f prevLoc, Mat prev_img, Mat current_img, Point2f detect_loc, Mat mask = Mat());

		// Method to call the proposed scheme tracker
		Point2f track_proposed_prob(Point2f prevLoc, Mat prev_img, Mat current_img, Point2f detect_loc, Mat mask = Mat());

		// Method to call the proposed scheme tracker
		Point2f track_proposed_withHOG_prob(Point2f prevLoc, Mat prev_img, Mat current_img, Point2f detect_loc, Mat mask = Mat());

		// Method to call the proposed scheme tracker
		Point2f track_proposed_withLBP_prob(Point2f prevLoc, Mat prev_img, Mat current_img, Point2f detect_loc, Mat mask = Mat());

public:
		//methods
		Point2f getLocation() const;
		Point2f getVelocity() const;
		RotatedRect getSearchRegion() const;

		void setLocation(Point2f location);
		void setVelocity(Point2f velocity);
		void setGlobalVelocity(Point2f gb_velocity);

		void setState(Point2f location,Point2f velocity);
		void setMeasurement(Point2f location);

		void setJointColor(Scalar color);

		// Match the previous and current image description - returns true if matched else false
		bool matchImageDescriptors(vector<float> feature1,vector<float> feature2, float& measure, bool flag_LBP = true);

		// Initialize Tracker - set num_of_parameters, initialize A,B,H,Q,R,x_k+,P_k+
		void initializeTracker(int dynamParams,int measureParams,int controlParams,int type);

		// Predict the joint position
		void predictJointPosition();

		// Correct the joint position
		void correctJointPosition();

		//function to plot descriptor
		void plotJointDescriptor(vector<float> fl);

		// Compute joint signature
		vector<float> computeSignature(Mat imag, bool flag_HOG = true);

		// Compute color descriptor
		vector<float> computeColorHistogram(Mat imag, Mat mask = Mat());

		// Compute search region
		void computeSearchRegion();

		// Compute search region
		RotatedRect computeSearchRegion2(Point2f updateLoc, Point2f detect_loc);

		// Extract Coordinates from image
		vector<Point2f> ExtractCoordsFromImage(Mat& image, Mat mask = Mat(), Rect roi_region = Rect());

		// TODO: WRITE METHODS TO LOAD/SAVE KALMAN FILTER AS AN XML FILE
		// Serialization
		void write(FileStorage& fs) const;
		void read(const FileNode& node);

};

#endif

