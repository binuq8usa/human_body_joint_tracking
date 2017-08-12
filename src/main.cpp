#include <iostream>
#include <fstream>
#include <cstdio>
#include <opencv/cv.h>
#include <string>
#include <sys/stat.h>
#include <sys/types.h>
#include <errno.h>
#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/nonfree/features2d.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include "opticalflowclass.h"
#include "JointDescriptor.h"
#include "bgmodel.h"
#include <string>
#include <fstream>

using namespace std;
using namespace cv;

// TODO: NEED TO CREATE A LOG FILE FOR EACH VIDEO CONTAINING CERTAIN OUTPUTS AND ERRORS FOR ANALYSIS

#define PATH "/home/bnair002/DropBox/Code/CSUProject"
#define PATH_FRAMES "/home/bnair002/DropBox/Code/CSUProject/Frames"
#define SOURCE_WIN "Optical Flow in Video"
#define SOURCE_WIN_GRAY "Video in Grayscale"
#define SOURCE_WIN_LBP "Video in LBP"
#define SOURCE_WIN_FIELD1 "Deinterlaced-Field1"
#define SOURCE_WIN_FIELD2 "Deinterlaced-Field2"
#define BG_WIN "Background Image"
#define FG_WIN "Foreground Video"
#define POINTS_WIN "Interest Points in Each Frame"
#define COMBINED_WIN "Combined Video"
#define VIDEOFILE "Test_Video.avi"

// Read and write functions for serialization in FileStorage
void write(FileStorage& fs,const std::string&, const JointDescriptor& x)
{
	x.write(fs);
}

void read(const FileNode& node, JointDescriptor& x, const JointDescriptor& default_value = JointDescriptor())
{
	if(node.empty())
		x = default_value;
	else
		x.read(node);
}

vector<Mat> deinterlaceImage(Mat image);

int main(int argc,char** argv)
{

	// Initiating modules non free
	initModule_nonfree();

	//declaring the video capture class
	VideoCapture capture;
	VideoWriter writer,writer2;
	VideoWriter writer_fg;
	VideoWriter writer_gray;
	FileStorage fs;

	// Global varaiables
	string des_name;
	string des_match_name;

	fs.open("Training.xml",FileStorage::WRITE);
	if(!fs.isOpened())
	{
		cerr << "Output XML file cannot be opened" <<endl;
		exit(1);
	}
	
	ifstream inputFile;
	ofstream outputFile;
	BGModel bg_model;
	OpticalFlow optFlow;
	Mat frame,frame_gray,bg_frame,bg_frame_gray,bg_frame_ycbcr,prev_frame,fg_mask,fg_mask_prev,frame_copy;
	Mat combined_frames;
	char video_file[200];
	int frame_count = 0;
	int num_of_points,trackFrame_begin;
	int num_of_framesTrack ;
	bool displayOn = true; // setting the display flag
	vector<Point2f> prevPts,currentPts;
	bool flag_ver1, flag_ver2, flag_ver3, flag_ver1_ver5;
	bool flagInitialize,flagPassThrough,flagMatch,flagRegionMatch2,flag_firstStageCheck;

	// flag which sets only the Kalman filter using manual points
	bool manual_kalman_tracker = false;

	// only either one of the flags is set
	bool flag_KLT_tracker = false;
	bool flag_HOG_tracker = false;
	bool flag_LBP_tracker = false;

	// for region based tracking
	bool flag_SIFT_tracker = false;
	bool flag_SURF_tracker = true;
	bool flag_BRIEF_tracker = false;
	bool flag_BRISK_tracker = false;
	bool flag_ORB_tracker = false;
	bool flag_FREAK_tracker = false;

	// for region based tracking and use of Kalman filter to keep tracks
	bool flag_HOG_tracker_kalman = false;
	bool flag_LBP_tracker_kalman = false;

	bool flag_SIFT_tracker_kalman = false;
	bool flag_SURF_tracker_kalman = false;
	bool flag_BRIEF_tracker_kalman = false;
	bool flag_BRISK_tracker_kalman = false;
	bool flag_ORB_tracker_kalman = false;
	bool flag_FREAK_tracker_kalman = false;

	// for proposed framework to keep tracks - Citation [1]
	bool flag_HOG_proposed = false;
	bool flag_LBP_proposed = false;

	bool flag_SIFT_proposed = false;
	bool flag_SURF_proposed = false;
	bool flag_BRIEF_proposed = false;
	bool flag_BRISK_proposed = false;
	bool flag_ORB_proposed = false;
	bool flag_FREAK_proposed = false;

	// for probabilistic proposed framework to keep tracks - Citation [2]
	bool flag_HOG_proposed_prob = false;
	bool flag_LBP_proposed_prob = false;

	bool flag_SIFT_proposed_prob = false;
	bool flag_SURF_proposed_prob = true;
	bool flag_BRIEF_proposed_prob = false;
	bool flag_BRISK_proposed_prob = false;
	bool flag_ORB_proposed_prob = false;
	bool flag_FREAK_proposed_prob = false;

	if(manual_kalman_tracker)
	{
		flag_ver1 = true;
		flag_ver2 = false;
		flag_ver1_ver5 = false;
		flag_ver3 = false;
		des_name = "SIFT";
		des_match_name = "BruteForce";
	}

	if(flag_KLT_tracker)
	{
		flag_ver1 = false;
		flag_ver2 = true;
		flag_ver1_ver5 = false;
		flag_ver3 = false;
		des_name = "SIFT";
		des_match_name = "BruteForce";
	}

	if(flag_HOG_tracker)
	{
		flag_ver1 = false;
		flag_ver2 = true;
		flag_ver1_ver5 = false;
		flag_ver3 = false;
		des_name = "SIFT";
		des_match_name = "BruteForce";
	}

	if(flag_LBP_tracker)
	{
		flag_ver1 = false;
		flag_ver2 = true;
		flag_ver1_ver5 = false;
		flag_ver3 = false;
		des_name = "SIFT"; // setting default values
		des_match_name = "BruteForce";
	}

	if(flag_SIFT_tracker)
	{
		flag_ver1 = false;
		flag_ver2 = true;
		flag_ver1_ver5 = false;
		flag_ver3 = false;
		des_name = "SIFT";
		des_match_name = "BruteForce";
	}

	if(flag_SURF_tracker)
	{
		flag_ver1 = false;
		flag_ver2 = true;
		flag_ver1_ver5 = false;
		flag_ver3 = false;
		des_name = "SURF";
		des_match_name = "BruteForce";
	}

	if(flag_BRIEF_tracker)
	{
		flag_ver1 = false;
		flag_ver2 = true;
		flag_ver1_ver5 = false;
		flag_ver3 = false;
		des_name = "BRIEF";
		des_match_name = "BruteForce-Hamming";
	}

	if(flag_BRISK_tracker)
	{
		flag_ver1 = false;
		flag_ver2 = true;
		flag_ver1_ver5 = false;
		flag_ver3 = false;
		des_name = "BRISK";
		des_match_name = "BruteForce-Hamming";
	}

	if(flag_ORB_tracker)
	{
		flag_ver1 = false;
		flag_ver2 = true;
		flag_ver1_ver5 = false;
		flag_ver3 = false;
		des_name = "ORB";
		des_match_name = "BruteForce-Hamming";
	}

//	flag_ver1 = true; // flag which tells us to use only manual points as measurement
//	flag_ver2 = false; // flag which takes only optical flow points as measurement
//	flag_ver3 = false; // flag which takes optical flow along with region matching to initialize
//	flag_ver1_ver5 = true; //flag which takes manual points as measurement but considers the other scenarios

	if(flag_ver1)
	{
		flagPassThrough = true; // no effect of bottom three flags if set
		flagMatch = false;
		flagRegionMatch2 = false;
		flagInitialize = false;
		flag_firstStageCheck = true;
		if(flag_ver1_ver5)
		{
			flagPassThrough = false;
			flagRegionMatch2 = true;
			flag_firstStageCheck = false;
		}
	}
	else if(flag_ver2)
	{
		flagPassThrough = true;
		flagMatch = false;
		flagRegionMatch2 = false;
		flagInitialize = false;
		flag_firstStageCheck = true;
	}
	else if(flag_ver3)
	{
		flagPassThrough = false;
		flagMatch = false; // not used currently.. always false
		flagRegionMatch2 = false; // does not use the region matching scheme 2 using manual points
		flagInitialize = true; // if region matching scheme 1 is used, then Kalman filter is initialized
		flag_firstStageCheck = true;
	}
	else
	{
		flagPassThrough = false;
		flagMatch = false;
		flagRegionMatch2 = true;
		flagInitialize = false; // flagRegionMatch2 sets it up
		flag_firstStageCheck = true;
	}
	
	if(displayOn)
	{
		namedWindow(SOURCE_WIN,CV_WINDOW_NORMAL);
		namedWindow(BG_WIN,CV_WINDOW_NORMAL);
		namedWindow(POINTS_WIN,CV_WINDOW_NORMAL);
	}

	
	//Reading the video once to get the median background image

/*	sprintf(video_file,"%s/%s",PATH,argv[1]);
	cout << video_file <<endl;
	capture.open(video_file);
*/
	//opening the inputs - video and text file
	capture.open(argv[1]);
	if(!capture.isOpened())
	{
		cerr << "Cannot read video file" <<endl;
		exit(1);
	}

	inputFile.open(argv[2]);
	if(!inputFile.is_open())
	{
		cerr << "Cannot read input file" <<endl;
		exit(1);
	}
	// file for storing the tracks
	outputFile.open(argv[3]);
	if(!outputFile.is_open())
	{
		cerr << "Cannot read output file" <<endl;
		exit(1);
	}

	inputFile >> num_of_framesTrack;
	inputFile >> num_of_points;
	inputFile >> trackFrame_begin;


	int num_of_frames_per_cycle = num_of_framesTrack-1;

	if(displayOn)
	{
		cout << "Number of points to track = " << num_of_points << endl;
		cout << "Number of frames to track = " << num_of_framesTrack << endl;
		cout << "Frame to start tracking = " << trackFrame_begin <<endl;
	}

	// Creating the Joint Descriptors
	JointDescriptor** joints = new JointDescriptor*[num_of_points];
	Scalar j_colors[num_of_points];
	int w_sizes[num_of_points];
	float m_threshold[num_of_points];
	if(num_of_points == 6)
	{
		j_colors[0] = Scalar(255,0,0); //shoulder
		j_colors[1] = Scalar(0,255,0); // elbow
		j_colors[2] = Scalar(0,0,255); // wrist
		j_colors[3] = Scalar(255,255,0); // hip
		j_colors[4] = Scalar(0,255,255); // knee
		j_colors[5] = Scalar(255,0,255); // ankle

		w_sizes[0] = DEFAULT_WINSIZE_SHOULDER;
		w_sizes[1] = DEFAULT_WINSIZE_ELBOW;
		w_sizes[2] = DEFAULT_WINSIZE_WRIST;
		w_sizes[3] = DEFAULT_WINSIZE_HIP;
		w_sizes[4] = DEFAULT_WINSIZE_KNEE;
		w_sizes[5] = DEFAULT_WINSIZE_ANKLE;

		m_threshold[0] = MATCH_THRESHOLD_SHOULDER;
		m_threshold[1] = MATCH_THRESHOLD_ELBOW;
		m_threshold[2] = MATCH_THRESHOLD_WRIST;
		m_threshold[3] = MATCH_THRESHOLD_HIP;
		m_threshold[4] = MATCH_THRESHOLD_KNEE;
		m_threshold[5] = MATCH_THRESHOLD_ANKLE;

	}
	else
	{
		for(int i = 0; i < num_of_points ; i++)
		{
			j_colors[i] = Scalar(0,255,0);
			w_sizes[i] = DEFAULT_WINSIZE_SHOULDER;
			m_threshold[i] = MATCH_THRESHOLD_SHOULDER;
		}
	}

	char optflow_file[400];
	char fg_file[400];
	char video_gray[400];
	char image_file[400];
	sprintf(optflow_file,"Test_Video_flow.avi");
	sprintf(fg_file,"Test_Video_fg.avi");
	sprintf(video_gray,"Test_Video_gray.avi");

	writer.open(argv[4],CV_FOURCC('M','J','P','G'),15,Size(capture.get(CV_CAP_PROP_FRAME_WIDTH),capture.get(CV_CAP_PROP_FRAME_HEIGHT)),1);
	writer2.open(argv[6],CV_FOURCC('M','J','P','G'),15,Size(capture.get(CV_CAP_PROP_FRAME_WIDTH),capture.get(CV_CAP_PROP_FRAME_HEIGHT)),1);

	//If file has been opened
	if(capture.isOpened())
	{
		// reading the video frames
		for(;;)
		{
			capture >> frame;
			frame_count++;
			if(frame.empty())
			{
				if(displayOn)
					cout << "End of Video File" <<endl;
				break;
			}

			//Process the frame
			//vector<Mat> fields = deinterlaceImage(frame);
			// TODO:HACK
			vector<Mat> fields;
			fields.push_back(frame);
			Mat frame_deinterlaced;
			//copy the odd field to the frame and use that
			fields[0].copyTo(frame_deinterlaced);
			bg_model.LoadAccumulator(frame_deinterlaced);
		}
		//computing the background
		bg_model.ComputeMeanBG();
		bg_frame = bg_model.GetBGFrame();
		if(displayOn)
		{
			imshow(BG_WIN,bg_frame);
			waitKey(0);
		}
	}	
	else cout << "Error Opening the video file" <<endl;
	
	
	//releasing the camera
	capture.release();
	
	cvtColor(bg_frame,bg_frame_ycbcr,CV_BGR2YCrCb);
	vector<Mat> temp_bg;
	split(bg_frame_ycbcr,temp_bg);
	temp_bg[0].copyTo(bg_frame_gray);
	frame_count = 0 ;

	// Computing the foreground segmentation and optical flow
	capture.open(argv[1]);
	
	if(capture.isOpened())
	{
		// reading the video frames
		for(;;)
		{
			Mat flow_mag,flow_dir,frame_ycbcr;
			capture >> frame;
			frame_count++;

			if(frame.empty())
			{
				cout << "End of Video File" <<endl;
				break;
			}

			//vector<Mat> fields = deinterlaceImage(frame);
			// TODO:HACK
			vector<Mat> fields;
			fields.push_back(frame);
			Mat frame_deinterlaced;
			//copy the odd field to the frame and use that
			fields[0].copyTo(frame_deinterlaced);

			frame_deinterlaced.copyTo(frame_copy);
			
			//Computing optical flow
			if(frame_count == trackFrame_begin)
			{
				cvtColor(frame_deinterlaced,frame_ycbcr,CV_BGR2YCrCb);
				vector<Mat> temp;
				split(frame_ycbcr,temp);
				temp[0].copyTo(frame_gray);

				optFlow.Initialize(frame_deinterlaced);

				fg_mask = abs(bg_frame_gray - frame_gray); // thresholding still in RGB space
				threshold(fg_mask,fg_mask,15,255,CV_THRESH_BINARY);
				erode(fg_mask,fg_mask,Mat(),Point(-1,-1),1);
				dilate(fg_mask,fg_mask,Mat(),Point(-1,-1),1);
				//save the current mask 
				fg_mask.copyTo(fg_mask_prev);

				// locating the previous  Points
				for(int k = 1; k <= num_of_points ; k++)
				{
					float x_cor,y_cor;
					inputFile >> x_cor;
					inputFile >> y_cor;
					circle(frame_copy,Point2f(x_cor,y_cor),1,Scalar(199,28,125),-1);
					prevPts.push_back(Point2f(x_cor,y_cor));
				}

				if(displayOn)
				{
					imshow(SOURCE_WIN,frame_deinterlaced);
					imshow(POINTS_WIN,frame_copy);
					waitKey(0);
				}


			}
			else if(frame_count == trackFrame_begin + 1) // to initialize the joints
			{
				vector<Point2f> trackedPts;

				writer2 << frame_deinterlaced;

				cvtColor(frame_deinterlaced,frame_ycbcr,CV_BGR2YCrCb);
				vector<Mat> temp;
				split(frame_ycbcr,temp);
				temp[0].copyTo(frame_gray);

				optFlow.UpdateImages(frame_deinterlaced); // optical flow in LBP mode

				fg_mask = abs(bg_frame_gray - frame_gray);

				threshold(fg_mask,fg_mask,15,255,CV_THRESH_BINARY);
				erode(fg_mask,fg_mask,Mat(),Point(-1,-1),1);
				dilate(fg_mask,fg_mask,Mat(),Point(-1,-1),1);

				// compute optical flow and find median velocity ( Here, no previous points)
				Point2f velXY;

				if(flag_KLT_tracker) //  KLT Tracker
					velXY = optFlow.ComputeOpticalFlowLK(frame_deinterlaced,fg_mask,fg_mask_prev,flow_mag,flow_dir,prevPts,trackedPts,false);
				else
				{
					// compute the average velocity of motion for updating velocity of joints
					vector<Point2f> temp_vector;
					velXY = optFlow.ComputeOpticalFlowLK(frame_deinterlaced,fg_mask,fg_mask_prev,flow_mag,flow_dir,temp_vector,temp_vector,false);
				}

				//Get the first set of points and Initialize the joint Descriptors with the velocity computed from the first two frames
				for(int k = 1; k <= num_of_points ; k++)
				{

					// Initialize the Joint Descriptor with the initial velocity computed from the first 2 frames and the coordinates from the first frame
					joints[k-1] = new JointDescriptor(prevPts[k-1],optFlow.frame_prev_color,w_sizes[k-1],m_threshold[k-1],velXY,displayOn,des_name,des_match_name);

					// set the opt flow global velocity of each joint for the prediction for instant k+1 after correction stage at k
					joints[k-1]->setGlobalVelocity(velXY);

					// Extracting the current frame coordinates of joint k
					float x_cor,y_cor;
					inputFile >> x_cor;
					inputFile >> y_cor;
					circle(frame_copy,Point2f(x_cor,y_cor),1,Scalar(199,28,125),-1);

					// draw the search region
					RotatedRect reg = joints[k-1]->getSearchRegion();
					ellipse(frame_copy,reg,j_colors[k-1]);

					// track the joints
					Point2f cors;
					if(flag_ver1)
						cors = Point2f(x_cor,y_cor);

					Point2f correctedLoc;
					if(flag_KLT_tracker) // KLT Tracker
					{
						cors = trackedPts[k-1];
						correctedLoc = cors;
					}
					else if(flag_SIFT_tracker || flag_SURF_tracker || flag_BRIEF_tracker || flag_BRISK_tracker || flag_ORB_tracker || flag_FREAK_tracker)
						// if using Kalman filter
						if(flag_SIFT_tracker_kalman || flag_SURF_tracker_kalman || flag_BRIEF_tracker_kalman || flag_BRISK_tracker_kalman || flag_ORB_tracker_kalman || flag_FREAK_tracker_kalman)
							correctedLoc = joints[k-1]->track_regionBasedKalman(prevPts[k-1],optFlow.frame_prev_color,optFlow.frame_color,Point2f(x_cor,y_cor));
						else if(flag_SIFT_proposed || flag_SURF_proposed || flag_BRIEF_proposed || flag_BRISK_proposed || flag_ORB_proposed || flag_FREAK_proposed)
							correctedLoc = joints[k-1]->track_proposed(prevPts[k-1],optFlow.frame_prev_color,optFlow.frame_color,Point2f(x_cor,y_cor));
						else if(flag_SIFT_proposed_prob || flag_SURF_proposed_prob || flag_BRIEF_proposed_prob || flag_BRISK_proposed_prob || flag_ORB_proposed_prob || flag_FREAK_proposed_prob)
							correctedLoc = joints[k-1]->track_proposed_prob(prevPts[k-1],optFlow.frame_prev_color,optFlow.frame_color,Point2f(x_cor,y_cor));
						else
							correctedLoc = joints[k-1]->track_regionBased(prevPts[k-1],optFlow.frame_prev_color,optFlow.frame_color,Point2f(x_cor,y_cor));
					else if(flag_HOG_tracker)
					{
						if(flag_HOG_tracker_kalman)
							correctedLoc = joints[k-1]->track_HOGBasedKalman(prevPts[k-1],optFlow.frame_prev_color,optFlow.frame_color,Point2f(x_cor,y_cor));
						else if(flag_HOG_proposed)
							correctedLoc = joints[k-1]->track_proposed_withHOG(prevPts[k-1],optFlow.frame_prev_color,optFlow.frame_color,Point2f(x_cor,y_cor));
						else if(flag_HOG_proposed_prob)
							correctedLoc = joints[k-1]->track_proposed_withHOG_prob(prevPts[k-1],optFlow.frame_prev_color,optFlow.frame_color,Point2f(x_cor,y_cor));
						else
							correctedLoc = joints[k-1]->track_HOGBased(prevPts[k-1],optFlow.frame_prev_color,optFlow.frame_color,Point2f(x_cor,y_cor));
					}
					else if(flag_LBP_tracker)
					{
						if(flag_LBP_tracker_kalman)
							correctedLoc = joints[k-1]->track_LBPBasedKalman(prevPts[k-1],optFlow.frame_prev_color,optFlow.frame_color,Point2f(x_cor,y_cor));
						else if(flag_LBP_proposed)
							correctedLoc = joints[k-1]->track_proposed_withLBP(prevPts[k-1],optFlow.frame_prev_color,optFlow.frame_color,Point2f(x_cor,y_cor));
						else if(flag_LBP_proposed_prob)
							correctedLoc = joints[k-1]->track_proposed_withLBP_prob(prevPts[k-1],optFlow.frame_prev_color,optFlow.frame_color,Point2f(x_cor,y_cor));
						else
							correctedLoc = joints[k-1]->track_LBPBased(prevPts[k-1],optFlow.frame_prev_color,optFlow.frame_color,Point2f(x_cor,y_cor));
					}
					else
						correctedLoc = joints[k-1]->track(cors,frame_deinterlaced,flagMatch,true); // here, just pass through

					if(trackedPts.size() != num_of_points)
						trackedPts.push_back(correctedLoc);
					else
					{
						trackedPts[k-1].x = correctedLoc.x;
						trackedPts[k-1].y = correctedLoc.y;
					}

					// output the corrected location
					outputFile << correctedLoc.x << "\t";
					outputFile << correctedLoc.y << "\t";

				}
				outputFile << "\n";
				cout << "\n" << endl;
				Scalar temp_j_colors[num_of_points];
				for(int k = 0; k < num_of_points ; k++)
				{
					temp_j_colors[k] = Scalar(200,215,0);
					circle(frame,trackedPts[k],1,j_colors[k]);
				}

				writer << frame_deinterlaced;

				//save the current mask and the current points
				fg_mask.copyTo(fg_mask_prev);

				// copying the prevPts to trackerPts
				prevPts.clear();
				for(int k = 0 ; k < trackedPts.size(); k++)
				{
					prevPts.push_back(trackedPts[k]);
				}
			    trackedPts.clear();

			    if(displayOn)
			    {
			    	imshow(SOURCE_WIN,frame_deinterlaced);
			    	imshow(POINTS_WIN,frame_copy);
			    	waitKey(0);
			    }
			}
			else if( (frame_count > trackFrame_begin + 1) && (frame_count < trackFrame_begin+num_of_framesTrack) ) //else if( (frame_count > trackFrame_begin + 1) && (frame_count <= trackFrame_begin+num_of_frames_per_cycle - 1) )  // TODO: repeat the same loop but with the correction from the manual points
			{
				try{
					vector<Point2f> trackedPts;
					writer2 << frame_deinterlaced;

					cvtColor(frame_deinterlaced,frame_ycbcr,CV_BGR2YCrCb);
					vector<Mat> temp;
					split(frame_ycbcr,temp);
					temp[0].copyTo(frame_gray);

					optFlow.UpdateImages(frame_deinterlaced); // optical flow in LBP mode
					fg_mask = abs(bg_frame_gray - frame_gray);

					threshold(fg_mask,fg_mask,15,255,CV_THRESH_BINARY);
					erode(fg_mask,fg_mask,Mat(),Point(-1,-1),1);
					dilate(fg_mask,fg_mask,Mat(),Point(-1,-1),1);

					// compute optical flow and find median velocity ( Here, no previous points)
					Point2f velXY;
					if(flag_KLT_tracker)
						velXY = optFlow.ComputeOpticalFlowLK(frame_deinterlaced,fg_mask,fg_mask_prev,flow_mag,flow_dir,prevPts,trackedPts,false);
					else
					{
						// compute the average velocity of motion for updating velocity of joints for region based trackers
						vector<Point2f> temp_vector;
						velXY = optFlow.ComputeOpticalFlowLK(frame_deinterlaced,fg_mask,fg_mask_prev,flow_mag,flow_dir,temp_vector,temp_vector,false);
					}

					//Get the first set of points and Initialize the joint Descriptors with the velocity computed from the first two frames
					for(int k = 1; k <= num_of_points ; k++)
					{
						//extracting the current frame coordinates and drawing them
						float x_cor,y_cor;
						inputFile >> x_cor;
						inputFile >> y_cor;
						circle(frame_copy,Point2f(x_cor,y_cor),1,Scalar(199,28,125),-1);

						// draw the search region
						RotatedRect reg = joints[k-1]->getSearchRegion();
						ellipse(frame_copy,reg,j_colors[k-1]);

						// set the average opt flow velocity of each joint
						joints[k-1]->setGlobalVelocity(velXY);
						Point2f cors;
						if(flag_ver1)
							cors = Point2f(x_cor,y_cor);

						Point2f correctedLoc;
						if(flag_KLT_tracker) // KLT Tracker
						{
							cors = trackedPts[k-1];
							correctedLoc = cors; // Here no kalman filtering is used. Just mere track estimate
						}
						else if(flag_SIFT_tracker || flag_SURF_tracker || flag_BRIEF_tracker || flag_BRISK_tracker || flag_ORB_tracker || flag_FREAK_tracker)

							// Use of Kalman filter
							if(flag_SIFT_tracker_kalman || flag_SURF_tracker_kalman || flag_BRIEF_tracker_kalman || flag_BRISK_tracker_kalman || flag_ORB_tracker_kalman || flag_FREAK_tracker_kalman)
								correctedLoc = joints[k-1]->track_regionBasedKalman(prevPts[k-1],optFlow.frame_prev_color,optFlow.frame_color,Point2f(x_cor,y_cor));
							else if(flag_SIFT_proposed || flag_SURF_proposed || flag_BRIEF_proposed || flag_BRISK_proposed || flag_ORB_proposed || flag_FREAK_proposed)
								correctedLoc = joints[k-1]->track_proposed(prevPts[k-1],optFlow.frame_prev_color,optFlow.frame_color,Point2f(x_cor,y_cor));
							else if(flag_SIFT_proposed_prob || flag_SURF_proposed_prob || flag_BRIEF_proposed_prob || flag_BRISK_proposed_prob || flag_ORB_proposed_prob || flag_FREAK_proposed_prob)
								correctedLoc = joints[k-1]->track_proposed_prob(prevPts[k-1],optFlow.frame_prev_color,optFlow.frame_color,Point2f(x_cor,y_cor));
							else
								correctedLoc = joints[k-1]->track_regionBased(prevPts[k-1],optFlow.frame_prev_color,optFlow.frame_color,Point2f(x_cor,y_cor));
						else if(flag_HOG_tracker)
						{
							if(flag_HOG_tracker_kalman)
								correctedLoc = joints[k-1]->track_HOGBasedKalman(prevPts[k-1],optFlow.frame_prev_color,optFlow.frame_color,Point2f(x_cor,y_cor));
							else if(flag_HOG_proposed)
								correctedLoc = joints[k-1]->track_proposed_withHOG(prevPts[k-1],optFlow.frame_prev_color,optFlow.frame_color,Point2f(x_cor,y_cor));
							else if(flag_HOG_proposed_prob)
								correctedLoc = joints[k-1]->track_proposed_withHOG_prob(prevPts[k-1],optFlow.frame_prev_color,optFlow.frame_color,Point2f(x_cor,y_cor));
							else
								correctedLoc = joints[k-1]->track_HOGBased(prevPts[k-1],optFlow.frame_prev_color,optFlow.frame_color,Point2f(x_cor,y_cor));
						}
						else if(flag_LBP_tracker)
						{
							if(flag_LBP_tracker_kalman)
								correctedLoc = joints[k-1]->track_LBPBasedKalman(prevPts[k-1],optFlow.frame_prev_color,optFlow.frame_color,Point2f(x_cor,y_cor));
							else if(flag_LBP_proposed)
								correctedLoc = joints[k-1]->track_proposed_withLBP(prevPts[k-1],optFlow.frame_prev_color,optFlow.frame_color,Point2f(x_cor,y_cor));
							else if(flag_LBP_proposed_prob)
								correctedLoc = joints[k-1]->track_proposed_withLBP_prob(prevPts[k-1],optFlow.frame_prev_color,optFlow.frame_color,Point2f(x_cor,y_cor));
							else
								correctedLoc = joints[k-1]->track_LBPBased(prevPts[k-1],optFlow.frame_prev_color,optFlow.frame_color,Point2f(x_cor,y_cor));
						}
						else
							correctedLoc = joints[k-1]->track(cors,frame_deinterlaced,flagMatch,flagPassThrough,Point2f(x_cor,y_cor),flagRegionMatch2,flagInitialize,flag_firstStageCheck);

						if(trackedPts.size() != num_of_points)
							trackedPts.push_back(correctedLoc);
						else
						{
							// This usually happens when the KLT tracker is called where the tracks are obtained before initialization of joints
							trackedPts[k-1].x = correctedLoc.x;
							trackedPts[k-1].y = correctedLoc.y;
						}

						if(displayOn)
							cout << "Manual Location : " << Point2f(x_cor,y_cor) << "\t Tracked Location : " << trackedPts[k-1] <<"\t Corrected Location : " << correctedLoc << endl;

						ellipse(frame_copy,joints[k-1]->search_region2,j_colors[k-1]);

						// output the corrected location
						outputFile << correctedLoc.x << "\t";
						outputFile << correctedLoc.y << "\t";
					}
					outputFile << "\n";
					if(displayOn)
						cout << "\n" << endl;
					Scalar temp_j_colors[num_of_points];
					for(int k = 0; k < num_of_points ; k++)
					{
						temp_j_colors[k] = Scalar(200,215,0);
						circle(frame_deinterlaced,trackedPts[k],1,j_colors[k]);
					}

					writer << frame_deinterlaced;
					//save the current mask and the current points
					fg_mask.copyTo(fg_mask_prev);

					// copying the prevPts to trackerPts
					prevPts.clear();
					for(int k = 0 ; k < trackedPts.size(); k++)
					{
						prevPts.push_back(trackedPts[k]);
					}
					trackedPts.clear();

					if(displayOn)
					{
						imshow(SOURCE_WIN,frame_deinterlaced);
						imshow(POINTS_WIN,frame_copy);
						waitKey(0);
					}
				}
				catch(cv::Exception& e)
				{
					const char* err_msg = e.what();
					std::cout << "exception caught: " << err_msg << std::endl;
					exit(0);
				}
			}
			else
			{
				try{
					cvtColor(frame_deinterlaced,frame_ycbcr,CV_BGR2YCrCb);
					vector<Mat> temp;
					split(frame_ycbcr,temp);
					temp[0].copyTo(frame_gray);

					if(displayOn)
					{
						imshow(SOURCE_WIN,frame_deinterlaced);
						imshow(POINTS_WIN,frame_copy);
					}
				}
				catch(cv::Exception& e)
				{
					const char* err_msg = e.what();
					std::cout << "exception caught: " << err_msg << std::endl;
					exit(0);
				}
			}
			
			if(waitKey(10) >= 0) break;

		}
		
	}	
	else cout << "Error Opening the video file" <<endl;
	
	//saving the Joint Descriptors (the trackers)
	string header = "Tracker Parameters for each Joint";
	fs << "Details" << header << "Joints" <<"[";
	for(int k = 1 ; k <= num_of_points; k++)
	{
		char joint_num_str[10];
		sprintf(joint_num_str,"Joint-%d",k);
		JointDescriptor joides = (*joints[k-1]);
		fs << joides;
	}
	fs << "]";
	//releasing the camera
	capture.release();
	fs.release();
	inputFile.close();
	outputFile.close();
}

//function definition
vector<Mat> deinterlaceImage(Mat image)
{
	unsigned int rows = (unsigned int)image.rows;
	unsigned int cols = (unsigned int)image.cols;
	int type = image.type();
	Mat field1 = Mat(rows/2,cols,type);
	Mat field2 = Mat(rows/2,cols,type);
	vector<Mat> fields;

	for(unsigned int i = 0; i < rows; i+=2)
	{
		unsigned char* ptr_img1 = (unsigned char*)(image.data + image.step[0]*i); // getting the pointer to the ith row
		unsigned char* ptr_img2 = (unsigned char*)(image.data + image.step[0]*(i+1));

		unsigned char* ptr_field1 = (unsigned char*)(field1.data+ field1.step[0]*(i>>1));
		unsigned char* ptr_field2 = (unsigned char*)(field2.data + field2.step[0]*((i>>1)));

		for(unsigned int j = 0 ; j < cols ; j++)
		{
			ptr_field1[3*j] = ptr_img1[3*j];
			ptr_field1[3*j+1] = ptr_img1[3*j + 1];
			ptr_field1[3*j+2] = ptr_img1[3*j + 2];

			ptr_field2[3*j] = ptr_img2[3*j];
			ptr_field2[3*j+1] = ptr_img2[3*j + 1];
			ptr_field2[3*j+2] = ptr_img2[3*j + 2];
		}
	}
	resize(field1,field1,Size(),1,2,INTER_CUBIC);
	resize(field2,field2,Size(),1,2,INTER_CUBIC);

	fields.push_back(field1);
	fields.push_back(field2);
	return fields;
}
