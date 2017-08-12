/*
 * JointDescriptor.cpp
 *
 *  Created on: Jul 17, 2013
 *      Author: bnair002
 */
#include "JointDescriptor.h"
#include "cvplot.h"

using namespace std;

// class definitions
JointDescriptor::JointDescriptor()
{
	dynamParams = 4;
	measureParams = 2;
	controlParams = 4;
	type = CV_32F;
	timeStep = 1;
	acceleration = 0.01;// ideally, it should be (v_(k+1) - v(k))/(time_step) where v(k) - optical flow magnitude at time k
	winSize = DEFAULT_WINSIZE;
	matchThreshold = MATCH_THRESHOLD;
	//TODO:Iniialize the description - Later

	state = Mat::zeros(dynamParams,1,type);
	measurement = Mat::zeros(measureParams,1,type);

	//Initialize the tracker
	initializeTracker(dynamParams,measureParams,controlParams,type);

	// setting default default descriptor names
	this->des_name = "SIFT";
	this->des_match_name = "BruteForce";

	// defining the descriptor extractors
	desc_hog = new visionlab::HOGDescriptor();
	desc_lbp = new LBPDescriptor(u2);
	generic_extractor = DescriptorExtractor::create(des_name);

	// defining the descriptor matcher
	generic_matcher = DescriptorMatcher::create(des_match_name);

}

JointDescriptor::JointDescriptor(Point2f loc,Mat imag,bool displayFlag,string des_name, string des_match_name)
{
	dynamParams = 4;
	measureParams = 2;
	controlParams = 4;
	type = CV_32F;
	timeStep = 1;
	acceleration = 0.01;// ideally, it should be (v_(k+1) - v(k))/(time_step) where v(k) - optical flow magnitude at time k
	winSize = DEFAULT_WINSIZE;
	matchThreshold = MATCH_THRESHOLD;
	displayOn = displayFlag;
	//TODO:Initialize the description

	// Initializing the state and location
	state = Mat::zeros(dynamParams,1,type);
	measurement = Mat::zeros(measureParams,1,type);
	setLocation(loc);
	setVelocity(Point2f(1.5,0.3)); //TODO : Find a suitable initialization velocity

	cout << location << endl;
	cout << velocity << endl;

	//Initialize the tracker
	initializeTracker(dynamParams,measureParams,controlParams,type);

	// setting default default descriptor names
	this->des_name = des_name;
	this->des_match_name = des_match_name;

	// defining the descriptor extractors
	desc_hog = new visionlab::HOGDescriptor();
	desc_lbp = new LBPDescriptor(u2);
	generic_extractor = DescriptorExtractor::create(des_name);

	// defining the descriptor matcher
	generic_matcher = DescriptorMatcher::create(des_match_name);

	//Initialize the region representation
	// Extract the joint region
	Mat img_crop(imag,Rect(floor(location.x) - winSize/2 ,floor(location.y) - winSize/2,winSize,winSize));
	this->img = img_crop.clone();

	//display image
	if(displayOn)
	{
		imshow("Joint",this->img);
		waitKey(0);
	}

	feature_vector = computeSignature(this->img);
}

JointDescriptor::JointDescriptor(Point2f loc,Mat imag, int win_size,bool displayFlag,string des_name, string des_match_name)
{
	dynamParams = 4;
	measureParams = 2;
	controlParams = 4;
	type = CV_32F;
	timeStep = 1;
	acceleration = 0.01;// ideally, it should be (v_(k+1) - v(k))/(time_step) where v(k) - optical flow magnitude at time k
	winSize = win_size;
	matchThreshold = MATCH_THRESHOLD;
	displayOn = displayFlag;
	//TODO:Initialize the description

	// Initializing the state and location
	state = Mat::zeros(dynamParams,1,type);
	measurement = Mat::zeros(measureParams,1,type);
	setLocation(loc);
	setVelocity(Point2f(1.5,0.3)); //TODO : Find a suitable initialization velocity

	cout << location << endl;
	cout << velocity << endl;

	//Initialize the tracker
	initializeTracker(dynamParams,measureParams,controlParams,type);

	// setting default default descriptor names
	this->des_name = des_name;
	this->des_match_name = des_match_name;

	// defining the descriptor extractors
	desc_hog = new visionlab::HOGDescriptor();
	desc_lbp = new LBPDescriptor(u2);
	generic_extractor = DescriptorExtractor::create(des_name);

	// defining the descriptor matcher
	generic_matcher = DescriptorMatcher::create(des_match_name);

	//Initialize the region representation
	// Extract the joint region
	Mat img_crop(imag,Rect(floor(location.x) - winSize/2 ,floor(location.y) - winSize/2,winSize,winSize));
	this->img = img_crop.clone();

	//display image
	if(displayOn)
	{
		imshow("Joint",this->img);
		waitKey(0);
	}

	feature_vector = computeSignature(this->img);
}

JointDescriptor::JointDescriptor(Point2f loc,Mat imag, int win_size, float match_threshold,bool displayFlag,string des_name, string des_match_name)
{
	dynamParams = 4;
	measureParams = 2;
	controlParams = 4;
	type = CV_32F;
	timeStep = 1;
	acceleration = 0.1;// ideally, it should be (v_(k+1) - v(k))/(time_step) where v(k) - optical flow magnitude at time k
	winSize = win_size;
	matchThreshold = match_threshold;
	displayOn = displayFlag;
	//TODO:Initialize the description

	// Initializing the state and location
	state = Mat::zeros(dynamParams,1,type);
	measurement = Mat::zeros(measureParams,1,type);
	setLocation(loc);
	setVelocity(Point2f(1.5,0.3)); //TODO : Find a suitable initialization velocity

	//cout << location << endl;
	//cout << velocity << endl;

	//Initialize the tracker
	initializeTracker(dynamParams,measureParams,controlParams,type);

	// setting default default descriptor names
	this->des_name = des_name;
	this->des_match_name = des_match_name;

	// defining the descriptor extractors
	desc_hog = new visionlab::HOGDescriptor();
	desc_lbp = new LBPDescriptor(u2);
	generic_extractor = DescriptorExtractor::create(des_name);

	// defining the descriptor matcher
	generic_matcher = DescriptorMatcher::create(des_match_name);

	//Initialize the region representation
	// Extract the joint region
	Mat img_crop(imag,Rect(floor(location.x) - winSize/2 ,floor(location.y) - winSize/2,winSize,winSize));
	this->img = img_crop.clone();

	//display image
	if(displayOn)
	{
		imshow("Joint",this->img);
		waitKey(0);
	}

	feature_vector = computeSignature(this->img);
}

JointDescriptor::JointDescriptor(Point2f loc,Mat imag, int win_size, float match_threshold, Point2f vel,bool displayFlag,string des_name, string des_match_name)
{
	dynamParams = 4;
	measureParams = 2;
	controlParams = 4;
	type = CV_32F;
	timeStep = 0.03333; // it is related to the frame rate : 30 frames per sec
	acceleration = ACCELERATION;// ideally, it should be (v_(k+1) - v(k))/(time_step) where v(k) - optical flow magnitude at time k
	winSize = win_size;
	matchThreshold = match_threshold;
	displayOn = displayFlag;
	firstTime = true;
	//TODO:Initialize the description

	// Initializing the state and location
	state = Mat::zeros(dynamParams,1,type);
	measurement = Mat::zeros(measureParams,1,type);
	setLocation(loc);
	Point2f velXY(vel.x/timeStep,vel.y/timeStep);
	setVelocity(velXY); //TODO : Find a suitable initialization velocity

	//cout << location << endl;
	//cout << velocity << endl;

	//Initialize the tracker
	initializeTracker(dynamParams,measureParams,controlParams,type);
	firstTime = false;

	//predict to next frame
	predictJointPosition();

	//Compute Search Region for next instant
	computeSearchRegion();

	// setting default default descriptor names
	this->des_name = des_name;
	this->des_match_name = des_match_name;

	// defining the descriptor extractors
	desc_hog = new visionlab::HOGDescriptor();
	desc_lbp = new LBPDescriptor(u2);
	generic_extractor = DescriptorExtractor::create(des_name);

	// defining the descriptor matcher
	generic_matcher = DescriptorMatcher::create(des_match_name);

	//Initialize the region representation
	// Extract the joint region
	Mat img_crop(imag,Rect(floor(location.x) - winSize/2 ,floor(location.y) - winSize/2,winSize,winSize));
	this->img = img_crop.clone();

	//display image
	if(displayOn)
	{
		imshow("Joint",this->img);
		waitKey(0);
	}

	//feature_vector = computeSignature(this->img);
}

JointDescriptor::~JointDescriptor()
{
	delete tracker;
}

Point2f JointDescriptor::getLocation() const
{
	return location;
}

Point2f JointDescriptor::getVelocity() const
{
	return velocity;
}

RotatedRect JointDescriptor::getSearchRegion() const
{
	return search_region;
}

void JointDescriptor::setLocation(Point2f loc)
{
	location = loc;
}

void JointDescriptor::setVelocity(Point2f vel)
{
	velocity = vel;
}

void JointDescriptor::setGlobalVelocity(Point2f gb_velocity)
{
	global_velocity = gb_velocity;
}

void JointDescriptor::setJointColor(Scalar color)
{
	joint_color = color;
}

void JointDescriptor::setState(Point2f loc,Point2f vel)
{
	// Setting the state with the location and velocity
	state.at<float>(0,0) = loc.x;
	state.at<float>(1,0) = loc.y;
	state.at<float>(2,0) = vel.x;
	state.at<float>(3,0) = vel.y;
}

void JointDescriptor::setMeasurement(Point2f loc)
{
	// Setting the measurement data
	measurement.at<float>(0,0) = loc.x;
	measurement.at<float>(1,0) = loc.y;
}

void JointDescriptor::initializeTracker(int dynamParams,int measureParams,int controlParams,int type)
{
	flagFirst = true;

	//Initializing the tracker
	if(firstTime)
		tracker = new KalmanFilter(dynamParams,measureParams,controlParams);
	else
		tracker->init(dynamParams,measureParams,controlParams);

	// Prediction Side
	tracker->statePre = Mat(dynamParams,1,type,0); //x_(k+1)-
	tracker->statePost = Mat(dynamParams,1,type,0); // x_(k)+
	// THIS REMAINS A CONSTANT THROUGHOUT!! ASSUMES A LINEAR RELATIONSHIP BETWEEN STATES
	tracker->transitionMatrix = (Mat_<float>(dynamParams,dynamParams) << 1,0,timeStep,0,0,1,0,timeStep,0,0,1,0,0,0,0,1); // A
	setIdentity(tracker->controlMatrix,Scalar(0));// B
	//setIdentity(tracker->processNoiseCov,Scalar(acceleration*acceleration)); // Q
	float noise_scale = acceleration*acceleration * timeStep/6;
	float val1 = noise_scale*2*timeStep*timeStep;
	float val2 = noise_scale*3*timeStep;
	float val3 = noise_scale * 6;
	// THIS REMAINS A CONSTANT THROUGHOUT AS WELL!
	tracker->processNoiseCov = (Mat_<float>(dynamParams,dynamParams) << val1,0,val2,0,0,val1,0,val2,val2,0,val3,0,0,val2,0,val3);
	if(errorCovPre.empty())
		setIdentity(tracker->errorCovPre,Scalar(1)); //P_k-
	else
		tracker->errorCovPre = errorCovPre.clone();

	// setting the gain
	if(!kGain.empty())
		tracker->gain = kGain.clone();

	//Correction Side
	// THIS IS ALSO CONSTANT.. REFERS TO THE STANDARD DEVIATION OF THE (X,Y) DIRECTIONS
	// IF (X,Y) FOLLOWS A MANIFOLD, IT WILL BE STANDARD DEVIATION ALONG THE MANIFOLD
	setIdentity(tracker->measurementNoiseCov,Scalar(1e-5)); // R

	// ERROR COVARIANCE CONVERGES TO A STABLE VALUE AFTER A FEW FRAMES
	// WE NEED TO SET A CONVERGENCE VALUE AFTER WHICH WE START USING THE KALMAN FILTER
	// TILL IT STABILIZES THE KALMAN FILTER SHOULD BE ASSUMED THAT ITS TRAINED
	// TRAINING OF KALMAN FILTER: WHAT DOES IT MEAN? COMPUTING OF STABLE KALMAN GAIN?
	// TRAINING OF KALMAN FILTER FOR AN INDIVIDUAL:
	// 			A) FINDING STABLE PRE-COVARIANCE AND POST COVARIANCE ERROR MATRICES FROM TRAINING DATA
	//			B) FINDING STABLE KALMAN GAIN VALUES
	// TRACKER NEEDS TO BE RE-INITIALIZAED (RE-FINDING OF STABLE CONVERGES KALMAN FILTER PARAMETERS)
	// ONLY IF THE PERSON MAKES A SUDDEN DRASTIC MOVEMENT
	// THE ESTIMATES OF THE KALMAN FILTER FROM THE TRAINING DATA CAN BE USED TO TEST ONE SIMILAR MOTION DATA
	// WHERE THERE IS NO DRASTIC DIFFERENCE BETWEEN THE TRAINING MOTION DATA AND TEST MOTION DATA
	// THE ONLY DIFFERENCE BEING THE SIMILAR MOTION CORRESPONDING TO A DIFFERENT INDIVIDUAL
	// HOW ABOUT FOR EACH TYPE OF ACTION, A DIFFERENT KALMAN FILTER CHARACTERISTICS:
	// FIRST IDENTIFY OR CLASSIFY THE ACTION AND THEN FIND THE TRACKS OF THE SEQUENCE
	// TRAINING WILL INVOLVE FINDING STABLE PARAMETERS OF THE KALMAN FILTER FOR EACH ACTION CLASS
	// TODO: TRAINING OF TRACKER FAULTY SINCE MANUAL POINTS ARE ALSO NOISY!!!!
	if(errorCovPost.empty())
		setIdentity(tracker->errorCovPost,Scalar(1)); // P_k+
	else
		tracker->errorCovPost = errorCovPost.clone();

	// THIS ALSO REMAINS CONSTANT : TELLS HOW THE STATE VECTOR IS RELATED TO MEASUREMENT
	tracker->measurementMatrix = (Mat_<float>(measureParams,dynamParams) << 1,0,0,0,0,1,0,0); // H // check the values

	// THE KALMAN GAIN ALSO CONVERGES TO A STABLE SET OF VALUES
	// KALMAN GAIN DIMENSIONS 4 X 2
	// THE KALMAN GAIN AND POST-CONVARIANCE MATRICES ARE THE SAME MAINLY BECAUSE OF THE SAME GLOBAL VELOCITY ASSOCIATED WITH IT
}

void JointDescriptor::predictJointPosition()
{
	// setting the state
	setState(location,velocity);

	//copying the initial state to statePost
	state.copyTo(tracker->statePost);

	// predicting the state pre (state at k) from the state post( state at k-1 ) using measurement
	tracker->predict();
}

bool JointDescriptor::matchImageDescriptors(vector<float> model,vector<float> sample, float& measure, bool flag_LBP)
{
    measure = 0.0;

    if(flag_LBP)
    {
		// LBP feature matching :  Chi-squared distance metric
		for(int b = 0 ; b < model.size() ; b++)
		{
			float Sb = sample[b];
			float Mb = model[b];

			if( (Sb==0) & (Mb == 0))
				measure += 0;
			else
				measure += ( (Sb - Mb)*(Sb - Mb) / (Sb + Mb) );

		}

		if(measure <= matchThreshold)
			return true;
		else
			return false;
    }
    else
    {
    	// HOG feature matching : Euclidean distance
		for(int b = 0 ; b < model.size() ; b++)
		{
			float Sb = sample[b];
			float Mb = model[b];

			if( (Sb==0) & (Mb == 0))
				measure += 0;
			else
				measure += (Sb - Mb)*(Sb - Mb);

		}

		measure = std::sqrt(measure);

		if(measure <= matchThreshold)
			return true;
		else
			return false;
    }
}

void JointDescriptor::correctJointPosition()
{
	// correcting the tracker and obtaining new value of statePost from statePre and measurement(new location) at instant k
	tracker->correct(measurement);

	//getting the updated location
	Point2f loc;
	loc.x = tracker->statePost.at<float>(0,0);
	loc.y = tracker->statePost.at<float>(1,0);

	//computing the velocity
	Point2f st(location.x,location.y);
	Point2f en(loc.x,loc.y);

	// BIG BLUNDER!! : Final Position vector(en) - Initial Position Vector . What I did was the reverse which gave negative velocity(opposite direction)
	float hypotenuse = sqrt( (en.y - st.y)*(en.y - st.y) + (en.x - st.x)*(en.x - st.x));
	float angle = atan2((en.y - st.y),(en.x - st.x));
	//Point2f velxy(hypotenuse*cos(angle)/timeStep,hypotenuse*sin(angle)/timeStep);
	//Point2f velxy(hypotenuse*cos(angle),hypotenuse*sin(angle));
	Point2f velxy(global_velocity.x/timeStep,global_velocity.y/timeStep);

	// TODO: Set velxy as the global velocity of the body computed from optical flow
	//setVelocity(global_velocity);
	// storing this location and velocity for the next instant
	setLocation(loc);
	setVelocity(velxy); // TODO: Need to store the global velocity
}

// the function used to call the track from the main function at a particular instant
Point2f JointDescriptor::track(Point2f currentLoc,Mat imag, bool flagMatch, bool flagPassThrough, Point2f detect_loc,bool flag_useDetectLoc, bool flagInitialize, bool flag_firstStageCheck)
{
	// TODO: Check if the current location is in the search region

	// If(inside search region)
	//		Use this estimate to correct it
	// Else
	// 		Compute descriptors within the search region at each point
	// 		Find the closest match to the previously detected joint region
	// 		Use that location as the estimate for correction

	// Check if current location is in the search region
	Mat temp_mask = Mat::zeros(imag.rows,imag.cols,CV_8UC1);
	ellipse(temp_mask,search_region,Scalar(255,255,255),-1);
	//imshow("Temporary Mask",temp_mask);
	search_region2 = RotatedRect();


	int x_cor = (int)(currentLoc.x);
	int y_cor = (int)(currentLoc.y);

	// CHeck if the location is inside the ellipse
	// If flagInitialize is 1, then its irrelevant if the point falls on the eclipse or not
	// If flagInitialize is 0, then the decision is based on whether the initialized points falls on it or not.
	if( ((temp_mask.at<unsigned char>(y_cor,x_cor) > 0) || flagPassThrough) && flag_firstStageCheck)
	{
		// Do the correction stage
		// Set measurement
		setMeasurement(currentLoc);
		//cout << "Joint State Corrected" <<endl;

		//correct location
		correctJointPosition();

		//update img and feature vector
		Mat img_crop_update(imag,Rect(floor(location.x) - winSize/2,floor(location.y) - winSize/2,winSize,winSize));
		this->img = img_crop_update.clone();
		//feature_vector = computeSignature(this->img);
	}
	else //  no location found and so compute signature at each point
	{
		vector<Point2f> search_coords = ExtractCoordsFromImage(imag,temp_mask);
		vector<float> c_measures;
		for(int k = 0; k < search_coords.size(); k++)
		{
			// get the point inside the elliptical search region and compute signature at that point
			Point2f interestPt = search_coords[k];

			Mat img_crop(imag,Rect(floor(interestPt.x) - DEFAULT_WINSIZE/2,floor(interestPt.y) - DEFAULT_WINSIZE/2,DEFAULT_WINSIZE,DEFAULT_WINSIZE));
			vector<float> current_feature = computeSignature(img_crop);

			// match this interest point descriptor with joint descriptor
			float measure;

			bool correctFlag = matchImageDescriptors(feature_vector,current_feature,measure);
			// Check if currentLoc is in the search region or not
			if(flagMatch == false)
				correctFlag = true;

			// If correct flag is true, then store the minimum measure and the coordinates
			if(correctFlag == false)
				search_coords.erase(search_coords.begin()+k);
			else
				c_measures.push_back(measure);
		}

		if(search_coords.size() > 0)// there is a point
		{
//			// set the location as the mean of the remaining points in the ellipse
			int x_match = 0;
			int y_match= 0;
//			for(int k = 0 ; k < search_coords.size(); k++)
//			{
//				Point2f intPt = search_coords[k];
//				x_match = x_match + intPt.x;
//				y_match = y_match + intPt.y;
//			}
//			x_match /= search_coords.size();
//			y_match /= search_coords.size();

			// To find the location with the minimum distance
			//float min_measure = *(min_element(c_measures.begin(),c_measures.end());

			int min_ind;
			float min_measure = 1000.0;
			for(int k = 0; k < c_measures.size(); k++)
			{
				float measure = c_measures[k];
				if(measure <= min_measure )
				{
					min_measure = measure;
					min_ind = k;
				}
			}

			//Point2f updateLoc = Point2f(x_match,y_match);
			Point2f updateLoc = search_coords[min_ind];
			if(displayOn)
				cout << "Minimum Region Based Measure for location within search region before tracking" <<updateLoc << " : " << min_measure << endl;
			// TODO: if no points are present (or minimum no:points criteria is not met)
			// TODO: Only predict

			// Need to find a way to extract using floating point : Extract the joint region :
//			Mat img_crop(imag,Rect(floor(updateLoc.x) - DEFAULT_WINSIZE/2,floor(updateLoc.y) - DEFAULT_WINSIZE/2,DEFAULT_WINSIZE,DEFAULT_WINSIZE));
//			vector<float> current_feature = computeSignature(img_crop);

			// TODO: Update Location is discrete
//			// Do the correction stage
//			// Set measurement
//			setMeasurement(updateLoc);
//			//cout << "Joint State Corrected" <<endl;
			float a_1 = search_region.size.width/2;
			float b_1 = search_region.size.height/2;
			float area1 = 3.14 * a_1 * b_1;
			if(displayOn)
				cout << "Area of First Ellipse : " << area1  << endl;
//
//			//correct location
//			correctJointPosition();
			if(flag_useDetectLoc) // if we are using the manual or detected location of joint(using human pose estimation)
			{
				// find the new updated location using the search region
				// extracting the foci of the elliptical search region
				Point2f f1,f2;
				// tracker->statePost gets changed after prediction
				f1.x = updateLoc.x;
				f1.y = updateLoc.y;
				f2.x = detect_loc.x;
				f2.y = detect_loc.y;

				// extracting the variance from the a-priori error covariance
				Point2f sigma2;
				sigma2.x = tracker->errorCovPre.at<float>(0,0);
				sigma2.y = tracker->errorCovPre.at<float>(1,1);

				// computing the center, angle and semi-major,minor axes
				Point2f center;
				center.x = f1.x + (std::abs(f2.x - f1.x)/2);
				center.y = f2.y + (std::abs(f2.y - f1.y)/2);

				// compute the angle
				float ang = std::atan2((f2.y - f1.y),(f2.x - f1.x));
				if(ang < 0)
					ang = 360 + ang;

				// compute the major and minor axis
				float c = std::sqrt((f2.x - f1.x)*(f2.x - f1.x) + (f2.y - f2.y)*(f2.y - f2.y));
				float b = c + std::sqrt(sigma2.x*sigma2.x + sigma2.y*sigma2.y);
				float a = std::sqrt(c*c + b*b);

				//cout << "center = " << center << endl;
				//cout << "c = " << c << " ; b = "<< b << " ; a = " << a << endl;
				search_region2 = RotatedRect(center,Size2f(2*a,2*b),ang);
				float area2 = 3.14 * a * b;
				if(displayOn)
					cout << "Area of Second Ellipse : " << area2  << endl;

				// If Area 2 is way bigger, then we should avoid search region 2


			    if(area2 <= ALPHA_P * area1) // here it shows that the region matched point fails since the manual points although noisy is more accurate than the region based estimation
			    	//flagInitialize = true;
			    	flag_useDetectLoc = true;
			    else
			    	flag_useDetectLoc = false;
			    	//flagInitialize = false;

			    // If area2 is much larger, then just use the updated location to correct thecker.. not reinitialize

			    // If area of the region based ellipse is greater than the area of optical flow ellipse,
			    // then the manual points are noisy and cannot be used to re-initialize the Kalman tracker
			    // Then, just do the linear Kalman Filter prediction

				// TODO: if search region is larger than a specified area, then
				// We need to predict
				// When do we re-initialize?

			    // Only if the area2 is not so large, will we compute an updated location in search region 2
			    if( !(area2 >= ALPHA_P*ALPHA_P*area1))
			    {

					// ellipse image
					Mat temp_mask2 = Mat::zeros(imag.rows,imag.cols,CV_8UC1);
					ellipse(temp_mask2,search_region2,Scalar(255,255,255),-1);

					// Computing region descriptors at every point on the second ellipe
					vector<Point2f> search_coords2 = ExtractCoordsFromImage(imag,temp_mask2);
					vector<float> c_measures2;
					for(int k = 0; k < search_coords2.size(); k++)
					{
						// get the point inside the elliptical search region and compute signature at that point
						Point2f interestPt = search_coords2[k];
						Mat img_crop(imag,Rect(floor(interestPt.x) - DEFAULT_WINSIZE/2,floor(interestPt.y) - DEFAULT_WINSIZE/2,DEFAULT_WINSIZE,DEFAULT_WINSIZE));
						vector<float> current_feature = computeSignature(img_crop);

						// match this interest point descriptor with joint descriptor
						float measure = 0.0;

						bool correctFlag = matchImageDescriptors(feature_vector,current_feature,measure);
						// Check if currentLoc is in the search region or not
						if(flagMatch == false)
							correctFlag = true;
						//cout << interestPt << ":"  << measure << endl;
						// If correct flag is true, then store the minimum measure and the coordinates
						if(correctFlag == false)
							search_coords2.erase(search_coords.begin()+k);
						else
							c_measures2.push_back(measure);
					}

					// Find the minimum distance
					min_measure = 1000.0;
					for(int k = 0; k < c_measures2.size(); k++)
					{
						float measure = c_measures2[k];
						if(measure <= min_measure )
						{
							min_measure = measure;
							min_ind = k;
						}
					}

					//Point2f updateLoc = Point2f(x_match,y_match);
					updateLoc = search_coords2[min_ind];
					if(displayOn)
						cout << "Minimum Region Based Measure 2 obtained in location after tracking" <<updateLoc << " : " << min_measure << endl;
			    }

			}

			if(flagInitialize)
			{
				//Reinitialize the tracker
				state = Mat::zeros(dynamParams,1,type);
				measurement = Mat::zeros(measureParams,1,type);
				setLocation(updateLoc);
				//setLocation(detect_loc);
				Point2f velxy(global_velocity.x/timeStep,global_velocity.y/timeStep);
				setVelocity(velxy); //TODO : Find a suitable initialization velocity
				initializeTracker(dynamParams,measureParams,controlParams,type);
				if(displayOn)
					cout << "Tracker Re-initialized" <<endl;

				//update img and feature vector
				Mat img_crop_update(imag,Rect(floor(location.x) - winSize/2,floor(location.y) - winSize/2,winSize,winSize));
				this->img = img_crop_update.clone();
				feature_vector = computeSignature(this->img);
			}
			else if(flag_useDetectLoc) // use the updated location to use this as measurement
			{
				// Set measurement
				setMeasurement(updateLoc);
				//setMeasurement(detect_loc);
				//correct location
				correctJointPosition();
				if(displayOn)
					cout << "Joint State Corrected with Region Updated Location" <<endl;

				//update img and feature vector
				Mat img_crop_update(imag,Rect(floor(location.x) - winSize/2,floor(location.y) - winSize/2,winSize,winSize));
				this->img = img_crop_update.clone();
				feature_vector = computeSignature(this->img);
			}
			else
			{
				// Set measurement
				//setMeasurement(updateLoc);
				setMeasurement(detect_loc);
				//correct location
				correctJointPosition();
				if(displayOn)
					cout << "Joint State Corrected with Manual Point Detected Location(Coarse)" <<endl;

				//update img and feature vector
				Mat img_crop_update(imag,Rect(floor(location.x) - winSize/2,floor(location.y) - winSize/2,winSize,winSize));
				this->img = img_crop_update.clone();
				feature_vector = computeSignature(this->img);
			}

		}
		else
		{
			// Copy Predicted state to current location and velocity
			//getting the updated location
			Point2f loc;
			loc.x = tracker->statePre.at<float>(0,0);
			loc.y = tracker->statePre.at<float>(1,0);

			//computing the velocity
			Point2f st(location.x,location.y); //  this is the previous location
			Point2f en(loc.x,loc.y);

			// BIG BLUNDER!! : Final Position vector(en) - Initial Position Vector . What I did was the reverse which gave negative velocity(opposite direction)
			float hypotenuse = sqrt( (en.y - st.y)*(en.y - st.y) + (en.x - st.x)*(en.x - st.x));
			float angle = atan2((en.y - st.y),(en.x - st.x));
			//Point2f velxy(hypotenuse*cos(angle)/timeStep,hypotenuse*sin(angle)/timeStep);
			Point2f velxy(global_velocity.x/timeStep,global_velocity.y/timeStep);

			// storing this location and velocity for the next instant
			setLocation(loc);
			setVelocity(velxy);
			//setVelocity(global_velocity);

			//Reinitialize the velocity
			//setVelocity(Point2f(1.5,0.3));
		}

	}

	//predict to next frame
	predictJointPosition();

	//Compute Search Region for next instant
	computeSearchRegion();

	return location;

}

// Type 1 method  : HOG descriptor
Point2f JointDescriptor::track_HOGBased(Point2f prevLoc, Mat prev_img, Mat current_img, Point2f detect_loc, Mat mask)
{
	// Compute the descriptor for the previous location
	Mat img_crop_prev(prev_img,Rect(floor(prevLoc.x) - DEFAULT_WINSIZE/2,floor(prevLoc.y) - DEFAULT_WINSIZE/2,DEFAULT_WINSIZE,DEFAULT_WINSIZE));
	vector<float> prev_feature = computeSignature(img_crop_prev);

	// Extract coordinates in the current image belonging to the computer search region S_reg1
	vector<Point2f> search_coords = ExtractCoordsFromImage(current_img,mask,Rect(floor(detect_loc.x) - winSize/2,floor(detect_loc.y) - winSize/2,winSize,winSize));

	// Compute the descriptors for the search coordinates in S_reg1
	vector<float> c_measures;
	for(int k = 0; k < search_coords.size(); k++)
	{
		// get the point inside the elliptical search region and compute signature at that point
		Point2f interestPt = search_coords[k];

		Mat img_crop(current_img,Rect(floor(interestPt.x) - DEFAULT_WINSIZE/2,floor(interestPt.y) - DEFAULT_WINSIZE/2,DEFAULT_WINSIZE,DEFAULT_WINSIZE));
		vector<float> current_feature = computeSignature(img_crop);

		// match this interest point descriptor with joint descriptor
		float measure;

		bool correctFlag = matchImageDescriptors(prev_feature,current_feature,measure,false);
		c_measures.push_back(measure);
	}

	// Compute the minimum distance measure
	int min_k;
	float min_dist = 1000.0;
	for(int k = 0; k < c_measures.size(); k++)
	{
		float measure = c_measures[k];
		if(measure <= min_dist )
		{
			min_dist = measure;
			min_k = k;
		}
	}

	setLocation(search_coords[min_k]);

	return location;
}

// Type 1 method  : LBP descriptor
Point2f JointDescriptor::track_LBPBased(Point2f prevLoc, Mat prev_img, Mat current_img, Point2f detect_loc, Mat mask)
{
	// Compute the descriptor for the previous location
	Mat img_crop_prev(prev_img,Rect(floor(prevLoc.x) - DEFAULT_WINSIZE/2,floor(prevLoc.y) - DEFAULT_WINSIZE/2,DEFAULT_WINSIZE,DEFAULT_WINSIZE));
	vector<float> prev_feature = computeSignature(img_crop_prev,false);

	// Extract coordinates in the current image belonging to the computer search region S_reg1
	vector<Point2f> search_coords = ExtractCoordsFromImage(current_img,mask,Rect(floor(detect_loc.x) - winSize/2,floor(detect_loc.y) - winSize/2,winSize,winSize));

	// Compute the descriptors for the search coordinates in S_reg1
	vector<float> c_measures;
	for(int k = 0; k < search_coords.size(); k++)
	{
		// get the point inside the elliptical search region and compute signature at that point
		Point2f interestPt = search_coords[k];

		Mat img_crop(current_img,Rect(floor(interestPt.x) - DEFAULT_WINSIZE/2,floor(interestPt.y) - DEFAULT_WINSIZE/2,DEFAULT_WINSIZE,DEFAULT_WINSIZE));
		vector<float> current_feature = computeSignature(img_crop,false);

		// match this interest point descriptor with joint descriptor
		float measure;

		bool correctFlag = matchImageDescriptors(prev_feature,current_feature,measure);
		c_measures.push_back(measure);
	}

	// Compute the minimum distance measure
	int min_k;
	float min_dist = 1000.0;
	for(int k = 0; k < c_measures.size(); k++)
	{
		float measure = c_measures[k];
		if(measure <= min_dist )
		{
			min_dist = measure;
			min_k = k;
		}
	}

	setLocation(search_coords[min_k]);

	return location;
}

// Type 1 method, which uses the detected location to get the search region
Point2f JointDescriptor::track_regionBased(Point2f prevLoc, Mat prev_img, Mat current_img, Point2f detect_loc, Mat mask)
{
	// Assign the keypoint
	KeyPoint kp = KeyPoint(prevLoc,winSize);
	vector<KeyPoint> kps_prev,kps_current;
	kps_prev.push_back(kp);

	// compute the descriptor in the previous location
	Mat prev_descriptors,curr_descriptors;
	generic_extractor->compute(prev_img,kps_prev,prev_descriptors);

	// Extract coordinates in the current image
	// set region fixed size around the discrete location
	vector<Point2f> search_coords = ExtractCoordsFromImage(current_img,mask,Rect(floor(detect_loc.x) - winSize/2,floor(detect_loc.y) - winSize/2,winSize,winSize));

	// Convert to KeyPoints
	for(int k = 0 ; k < search_coords.size(); k++)
	{
		KeyPoint kp_temp = KeyPoint(search_coords[k],winSize);
		kps_current.push_back(kp_temp);
	}

	// Compute the descriptors
	generic_extractor->compute(current_img,kps_current,curr_descriptors);

	// Perform descriptor matching and find the minimum location
	vector<DMatch> matches;
	generic_matcher->match(curr_descriptors,prev_descriptors,matches,Mat());

	// obtain and analyze the matches
	float min_dist = 10000;
	int min_k = 0;
	for(int k = 0 ; k < matches.size(); k++)
	{
		float dist = matches[k].distance;

		if(dist < min_dist)
		{
			min_dist = dist; //  computing the new minimum distance
			min_k = k;
		}
	}

	setLocation(search_coords[min_k]);

	// set the matched location
	return location;
}

// Type 2 method which uses the Kalman filter to obtain the search region . The kalman filter is designed appropriately to model frame-to-frame body joint state transition
Point2f JointDescriptor::track_regionBasedKalman(Point2f prevLoc, Mat prev_img, Mat current_img, Point2f detect_loc, Mat mask)
{
	// Here, we are not using the detect_loc at all : for future purposes
	// Obtain the mask simulating the search region in the current image
	Mat temp_mask = Mat::zeros(current_img.rows,current_img.cols,CV_8UC1);
	ellipse(temp_mask,search_region,Scalar(255,255,255),-1);

	Point2f updateLoc;

	// Assign the keypoint
	KeyPoint kp = KeyPoint(prevLoc,winSize);
	vector<KeyPoint> kps_prev,kps_current;
	kps_prev.push_back(kp);

	// compute the descriptor in the previous location
	Mat prev_descriptors,curr_descriptors;
	generic_extractor->compute(prev_img,kps_prev,prev_descriptors);

	// Extract coordinates in the current image belonging to the computer search region
	vector<Point2f> search_coords = ExtractCoordsFromImage(current_img,temp_mask,Rect());

	// Convert to KeyPoints
	for(int k = 0 ; k < search_coords.size(); k++)
	{
		KeyPoint kp_temp = KeyPoint(search_coords[k],winSize);
		kps_current.push_back(kp_temp);
	}

	// Compute the descriptors
	generic_extractor->compute(current_img,kps_current,curr_descriptors);

	// Perform descriptor matching and find the minimum location
	vector<DMatch> matches;
	generic_matcher->match(curr_descriptors,prev_descriptors,matches,Mat());

	// obtain and analyze the matches
	float min_dist = 10000;
	int min_k = 0;
	for(int k = 0 ; k < matches.size(); k++)
	{
		float dist = matches[k].distance;

		if(dist < min_dist)
		{
			min_dist = dist; //  computing the new minimum distance
			min_k = k;
		}
	}

	updateLoc = search_coords[min_k];

	// after getting the updated location, update the measurement
	setMeasurement(updateLoc);

	//correct location
	correctJointPosition();

	if(displayOn)
		printf("Joint state corrected with the region updated estimate : Location = (%f,%f) ; measure = %f\n",updateLoc.x,updateLoc.y,min_dist);

	//update img and feature vector
	Mat img_crop_update(current_img,Rect(floor(location.x) - winSize/2,floor(location.y) - winSize/2,winSize,winSize));
	this->img = img_crop_update.clone();

	//predict to next frame
	predictJointPosition();

	//Compute Search Region for next instant
	computeSearchRegion();

	return location;
}

Point2f JointDescriptor::track_HOGBasedKalman(Point2f prevLoc, Mat prev_img, Mat current_img, Point2f detect_loc, Mat mask)
{
	// Obtain the mask simulating the search region ; S_reg1 in the current image
	Mat temp_mask = Mat::zeros(current_img.rows,current_img.cols,CV_8UC1);
	ellipse(temp_mask,search_region,Scalar(255,255,255),-1);

	// Compute the descriptor for the previous location
	Mat img_crop_prev(prev_img,Rect(floor(prevLoc.x) - DEFAULT_WINSIZE/2,floor(prevLoc.y) - DEFAULT_WINSIZE/2,DEFAULT_WINSIZE,DEFAULT_WINSIZE));
	vector<float> prev_feature = computeSignature(img_crop_prev);

	Point2f updateLoc;

	// Extract coordinates in the current image belonging to the computer search region S_reg1
	vector<Point2f> search_coords = ExtractCoordsFromImage(current_img,temp_mask,Rect());

	// Compute the descriptors for the search coordinates in S_reg1
	vector<float> c_measures;
	for(int k = 0; k < search_coords.size(); k++)
	{
		// get the point inside the elliptical search region and compute signature at that point
		Point2f interestPt = search_coords[k];

		Mat img_crop(current_img,Rect(floor(interestPt.x) - DEFAULT_WINSIZE/2,floor(interestPt.y) - DEFAULT_WINSIZE/2,DEFAULT_WINSIZE,DEFAULT_WINSIZE));
		vector<float> current_feature = computeSignature(img_crop);

		// match this interest point descriptor with joint descriptor
		float measure;

		bool correctFlag = matchImageDescriptors(prev_feature,current_feature,measure,false);
		c_measures.push_back(measure);
	}

	// Compute the minimum distance measure
	int min_k;
	float min_dist = 1000.0;
	for(int k = 0; k < c_measures.size(); k++)
	{
		float measure = c_measures[k];
		if(measure <= min_dist )
		{
			min_dist = measure;
			min_k = k;
		}
	}

	updateLoc = search_coords[min_k];

	// after getting the updated location, update the measurement
	setMeasurement(updateLoc);

	//correct location
	correctJointPosition();

	if(displayOn)
		std::printf("Joint state corrected with the HOG region estimate : Location = (%f,%f) ; measure = %f\n",updateLoc.x,updateLoc.y,min_dist);

	//update img and feature vector
	Mat img_crop_update(current_img,Rect(floor(location.x) - winSize/2,floor(location.y) - winSize/2,winSize,winSize));
	this->img = img_crop_update.clone();

	//predict to next frame
	predictJointPosition();

	//Compute Search Region for next instant
	computeSearchRegion();

	return location;
}

Point2f JointDescriptor::track_LBPBasedKalman(Point2f prevLoc, Mat prev_img, Mat current_img, Point2f detect_loc, Mat mask)
{
	// Obtain the mask simulating the search region ; S_reg1 in the current image
	Mat temp_mask = Mat::zeros(current_img.rows,current_img.cols,CV_8UC1);
	ellipse(temp_mask,search_region,Scalar(255,255,255),-1);

	// Compute the descriptor for the previous location
	Mat img_crop_prev(prev_img,Rect(floor(prevLoc.x) - DEFAULT_WINSIZE/2,floor(prevLoc.y) - DEFAULT_WINSIZE/2,DEFAULT_WINSIZE,DEFAULT_WINSIZE));
	vector<float> prev_feature = computeSignature(img_crop_prev,false);

	Point2f updateLoc;

	// Extract coordinates in the current image belonging to the computer search region S_reg1
	vector<Point2f> search_coords = ExtractCoordsFromImage(current_img,temp_mask,Rect());

	// Compute the descriptors for the search coordinates in S_reg1
	vector<float> c_measures;
	for(int k = 0; k < search_coords.size(); k++)
	{
		// get the point inside the elliptical search region and compute signature at that point
		Point2f interestPt = search_coords[k];

		Mat img_crop(current_img,Rect(floor(interestPt.x) - DEFAULT_WINSIZE/2,floor(interestPt.y) - DEFAULT_WINSIZE/2,DEFAULT_WINSIZE,DEFAULT_WINSIZE));
		vector<float> current_feature = computeSignature(img_crop,false);

		// match this interest point descriptor with joint descriptor
		float measure;

		bool correctFlag = matchImageDescriptors(prev_feature,current_feature,measure);
		c_measures.push_back(measure);
	}

	// Compute the minimum distance measure
	int min_k;
	float min_dist = 1000.0;
	for(int k = 0; k < c_measures.size(); k++)
	{
		float measure = c_measures[k];
		if(measure <= min_dist )
		{
			min_dist = measure;
			min_k = k;
		}
	}

	updateLoc = search_coords[min_k];

	// after getting the updated location, update the measurement
	setMeasurement(updateLoc);

	//correct location
	correctJointPosition();

	if(displayOn)
		std::printf("Joint state corrected with the HOG region estimate : Location = (%f,%f) ; measure = %f\n",updateLoc.x,updateLoc.y,min_dist);

	//update img and feature vector
	Mat img_crop_update(current_img,Rect(floor(location.x) - winSize/2,floor(location.y) - winSize/2,winSize,winSize));
	this->img = img_crop_update.clone();

	//predict to next frame
	predictJointPosition();

	//Compute Search Region for next instant
	computeSearchRegion();

	return location;
}

// THIS IS THE PROPOSED TRACKING SCHEME
Point2f JointDescriptor::track_proposed(Point2f prevLoc, Mat prev_img, Mat current_img, Point2f detect_loc, Mat mask)
{
	// Find the region based estimate in the search region
	bool scene_1, scene_2a, scene_2b;
	scene_1 = false; scene_2a = false; scene_2b = false;
	float a_s_reg1, a_s_reg2; // area of two regions

	// Obtain the mask simulating the search region ; S_reg1 in the current image
	Mat temp_mask = Mat::zeros(current_img.rows,current_img.cols,CV_8UC1);
	ellipse(temp_mask,search_region,Scalar(255,255,255),-1);

	Point2f updateLoc;

	// Assign the keypoint
	KeyPoint kp = KeyPoint(prevLoc,winSize);
	vector<KeyPoint> kps_prev,kps_current;
	kps_prev.push_back(kp);

	// compute the descriptor in the previous location
	Mat prev_descriptors,curr_descriptors;
	generic_extractor->compute(prev_img,kps_prev,prev_descriptors);

	// Extract coordinates in the current image belonging to the computer search region
	vector<Point2f> search_coords = ExtractCoordsFromImage(current_img,temp_mask,Rect());

	// Convert to KeyPoints
	for(int k = 0 ; k < search_coords.size(); k++)
	{
		KeyPoint kp_temp = KeyPoint(search_coords[k],winSize);
		kps_current.push_back(kp_temp);
	}

	// Compute the descriptors
	generic_extractor->compute(current_img,kps_current,curr_descriptors);

	// Perform descriptor matching and find the minimum location
	vector<DMatch> matches;
	generic_matcher->match(curr_descriptors,prev_descriptors,matches,Mat());

	// obtain and analyze the matches
	float min_dist = 10000;
	int min_k = 0;
	for(int k = 0 ; k < matches.size(); k++)
	{
		float dist = matches[k].distance;

		if(dist < min_dist)
		{
			min_dist = dist; //  computing the new minimum distance
			min_k = k;
		}
	}

	updateLoc = search_coords[min_k];

	// Displaying the measure and location obtained from the first search region
	if(displayOn)
		std::printf("Region based estimate within S_reg1 => Location : [%f,%f] ; Measure : %f \n",updateLoc.x,updateLoc.y,min_dist);

	// Computing the second search region ; S_reg2 again within the current image
	search_region2 = computeSearchRegion2(updateLoc,detect_loc);

	// computing the area of the two regions
	float a_1 = search_region.size.width/2;
	float b_1 = search_region.size.height/2;
	a_s_reg1 = 3.14 * a_1 * b_1;

	float a_2 = search_region2.size.width/2;
	float b_2 = search_region2.size.height/2;
	a_s_reg2 = 3.14 * a_2 * b_2;

	// Entering the specific scenario
	if(a_s_reg2 < a_s_reg1)
		scene_1 = true;
	else if( (a_s_reg2 >= a_s_reg1) && (a_s_reg2 <= ALPHA_P*ALPHA_P*a_s_reg1) )
		scene_2a = true;
	else if( a_s_reg2 > ALPHA_P*ALPHA_P * a_s_reg1 )
		scene_2b = true;

	// Displaying the areas of the computed search region
	if(displayOn)
	{
		std::printf("Area of search region 1 ; S_reg1 = %f\n",a_s_reg1);
		std::printf("Area of search region 2 ; S_reg2 = %f\n",a_s_reg2);
		if(scene_1)
			std::printf("Entering Scenario 1\n");
		else if(scene_2a)
			std::printf("Entering Scenario 2a\n");
		else if(scene_2b)
			std::printf("Entering Scenario 2b\n");
	}

	// Selecting the measurement to correct
	if(scene_1) // use the region based estimate
	{
		// Set measurement
		setMeasurement(updateLoc);

		//correct location
		correctJointPosition();
		if(displayOn)
			std::printf("Joint State Corrected with region-based estimate in Scenario 1\n");

		//update img and feature vector
		Mat img_crop_update(current_img,Rect(floor(location.x) - winSize/2,floor(location.y) - winSize/2,winSize,winSize));
		this->img = img_crop_update.clone();
		//feature_vector = computeSignature(this->img);
	}
	else if(scene_2b)
	{
		// Set measurement
		setMeasurement(detect_loc);

		//correct location
		correctJointPosition();

		if(displayOn)
			std::printf("Joint State Corrected with discrete pose estimate in Scenario 2b\n");

		//update img and feature vector
		Mat img_crop_update(current_img,Rect(floor(location.x) - winSize/2,floor(location.y) - winSize/2,winSize,winSize));
		this->img = img_crop_update.clone();
		//feature_vector = computeSignature(this->img);
	}
	else if(scene_2a)
	{
		// TODO: This is the part of the code which can be replaced for better estimate
		// For now, we just recompute the region based estimate

		// Obtain the mask simulating the search region ; S_reg2 in the current image
		Mat temp_mask2 = Mat::zeros(current_img.rows,current_img.cols,CV_8UC1);
		ellipse(temp_mask2,search_region2,Scalar(255,255,255),-1);

		Point2f updateLoc2;
		Mat curr_descriptors2;

		vector<KeyPoint> kps_current2;

		// Extract coordinates in the current image belonging to the search region ; S_Reg2
		vector<Point2f> search_coords2 = ExtractCoordsFromImage(current_img,temp_mask2,Rect());

		// Convert to KeyPoints
		for(int k = 0 ; k < search_coords2.size(); k++)
		{
			KeyPoint kp_temp = KeyPoint(search_coords2[k],winSize);
			kps_current2.push_back(kp_temp);
		}

		// Compute the descriptors
		generic_extractor->compute(current_img,kps_current2,curr_descriptors2);

		// Perform descriptor matching and find the minimum location
		vector<DMatch> matches2;
		generic_matcher->match(curr_descriptors2,prev_descriptors,matches2,Mat());

		// obtain and analyze the matches :  only considering the minimum distance measure
		float min_dist2 = 10000;
		int min_k2 = 0;
		for(int k = 0 ; k < matches2.size(); k++)
		{
			float dist = matches2[k].distance;

			if(dist < min_dist2)
			{
				min_dist2 = dist; //  computing the new minimum distance
				min_k2 = k;
			}
		}

		updateLoc2 = search_coords2[min_k2];

		// Displaying the measure and location obtained from the first search region
		if(displayOn)
			std::printf("Region based estimate within S_reg2 => Location : [%f,%f] ; Measure : %f \n",updateLoc2.x,updateLoc2.y,min_dist2);

		// Set measurement
		setMeasurement(detect_loc);

		//correct location
		correctJointPosition();

		if(displayOn)
			std::printf("Joint State Corrected with second region based estimate in Scenario 2a\n");

		//update img and feature vector
		Mat img_crop_update(current_img,Rect(floor(location.x) - winSize/2,floor(location.y) - winSize/2,winSize,winSize));
		this->img = img_crop_update.clone();
	}

	//predict to next frame
	predictJointPosition();

	//Compute Search Region for next instant
	computeSearchRegion();

	return location;

}

Point2f JointDescriptor::track_proposed_withHOG(Point2f prevLoc, Mat prev_img, Mat current_img, Point2f detect_loc, Mat mask)
{
	// Find the region based estimate in the search region
	bool scene_1, scene_2a, scene_2b;
	scene_1 = false; scene_2a = false; scene_2b = false;
	float a_s_reg1, a_s_reg2; // area of two regions

	// Obtain the mask simulating the search region ; S_reg1 in the current image
	Mat temp_mask = Mat::zeros(current_img.rows,current_img.cols,CV_8UC1);
	ellipse(temp_mask,search_region,Scalar(255,255,255),-1);

	// Compute the descriptor for the previous location
	Mat img_crop_prev(prev_img,Rect(floor(prevLoc.x) - DEFAULT_WINSIZE/2,floor(prevLoc.y) - DEFAULT_WINSIZE/2,DEFAULT_WINSIZE,DEFAULT_WINSIZE));
	vector<float> prev_feature = computeSignature(img_crop_prev);

	Point2f updateLoc;

	// Extract coordinates in the current image belonging to the computer search region S_reg1
	vector<Point2f> search_coords = ExtractCoordsFromImage(current_img,temp_mask,Rect());

	// Compute the descriptors for the search coordinates in S_reg1
	vector<float> c_measures;
	for(int k = 0; k < search_coords.size(); k++)
	{
		// get the point inside the elliptical search region and compute signature at that point
		Point2f interestPt = search_coords[k];

		Mat img_crop(current_img,Rect(floor(interestPt.x) - DEFAULT_WINSIZE/2,floor(interestPt.y) - DEFAULT_WINSIZE/2,DEFAULT_WINSIZE,DEFAULT_WINSIZE));
		vector<float> current_feature = computeSignature(img_crop);

		// match this interest point descriptor with joint descriptor
		float measure;

		bool correctFlag = matchImageDescriptors(prev_feature,current_feature,measure,false);
		c_measures.push_back(measure);
	}

	// Compute the minimum distance measure
	int min_k;
	float min_dist = 1000.0;
	for(int k = 0; k < c_measures.size(); k++)
	{
		float measure = c_measures[k];
		if(measure <= min_dist )
		{
			min_dist = measure;
			min_k = k;
		}
	}

	updateLoc = search_coords[min_k];

	// Displaying the measure and location obtained from the first search region
	if(displayOn)
		std::printf("HOG based estimate within S_reg1 => Location : [%f,%f] ; Measure : %f \n",updateLoc.x,updateLoc.y,min_dist);

	// Computing the second search region ; S_reg2 again within the current image
	search_region2 = computeSearchRegion2(updateLoc,detect_loc);

	// computing the area of the two regions
	float a_1 = search_region.size.width/2;
	float b_1 = search_region.size.height/2;
	a_s_reg1 = 3.14 * a_1 * b_1;

	float a_2 = search_region2.size.width/2;
	float b_2 = search_region2.size.height/2;
	a_s_reg2 = 3.14 * a_2 * b_2;

	// Entering the specific scenario
	if(a_s_reg2 < a_s_reg1)
		scene_1 = true;
	else if( (a_s_reg2 >= a_s_reg1) && (a_s_reg2 <= ALPHA_P*ALPHA_P*a_s_reg1) )
		scene_2a = true;
	else if( a_s_reg2 > ALPHA_P*ALPHA_P * a_s_reg1 )
		scene_2b = true;

	// Displaying the areas of the computed search region
	if(displayOn)
	{
		std::printf("Area of search region 1 ; S_reg1 = %f\n",a_s_reg1);
		std::printf("Area of search region 2 ; S_reg2 = %f\n",a_s_reg2);
		if(scene_1)
			std::printf("Entering Scenario 1\n");
		else if(scene_2a)
			std::printf("Entering Scenario 2a\n");
		else if(scene_2b)
			std::printf("Entering Scenario 2b\n");
	}

	// Selecting the measurement to correct
	if(scene_1) // use the region based estimate
	{
		// Set measurement
		setMeasurement(updateLoc);

		//correct location
		correctJointPosition();
		if(displayOn)
			std::printf("Joint State Corrected with HOG-based estimate in Scenario 1\n");

		//update img and feature vector
		Mat img_crop_update(current_img,Rect(floor(location.x) - winSize/2,floor(location.y) - winSize/2,winSize,winSize));
		this->img = img_crop_update.clone();
		//feature_vector = computeSignature(this->img);
	}
	else if(scene_2b)
	{
		// Set measurement
		setMeasurement(detect_loc);

		//correct location
		correctJointPosition();

		if(displayOn)
			std::printf("Joint State Corrected with discrete pose estimate in Scenario 2b\n");

		//update img and feature vector
		Mat img_crop_update(current_img,Rect(floor(location.x) - winSize/2,floor(location.y) - winSize/2,winSize,winSize));
		this->img = img_crop_update.clone();
		//feature_vector = computeSignature(this->img);
	}
	else if(scene_2a)
	{
		// TODO: This is the part of the code which can be replaced for better estimate
		// For now, we just recompute the region based estimate

		// Obtain the mask simulating the search region ; S_reg2 in the current image
		Mat temp_mask2 = Mat::zeros(current_img.rows,current_img.cols,CV_8UC1);
		ellipse(temp_mask2,search_region2,Scalar(255,255,255),-1);

		Point2f updateLoc2;

		// Extract coordinates in the current image belonging to the search region ; S_Reg2
		vector<Point2f> search_coords2 = ExtractCoordsFromImage(current_img,temp_mask2,Rect());

		// Compute the descriptors for the search coordinates in S_reg2
		vector<float> c_measures2;
		for(int k = 0; k < search_coords2.size(); k++)
		{
			// get the point inside the elliptical search region and compute signature at that point
			Point2f interestPt = search_coords2[k];

			Mat img_crop(current_img,Rect(floor(interestPt.x) - DEFAULT_WINSIZE/2,floor(interestPt.y) - DEFAULT_WINSIZE/2,DEFAULT_WINSIZE,DEFAULT_WINSIZE));
			vector<float> current_feature = computeSignature(img_crop);

			// match this interest point descriptor with joint descriptor
			float measure;

			bool correctFlag = matchImageDescriptors(prev_feature,current_feature,measure,false);
			c_measures2.push_back(measure);
		}

		// Compute the minimum distance measure
		int min_k2;
		float min_dist2 = 1000.0;
		for(int k = 0; k < c_measures2.size(); k++)
		{
			float measure = c_measures2[k];
			if(measure <= min_dist2 )
			{
				min_dist2 = measure;
				min_k2 = k;
			}
		}

		updateLoc2 = search_coords2[min_k2];

		// Displaying the measure and location obtained from the first search region
		if(displayOn)
			std::printf("HOG based estimate within S_reg2 => Location : [%f,%f] ; Measure : %f \n",updateLoc2.x,updateLoc2.y,min_dist2);

		// Set measurement
		setMeasurement(updateLoc2);

		//correct location
		correctJointPosition();

		if(displayOn)
			std::printf("Joint State Corrected with second region based estimate in Scenario 2a\n");

		//update img and feature vector
		Mat img_crop_update(current_img,Rect(floor(location.x) - winSize/2,floor(location.y) - winSize/2,winSize,winSize));
		this->img = img_crop_update.clone();
	}

	//predict to next frame
	predictJointPosition();

	//Compute Search Region for next instant
	computeSearchRegion();

	return location;
}

Point2f JointDescriptor::track_proposed_withLBP(Point2f prevLoc, Mat prev_img, Mat current_img, Point2f detect_loc, Mat mask)
{
	// Find the region based estimate in the search region
	bool scene_1, scene_2a, scene_2b;
	scene_1 = false; scene_2a = false; scene_2b = false;
	float a_s_reg1, a_s_reg2; // area of two regions

	// Obtain the mask simulating the search region ; S_reg1 in the current image
	Mat temp_mask = Mat::zeros(current_img.rows,current_img.cols,CV_8UC1);
	ellipse(temp_mask,search_region,Scalar(255,255,255),-1);

	// Compute the descriptor for the previous location
	Mat img_crop_prev(prev_img,Rect(floor(prevLoc.x) - DEFAULT_WINSIZE/2,floor(prevLoc.y) - DEFAULT_WINSIZE/2,DEFAULT_WINSIZE,DEFAULT_WINSIZE));
	vector<float> prev_feature = computeSignature(img_crop_prev,false);

	Point2f updateLoc;

	// Extract coordinates in the current image belonging to the computer search region S_reg1
	vector<Point2f> search_coords = ExtractCoordsFromImage(current_img,temp_mask,Rect());

	// Compute the descriptors for the search coordinates in S_reg1
	vector<float> c_measures;
	for(int k = 0; k < search_coords.size(); k++)
	{
		// get the point inside the elliptical search region and compute signature at that point
		Point2f interestPt = search_coords[k];

		Mat img_crop(current_img,Rect(floor(interestPt.x) - DEFAULT_WINSIZE/2,floor(interestPt.y) - DEFAULT_WINSIZE/2,DEFAULT_WINSIZE,DEFAULT_WINSIZE));
		vector<float> current_feature = computeSignature(img_crop,false);

		// match this interest point descriptor with joint descriptor
		float measure;

		bool correctFlag = matchImageDescriptors(prev_feature,current_feature,measure);
		c_measures.push_back(measure);
	}

	// Compute the minimum distance measure
	int min_k;
	float min_dist = 1000.0;
	for(int k = 0; k < c_measures.size(); k++)
	{
		float measure = c_measures[k];
		if(measure <= min_dist )
		{
			min_dist = measure;
			min_k = k;
		}
	}

	updateLoc = search_coords[min_k];

	// Displaying the measure and location obtained from the first search region
	if(displayOn)
		std::printf("LBP based estimate within S_reg1 => Location : [%f,%f] ; Measure : %f \n",updateLoc.x,updateLoc.y,min_dist);

	// Computing the second search region ; S_reg2 again within the current image
	search_region2 = computeSearchRegion2(updateLoc,detect_loc);

	// computing the area of the two regions
	float a_1 = search_region.size.width/2;
	float b_1 = search_region.size.height/2;
	a_s_reg1 = 3.14 * a_1 * b_1;

	float a_2 = search_region2.size.width/2;
	float b_2 = search_region2.size.height/2;
	a_s_reg2 = 3.14 * a_2 * b_2;

	// Entering the specific scenario
	if(a_s_reg2 < a_s_reg1)
		scene_1 = true;
	else if( (a_s_reg2 >= a_s_reg1) && (a_s_reg2 <= ALPHA_P*ALPHA_P*a_s_reg1) )
		scene_2a = true;
	else if( a_s_reg2 > ALPHA_P*ALPHA_P * a_s_reg1 )
		scene_2b = true;

	// Displaying the areas of the computed search region
	if(displayOn)
	{
		std::printf("Area of search region 1 ; S_reg1 = %f\n",a_s_reg1);
		std::printf("Area of search region 2 ; S_reg2 = %f\n",a_s_reg2);
		if(scene_1)
			std::printf("Entering Scenario 1\n");
		else if(scene_2a)
			std::printf("Entering Scenario 2a\n");
		else if(scene_2b)
			std::printf("Entering Scenario 2b\n");
	}

	// Selecting the measurement to correct
	if(scene_1) // use the region based estimate
	{
		// Set measurement
		setMeasurement(updateLoc);

		//correct location
		correctJointPosition();
		if(displayOn)
			std::printf("Joint State Corrected with region-based estimate in Scenario 1\n");

		//update img and feature vector
		Mat img_crop_update(current_img,Rect(floor(location.x) - winSize/2,floor(location.y) - winSize/2,winSize,winSize));
		this->img = img_crop_update.clone();
		//feature_vector = computeSignature(this->img);
	}
	else if(scene_2b)
	{
		// Set measurement
		setMeasurement(detect_loc);

		//correct location
		correctJointPosition();

		if(displayOn)
			std::printf("Joint State Corrected with discrete pose estimate in Scenario 2b\n");

		//update img and feature vector
		Mat img_crop_update(current_img,Rect(floor(location.x) - winSize/2,floor(location.y) - winSize/2,winSize,winSize));
		this->img = img_crop_update.clone();
		//feature_vector = computeSignature(this->img);
	}
	else if(scene_2a)
	{
		// TODO: This is the part of the code which can be replaced for better estimate
		// For now, we just recompute the region based estimate

		// Obtain the mask simulating the search region ; S_reg2 in the current image
		Mat temp_mask2 = Mat::zeros(current_img.rows,current_img.cols,CV_8UC1);
		ellipse(temp_mask2,search_region2,Scalar(255,255,255),-1);

		Point2f updateLoc2;

		// Extract coordinates in the current image belonging to the search region ; S_Reg2
		vector<Point2f> search_coords2 = ExtractCoordsFromImage(current_img,temp_mask2,Rect());

		// Compute the descriptors for the search coordinates in S_reg2
		vector<float> c_measures2;
		for(int k = 0; k < search_coords2.size(); k++)
		{
			// get the point inside the elliptical search region and compute signature at that point
			Point2f interestPt = search_coords2[k];

			Mat img_crop(current_img,Rect(floor(interestPt.x) - DEFAULT_WINSIZE/2,floor(interestPt.y) - DEFAULT_WINSIZE/2,DEFAULT_WINSIZE,DEFAULT_WINSIZE));
			vector<float> current_feature = computeSignature(img_crop,false);

			// match this interest point descriptor with joint descriptor
			float measure;

			bool correctFlag = matchImageDescriptors(prev_feature,current_feature,measure);
			c_measures2.push_back(measure);
		}

		// Compute the minimum distance measure
		int min_k2;
		float min_dist2 = 1000.0;
		for(int k = 0; k < c_measures2.size(); k++)
		{
			float measure = c_measures2[k];
			if(measure <= min_dist2 )
			{
				min_dist2 = measure;
				min_k2 = k;
			}
		}

		updateLoc2 = search_coords2[min_k2];

		// Displaying the measure and location obtained from the first search region
		if(displayOn)
			std::printf("LBP based estimate within S_reg2 => Location : [%f,%f] ; Measure : %f \n",updateLoc2.x,updateLoc2.y,min_dist2);

		// Set measurement
		setMeasurement(detect_loc);

		//correct location
		correctJointPosition();

		if(displayOn)
			std::printf("Joint State Corrected with second region based estimate in Scenario 2a\n");

		//update img and feature vector
		Mat img_crop_update(current_img,Rect(floor(location.x) - winSize/2,floor(location.y) - winSize/2,winSize,winSize));
		this->img = img_crop_update.clone();
	}

	//predict to next frame
	predictJointPosition();

	//Compute Search Region for next instant
	computeSearchRegion();

	return location;
}

// Method to call the proposed scheme tracker
Point2f JointDescriptor::track_proposed_prob(Point2f prevLoc, Mat prev_img, Mat current_img, Point2f detect_loc, Mat mask)
{
	// Find the region based estimate in the search region
	bool scene_1, scene_2a, scene_2b;
	scene_1 = false; scene_2a = false; scene_2b = false;
	float a_s_reg1, a_s_reg2; // area of two regions

	// Obtain the mask simulating the search region ; S_reg1 in the current image
	Mat temp_mask = Mat::zeros(current_img.rows,current_img.cols,CV_8UC1);
	ellipse(temp_mask,search_region,Scalar(255,255,255),-1);

	Point2f updateLoc;

	// Assign the keypoint
	KeyPoint kp = KeyPoint(prevLoc,winSize);
	KeyPoint kp_det = KeyPoint(detect_loc,winSize);

	vector<KeyPoint> kps_prev,kps_current, kps_detect;
	kps_prev.push_back(kp);
	kps_detect.push_back(kp_det);

	// compute the descriptor in the previous location and the detected location
	Mat prev_descriptors,curr_descriptors,det_descriptors;
	generic_extractor->compute(prev_img,kps_prev,prev_descriptors);
	generic_extractor->compute(prev_img,kps_detect,det_descriptors);

	// Extract coordinates in the current image belonging to the computer search region
	vector<Point2f> search_coords = ExtractCoordsFromImage(current_img,temp_mask,Rect());

	// Convert to KeyPoints
	for(int k = 0 ; k < search_coords.size(); k++)
	{
		KeyPoint kp_temp = KeyPoint(search_coords[k],winSize);
		kps_current.push_back(kp_temp);
	}

	// Compute the descriptors
	generic_extractor->compute(current_img,kps_current,curr_descriptors);

	// Perform descriptor matching and find the minimum location
	vector<DMatch> matches,match_detect;
	generic_matcher->match(curr_descriptors,prev_descriptors,matches,Mat());
	generic_matcher->match(det_descriptors,prev_descriptors,match_detect,Mat());

	// obtain and analyze the matches
	float min_dist = 10000;
	int min_k = 0;
	for(int k = 0 ; k < matches.size(); k++)
	{
		float dist = matches[k].distance;

		if(dist < min_dist)
		{
			min_dist = dist; //  computing the new minimum distance
			min_k = k;
		}
	}

	updateLoc = search_coords[min_k];

	// Displaying the measure and location obtained from the first search region
	if(displayOn)
		std::printf("Region based estimate within S_reg1 => Location : [%f,%f] ; Measure : %f \n",updateLoc.x,updateLoc.y,min_dist);

	// Computing the second search region ; S_reg2 again within the current image
	search_region2 = computeSearchRegion2(updateLoc,detect_loc);

	// computing the area of the two regions
	float a_1 = search_region.size.width/2;
	float b_1 = search_region.size.height/2;
	a_s_reg1 = 3.14 * a_1 * b_1;

	float a_2 = search_region2.size.width/2;
	float b_2 = search_region2.size.height/2;
	a_s_reg2 = 3.14 * a_2 * b_2;

	// Entering the specific scenario
	if(a_s_reg2 < a_s_reg1)
		scene_1 = true;
	else if( (a_s_reg2 >= a_s_reg1) && (a_s_reg2 <= ALPHA_P*ALPHA_P*a_s_reg1) )
		scene_2a = true;
	else if( a_s_reg2 > ALPHA_P*ALPHA_P * a_s_reg1 )
		scene_2b = true;

	// Displaying the areas of the computed search region
	if(displayOn)
	{
		std::printf("Area of search region 1 ; S_reg1 = %f\n",a_s_reg1);
		std::printf("Area of search region 2 ; S_reg2 = %f\n",a_s_reg2);
		if(scene_1)
			std::printf("Entering Scenario 1\n");
		else if(scene_2a)
			std::printf("Entering Scenario 2a\n");
		else if(scene_2b)
			std::printf("Entering Scenario 2b\n");
	}

	// Selecting the measurement to correct
	if(scene_1) // use the region based estimate
	{
		// Set measurement
		setMeasurement(updateLoc);

		//correct location
		correctJointPosition();
		if(displayOn)
			std::printf("Joint State Corrected with region-based estimate in Scenario 1\n");

		//update img and feature vector
		Mat img_crop_update(current_img,Rect(floor(location.x) - winSize/2,floor(location.y) - winSize/2,winSize,winSize));
		this->img = img_crop_update.clone();
		//feature_vector = computeSignature(this->img);
	}
	else if(scene_2b)
	{
		// Set measurement
		setMeasurement(detect_loc);

		//correct location
		correctJointPosition();

		if(displayOn)
			std::printf("Joint State Corrected with discrete pose estimate in Scenario 2b\n");

		//update img and feature vector
		Mat img_crop_update(current_img,Rect(floor(location.x) - winSize/2,floor(location.y) - winSize/2,winSize,winSize));
		this->img = img_crop_update.clone();
		//feature_vector = computeSignature(this->img);
	}
	else if(scene_2a)
	{
		// TODO: This is the part of the code which can be replaced for better estimate
		// For now, we just recompute the region based estimate

		// Obtain the mask simulating the search region ; S_reg2 in the current image
		Mat temp_mask2 = Mat::zeros(current_img.rows,current_img.cols,CV_8UC1);
		ellipse(temp_mask2,search_region2,Scalar(255,255,255),-1);

		Point2f updateLoc2;
		Mat curr_descriptors2;

		vector<KeyPoint> kps_current2;

		// Extract coordinates in the current image belonging to the search region ; S_Reg2
		vector<Point2f> search_coords2 = ExtractCoordsFromImage(current_img,temp_mask2,Rect());

		// Convert to KeyPoints
		for(int k = 0 ; k < search_coords2.size(); k++)
		{
			float dist_to_detect = (search_coords2[k].x - detect_loc.x)*(search_coords2[k].x - detect_loc.x) + (search_coords2[k].y - detect_loc.y)*(search_coords2[k].y - detect_loc.y);
			if(dist_to_detect <= 10e-4)
				continue;

			KeyPoint kp_temp = KeyPoint(search_coords2[k],winSize);
			kps_current2.push_back(kp_temp);
		}

		// Compute the descriptors
		generic_extractor->compute(current_img,kps_current2,curr_descriptors2);

		// Perform descriptor matching and find the minimum location
		vector<DMatch> matches2;
		generic_matcher->match(curr_descriptors2,prev_descriptors,matches2,Mat());

		// Here is what needs to be changed
		// Instead of using the minimum distance (region based estimate again)
		// Find the posterior distribution of the detected location with respect to each particle in the neighborhood
		// that is p([x_det,y_det_d_det] | x_hat,y_hat, d_hat) where (x_det, y_det, d_det) is the location and distance measure of the detected point
		// x_hat, y_hat, d_hat is the location and distance measure of a sample particle in S_reg2

		// Storing the information as the 3D points
		Point3f p_det;

		p_det.x = detect_loc.x;
		p_det.y = detect_loc.y;
		p_det.z = match_detect[0].distance;

		// obtaining the standard deviation of x and y from the tracker
		delta_x = tracker->errorCovPre.at<float>(0,0);
		delta_y = tracker->errorCovPre.at<float>(1,1);

		// obtaining the standard deviation of the distance measure
		float temp_mean = 0.0;
		float temp_var = 0.0;
		for(int k = 0; k< matches2.size(); k++)
		{
			temp_mean = temp_mean + matches2[k].distance;
		}
		temp_mean = temp_mean / matches2.size();

		for (int k =0 ; k< matches2.size(); k++)
		{
			temp_var = temp_var + (matches2[k].distance - temp_mean)*(matches2[k].distance - temp_mean);
		}
		temp_var = temp_var / matches2.size();
		delta_z = temp_var;

		// computing the posterior probability of a particle with respect to each point
		vector<float> post_prob;
		for(int k = 0; k < matches2.size(); k++)
		{
			// get the point
			Point3f p_temp;
			p_temp.x = kps_current[k].pt.x;
			p_temp.y = kps_current[k].pt.y;
			p_temp.z = matches2[k].distance;

			float dist_val = 0;
			dist_val = dist_val + ((p_temp.x - p_det.x)*(p_temp.x - p_det.x))/(2 * delta_x);
			dist_val = dist_val + ((p_temp.y - p_det.y)*(p_temp.y - p_det.y))/(2 * delta_y);
			dist_val = dist_val + ((p_temp.z - p_det.z)*(p_temp.z - p_det.z))/(2 * delta_z);

			dist_val = std::exp(-1 * dist_val);
			post_prob.push_back(dist_val);
		}

		// obtaining the maximum value of the probability
		float max_prob_dist = 0;
		int max_k2 = 0;
		for(int k = 0 ;  k< post_prob.size(); k++)
		{
			if(post_prob[k] > max_prob_dist)
			{
				max_prob_dist = post_prob[k];
				max_k2 = k;
			}
		}

		updateLoc2 = search_coords2[max_k2];

		// Displaying the measure and location obtained from the first search region
		if(displayOn)
			std::printf("Region based estimate within S_reg2 => Location : [%f,%f] ; Measure : %f \n",updateLoc2.x,updateLoc2.y,matches2[max_k2].distance);

		// Set measurement
		setMeasurement(detect_loc);

		//correct location
		correctJointPosition();

		if(displayOn)
			std::printf("Joint State Corrected with second region based estimate in Scenario 2a\n");

		//update img and feature vector
		Mat img_crop_update(current_img,Rect(floor(location.x) - winSize/2,floor(location.y) - winSize/2,winSize,winSize));
		this->img = img_crop_update.clone();
	}

	//predict to next frame
	predictJointPosition();

	//Compute Search Region for next instant
	computeSearchRegion();

	return location;
}

// Method to call the proposed scheme tracker
Point2f JointDescriptor::track_proposed_withHOG_prob(Point2f prevLoc, Mat prev_img, Mat current_img, Point2f detect_loc, Mat mask)
{
	// Find the region based estimate in the search region
	bool scene_1, scene_2a, scene_2b;
	scene_1 = false; scene_2a = false; scene_2b = false;
	float a_s_reg1, a_s_reg2; // area of two regions

	// Obtain the mask simulating the search region ; S_reg1 in the current image
	Mat temp_mask = Mat::zeros(current_img.rows,current_img.cols,CV_8UC1);
	ellipse(temp_mask,search_region,Scalar(255,255,255),-1);

	// Compute the descriptor for the previous location
	Mat img_crop_prev(prev_img,Rect(floor(prevLoc.x) - DEFAULT_WINSIZE/2,floor(prevLoc.y) - DEFAULT_WINSIZE/2,DEFAULT_WINSIZE,DEFAULT_WINSIZE));
	vector<float> prev_feature = computeSignature(img_crop_prev);

	// Compute the descriptor for the detected location
	Mat img_crop_detect(current_img,Rect(floor(detect_loc.x) - DEFAULT_WINSIZE/2,floor(detect_loc.y) - DEFAULT_WINSIZE/2,DEFAULT_WINSIZE,DEFAULT_WINSIZE));
	vector<float> detect_feature = computeSignature(img_crop_detect);

	float c_measure_detect;
	bool correctFlag = matchImageDescriptors(prev_feature,detect_feature,c_measure_detect,false);

	Point2f updateLoc;

	// Extract coordinates in the current image belonging to the computer search region S_reg1
	vector<Point2f> search_coords = ExtractCoordsFromImage(current_img,temp_mask,Rect());

	// Compute the descriptors for the search coordinates in S_reg1
	vector<float> c_measures;
	for(int k = 0; k < search_coords.size(); k++)
	{
		// get the point inside the elliptical search region and compute signature at that point
		Point2f interestPt = search_coords[k];

		Mat img_crop(current_img,Rect(floor(interestPt.x) - DEFAULT_WINSIZE/2,floor(interestPt.y) - DEFAULT_WINSIZE/2,DEFAULT_WINSIZE,DEFAULT_WINSIZE));
		vector<float> current_feature = computeSignature(img_crop);

		// match this interest point descriptor with joint descriptor
		float measure;

		bool correctFlag = matchImageDescriptors(prev_feature,current_feature,measure,false);
		c_measures.push_back(measure);
	}

	// Compute the minimum distance measure
	int min_k;
	float min_dist = 1000.0;
	for(int k = 0; k < c_measures.size(); k++)
	{
		float measure = c_measures[k];
		if(measure <= min_dist )
		{
			min_dist = measure;
			min_k = k;
		}
	}

	updateLoc = search_coords[min_k];

	// Displaying the measure and location obtained from the first search region
	if(displayOn)
		std::printf("HOG based estimate within S_reg1 => Location : [%f,%f] ; Measure : %f \n",updateLoc.x,updateLoc.y,min_dist);

	// Computing the second search region ; S_reg2 again within the current image
	search_region2 = computeSearchRegion2(updateLoc,detect_loc);

	// computing the area of the two regions
	float a_1 = search_region.size.width/2;
	float b_1 = search_region.size.height/2;
	a_s_reg1 = 3.14 * a_1 * b_1;

	float a_2 = search_region2.size.width/2;
	float b_2 = search_region2.size.height/2;
	a_s_reg2 = 3.14 * a_2 * b_2;

	// Entering the specific scenario
	if(a_s_reg2 < a_s_reg1)
		scene_1 = true;
	else if( (a_s_reg2 >= a_s_reg1) && (a_s_reg2 <= ALPHA_P*ALPHA_P*a_s_reg1) )
		scene_2a = true;
	else if( a_s_reg2 > ALPHA_P*ALPHA_P * a_s_reg1 )
		scene_2b = true;

	// Displaying the areas of the computed search region
	if(displayOn)
	{
		std::printf("Area of search region 1 ; S_reg1 = %f\n",a_s_reg1);
		std::printf("Area of search region 2 ; S_reg2 = %f\n",a_s_reg2);
		if(scene_1)
			std::printf("Entering Scenario 1\n");
		else if(scene_2a)
			std::printf("Entering Scenario 2a\n");
		else if(scene_2b)
			std::printf("Entering Scenario 2b\n");
	}

	// Selecting the measurement to correct
	if(scene_1) // use the region based estimate
	{
		// Set measurement
		setMeasurement(updateLoc);

		//correct location
		correctJointPosition();
		if(displayOn)
			std::printf("Joint State Corrected with HOG-based estimate in Scenario 1\n");

		//update img and feature vector
		Mat img_crop_update(current_img,Rect(floor(location.x) - winSize/2,floor(location.y) - winSize/2,winSize,winSize));
		this->img = img_crop_update.clone();
		//feature_vector = computeSignature(this->img);
	}
	else if(scene_2b)
	{
		// Set measurement
		setMeasurement(detect_loc);

		//correct location
		correctJointPosition();

		if(displayOn)
			std::printf("Joint State Corrected with discrete pose estimate in Scenario 2b\n");

		//update img and feature vector
		Mat img_crop_update(current_img,Rect(floor(location.x) - winSize/2,floor(location.y) - winSize/2,winSize,winSize));
		this->img = img_crop_update.clone();
		//feature_vector = computeSignature(this->img);
	}
	else if(scene_2a)
	{
		// TODO: This is the part of the code which can be replaced for better estimate
		// For now, we just recompute the region based estimate

		// Obtain the mask simulating the search region ; S_reg2 in the current image
		Mat temp_mask2 = Mat::zeros(current_img.rows,current_img.cols,CV_8UC1);
		ellipse(temp_mask2,search_region2,Scalar(255,255,255),-1);

		Point2f updateLoc2;

		// Extract coordinates in the current image belonging to the search region ; S_Reg2
		vector<Point2f> search_coords2 = ExtractCoordsFromImage(current_img,temp_mask2,Rect());
		vector<Point2f> search_coords2_new;

		// Convert to KeyPoints
		for(int k = 0 ; k < search_coords2.size(); k++)
		{
			float dist_to_detect = (search_coords2[k].x - detect_loc.x)*(search_coords2[k].x - detect_loc.x) + (search_coords2[k].y - detect_loc.y)*(search_coords2[k].y - detect_loc.y);
			if(dist_to_detect <= 10e-4)
				continue;

			search_coords2_new.push_back(search_coords2[k]);
		}

		// Compute the descriptors for the search coordinates in S_reg2
		vector<float> c_measures2;
		for(int k = 0; k < search_coords2_new.size(); k++)
		{
			// get the point inside the elliptical search region and compute signature at that point
			Point2f interestPt = search_coords2_new[k];

			Mat img_crop(current_img,Rect(floor(interestPt.x) - DEFAULT_WINSIZE/2,floor(interestPt.y) - DEFAULT_WINSIZE/2,DEFAULT_WINSIZE,DEFAULT_WINSIZE));
			vector<float> current_feature = computeSignature(img_crop);

			// match this interest point descriptor with joint descriptor
			float measure;

			bool correctFlag = matchImageDescriptors(prev_feature,current_feature,measure,false);

			c_measures2.push_back(measure);
		}

		// Storing the information as the 3D points
		Point3f p_det;

		p_det.x = detect_loc.x;
		p_det.y = detect_loc.y;
		p_det.z = c_measure_detect;

		// obtaining the standard deviation of x and y from the tracker
		delta_x = tracker->errorCovPre.at<float>(0,0);
		delta_y = tracker->errorCovPre.at<float>(1,1);

		// obtaining the standard deviation of the distance measure
		float temp_mean = 0.0;
		float temp_var = 0.0;
		for(int k = 0; k< c_measures2.size(); k++)
		{
			temp_mean = temp_mean + c_measures2[k];
		}
		temp_mean = temp_mean / c_measures2.size();

		for (int k =0 ; k< c_measures2.size(); k++)
		{
			temp_var = temp_var + (c_measures2[k] - temp_mean)*(c_measures2[k] - temp_mean);
		}
		temp_var = temp_var / c_measures2.size();
		delta_z = temp_var;

		// computing the posterior probability of a particle with respect to each point
		vector<float> post_prob;
		for(int k = 0; k < c_measures2.size(); k++)
		{
			// get the point
			Point3f p_temp;
			p_temp.x = search_coords2_new[k].x;
			p_temp.y = search_coords2_new[k].y;
			p_temp.z = c_measures2[k];

			float dist_val = 0;
			dist_val = dist_val + ((p_temp.x - p_det.x)*(p_temp.x - p_det.x))/(2 * delta_x);
			dist_val = dist_val + ((p_temp.y - p_det.y)*(p_temp.y - p_det.y))/(2 * delta_y);
			dist_val = dist_val + ((p_temp.z - p_det.z)*(p_temp.z - p_det.z))/(2 * delta_z);

			dist_val = std::exp(-1 * dist_val);
			post_prob.push_back(dist_val);
		}

		// obtaining the maximum value of the probability
		float max_prob_dist = 0;
		int max_k2 = 0;
		for(int k = 0 ;  k< post_prob.size(); k++)
		{
			if(post_prob[k] > max_prob_dist)
			{
				max_prob_dist = post_prob[k];
				max_k2 = k;
			}
		}

		updateLoc2 = search_coords2_new[max_k2];

		// Displaying the measure and location obtained from the first search region
		if(displayOn)
			std::printf("HOG based estimate within S_reg2 => Location : [%f,%f] ; Measure : %f \n",updateLoc2.x,updateLoc2.y,c_measures2[max_k2]);

		// Set measurement
		setMeasurement(updateLoc2);

		//correct location
		correctJointPosition();

		if(displayOn)
			std::printf("Joint State Corrected with second region based estimate in Scenario 2a\n");

		//update img and feature vector
		Mat img_crop_update(current_img,Rect(floor(location.x) - winSize/2,floor(location.y) - winSize/2,winSize,winSize));
		this->img = img_crop_update.clone();
	}

	//predict to next frame
	predictJointPosition();

	//Compute Search Region for next instant
	computeSearchRegion();

	return location;
}

// Method to call the proposed scheme tracker
Point2f JointDescriptor::track_proposed_withLBP_prob(Point2f prevLoc, Mat prev_img, Mat current_img, Point2f detect_loc, Mat mask)
{
	// Find the region based estimate in the search region
	bool scene_1, scene_2a, scene_2b;
	scene_1 = false; scene_2a = false; scene_2b = false;
	float a_s_reg1, a_s_reg2; // area of two regions

	// Obtain the mask simulating the search region ; S_reg1 in the current image
	Mat temp_mask = Mat::zeros(current_img.rows,current_img.cols,CV_8UC1);
	ellipse(temp_mask,search_region,Scalar(255,255,255),-1);

	// Compute the descriptor for the previous location
	Mat img_crop_prev(prev_img,Rect(floor(prevLoc.x) - DEFAULT_WINSIZE/2,floor(prevLoc.y) - DEFAULT_WINSIZE/2,DEFAULT_WINSIZE,DEFAULT_WINSIZE));
	vector<float> prev_feature = computeSignature(img_crop_prev,false);

	// Compute the descriptor for the detected location
	Mat img_crop_detect(current_img,Rect(floor(detect_loc.x) - DEFAULT_WINSIZE/2,floor(detect_loc.y) - DEFAULT_WINSIZE/2,DEFAULT_WINSIZE,DEFAULT_WINSIZE));
	vector<float> detect_feature = computeSignature(img_crop_detect,false);

	float c_measure_detect;
	bool correctFlag = matchImageDescriptors(prev_feature,detect_feature,c_measure_detect);

	Point2f updateLoc;

	// Extract coordinates in the current image belonging to the computer search region S_reg1
	vector<Point2f> search_coords = ExtractCoordsFromImage(current_img,temp_mask,Rect());

	// Compute the descriptors for the search coordinates in S_reg1
	vector<float> c_measures;
	for(int k = 0; k < search_coords.size(); k++)
	{
		// get the point inside the elliptical search region and compute signature at that point
		Point2f interestPt = search_coords[k];

		Mat img_crop(current_img,Rect(floor(interestPt.x) - DEFAULT_WINSIZE/2,floor(interestPt.y) - DEFAULT_WINSIZE/2,DEFAULT_WINSIZE,DEFAULT_WINSIZE));
		vector<float> current_feature = computeSignature(img_crop,false);

		// match this interest point descriptor with joint descriptor
		float measure;

		bool correctFlag = matchImageDescriptors(prev_feature,current_feature,measure);
		c_measures.push_back(measure);
	}

	// Compute the minimum distance measure
	int min_k;
	float min_dist = 1000.0;
	for(int k = 0; k < c_measures.size(); k++)
	{
		float measure = c_measures[k];
		if(measure <= min_dist )
		{
			min_dist = measure;
			min_k = k;
		}
	}

	updateLoc = search_coords[min_k];

	// Displaying the measure and location obtained from the first search region
	if(displayOn)
		std::printf("LBP based estimate within S_reg1 => Location : [%f,%f] ; Measure : %f \n",updateLoc.x,updateLoc.y,min_dist);

	// Computing the second search region ; S_reg2 again within the current image
	search_region2 = computeSearchRegion2(updateLoc,detect_loc);

	// computing the area of the two regions
	float a_1 = search_region.size.width/2;
	float b_1 = search_region.size.height/2;
	a_s_reg1 = 3.14 * a_1 * b_1;

	float a_2 = search_region2.size.width/2;
	float b_2 = search_region2.size.height/2;
	a_s_reg2 = 3.14 * a_2 * b_2;

	// Entering the specific scenario
	if(a_s_reg2 < a_s_reg1)
		scene_1 = true;
	else if( (a_s_reg2 >= a_s_reg1) && (a_s_reg2 <= ALPHA_P*ALPHA_P*a_s_reg1) )
		scene_2a = true;
	else if( a_s_reg2 > ALPHA_P*ALPHA_P * a_s_reg1 )
		scene_2b = true;

	// Displaying the areas of the computed search region
	if(displayOn)
	{
		std::printf("Area of search region 1 ; S_reg1 = %f\n",a_s_reg1);
		std::printf("Area of search region 2 ; S_reg2 = %f\n",a_s_reg2);
		if(scene_1)
			std::printf("Entering Scenario 1\n");
		else if(scene_2a)
			std::printf("Entering Scenario 2a\n");
		else if(scene_2b)
			std::printf("Entering Scenario 2b\n");
	}

	// Selecting the measurement to correct
	if(scene_1) // use the region based estimate
	{
		// Set measurement
		setMeasurement(updateLoc);

		//correct location
		correctJointPosition();
		if(displayOn)
			std::printf("Joint State Corrected with region-based estimate in Scenario 1\n");

		//update img and feature vector
		Mat img_crop_update(current_img,Rect(floor(location.x) - winSize/2,floor(location.y) - winSize/2,winSize,winSize));
		this->img = img_crop_update.clone();
		//feature_vector = computeSignature(this->img);
	}
	else if(scene_2b)
	{
		// Set measurement
		setMeasurement(detect_loc);

		//correct location
		correctJointPosition();

		if(displayOn)
			std::printf("Joint State Corrected with discrete pose estimate in Scenario 2b\n");

		//update img and feature vector
		Mat img_crop_update(current_img,Rect(floor(location.x) - winSize/2,floor(location.y) - winSize/2,winSize,winSize));
		this->img = img_crop_update.clone();
		//feature_vector = computeSignature(this->img);
	}
	else if(scene_2a)
	{
		// TODO: This is the part of the code which can be replaced for better estimate
		// For now, we just recompute the region based estimate

		// Obtain the mask simulating the search region ; S_reg2 in the current image
		Mat temp_mask2 = Mat::zeros(current_img.rows,current_img.cols,CV_8UC1);
		ellipse(temp_mask2,search_region2,Scalar(255,255,255),-1);

		Point2f updateLoc2;

		// Extract coordinates in the current image belonging to the search region ; S_Reg2
		vector<Point2f> search_coords2 = ExtractCoordsFromImage(current_img,temp_mask2,Rect());
		vector<Point2f> search_coords2_new;

		// Convert to KeyPoints
		for(int k = 0 ; k < search_coords2.size(); k++)
		{
			float dist_to_detect = (search_coords2[k].x - detect_loc.x)*(search_coords2[k].x - detect_loc.x) + (search_coords2[k].y - detect_loc.y)*(search_coords2[k].y - detect_loc.y);
			if(dist_to_detect <= 10e-4)
				continue;

			search_coords2_new.push_back(search_coords2[k]);
		}

		// Compute the descriptors for the search coordinates in S_reg2
		vector<float> c_measures2;
		for(int k = 0; k < search_coords2_new.size(); k++)
		{
			// get the point inside the elliptical search region and compute signature at that point
			Point2f interestPt = search_coords2_new[k];

			Mat img_crop(current_img,Rect(floor(interestPt.x) - DEFAULT_WINSIZE/2,floor(interestPt.y) - DEFAULT_WINSIZE/2,DEFAULT_WINSIZE,DEFAULT_WINSIZE));
			vector<float> current_feature = computeSignature(img_crop,false);

			// match this interest point descriptor with joint descriptor
			float measure;

			bool correctFlag = matchImageDescriptors(prev_feature,current_feature,measure);
			c_measures2.push_back(measure);
		}

		// Storing the information as the 3D points
		Point3f p_det;

		p_det.x = detect_loc.x;
		p_det.y = detect_loc.y;
		p_det.z = c_measure_detect;

		// obtaining the standard deviation of x and y from the tracker
		delta_x = tracker->errorCovPre.at<float>(0,0);
		delta_y = tracker->errorCovPre.at<float>(1,1);

		// obtaining the standard deviation of the distance measure
		float temp_mean = 0.0;
		float temp_var = 0.0;
		for(int k = 0; k< c_measures2.size(); k++)
		{
			temp_mean = temp_mean + c_measures2[k];
		}
		temp_mean = temp_mean / c_measures2.size();

		for (int k =0 ; k< c_measures2.size(); k++)
		{
			temp_var = temp_var + (c_measures2[k] - temp_mean)*(c_measures2[k] - temp_mean);
		}
		temp_var = temp_var / c_measures2.size();
		delta_z = temp_var;

		// computing the posterior probability of a particle with respect to each point
		vector<float> post_prob;
		for(int k = 0; k < c_measures2.size(); k++)
		{
			// get the point
			Point3f p_temp;
			p_temp.x = search_coords2_new[k].x;
			p_temp.y = search_coords2_new[k].y;
			p_temp.z = c_measures2[k];

			float dist_val = 0;
			dist_val = dist_val + ((p_temp.x - p_det.x)*(p_temp.x - p_det.x))/(2 * delta_x);
			dist_val = dist_val + ((p_temp.y - p_det.y)*(p_temp.y - p_det.y))/(2 * delta_y);
			dist_val = dist_val + ((p_temp.z - p_det.z)*(p_temp.z - p_det.z))/(2 * delta_z);

			dist_val = std::exp(-1 * dist_val);
			post_prob.push_back(dist_val);
		}

		// obtaining the maximum value of the probability
		float max_prob_dist = 0;
		int max_k2 = 0;
		for(int k = 0 ;  k< post_prob.size(); k++)
		{
			if(post_prob[k] > max_prob_dist)
			{
				max_prob_dist = post_prob[k];
				max_k2 = k;
			}
		}

		updateLoc2 = search_coords2_new[max_k2];

		// Displaying the measure and location obtained from the first search region
		if(displayOn)
			std::printf("LBP based estimate within S_reg2 => Location : [%f,%f] ; Measure : %f \n",updateLoc2.x,updateLoc2.y,c_measures2[max_k2]);


		// Set measurement
		setMeasurement(detect_loc);

		//correct location
		correctJointPosition();

		if(displayOn)
			std::printf("Joint State Corrected with second region based estimate in Scenario 2a\n");

		//update img and feature vector
		Mat img_crop_update(current_img,Rect(floor(location.x) - winSize/2,floor(location.y) - winSize/2,winSize,winSize));
		this->img = img_crop_update.clone();
	}

	//predict to next frame
	predictJointPosition();

	//Compute Search Region for next instant
	computeSearchRegion();

	return location;
}

vector<float> JointDescriptor::computeSignature(Mat imag, bool flag_HOG)
{
	// TODO: COMPUTE HOG DESCRIPTOR
	vector<Mat> imag_grey;
	Mat imag_YCrCb;
	vector<float> feature_hog;
	vector<float> feature_lbp;
	vector<float> feature;

	if(flag_HOG) // if the HOG descriptor is being computed
	{
		if(imag.channels() > 1)
		{
			cvtColor(imag,imag_YCrCb,CV_BGR2YCrCb);
			split(imag_YCrCb,imag_grey);

			IplImage img_c = imag_grey[0];
			//clear the bins and data stored from previous iteration
			desc_hog->data.clear();
			desc_hog->ClearOrientationBins();
			vector<float> feature_hog_0 = desc_hog->ComputeGlobalHOGDescriptor(&img_c);
			//vector<float> feature_hog_0 = desc_hog->ComputeHOGDescriptor(&img_c);

			img_c = imag_grey[1];
			//clear the bins and data stored from previous iteration
			desc_hog->data.clear();
			desc_hog->ClearOrientationBins();
			vector<float> feature_hog_1 = desc_hog->ComputeGlobalHOGDescriptor(&img_c);
			//vector<float> feature_hog_0 = desc_hog->ComputeHOGDescriptor(&img_c);

			img_c = imag_grey[2];
			//clear the bins and data stored from previous iteration
			desc_hog->data.clear();
			desc_hog->ClearOrientationBins();
			vector<float> feature_hog_2 = desc_hog->ComputeGlobalHOGDescriptor(&img_c);
			//vector<float> feature_hog_0 = desc_hog->ComputeHOGDescriptor(&img_c);

			feature_hog.insert(feature_hog.begin(),feature_hog_0.begin(),feature_hog_0.end());
			feature_hog.insert(feature_hog.end(),feature_hog_1.begin(),feature_hog_1.end());
			feature_hog.insert(feature_hog.end(),feature_hog_2.begin(),feature_hog_2.end());
		}
		else
		{
			IplImage img_c = imag;
			//clear the bins and data stored from previous iteration
			desc_hog->data.clear();
			desc_hog->ClearOrientationBins();
			vector<float> feature_hog_0 = desc_hog->ComputeGlobalHOGDescriptor(&img_c);
			feature_hog.insert(feature_hog.begin(),feature_hog_0.begin(),feature_hog_0.end());
		}

		feature.insert(feature.begin(),feature_hog.begin(),feature_hog.end());
	}
	else // if the LBP is being computer
	{
		// TODO: COMPUTE LBP DESCRIPTOR
		imag_grey.clear();
		Mat dest_img;

		if(imag.channels() > 1)
		{
			cvtColor(imag,imag_YCrCb,CV_BGR2YCrCb);
			split(imag_YCrCb,imag_grey);
			feature_lbp = desc_lbp->ComputeLBPImage(imag_grey[0],dest_img);
		}
		else
		{
			feature_lbp = desc_lbp->ComputeLBPImage(imag,dest_img);
			//feature_lbp = desc_lbp->ComputeHistogram(imag);
		}

		feature.insert(feature.begin(),feature_lbp.begin(),feature_lbp.end());
	}

	//vector<float> feature_lbp;
    //vector<float> feature_lbp_0 = desc_lbp->ComputeLBPImage(imag_grey[0],dest_img);
    //vector<float> feature_lbp_1 = desc_lbp->ComputeLBPImage(imag_grey[1],dest_img);
    //vector<float> feature_lbp_2 = desc_lbp->ComputeLBPImage(imag_grey[2],dest_img);

//    feature_lbp.insert(feature_lbp.begin(),feature_lbp_0.begin(),feature_lbp_0.end());
//    feature_lbp.insert(feature_lbp.end(),feature_lbp_1.begin(),feature_lbp_1.end());
//    feature_lbp.insert(feature_lbp.end(),feature_lbp_2.begin(),feature_lbp_2.end());


	// TODO: COMPUTE COLOR DESCRIPTOR
	//vector<float> feature_color = computeColorHistogram(imag);

	// TODO: Combine both rotation invariant LBP and HOG(with 18/36 bins)
	//feature.insert(feature.begin(),feature_lbp.begin(),feature_lbp.end());

	//feature.insert(feature.begin(),feature_hog.begin(),feature_hog.end());

	//feature.insert(feature.end(),feature_hog.begin(),feature_hog.end());

	//feature.insert(feature.end(),feature_color.begin(),feature_color.end());

	//feature.insert(feature.begin(),feature_color.begin(),feature_color.end());

	//plotJointDescriptor(feature);
	return feature;

}

vector<float> JointDescriptor::computeColorHistogram(Mat imag, Mat mask)
{
	Mat img_float = Mat::zeros(Size(imag.cols,imag.rows),CV_32FC3);
	Mat img_cie;
	imag.copyTo(img_float);
	vector<float> c_h;

	//convert toe CIE-LAB colorspace
	//img_float *= 1./255;
	cvtColor(imag,img_cie,CV_BGR2HSV);

	//setting the parameters for computing histograms
	int l_bins = 20;
	int a_bins = 48;
	int b_bins = 48;
	int histSize[] = {a_bins, b_bins};

	float a_ranges[] = {0,256};
	float b_ranges[] = {0,256};
	const float* ranges[] = {a_ranges,b_ranges};

	MatND hist;
	int channels[] = {0,1};

	calcHist(&img_cie,1,channels,mask,hist,2,histSize,ranges,true,false);

	//finding the number of pixels
	int num_pixels = 0;
	if(!mask.empty())
	{
		for(int i = 0 ; i < mask.rows ; i ++)
			for(int j = 0 ; j < mask.cols; j++)
			{
				unsigned char pixel_mask = mask.at<unsigned char>(i,j);
				if(pixel_mask != 0)
					num_pixels++;
			}
	}
	else
	{
		num_pixels = imag.rows * imag.cols;
	}
	double maxVal = 0;
	minMaxLoc(hist,0,&maxVal,0,0);
	//converting the histogram to
	for(int a = 0 ; a < a_bins ; a++)
		for(int b = 0 ; b < b_bins; b++)
		{
			float binVal = (hist.at<float>(a,b))/num_pixels;
			c_h.push_back(binVal);
		}

	//return the histogram
	return c_h;
}

void JointDescriptor::computeSearchRegion()
{
	// extracting the foci of the elliptical search region
	Point2f f1,f2;
	// tracker->statePost gets changed after prediction
	f1.x = state.at<float>(0,0);
	f1.y = state.at<float>(1,0);
	f2.x = tracker->statePre.at<float>(0,0);
	f2.y = tracker->statePre.at<float>(1,0);

	// extracting the variance from the a-priori error covariance
	Point2f sigma2;
	sigma2.x = tracker->errorCovPre.at<float>(0,0);
	sigma2.y = tracker->errorCovPre.at<float>(1,1);

	// computing the center, angle and semi-major,minor axes
	Point2f center;
	center.x = f1.x + (std::abs(f2.x - f1.x)/2);
	center.y = f2.y + (std::abs(f2.y - f1.y)/2);

    // compute the angle
	float ang = std::atan2((f2.y - f1.y),(f2.x - f1.x));
	if(ang < 0)
		ang = 360 + ang;

    // compute the major and minor axis
	float c = std::sqrt((f2.x - f1.x)*(f2.x - f1.x) + (f2.y - f2.y)*(f2.y - f2.y));
	float b = c + std::sqrt(sigma2.x*sigma2.x + sigma2.y*sigma2.y);
	float a = std::sqrt(c*c + b*b);

	//cout << "center = " << center << endl;
	//cout << "c = " << c << " ; b = "<< b << " ; a = " << a << endl;
	search_region = RotatedRect(center,Size2f(2*a,2*b),ang);
	// TODO: The scale factor can depend on how far the person is from the camera: i.e. from the depth image

}

RotatedRect JointDescriptor::computeSearchRegion2(Point2f updateLoc, Point2f detect_loc)
{
	// extracting the foci of the elliptical search region
	Point2f f1,f2;
	// tracker->statePost gets changed after prediction
	f1.x = updateLoc.x;
	f1.y = updateLoc.y;
	f2.x = detect_loc.x;
	f2.y = detect_loc.y;

	// extracting the variance from the a-priori error covariance
	Point2f sigma2;
	sigma2.x = tracker->errorCovPre.at<float>(0,0);
	sigma2.y = tracker->errorCovPre.at<float>(1,1);

	// computing the center, angle and semi-major,minor axes
	Point2f center;
	center.x = f1.x + (std::abs(f2.x - f1.x)/2);
	center.y = f2.y + (std::abs(f2.y - f1.y)/2);

    // compute the angle
	float ang = std::atan2((f2.y - f1.y),(f2.x - f1.x));
	if(ang < 0)
		ang = 360 + ang;

    // compute the major and minor axis
	float c = std::sqrt((f2.x - f1.x)*(f2.x - f1.x) + (f2.y - f2.y)*(f2.y - f2.y));
	float b = c + std::sqrt(sigma2.x*sigma2.x + sigma2.y*sigma2.y);
	float a = std::sqrt(c*c + b*b);

	//cout << "center = " << center << endl;
	//cout << "c = " << c << " ; b = "<< b << " ; a = " << a << endl;
	RotatedRect sh_reg = RotatedRect(center,Size2f(2*a,2*b),ang);
	// TODO: The scale factor can depend on how far the person is from the camera: i.e. from the depth image

	return sh_reg;
}

void JointDescriptor::plotJointDescriptor(vector<float> fl)
{
	unsigned int* pb = new unsigned int[fl.size()];
	for(int i = 0 ; i < fl.size(); i++)
		pb[i] = (unsigned int)fl[i];

//	//plot
//	CvPlot::plot("Feature Vector",pb,fl.size(),3,1,1,1);
//	CvPlot::label("B");
//	char key = waitKey(0);
//	if(key == 32)
//		CvPlot::clear("Feature Vector");

	delete pb;

}

vector<Point2f> JointDescriptor::ExtractCoordsFromImage(Mat& Image, Mat mask, Rect roi_region)
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

// Write Serialization for this class
void JointDescriptor::write(FileStorage& fs) const
{
	// open bracket for class
	fs << "{";
	fs << "dynamParams" << dynamParams;
	fs << "measureParams" << measureParams;
	fs << "controlParams" << controlParams;
	//fs << "KalmanFilter" << tracker;
	fs << "errorCovPre" << tracker->errorCovPre;
	fs << "errorCovPost" << tracker->errorCovPost;
	fs << "KGain" << tracker->gain;

	fs << "}";

}

// Read Serialization for this class
void JointDescriptor::read(const FileNode& node)
{
	dynamParams = (int)node["dynamParams"];
	measureParams = (int)node["measureParams"];
	controlParams = (int)node["controlParams"];

	//node["KalmanFilter"] >> tracker;
	node["errorCovPre"] >> errorCovPre;
	node["errorCovPost"] >> errorCovPost;
	node["KGain"] >> kGain;
}
