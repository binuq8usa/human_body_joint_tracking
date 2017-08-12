#include "lbpdescriptor.h"

float findMax(float i, float j)
{
	if(i > j)
		return i;
	else
		return j;
}

float findMin(float i, float j)
{
	if(i < j)
		return i;
	else
		return j;
}

//class definition
//constructor - default //regular lbp - rectangular grid
LBPDescriptor::LBPDescriptor(_maptype m_type)
{
	radius = 1;
	neighbors = 8;
	

	spoints = (Mat_<float>(neighbors,2) << -1, -1, -1, 0, -1, 1, 0, -1, -0, 1, 1, -1, 1, 0, 1, 1 );
//	spoints = Mat::zeros(neighbors,2,CV_32F);
//	for(int i = 0 ; i < neighbors; i++)
//	{
//		float ptr_0 = -1 * radius * sin(2*M_PI*i/neighbors);
//		float ptr_1 =  1 * radius * cos(2*M_PI*i/neighbors);
//
//		spoints.at<float>(i,0) = ptr_0;
//		spoints.at<float>(i,1) = ptr_1;
//
//	}


	//setting the mode
	mode = nh;
	maptype = m_type;
	
	Mat col_0 = spoints.col(0);
	Mat col_1 = spoints.col(1);

	Point min_loc,max_loc;
	minMaxLoc(col_0,&miny,&maxy,&min_loc,&max_loc);
	minMaxLoc(col_1,&minx,&maxx,&min_loc,&max_loc);

	//computing the block size
	maxy = findMax(maxy,0);
	miny = findMin(miny,0);
	maxx = findMax(maxx,0);
	minx = findMin(minx,0);


	bsizey = ceil(maxy) - floor(miny) + 1;
	bsizex = ceil(maxx) - floor(minx) + 1;

	//coordinates of origin in the block
	origy =  -1* floor(miny); // actually its 1-floor() but since c++ follows a zero-based indexing , we omit '1-'
	origx =  -1* floor(minx);

	bins = pow(2,neighbors);

	//compute mapping
	ComputeMapping(neighbors);

}

//destructor
LBPDescriptor::~LBPDescriptor()
{

}

//computation of mapping
void LBPDescriptor::ComputeMapping(int samples)
{
	int newMax = 0;
	int index = 0;
	int orig_num = pow(2,samples);

	if(maptype == n)
	{
		mapping.flag = false;
		return;
	}
	//compare the maptype with 'riu2'
	else if(maptype == u2)
	{
		mapping.flag = true;
		mapping.samples = samples;
		newMax = samples * (samples-1) + 3;
		mapping.num = newMax;
		mapping.table = vector<int>(orig_num,0);

		// Need to generate automatically ..
		for(int i = 0 ; i < orig_num ; i++)
		{
			//rotate left
			unsigned char v = (unsigned char)(i);
			unsigned char j = (v << 1) | (v >> (8-1)); // (v << 1) - shift left by 1 ; (v >> 8) - shift right by 8

			// the number of 1->0 and 0->1 transitions in binary string 'x' is equal to the number of 1-bits in XOR(x,rotateleft(x));
			unsigned char k = (v ^ j);
			bitset<8> val(k);
			int numt = val.count();

			if(numt <=2 )
			{
				mapping.table[i] = index;
				index++;
			}
			else
				mapping.table[i] = newMax - 1;
		}
		return;
	}
	else if(maptype == ri) //rotation invariant
	{
		mapping.flag = true;
		mapping.table = vector<int>(orig_num,0);
		//initializing the temporary map
		vector<int> tmpMap(orig_num,-1);
		for(int i = 0; i < orig_num; i++)
		{
			unsigned char rm = (unsigned char)(i);
			unsigned char r = (unsigned char)(i);
			for(int j = 0; j < samples-1; j++)
			{
				//rotate left
				unsigned char v = (unsigned char)(r);
				r = (v << 1) | (v >> (8-1)); // (v << 1) - shift left by 1 ; (v >> 8) - shift right by 8

				if(r < rm)
					rm = r;
			}
			if(tmpMap[rm] < 0)
			{
				tmpMap[rm] = newMax;
				newMax = newMax + 1;
			}
			mapping.table[i] = tmpMap[rm];

			//cout << "Mapping["<<i<<"] = " <<mapping.table[i] << endl;
		}
		mapping.samples = samples;
		mapping.num = newMax;
		return;
	}
	else if(maptype == riu2) //uniform rotation invariant
	{
		mapping.flag = true;
		mapping.table = vector<int>(orig_num,0);
		newMax = samples + 2;
		for(int i = 0; i < orig_num; i++)
		{
			//rotate left
			unsigned char v = (unsigned char)(i);
			unsigned char j = (v << 1) | (v >> (8-1)); // (v << 1) - shift left by 1 ; (v >> 8) - shift right by 8

			// the number of 1->0 and 0->1 transitions in binary string 'x' is equal to the number of 1-bits in XOR(x,rotateleft(x));
			unsigned char k = (v ^ j);
			bitset<8> val(k);
			int numt = val.count();

			bitset<8> val_i(i);
			if(numt <= 2)
				mapping.table[i] = val_i.count();
			else
				mapping.table[i] = samples + 1;

			//cout << mapping.table[i] << endl;
		}

		mapping.samples = samples;
		mapping.num = newMax;
		return;
	}

}

//computation of LBP
// w_image - weights for histogram computation
vector<float> LBPDescriptor::ComputeLBPImage(Mat source_img,Mat& dest_img,Mat mask, Mat w_image)
{
	//setting the rows and cols
	xsize = source_img.cols;
	ysize = source_img.rows;

	//converting the source image to gray scale
	Mat gray_img,d_dest_img;
	gray_img = source_img.clone();

	//cvtColor(source_img,gray_img,CV_BGR2GRAY);

	if(xsize < bsizex || ysize < bsizey)
		cerr << "Too small input image. SHould be at least (2*radius + 1) x (2*radius+1) " <<endl;

	dx = xsize - bsizex;
	dy = ysize - bsizey;

	//creating the center matrix
	Mat temp = gray_img(Range(origy,origy + dy + 1), Range(origx, origx + dx + 1));
	Mat C = temp.clone();
	Mat d_C,d_image;
	Mat N,D;
	C.convertTo(d_C,CV_32F);
	gray_img.convertTo(d_image,CV_32F);

    d_dest_img = Mat::zeros(dy + 1,dx + 1, CV_32F);

    // Computing the LBP coded Image
    for(int i = 0; i< neighbors; i++)
    {
    	int fy,cy,ry,fx,cx,rx;
    	double w1,w2,w3,w4,ty,tx;

    	float y = spoints.at<float>(i,0) + origy;
    	float x = spoints.at<float>(i,1) + origx;

    	//calculate the floors,ceils and rounds for the x and y
    	fy = floor(y); cy = ceil(y); ry = round(y);
    	fx = floor(x); cx = ceil(x); rx = round(x);

    	//check if interpolation is needed
    	if( (abs(x - rx) < 1e-6) && (abs(y-ry) < 1e-6) )
    	{
    		N = d_image(Range(ry,ry + dy + 1),Range(rx,rx + dx + 1) );
    		compare(N,d_C,D,CMP_GE); //comparing the two arrays N and d_C.
    	}
    	else //this loop has some problems - not sure where it goes wrong but gives wrong values for interpolation
    	{
    		// interpolation needed.. use double image types
    		ty = y - fy;
    		tx = x - fx;

    		//calculate interpolation weights
    		w1 = (1 - tx) * (1 - ty);
    		w2 = tx * (1-ty);
    		w3 = (1 - tx) * ty;
    		w4 = tx * ty;

    		//compute interpolated pixel values
    		//Direct assigment such as Mat W1 = d_image(Range(.....),Range(..)) only copies the header to W1
    		Mat W1,W2,W3,W4;
    		d_image(Range(fy,fy + dy + 1), Range(fx,fx + dx + 1)).copyTo(W1);
    		d_image(Range(fy,fy + dy + 1), Range(cx,cx + dx + 1)).copyTo(W2);
    		d_image(Range(cy,cy + dy + 1), Range(fx,fx + dx + 1)).copyTo(W3);
    		d_image(Range(cy,cy + dy + 1), Range(cx,cx + dx + 1)).copyTo(W4);

    		N = W1 * w1 + W2 * w2 + W3 * w3 + W4 * w4;

    		compare(N,d_C,D,CMP_GE); //comparing the two arrays N and d_C. or D = (N >= d_C)
    	}

    	//update the result matrix
    	int v = pow(2,i);
    	Mat d_D;
    	D.convertTo(d_D,CV_32F);

    	//Value of D is 255
    	double alpha = double(1)/double(255);
    	d_D = d_D * alpha;
    	double min_D,max_D;
    	minMaxLoc(d_D,&min_D,&max_D,NULL,NULL);

    	d_dest_img = d_dest_img + d_D * v;
    }

    //convert it back to integer format
    if(neighbors == 8)
    	d_dest_img.convertTo(dest_img,CV_8U);
    else if(neighbors == 16)
    	d_dest_img.convertTo(dest_img,CV_16U);

    //apply mapping if any
    if(mapping.flag)
    {
    	for(int i = 0 ; i < dest_img.rows; i++)
    		for(int j = 0; j < dest_img.cols ; j++ )
    		{
    			int ind = (int)(dest_img.at<unsigned char>(i,j));
    			int val = mapping.table[ind];
    			dest_img.at<unsigned char>(i,j) = (unsigned char)(val);
    		}
    }

    if( (mode == h) || (mode == nh) )
    {
    	img_hist = vector<float>(mapping.num,0);
    	int num_pixels = 0;
    	//compute the histogram of dest_img
    	for(int i = 0 ; i < dest_img.rows; i++)
    		for(int j = 0 ; j < dest_img.cols; j++)
    		{
    			unsigned char pixel = dest_img.at<unsigned char>(i,j);
    			float w_pixel;
    			//check for weights
    		    if(w_image.empty())
    		    	w_pixel = 1.0;
    		    else
    		    	w_pixel = w_image.at<float>(i,j);

    			if(mask.empty())
    				img_hist[pixel] +=  w_pixel;
    			else
    			{
    				unsigned char pixel_mask = mask.at<unsigned char>(i,j);
    				if(pixel_mask != 0)
    				{
    					img_hist[pixel] +=  w_pixel;
    					num_pixels++;
    				}
    			}
    		}
    	//normalize the histogram
    	if(mode == nh)
    	{
    		for(int i = 0 ; i < mapping.num ; i++)
    		{
    			if(mask.empty())
    				img_hist[i] = img_hist[i]/(dest_img.rows * dest_img.cols);
    			else
    				img_hist[i] = img_hist[i]/num_pixels;
    		}
    	}
    }

    return img_hist;

}

vector<float> LBPDescriptor::ComputeHistogram(Mat dest_img,Mat mask, Mat w_image)
{
	img_hist = vector<float>(256,0); // 255 is the maximum value
	int num_pixels = 0;
	//compute the histogram of dest_img
	for(int i = 0 ; i < dest_img.rows; i++)
		for(int j = 0 ; j < dest_img.cols; j++)
		{
			unsigned char pixel = dest_img.at<unsigned char>(i,j);
			float w_pixel;
			//check for weights
			if(w_image.empty())
				w_pixel = 1.0;
			else
				w_pixel = w_image.at<float>(i,j);

			if(mask.empty())
				img_hist[pixel] +=  w_pixel;
			else
			{
				unsigned char pixel_mask = mask.at<unsigned char>(i,j);
				if(pixel_mask != 0)
				{
					img_hist[pixel] +=  w_pixel;
					num_pixels++;
				}
			}
		}

	for(int i = 0 ; i < mapping.num ; i++)
	{
		if(mask.empty())
			img_hist[i] = img_hist[i]/(dest_img.rows * dest_img.cols);
		else
			img_hist[i] = img_hist[i]/num_pixels;
	}

	return img_hist;
}
