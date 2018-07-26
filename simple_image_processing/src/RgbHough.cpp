#include "RgbHough.h"

using namespace cv;

namespace simple_image_processing {

RgbHough::RgbHough(ros::NodeHandle n, ros::NodeHandle pn)
{
  
  sub_image_ = n.subscribe("image_in", 1, &RgbHough::recvImage, this);
  
  srv_.setCallback(boost::bind(&RgbHough::reconfig, this, _1, _2));

  
  pub_image_ = n.advertise<sensor_msgs::Image>("/flipped_image",1);
  
}

void RgbHough::recvImage(const sensor_msgs::ImageConstPtr& msg)
{
  cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  cv::Mat raw_img = cv_ptr->image;
  
  //cv::Mat src = cv::imread("im.png", CV_LOAD_IMAGE_UNCHANGED);
    double angle = -180;

    // get rotation matrix for rotating the image around its center
    cv::Point2f center(raw_img.cols/2.0, raw_img.rows/2.0);
    cv::Mat rot = cv::getRotationMatrix2D(center, angle, 1.0);
    // determine bounding rectangle
    cv::Rect bbox = cv::RotatedRect(center,raw_img.size(), angle).boundingRect();
    // adjust transformation matrix
    rot.at<double>(0,2) += bbox.width/2.0 - center.x;
    rot.at<double>(1,2) += bbox.height/2.0 - center.y;

    cv::warpAffine(raw_img, raw_img, rot, bbox.size());
  
    
    raw_camera_image_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", raw_img).toImageMsg();
    
    pub_image_.publish(raw_camera_image_msg);
    
  //cv::imshow("Output Image", raw_img);
  //cv::waitKey(1);
  
  
  /***************************************************************************************************************************/
  
  // Detecting yellow lines
//Scalar lowerYellowScalar = Scalar(0, cfg_.lwr_ylw_sclr_g, cfg_.lwr_ylw_sclr_r); //Scalar(0, 50, 50);
//Scalar higherYellowScalar = Scalar(cfg_.hgr_ylw_sclr_b, hgr_ylw_sclr_g, hgr_ylw_sclr_r); 
// Scalar(90, 255, 255);

// Grayscale in Range thresholding
//int grayScaleThresholdLow = cfg_.gray_thresh_low; //150;
//int grayScaleThresholdHigh = 255;

// Gaussian Blur filter size
const int gaussBlurSizeX = 3;
const int gaussBlurSizeY = 3;

// Thresholding params
//const int lowThreshVal = cfg_.lowThreshVal; // 50
//const int targetThreshVal = cfg_.targetThreshVal; // 255;

// Canny Edge thresholding params
//const int cannyLowThreshold = cfg_.canny_thresh_low ; //50;
//const int cannyHighThreshold = cfg_.canny_thresh_high ; //150;

// Hough Transform params
//double rho = cfg_.hough_rho_res;  //1;
//double theta = cfg_.hough_theta_res ; //CV_PI / 180;
//double thresholdHT = cfg_.hough_threshold; //100;
//int maxLengthGapHT = cfg_.hough_max_gap; //100;
//int minLineLengthHT = cfg_.hough_min_length ; //5;

const String procsdWindwName = "Processed Image";
const String origWndwName = "Original Image";



//const Scalar rectColor = Scalar(255);
//const int rectThickness = 1;
const double sigmaX = 0.0;    // std deviation in X
const double sigmaY = 0.0;    // std deviation in Y
const double alpha = 0.55;    // weight of the first array elements
const double beta = 0.45;    // weight of the second array elements
const double gama = 0.0;    // scalar added to each sum

// ROI params
const double topWidthROI = 0.1;        // As a percentage, 0 --> 1
const double bottomWidthROI = 0.9;    // As a percentage, 0 --> 1
const double heightOfROI = 0.22;    // As a percentage, 0 --> 1
const double yBottomStart = 0.72;    // As a percentage, 0 --> 1

const double fontScale = 0.7;

// Resize window for convenience
const int resizeWidth = 960;
const int resizeHeight = 540;


		// required integer params
		int xTopLeft, xBottomLeft, xTopRight, xBottomRight;
		int yTopLeft, yTopRight, yBottomLeft, yBottomRight;
		char loopCount = 0;

		// Start and end times
		clock_t start, end;
		double msecs;

		// Required image holders for processing
		Mat origImg, origImgCpy, origImg2;
		Mat myImgGray;
		Mat histEqualized;
		Mat gaussianBlurred;
		Mat thresholded;
		Mat cannyEdgesOut, cannyEdgesForHT, testMat;
		Mat addweight;
		
		origImg = raw_img.clone();
		
  			// Set ROI params
			xTopLeft = (int)(((double)origImg.cols) * (0.5 - (topWidthROI / 2)));//* 0.375);//
			xBottomLeft = (int)(((double)origImg.cols)* (0.5 - (bottomWidthROI / 2)));//0.05);
			xTopRight = (int)(((double)origImg.cols) * (0.5 + (topWidthROI / 2)));//0.625);//
			xBottomRight = (int)(((double)origImg.cols)* (0.5 + (bottomWidthROI / 2)));    //0.95);

			yBottomLeft = (int)((double)origImg.rows)*yBottomStart;
			yBottomRight = yBottomLeft;
			yTopLeft = (int)(((double)origImg.rows)* (yBottomStart - heightOfROI));
			yTopRight = yTopLeft;

			// Create ROI
			Point polygonPoints[1][4];
			polygonPoints[0][0] = Point(xBottomLeft, yBottomLeft);
			polygonPoints[0][1] = Point(xTopLeft, yTopLeft);
			polygonPoints[0][2] = Point(xTopRight, yTopRight);
			polygonPoints[0][3] = Point(xBottomRight, yBottomRight);

			const Point* ppt[1] = { polygonPoints[0] };
			const int npts = 4;
			const int numPolygons = 1;
			Mat myROIImg(origImg.rows, origImg.cols, CV_8UC3, Scalar(0, 0, 0));
			fillPoly(myROIImg, ppt, &npts, numPolygons, Scalar(255, 255, 255), LINE_8, 0);
			//imshow("After fillPoly", myROIImg); //Works perfect till here

			bitwise_and(origImg, myROIImg, myROIImg);
			//imshow("After bitwise_and & fillPoly", myROIImg); //Works perfect till here

			Rect rec(xBottomLeft, yTopLeft, (xBottomRight - xBottomLeft), (yBottomLeft - yTopLeft));
			rectangle(myROIImg, rec, Scalar(255, 0, 0), 2, 8, 0);
			myROIImg = myROIImg(rec);
			imshow("Polygon Image with cropped rectangle", myROIImg); //Works perfect till here
			cv::waitKey(1);

			// Extract yellow lines by creating mask
			Mat lowerYellow(myROIImg.rows, myROIImg.cols, CV_8UC3, Scalar(0,cfg_.lwr_ylw_sclr_g, cfg_.lwr_ylw_sclr_r));
			Mat higherYellow(myROIImg.rows, myROIImg.cols, CV_8UC3, Scalar(cfg_.hgr_ylw_sclr_b, cfg_.hgr_ylw_sclr_g, cfg_.hgr_ylw_sclr_r));
			Mat maskYellow(myROIImg.rows, myROIImg.cols, CV_8UC3, Scalar(0));
			inRange(myROIImg, lowerYellow, higherYellow, maskYellow);
			//imshow("Extracted yellow", maskYellow);	//Works perfect till here
			//cv::waitKey(1);

			// Convert to grayscale
			cvtColor(myROIImg, myImgGray, COLOR_BGR2GRAY);
			Mat maskWhite(myROIImg.rows, myROIImg.cols, CV_8UC3, Scalar(0));
			inRange(myImgGray, cfg_.gray_thresh_low, cfg_.gray_thresh_high, maskWhite);
			//imshow("Gray scale", maskWhite);	//Works perfect till here
			//cv::waitKey(1);

			Mat maskWhiteYellow(myROIImg.rows, myROIImg.cols, CV_8UC3, Scalar(0));
			bitwise_or(maskWhite, maskYellow, maskWhiteYellow);
			
			
			bitwise_and(myImgGray, maskWhiteYellow, myImgGray);	//bitwise_and
			//imshow("Extracted white and yellow", myImgGray);	// In challenge a lot of extra pixels from road are still in the picture
			//cv::waitKey(1);

			// Histogram equalization
			cv::adaptiveThreshold(myImgGray,histEqualized, 255, cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY_INV, 11, 2);
			//equalizeHist(myImgGray, histEqualized);
			//cv::imshow("Histogram Equalized", histEqualized);	
			//cv::waitKey(1);

			// Add in range here
			//inRange(histEqualized, (grayScaleThresholdHigh*0.66), grayScaleThresholdHigh, histEqualized);

			// Use Gaussian blurr to remove noise
			GaussianBlur(histEqualized, gaussianBlurred, Size(gaussBlurSizeX, gaussBlurSizeY), sigmaX, sigmaY, BORDER_DEFAULT);
			//imshow(procsdWindwName, gaussianBlurred);

			// Thresholding
			threshold(gaussianBlurred, thresholded, cfg_.lowThreshVal, cfg_.targetThreshVal, THRESH_BINARY);
			//imshow("Thresholded", thresholded);		// Doesnt work well with challenge video
			//cv::waitKey(1);
			
			// Canny edge
			Canny(thresholded, cannyEdgesOut, cfg_.canny_thresh_low, cfg_.canny_thresh_high);
			cannyEdgesForHT = cannyEdgesOut.clone();
			imshow("Canny Edges", cannyEdgesForHT);		// Doesnt work well with challenge video
			cv::waitKey(1);

			cvtColor(cannyEdgesOut, cannyEdgesOut, CV_GRAY2BGR);    // convert to RGB space
			testMat = cannyEdgesOut.clone();
			testMat = Scalar(0, 0, 0);

			// Hough Transform
#ifdef houghLines
			vector<Vec2f> linesVector;
			HoughLines(cannyEdgesForHT, linesVector, 1, CV_PI / 180, 100, 0, 0);
			for (size_t i = 0; i < linesVector.size(); i++)
			{
				float rho = linesVector[i][0], theta = linesVector[i][1];
				Point pt1, pt2;
				double a = cos(theta), b = sin(theta);
				double x0 = a*rho, y0 = b*rho;
				pt1.x = cvRound(x0 + 1000 * (-b));
				pt1.y = cvRound(y0 + 1000 * (a));
				pt2.x = cvRound(x0 - 1000 * (-b));
				pt2.y = cvRound(y0 - 1000 * (a));
				line(testMat, pt1, pt2, Scalar(0, 0, 255), 3, CV_AA);
			}
#else // houghLinesP
			
			std::vector<Vec4i> linesVector;
			HoughLinesP(cannyEdgesForHT, linesVector, cfg_.hough_rho_res, cfg_.hough_theta_res,\
					 cfg_.hough_threshold, cfg_.hough_min_length, cfg_.hough_max_gap);
			
			//HoughLines(cannyEdgesForHT, linesVector, CV_HOUGH_PROBABILISTIC, rho, theta, thresholdHT);//, minLineLengthHT, maxLengthGapHT);
			for (size_t i = 0; i < linesVector.size(); i++)
			{
				line(testMat, Point(linesVector[i][0], linesVector[i][1]),
					Point(linesVector[i][2], linesVector[i][3]), Scalar(0, 0, 255), 3, 8);
			}
#endif
			// Check quality of lane detection here
			//imshow("Canny Edges with HT overlay",testMat);

			// Overlay in original image
			Rect WhereRec(xBottomLeft, yTopLeft, myROIImg.cols, myROIImg.rows);    // Create rectangle with overlay coordinates

			addWeighted(origImg(WhereRec), alpha, testMat, beta, gama, addweight);

			addweight.copyTo(origImg(WhereRec));
  
			 //cv::imshow(procsdWindwName, origImg);
			 //cv::waitKey(1);
  
}

void RgbHough::reconfig(simple_image_processing::RgbHoughConfig& config, uint32_t level)
{
  cfg_ = config;
}

}
