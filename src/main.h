#include "ardrone/ardrone.h"
#include <vector>

#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "cameraparams.h"
#include "patterndetector.h"

using namespace std;
using namespace cv;
using namespace ARma;

typedef std::vector <std::vector<int>> colorsMatrix;

// AR.Drone class
ARDrone ardrone;

static int loadPattern(const char*, std::vector<cv::Mat>&, int&);
static void DoIfSees(int patterID);

// movement
static void PitchForwards();
static void PitchBackwards();
static void RollLeft();
static void RollRight();
static void YawCClockwise();
static void YawClockwise();
static void GainAltitude();
static void LooseAltitude();

static void KeyControlls();

bool IsTooLow();
bool IsTooHigh();
static void KeepGoodAltitude();


std::vector<cv::Mat> patternLibrary;
std::vector<Pattern> detectedPattern;

bool quitProgram;

























//CvCapture* webcam = cvCaptureFromCAM(CV_CAP_ANY);

//// Number of colors
//unsigned int nColors = 6;
//std::vector<CvKalman> vKalman;
//std::vector<IplImage> binalizedImages;

//int blackHSV[3][2];
//int redHSV[3][2];
//int blueHSV[3][2];
//int yellowHSV[3][2];
//int orangeHSV[3][2];
//int violetHSV[3][2];
//int greenHSV[3][2];

//for (unsigned int i = 0; i < nColors; i++) {
//	// Kalman filters
//	CvKalman *kalman = cvCreateKalman(4, 2);

//	// Setup
//	cvSetIdentity(kalman->measurement_matrix, cvRealScalar(1.0));
//	cvSetIdentity(kalman->process_noise_cov, cvRealScalar(1e-5));
//	cvSetIdentity(kalman->measurement_noise_cov, cvRealScalar(0.1));
//	cvSetIdentity(kalman->error_cov_post, cvRealScalar(1.0));

//	// Linear system
//	kalman->DynamMatr[0] = 1.0; kalman->DynamMatr[1] = 0.0; kalman->DynamMatr[2] = 1.0; kalman->DynamMatr[3] = 0.0;
//	kalman->DynamMatr[4] = 0.0; kalman->DynamMatr[5] = 1.0; kalman->DynamMatr[6] = 0.0; kalman->DynamMatr[7] = 1.0;
//	kalman->DynamMatr[8] = 0.0; kalman->DynamMatr[9] = 0.0; kalman->DynamMatr[10] = 1.0; kalman->DynamMatr[11] = 0.0;
//	kalman->DynamMatr[12] = 0.0; kalman->DynamMatr[13] = 0.0; kalman->DynamMatr[14] = 0.0; kalman->DynamMatr[15] = 1.0;

//	vKalman.push_back(*kalman);


//	// Color thresholds
//	// black
//	switch (i) {
//	case 0:
//		blackHSV[0][0] = 0;  // minH
//		blackHSV[0][1] = 25; // maxH
//		blackHSV[1][0] = 0;  // minS
//		blackHSV[1][1] = 25; // maxS
//		blackHSV[2][0] = 0;  // minV
//		blackHSV[2][1] = 25; // maxV
//		break;

//	// primary colors
//	// red
//	case 1:
//		redHSV[0][0] = 0;  // minH
//		redHSV[0][1] = 25; // maxH
//		redHSV[1][0] = 0;  // minS
//		redHSV[1][1] = 25; // maxS
//		redHSV[2][0] = 0;  // minV
//		redHSV[2][1] = 25; // maxV
//		break;

//	// blue
//	case 2:
//		blueHSV[0][0] = 0;  // minH
//		blueHSV[0][1] = 25; // maxH
//		blueHSV[1][0] = 0;  // minS
//		blueHSV[1][1] = 25; // maxS
//		blueHSV[2][0] = 0;  // minV
//		blueHSV[2][1] = 25; // maxV
//		break;

//	// yellow
//	case 3:
//		yellowHSV[0][0] = 0;  // minH
//		yellowHSV[0][1] = 25; // maxH
//		yellowHSV[1][0] = 0;  // minS
//		yellowHSV[1][1] = 25; // maxS
//		yellowHSV[2][0] = 0;  // minV
//		yellowHSV[2][1] = 25; // maxV
//		break;

//	// secondary colors
//	// orange
//	case 4:
//		orangeHSV[0][0] = 0;  // minH
//		orangeHSV[0][1] = 25; // maxH
//		orangeHSV[1][0] = 0;  // minS
//		orangeHSV[1][1] = 25; // maxS
//		orangeHSV[2][0] = 0;  // minV
//		orangeHSV[2][1] = 25; // maxV
//		break;

//	// violet
//	case 5:
//		violetHSV[0][0] = 0;  // minH
//		violetHSV[0][1] = 25; // maxH
//		violetHSV[1][0] = 0;  // minS
//		violetHSV[1][1] = 25; // maxS
//		violetHSV[2][0] = 0;  // minV
//		violetHSV[2][1] = 25; // maxV
//		break;

//	// green
//	case 6:
//		greenHSV[0][0] = 0;  // minH
//		greenHSV[0][1] = 25; // maxH
//		greenHSV[1][0] = 0;  // minS
//		greenHSV[1][1] = 25; // maxS
//		greenHSV[2][0] = 0;  // minV
//		greenHSV[2][1] = 25; // maxV
//		break;

//	default:
//		break;
//	}
//}

//// +++
//CvKalman *akalman = cvCreateKalman(4, 2);
//CvKalman *bkalman = cvCreateKalman(4, 2);

//// Setup
//cvSetIdentity(akalman->measurement_matrix, cvRealScalar(1.0));
//cvSetIdentity(akalman->process_noise_cov, cvRealScalar(1e-5));
//cvSetIdentity(akalman->measurement_noise_cov, cvRealScalar(0.1));
//cvSetIdentity(akalman->error_cov_post, cvRealScalar(1.0));

//cvSetIdentity(bkalman->measurement_matrix, cvRealScalar(1.0));
//cvSetIdentity(bkalman->process_noise_cov, cvRealScalar(1e-5));
//cvSetIdentity(bkalman->measurement_noise_cov, cvRealScalar(0.1));
//cvSetIdentity(bkalman->error_cov_post, cvRealScalar(1.0));

//// Linear system
//akalman->DynamMatr[0] = 1.0; akalman->DynamMatr[1] = 0.0; akalman->DynamMatr[2] = 1.0; akalman->DynamMatr[3] = 0.0;
//akalman->DynamMatr[4] = 0.0; akalman->DynamMatr[5] = 1.0; akalman->DynamMatr[6] = 0.0; akalman->DynamMatr[7] = 1.0;
//akalman->DynamMatr[8] = 0.0; akalman->DynamMatr[9] = 0.0; akalman->DynamMatr[10] = 1.0; akalman->DynamMatr[11] = 0.0;
//akalman->DynamMatr[12] = 0.0; akalman->DynamMatr[13] = 0.0; akalman->DynamMatr[14] = 0.0; akalman->DynamMatr[15] = 1.0;

//bkalman->DynamMatr[0] = 1.0; bkalman->DynamMatr[1] = 0.0; bkalman->DynamMatr[2] = 1.0; bkalman->DynamMatr[3] = 0.0;
//bkalman->DynamMatr[4] = 0.0; bkalman->DynamMatr[5] = 1.0; bkalman->DynamMatr[6] = 0.0; bkalman->DynamMatr[7] = 1.0;
//bkalman->DynamMatr[8] = 0.0; bkalman->DynamMatr[9] = 0.0; bkalman->DynamMatr[10] = 1.0; bkalman->DynamMatr[11] = 0.0;
//bkalman->DynamMatr[12] = 0.0; bkalman->DynamMatr[13] = 0.0; bkalman->DynamMatr[14] = 0.0; bkalman->DynamMatr[15] = 1.0;

//// Thresholds

//// black
//int minHb = 0, maxHb = 25;
//int minSb = 0, maxSb = 25;
//int minVb = 0, maxVb = 25;

//// blue
//int minHa = 50, maxHa = 90;
//int minSa = 40, maxSa = 60;
//int minVa = 20, maxVa = 50;

////// pink
////int minHb = 60, maxHb = 130;
////int minSb = 40, maxSb = 100;
////int minVb = 140, maxVb = 200;

////// light blue
////int minHa = 130, maxHa = 160;
////int minSa = 70, maxSa = 120;
////int minVa = 70, maxVa = 120;

//// Create windows
//cvNamedWindow("abinalized");
//cvResizeWindow("abinalized", 0, 0);

//cvNamedWindow("bbinalized");
//cvResizeWindow("bbinalized", 0, 0);

//cvNamedWindow("hsv");
//cvCreateTrackbar("H max", "hsv", &maxHa, 255);
//cvCreateTrackbar("H min", "hsv", &minHa, 255);
//cvCreateTrackbar("S max", "hsv", &maxSa, 255);
//cvCreateTrackbar("S min", "hsv", &minSa, 255);
//cvCreateTrackbar("V max", "hsv", &maxVa, 255);
//cvCreateTrackbar("V min", "hsv", &minVa, 255);


//// Main loop
//while (1) {
//	// Key input
//	int key = cvWaitKey(1);
//	if (key == 0x1b) break;

//	// Update
//	//if (!ardrone.update()) break;

//	// Get an image
//	//IplImage *image = ardrone.getImage();
//	IplImage *image = cvQueryFrame(webcam);

//	// HSV image
//	IplImage *hsv = cvCloneImage(image);
//	cvCvtColor(image, hsv, CV_RGB2HSV_FULL);



//	for (unsigned int i = 0; i < nColors; i++) {
//		// Binalized images
//		IplImage *binalized = cvCreateImage(cvGetSize(image), IPL_DEPTH_8U, 1);
//		binalizedImages.push_back(*binalized);

//		// Binalize
//		CvScalar lower = cvScalar(minHa, minSa, minVa);
//		CvScalar upper = cvScalar(maxHa, maxSa, maxVa);
//		
//		switch (i) {
//		case 0:
//			blackHSV[0][0] = 0;  // minH
//			blackHSV[0][1] = 25; // maxH
//			blackHSV[1][0] = 0;  // minS
//			blackHSV[1][1] = 25; // maxS
//			blackHSV[2][0] = 0;  // minV
//			blackHSV[2][1] = 25; // maxV
//			break;

//			// primary colors
//			// red
//		case 1:
//			redHSV[0][0] = 0;  // minH
//			redHSV[0][1] = 25; // maxH
//			redHSV[1][0] = 0;  // minS
//			redHSV[1][1] = 25; // maxS
//			redHSV[2][0] = 0;  // minV
//			redHSV[2][1] = 25; // maxV
//			break;

//			// blue
//		case 2:
//			blueHSV[0][0] = 0;  // minH
//			blueHSV[0][1] = 25; // maxH
//			blueHSV[1][0] = 0;  // minS
//			blueHSV[1][1] = 25; // maxS
//			blueHSV[2][0] = 0;  // minV
//			blueHSV[2][1] = 25; // maxV
//			break;

//			// yellow
//		case 3:
//			yellowHSV[0][0] = 0;  // minH
//			yellowHSV[0][1] = 25; // maxH
//			yellowHSV[1][0] = 0;  // minS
//			yellowHSV[1][1] = 25; // maxS
//			yellowHSV[2][0] = 0;  // minV
//			yellowHSV[2][1] = 25; // maxV
//			break;

//			// secondary colors
//			// orange
//		case 4:
//			orangeHSV[0][0] = 0;  // minH
//			orangeHSV[0][1] = 25; // maxH
//			orangeHSV[1][0] = 0;  // minS
//			orangeHSV[1][1] = 25; // maxS
//			orangeHSV[2][0] = 0;  // minV
//			orangeHSV[2][1] = 25; // maxV
//			break;

//			// violet
//		case 5:
//			violetHSV[0][0] = 0;  // minH
//			violetHSV[0][1] = 25; // maxH
//			violetHSV[1][0] = 0;  // minS
//			violetHSV[1][1] = 25; // maxS
//			violetHSV[2][0] = 0;  // minV
//			violetHSV[2][1] = 25; // maxV
//			break;

//			// green
//		case 6:
//			greenHSV[0][0] = 0;  // minH
//			greenHSV[0][1] = 25; // maxH
//			greenHSV[1][0] = 0;  // minS
//			greenHSV[1][1] = 25; // maxS
//			greenHSV[2][0] = 0;  // minV
//			greenHSV[2][1] = 25; // maxV
//			break;

//		default:
//			break;
//		}

//		cvInRangeS(image, lower, upper, binalized);
//	}

//	IplImage *abinalized = cvCreateImage(cvGetSize(image), IPL_DEPTH_8U, 1);
//	IplImage *bbinalized = cvCreateImage(cvGetSize(image), IPL_DEPTH_8U, 1);

//	// Binalize
//	CvScalar alower = cvScalar(minHa, minSa, minVa);
//	CvScalar aupper = cvScalar(maxHa, maxSa, maxVa);

//	CvScalar blower = cvScalar(minHb, minSb, minVb);
//	CvScalar bupper = cvScalar(maxHb, maxSb, maxVb);

//	cvInRangeS(image, alower, aupper, abinalized);
//	cvInRangeS(image, blower, bupper, bbinalized);

//	// Show result
//	cvShowImage("abinalized", abinalized);
//	cvShowImage("bbinalized", bbinalized);

//	// De-noising
//	cvMorphologyEx(abinalized, abinalized, NULL, NULL, CV_MOP_CLOSE);
//	cvMorphologyEx(bbinalized, bbinalized, NULL, NULL, CV_MOP_CLOSE);

//	// Detect contours
//	CvSeq *acontour = NULL, *amaxContour = NULL;
//	CvSeq *bcontour = NULL, *bmaxContour = NULL;
//	CvMemStorage *contourStorage = cvCreateMemStorage();
//	cvFindContours(abinalized, contourStorage, &acontour, sizeof(CvContour), CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
//	cvFindContours(bbinalized, contourStorage, &bcontour, sizeof(CvContour), CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

//	// Find largest contour
//	double amax_area = 0.0;
//	while (acontour) {
//		double aarea = fabs(cvContourArea(acontour));
//		if (aarea > amax_area) {
//			amaxContour = acontour;
//			amax_area = aarea;
//		}
//		acontour = acontour->h_next;
//	}

//	double bmax_area = 0.0;
//	while (bcontour) {
//		double barea = fabs(cvContourArea(bcontour));
//		if (barea > bmax_area) {
//			bmaxContour = bcontour;
//			bmax_area = barea;
//		}
//		bcontour = bcontour->h_next;
//	}

//	// Object detected
//	//if (maxContour) {
//	if (amaxContour || bmaxContour) {
//		// Draw a contour
//		cvZero(abinalized);
//		cvZero(bbinalized);
//		cvDrawContours(abinalized, amaxContour, cvScalarAll(255), cvScalarAll(255), 0, CV_FILLED);
//		cvDrawContours(bbinalized, bmaxContour, cvScalarAll(255), cvScalarAll(255), 0, CV_FILLED);

//		// Calculate the moments
//		CvMoments amoments;
//		CvMoments bmoments;

//		cvMoments(abinalized, &amoments, 1);
//		cvMoments(bbinalized, &bmoments, 1);

//		int amy = (int)(amoments.m01 / amoments.m00);
//		int amx = (int)(amoments.m10 / amoments.m00);

//		int bmy = (int)(bmoments.m01 / bmoments.m00);
//		int bmx = (int)(bmoments.m10 / bmoments.m00);


//		// Measurements
//		float am[] = { amx, amy };
//		float bm[] = { bmx, bmy };
//		CvMat ameasurement = cvMat(2, 1, CV_32FC1, am);
//		CvMat bmeasurement = cvMat(2, 1, CV_32FC1, bm);

//		// Correct phase
//		const CvMat *acorrection = cvKalmanCorrect(akalman, &ameasurement);
//		const CvMat *bcorrection = cvKalmanCorrect(bkalman, &bmeasurement);
//	}

//	// Prediction phase
//	const CvMat *aprediction = cvKalmanPredict(akalman);
//	const CvMat *bprediction = cvKalmanPredict(bkalman);

//	// Display the image
//	cvCircle(image, cvPointFrom32f(cvPoint2D32f(aprediction->data.fl[0], aprediction->data.fl[1])), 10, CV_RGB(0, 255, 0));
//	cvCircle(image, cvPointFrom32f(cvPoint2D32f(bprediction->data.fl[0], bprediction->data.fl[1])), 10, CV_RGB(0, 255, 0));
//	cvShowImage("camera", image);

//	// Release the memories
//	cvReleaseImage(&hsv);
//	cvReleaseImage(&abinalized);
//	cvReleaseImage(&bbinalized);
//	cvReleaseMemStorage(&contourStorage);
//}

//// Release the kalman filter
//cvReleaseKalman(&akalman);
//cvReleaseKalman(&bkalman);