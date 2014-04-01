#include "main.h"

#define PAT_SIZE 64//equal to pattern_size variable (see below)
#define NUM_OF_PATTERNS 25// define the number of patterns you want to use
#define MOVEMENT_SPEED 0.5 // define the drone movement speed

char* filename1 = "..\\..\\src\\resource\\1.png";//id=1
char* filename2 = "..\\..\\src\\resource\\2.png";//id=2
char* filename3 = "..\\..\\src\\resource\\3.png";//id=3
char* filename4 = "..\\..\\src\\resource\\4.png";//id=4
char* filename5 = "..\\..\\src\\resource\\5.png";//id=5
char* filename6 = "..\\..\\src\\resource\\6.png";//id=6
char* filename7 = "..\\..\\src\\resource\\7.png";//id=7
char* filename8 = "..\\..\\src\\resource\\8.png";//id=8
char* filename9 = "..\\..\\src\\resource\\9.png";//id=9
char* filename10 = "..\\..\\src\\resource\\10.png";//id=10
char* filename11 = "..\\..\\src\\resource\\11.png";//id=11
char* filename12 = "..\\..\\src\\resource\\12.png";//id=12
char* filename13 = "..\\..\\src\\resource\\13.png";//id=13
char* filename14 = "..\\..\\src\\resource\\14.png";//id=14
char* filename15 = "..\\..\\src\\resource\\15.png";//id=15
char* filename16 = "..\\..\\src\\resource\\16.png";//id=16
char* filename17 = "..\\..\\src\\resource\\17.png";//id=17
char* filename18 = "..\\..\\src\\resource\\18.png";//id=18
char* filename19 = "..\\..\\src\\resource\\19.png";//id=19
char* filename20 = "..\\..\\src\\resource\\20.png";//id=20
char* filename21 = "..\\..\\src\\resource\\21.png";//id=21
char* filename22 = "..\\..\\src\\resource\\22.png";//id=22
char* filename23 = "..\\..\\src\\resource\\23.png";//id=23
char* filename24 = "..\\..\\src\\resource\\24.png";//id=24
char* filename25 = "..\\..\\src\\resource\\25.png";//id=25

// --------------------------------------------------------------------------
// main(Number of arguments, Argument values)
// Description  : This is the entry point of the program.
// Return value : SUCCESS:0  ERROR:-1
// --------------------------------------------------------------------------
int main(int argc, char **argv) {

	// Initialize
	// If drone is not connected, initialise webcam
	//if (!ardrone.open()) {
	//	printf("Drone failed to connect.\n");
	//	isDroneConnected = false;
	//} else isDroneConnected = true;

	// DEBUGGING
	isDroneConnected = false;


	quitProgram = false;
	int patternCount = 0;
	visiblePattern = 0;
	controlling = false;

	// Loading patterns
	LoadPattern(filename1, patternLibrary, patternCount);
#if (NUM_OF_PATTERNS==25)
	LoadPattern(filename2, patternLibrary, patternCount);
	LoadPattern(filename3, patternLibrary, patternCount);
	LoadPattern(filename4, patternLibrary, patternCount);
	LoadPattern(filename5, patternLibrary, patternCount);
	LoadPattern(filename6, patternLibrary, patternCount);
	LoadPattern(filename7, patternLibrary, patternCount);
	LoadPattern(filename8, patternLibrary, patternCount);
	LoadPattern(filename9, patternLibrary, patternCount);
	LoadPattern(filename10, patternLibrary, patternCount);
	LoadPattern(filename11, patternLibrary, patternCount);
	LoadPattern(filename12, patternLibrary, patternCount);
	LoadPattern(filename13, patternLibrary, patternCount);
	LoadPattern(filename14, patternLibrary, patternCount);
	LoadPattern(filename15, patternLibrary, patternCount);
	LoadPattern(filename16, patternLibrary, patternCount);
	LoadPattern(filename17, patternLibrary, patternCount);
	LoadPattern(filename18, patternLibrary, patternCount);
	LoadPattern(filename19, patternLibrary, patternCount);
	LoadPattern(filename20, patternLibrary, patternCount);
	LoadPattern(filename21, patternLibrary, patternCount);
	LoadPattern(filename22, patternLibrary, patternCount);
	LoadPattern(filename23, patternLibrary, patternCount);
	LoadPattern(filename24, patternLibrary, patternCount);
	LoadPattern(filename25, patternLibrary, patternCount);
#endif
	cout << patternCount << " patterns are loaded." << endl;


	// Pattern detector arguments
	int norm_pattern_size = PAT_SIZE;
	double fixed_thresh = 40;
	double adapt_thresh = 5;//non-used with FIXED_THRESHOLD mode
	int adapt_block_size = 45;//non-used with FIXED_THRESHOLD mode
	double confidenceThreshold = 0.35;
	int mode = 2;//1:FIXED_THRESHOLD, 2: ADAPTIVE_THRESHOLD

	// Initialise PatternDetector
	PatternDetector myDetector(fixed_thresh, adapt_thresh, adapt_block_size, confidenceThreshold, norm_pattern_size, mode);

	// Matrix for OpenCV image
	Mat imgMat;

	// Start main loop
	while (1) {

		// check to terminate program
		if (quitProgram) {
			Stop();
			break;
		}

		// OpenCV image
		IplImage *img;

		// Check which image source to feed to OpenCV
		if (isDroneConnected) {
			// Get drone image
			img = ardrone.getImage();
		} else {
			// Capture webcam feed
			CvCapture *webcamCapture = cvCreateCameraCapture(0);
			// Set custom webcam size
			cvSetCaptureProperty(webcamCapture, CV_CAP_PROP_FRAME_WIDTH, 640);
			cvSetCaptureProperty(webcamCapture, CV_CAP_PROP_FRAME_HEIGHT, 480);

			// Get webcam image
			img = cvQueryFrame(webcamCapture);
		}

		// Create image matrix
		Mat imgMat = Mat(img);
		// Timer for pattern detection time
		double tic = (double)cvGetTickCount();

		// Run the detector
		myDetector.detect(imgMat, cameraMatrix, distortions, patternLibrary, detectedPattern);

		// Show the pattern detection time
		double toc = (double)cvGetTickCount();
		double detectionTime = (toc - tic) / ((double)cvGetTickFrequency() * 1000);
		cout << "Detected Patterns: " << detectedPattern.size() << endl;
		cout << "Detection time: " << detectionTime << endl;

		// Some usefull debug printfs
		printf("Battery = %d%%\n", ardrone.getBatteryPercentage());
		printf("Altitude = %d%%\n", ardrone.getAltitude());
		printf("Position = %d%%\n", ardrone.getPosition());


		// Augment the input frame (and print out the properties of pattern if you want)
		if (detectedPattern.size()) {
			for (unsigned int i = 0; i < detectedPattern.size(); i++){
				// Draw a cube over patterns
				detectedPattern.at(i).showPattern();
				detectedPattern.at(i).draw(imgMat, cameraMatrix, distortions);

				// Toggling which pattern is visible
				SetVisiblePattern(detectedPattern[i].id);
			}
		} else visiblePattern = 0; // reset visible pattern

		// Providing key controlls
		KeyControlls();

		// Autonomous drone controll
		KeepGoodAltitude();
		AutoAdjustPosition();

		// Show the OpenCV image in a new window AR.Drone
		imshow("AR.Drone", imgMat);
		// Give HighGUI to process the draw requests
		cvWaitKey(1);

		// Clear pattern for next tick
		detectedPattern.clear();
	}

	// Stop processes before quitting
	Stop();
	return 0;
}




// Movement methods
void PitchBackwards() {
	ardrone.move3D(MOVEMENT_SPEED, 0.0, 0.0, 0.0);
}

void PitchForwards() {
	ardrone.move3D(-MOVEMENT_SPEED, 0.0, 0.0, 0.0);
}

void RollLeft() {
	ardrone.move3D(0.0, -MOVEMENT_SPEED, 0.0, 0.0);
}

void RollRight() {
	ardrone.move3D(0.0, MOVEMENT_SPEED, 0.0, 0.0);
}

void YawCClockwise() {
	ardrone.move3D(0.0, 0.0, 0.0, -MOVEMENT_SPEED);
}

void YawClockwise() {
	ardrone.move3D(0.0, 0.0, 0.0, MOVEMENT_SPEED);
}

void GainAltitude() {
	ardrone.move3D(0.0, 0.0, MOVEMENT_SPEED, 0.0);
}

void LooseAltitude() {
	ardrone.move3D(0.0, 0.0, -MOVEMENT_SPEED, 0.0);
}

void Hover() {
	ardrone.move3D(0.0, 0.0, 0.0, 0.0);
}

void KeyControlls() {
	// Key input
	int key = cvWaitKey(33);

	// Quit if ESC key
	if (key == 0x1b) quitProgram = true;


	// Change camera - C key
	static int mode = 0;
	if (key == 'c') ardrone.setCamera(++mode % 4);

	// Take off / Landing - SPACE key
	if (key == ' ') {
		if (ardrone.onGround()) ardrone.takeoff();
		else                    ardrone.landing();
	}


	// Movement
	// Up arrow
	// Gain altitude
	if (key == 0x260000 || key == 'r' && !IsTooHigh()) GainAltitude();
	// Down arrow
	// Loose altitude
	if (key == 0x280000 || key == 'f' && !IsTooLow()) LooseAltitude();

	// Game controlls - need to be within bounds to opperate
	// Left/right controlls
	if (IsWithinBounds()) {
		// Left arrow
		// Roll left
		if (key == 0x250000) {
			RollLeft();
			controlling = true;
		} else controlling = false;
		// Right arrow
		// Roll right
		if (key == 0x270000) {
			RollRight();
			controlling = true;
		} else controlling = false;
	}

	// Emergency controlls - always available
	// Left arrow | a key
	// Roll left
	if (key == 'a') {
		RollLeft();
		controlling = true;
	} else controlling = false;
	// Right arrow | d key
	// Roll right
	if (key == 'd') {
		RollRight();
		controlling = true;
	} else controlling = false;
	// w key
	// Pitch forwards
	if (key == 'w') {
		PitchForwards();
		controlling = true;
	} else controlling = false;
	// s key
	// Pitch backwards
	if (key == 's') {
		PitchBackwards();
		controlling = true;
	} else controlling = false;
	// q key
	// Yaw c-clockwise
	if (key == 'q') {
		YawCClockwise();
		controlling = true;
	} else controlling = false;
	// e key
	// Yaw clockwise
	if (key == 'e') {
		YawClockwise();
		controlling = true;
	} else controlling = false;

	// Always go back to hovering if no user input
	Hover();
}

// Sets the state of the visible pattern
void SetVisiblePattern(int patterID) {
	std::stringstream s;

	// Only auto-correct if user is not controlling drone
	if (!controlling) {
		switch (patterID) {
		case 1:
			visiblePattern = 1;
			s << "seeing patter 1" << '\n';
			OutputDebugString(s.str().c_str());
			break;
		case 2:
			visiblePattern = 2;
			s << "seeing patter 2" << '\n';
			OutputDebugString(s.str().c_str());
			break;
		case 3:
			visiblePattern = 3;
			s << "seeing patter 3" << '\n';
			OutputDebugString(s.str().c_str());
			break;
		case 4:
			visiblePattern = 4;
			s << "seeing patter 4" << '\n';
			OutputDebugString(s.str().c_str());
			break;
		case 5:
			visiblePattern = 5;
			s << "seeing patter 5" << '\n';
			OutputDebugString(s.str().c_str());
			break;
		case 6:
			visiblePattern = 6;
			s << "seeing patter 6" << '\n';
			OutputDebugString(s.str().c_str());
			break;
		case 7:
			visiblePattern = 7;
			s << "seeing patter 7" << '\n';
			OutputDebugString(s.str().c_str());
			break;
		case 8:
			visiblePattern = 8;
			s << "seeing patter 8" << '\n';
			OutputDebugString(s.str().c_str());
			break;
		case 9:
			visiblePattern = 9;
			s << "seeing patter 9" << '\n';
			OutputDebugString(s.str().c_str());
			break;
		case 10:
			visiblePattern = 10;
			s << "seeing patter 10" << '\n';
			OutputDebugString(s.str().c_str());
			break;
		case 11:
			visiblePattern = 11;
			s << "seeing patter 11" << '\n';
			OutputDebugString(s.str().c_str());
			break;
		case 12:
			visiblePattern = 12;
			s << "seeing patter 12" << '\n';
			OutputDebugString(s.str().c_str());
			break;
		case 13:
			visiblePattern = 13;
			s << "seeing patter 13" << '\n';
			OutputDebugString(s.str().c_str());
			break;
		case 14:
			visiblePattern = 14;
			s << "seeing patter 14" << '\n';
			OutputDebugString(s.str().c_str());
			break;
		case 15:
			visiblePattern = 15;
			s << "seeing patter 15" << '\n';
			OutputDebugString(s.str().c_str());
			break;
		case 16:
			visiblePattern = 16;
			s << "seeing patter 16" << '\n';
			OutputDebugString(s.str().c_str());
			break;
		case 17:
			visiblePattern = 17;
			s << "seeing patter 17" << '\n';
			OutputDebugString(s.str().c_str());
			break;
		case 18:
			visiblePattern = 18;
			s << "seeing patter 18" << '\n';
			OutputDebugString(s.str().c_str());
			break;
		case 19:
			visiblePattern = 19;
			s << "seeing patter 19" << '\n';
			OutputDebugString(s.str().c_str());
			break;
		case 20:
			visiblePattern = 20;
			s << "seeing patter 20" << '\n';
			OutputDebugString(s.str().c_str());
			break;
		case 21:
			visiblePattern = 21;
			s << "seeing patter 21" << '\n';
			OutputDebugString(s.str().c_str());
			break;
		case 22:
			visiblePattern = 22;
			s << "seeing patter 22" << '\n';
			OutputDebugString(s.str().c_str());
			break;
		case 23:
			visiblePattern = 23;
			s << "seeing patter 23" << '\n';
			OutputDebugString(s.str().c_str());
			break;
		case 24:
			visiblePattern = 24;
			s << "seeing patter 24" << '\n';
			OutputDebugString(s.str().c_str());
			break;
		case 25:
			visiblePattern = 25;
			s << "seeing patter 25" << '\n';
			OutputDebugString(s.str().c_str());
			break;
		default:
			break;
		}
	}
}

// Autonomous drone controll
void AutoAdjustPosition() {
	std::stringstream s;

	// Only auto-correct if user is not controlling drone
	if (!controlling) {
		switch (visiblePattern) {

		case 1:
		case 17:
			// If sees pattern 1 or 17 do
			s << "go diagonally backward right " << '\n';
			OutputDebugString(s.str().c_str());
			PitchBackwards();
			RollRight();
			break;

		case 2:
		case 3:
		case 4:
		case 18:
			// If sees pattern 2, 3, 4 or 18 do
			s << "go straight back" << '\n';
			OutputDebugString(s.str().c_str());
			PitchBackwards();
			break;

		case 5:
		case 19:
			// If sees pattern 5 or 19 do
			s << "go diagonally backward left" << '\n';
			OutputDebugString(s.str().c_str());
			PitchBackwards();
			RollLeft();
			break;

		case 6:
		case 7:
		case 8:
		case 20:
			// If sees pattern 6, 7, 8 or 20 do
			s << "go straight left" << '\n';
			OutputDebugString(s.str().c_str());
			RollLeft();
			break;

		case 9:
		case 21:
			// If sees pattern 9 or 21 do
			s << "go diagonally forward left" << '\n';
			OutputDebugString(s.str().c_str());
			PitchForwards();
			RollLeft();
			break;

		case 10:
		case 11:
		case 12:
		case 22:
			// If sees pattern 10, 11, 12 or 22 do
			s << "go straight forward" << '\n';
			OutputDebugString(s.str().c_str());
			PitchForwards();
			break;

		case 13:
		case 23:
			// If sees pattern 13 or 23 do
			s << "go diagonally forward right" << '\n';
			OutputDebugString(s.str().c_str());
			PitchForwards();
			RollRight();
			break;

		case 14:
		case 15:
		case 16:
		case 24:
			// If sees pattern 14, 15, 16 or 24 do
			s << "go straight right" << '\n';
			OutputDebugString(s.str().c_str());
			RollRight();
			break;

		case 25:
			// If sees pattern 25 do
			s << "do nothing - hover" << '\n';
			OutputDebugString(s.str().c_str());
			Hover();
			break;
		default:
			s << "seeing nothing" << '\n';
			OutputDebugString(s.str().c_str());
			Hover();
			break;
		}
	}
}

// Return true if drone altitude is under .5 m
bool IsTooLow() {
	double altitude = ardrone.getAltitude();

	if (altitude < 0.5) return true;
	else return false;
}

// Return true if drone altitude is over 1.5 m
bool IsTooHigh() {
	double altitude = ardrone.getAltitude();

	if (altitude > 1.5) return true;
	else return false;
}

// Autonomous drone altitude controll
void KeepGoodAltitude() {
	// Lower the drone
	if (IsTooHigh()) LooseAltitude();

	// Raise the drone
	if (IsTooLow()) GainAltitude();
}

// Check if coords of the pattern is within bounds
bool IsWithinBounds() {
	int x = 50;
	int y = 50;

	if (false) {
		switch (visiblePattern)
		{

			// Upper left corner
		case 1:
			if (x > 75 || y < 25) return false;
			else return true;
			break;

			// Upper 3
		case 2:
		case 3:
		case 4:
			if (y < 25) return false;
			else return true;

			// Upper right corner
		case 5:
			if (x < 25 || y < 25) return false;
			else return true;
			break;

			// Right 3
		case 6:
		case 7:
		case 8:
			if (x < 25) return false;
			else return true;
			break;

			// Lower right corner
		case 9:
			if (x < 25 || y > 75) return false;
			else return true;
			break;

			// Lower 3
		case 10:
		case 11:
		case 12:
			if (y > 75) return false;
			else return true;
			break;

			// Lower left corner
		case 13:
			if (x > 75 || y > 75) return false;
			else return true;
			break;

			// Left 3
		case 14:
		case 15:
		case 16:
			if (x > 75) return false;
			else return true;
			break;

		default:
			break;
		}
	}

	// If not patterns are detected, consider drone is within bounds.
	return true;
}

// Stop processes
void Stop() {
	cvReleaseCapture(&webcamCapture);
	// See you
	ardrone.close();
}




// Loads pattern images for detection
int LoadPattern(const char* filename, std::vector<cv::Mat>& library, int& patternCount) {
	Mat img = imread(filename, 0);

	if (img.cols != img.rows){
		return -1;
		printf("Not a square pattern");
	}

	int msize = PAT_SIZE;

	Mat src(msize, msize, CV_8UC1);
	Point2f center((msize - 1) / 2.0f, (msize - 1) / 2.0f);
	Mat rot_mat(2, 3, CV_32F);

	resize(img, src, Size(msize, msize));
	Mat subImg = src(Range(msize / 4, 3 * msize / 4), Range(msize / 4, 3 * msize / 4));
	library.push_back(subImg);

	rot_mat = getRotationMatrix2D(center, 90, 1.0);

	for (int i = 1; i<4; i++){
		Mat dst = Mat(msize, msize, CV_8UC1);
		rot_mat = getRotationMatrix2D(center, -i * 90, 1.0);
		warpAffine(src, dst, rot_mat, Size(msize, msize));
		Mat subImg = dst(Range(msize / 4, 3 * msize / 4), Range(msize / 4, 3 * msize / 4));
		library.push_back(subImg);
	}

	patternCount++;
	return 1;
}