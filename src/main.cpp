#include "main.h"

#define PAT_SIZE 64 // equal to pattern_size variable (see below)
#define NUM_OF_PATTERNS 25 // define the number of patterns you want to use
#define MOVEMENT_SPEED 0.5 // define the drone movement speed
#define ALTITUDE_SPEED 0.5 // define the drone altitude gain/loose speed
#define RESET_TIMER 2 // define a reset timer for autonomous control
#define WIDTH 640 // define window width 
#define HEIGHT 480 // define window heigth

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

	//// Initialize
	//// If drone is not connected, initialise webcam
	//if (!ardrone.open()) {
	//	printf("Drone failed to connect.\n");
	//	isDroneConnected = false;
	//} else isDroneConnected = true;

	// DEBUGGING
	isDroneConnected = false;


	quitProgram = false;
	int patternCount = 0;
	visiblePattern = 0;
	lastVisiblePattern = 0;
	controlling = false;
	absoluteControl = false;
	vx = 0.0, vy = 0.0, vz = 0.0, vr = 0.0;


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

	// Initialise each pattern timer
	int timer;
	for (int i = 0; i < patternCount; i++) {
		timers.push_back(timer);
	}

	// Matrix for OpenCV image
	Mat imgMat;

	// Start main loop
	while (1) {

		// check to terminate program
		if (quitProgram) break;

		// Update
		if (!ardrone.update()) break;

		// OpenCV image
		IplImage *img;

		// Check which image source to feed to OpenCV
		if (isDroneConnected) {
			// Get drone image
			img = ardrone.getImage();
		} else {
			// Capture webcam feed
			CvCapture* webcamCapture = cvCaptureFromCAM(0);

			// Get webcam image
			img = cvQueryFrame(webcamCapture);
		}



		// Create image matrix
		Mat imgMat = Mat(img);

		// Render the HUD
		Mat result = HUD(imgMat, WIDTH, HEIGHT);

		// Run the detector
		myDetector.detect(imgMat, cameraMatrix, distortions, patternLibrary, detectedPattern);

		// Some usefull info
		cout << "Battery: " << ardrone.getBatteryPercentage() << endl;
		cout << "altitude = " << ardrone.getAltitude() << endl;

		// Augment the input frame (and print out the properties of pattern if you want)
		if (detectedPattern.size()) {
			for (unsigned int i = 0; i < detectedPattern.size(); i++) {
				
				// Start current pattern seen timer
				int now = cvGetTickCount();
				int passedSinceSeen = ((now - timers[i]) / (cvGetTickFrequency() * 1000)) / 1000;

				// Only set visible pattern if detected a pattern for more than 1 second 
				if (passedSinceSeen > 1.0) SetVisiblePattern(detectedPattern[i].id);

				// Draw a cube over patterns
				//detectedPattern.at(i).showPattern();
				detectedPattern.at(i).draw(result, cameraMatrix, distortions);
			}
		} else {
			// reset pattern timers
			for (int i = 0; i < patternCount; i++)
				timers[i] = cvGetTickCount();

			// Reset visible pattern
			visiblePattern = 0;
		}



		// Get elapsed time since last saw pattern
		lastVPElapsed = cvGetTickCount();
		int elapsedTime = ((lastVPElapsed - lastVPStart) / (cvGetTickFrequency() * 1000)) / 1000;

		// Check if reset timer limit passed and reset last visible pattern
		if (elapsedTime < 300)
			if (RESET_TIMER - elapsedTime < 0) lastVisiblePattern = 0;
		


		// Get key input
		int key = cvWaitKey(33);

		// Providing key controls
		KeyControls(key);

		// Drone movement
		ardrone.move3D(vx, vy, vz, vr);


		// Autonomous drone control
		KeepGoodAltitude();
		AutoAdjustPosition();

		// Always go back to hovering if no user input, no pattern visible and too old last visible pattern (0)
		if (!controlling && visiblePattern == 0 && lastVisiblePattern == 0) Hover();
		



		// Create a buffer of the image
		Mat copy;
		result.copyTo(copy);

		// Render circle on buffer
		circle(copy, Point(result.cols / 2, result.rows / 2), 20, Scalar(255, 0, 0), -1);

		// Combine buffer with original image + opacity
		double opacity = 0.2;
		addWeighted(copy, opacity, result, 1 - opacity, 0, result);


		// Initialise window with OpenGL support
		namedWindow("AR.Drone", WINDOW_OPENGL);
		// Set window size 
		resizeWindow("AR.Drone", WIDTH, HEIGHT);
		// Show the OpenCV image in a new window AR.Drone
		imshow("AR.Drone", result);


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
	vx = -MOVEMENT_SPEED;
	s << "pitch back" << '\n';
	OutputDebugString(s.str().c_str());
}

void PitchForwards() {
	vx = MOVEMENT_SPEED;
	s << "pitch front" << '\n';
	OutputDebugString(s.str().c_str());
}

void RollLeft() {
	vy = MOVEMENT_SPEED;
	s << "roll left" << '\n';
	OutputDebugString(s.str().c_str());
}

void RollRight() {
	vy = -MOVEMENT_SPEED;
	s << "roll right" << '\n';
	OutputDebugString(s.str().c_str());
}

void YawCClockwise() {
	vr = MOVEMENT_SPEED;
	s << "yaw cc" << '\n';
	OutputDebugString(s.str().c_str());
}

void YawClockwise() {
	vr = -MOVEMENT_SPEED;
	s << "yaw c" << '\n';
	OutputDebugString(s.str().c_str());
}

void GainAltitude() {
	vz = ALTITUDE_SPEED;
	s << "gain alt" << '\n';
	OutputDebugString(s.str().c_str());
}

void LooseAltitude() {
	vz = -ALTITUDE_SPEED;
	s << "loose alt" << '\n';
	OutputDebugString(s.str().c_str());
}

void Hover() {
	vx = 0.0, vy = 0.0, vz = 0.0, vr = 0.0;
	s << "hover" << '\n';
	OutputDebugString(s.str().c_str());
}

void KeyControls(int key) {

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

	// Emergency stop
	// x key
	if (key == 'x') ardrone.emergency();

	// Reset patterns
	// p key
	if (key == 'p') {
		visiblePattern = 0;
		lastVisiblePattern = 0;
	}

	// Start hovering
	// h key
	if (key == 'h') Hover();

	// Movement
	// Up arrow
	// Gain altitude
	if (!IsTooHigh())
		if (key == 0x260000 || key == 'r') GainAltitude();

	// Down arrow
	// Loose altitude
	if (!IsTooLow())
		if (key == 0x280000 || key == 'f') LooseAltitude();

	// Game controls - need to be within bounds to opperate
	// Left/right controls
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

	// Toggle absolute control
	// o key
	if (key == 'o') {
		if (absoluteControl) absoluteControl = false;
		else absoluteControl = true;
	}

	// Emergency controls
	// Available only in absoulte control mode
	if (absoluteControl) {
		// Left arrow | a key
		// Roll left
		if (key == 'a') {
			RollLeft();
			controlling = true;
		}
		else controlling = false;
		// Right arrow | d key
		// Roll right
		if (key == 'd') {
			RollRight();
			controlling = true;
		}
		else controlling = false;
		// w key
		// Pitch forwards
		if (key == 'w') {
			PitchForwards();
			controlling = true;
		}
		else controlling = false;
		// s key
		// Pitch backwards
		if (key == 's') {
			PitchBackwards();
			controlling = true;
		}
		else controlling = false;
		// q key
		// Yaw c-clockwise
		if (key == 'q') {
			YawCClockwise();
			controlling = true;
		}
		else controlling = false;
		// e key
		// Yaw clockwise
		if (key == 'e') {
			YawClockwise();
			controlling = true;
		}
		else controlling = false;
	}
}

// Sets the state of the visible pattern
void SetVisiblePattern(int patterID) {
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

		// store last visible pattern
		lastVisiblePattern = visiblePattern;

		// Start reset timer since last saw pattern
		lastVPStart = cvGetTickCount();
	}
}

// Autonomous drone control
void AutoAdjustPosition() {
	// Only auto-correct if user is not controlling drone
	if (!controlling) {

		// Switch variable
		int patternSwitch = 0;

		// Check if there is no pattern visible, but there was one store previously
		if (visiblePattern == 0 && lastVisiblePattern != 0) {
			// If so, execute action of last pattern
			patternSwitch = lastVisiblePattern;
		// Else, execute current action
		} else patternSwitch = visiblePattern;

		switch (patternSwitch) {

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
		}

		s << "last visible " << lastVisiblePattern << '\n';
		OutputDebugString(s.str().c_str());
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

// Autonomous drone altitude control
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
int LoadPattern(const char* filename, vector<Mat>& library, int& patternCount) {
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


// Overlay image method
void OverlayImage(const Mat &background, const Mat &foreground, Mat &output, Point2i location)
{
	background.copyTo(output);


	// Start at the row indicated by location, or at row 0 if location.y is negative.
	for (int y = max(location.y, 0); y < background.rows; ++y)
	{
		int fY = y - location.y; // because of the translation

		// Stop if all rows of the foreground image are processed.
		if (fY >= foreground.rows)
			break;

		// Start at the column indicated by location, 
		// Or at column 0 if location.x is negative.
		for (int x = max(location.x, 0); x < background.cols; ++x)
		{
			int fX = x - location.x; // because of the translation.

			// Stop with this row if the column is outside of the foreground image.
			if (fX >= foreground.cols)
				break;

			// Determine the opacity of the foregrond pixel, using its fourth (alpha) channel.
			double opacity = ((double)foreground.data[fY * foreground.step + fX * foreground.channels() + 3]) / 255.;


			// And now combine the background and foreground pixel, using the opacity, 
			// But only if opacity > 0.
			for (int c = 0; opacity > 0 && c < output.channels(); ++c)
			{
				unsigned char foregroundPx = foreground.data[fY * foreground.step + fX * foreground.channels() + c];
				unsigned char backgroundPx = background.data[y * background.step + x * background.channels() + c];
				output.data[y*output.step + output.channels()*x + c] = backgroundPx * (1. - opacity) + foregroundPx * opacity;
			}
		}
	}
}

Mat HUD(Mat videoFeed, int sizex, int sizey) {
	// Read HUD image
	Mat image = imread("..\\..\\src\\resource\\hud.png", -1);

	// Resized HUD image
	Mat rImage;

	// Resize HUD to fit window
	resize(image, rImage, Size(sizex, sizey), 0, 0, INTER_CUBIC);

	// Create a matrix to mix the video feed with the HUD image
	Mat result;

	// Overlay the HUD over the video feed
	OverlayImage(videoFeed, rImage, result, Point(0, 0));


	// Display info on to HUD
	ostringstream str; // string stream

	str << "Absolute control : " << absoluteControl;
	putText(result, str.str(), Point(10, 30), CV_FONT_HERSHEY_PLAIN, 1, CV_RGB(0, 250, 0));

	return result;
}