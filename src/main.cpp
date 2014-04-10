#include "main.h"

#define PAT_SIZE 64 // equal to pattern_size variable (see below)
#define MOVEMENT_SPEED 0.4 // define the drone self correction movement speed
#define ABSOLUTE_CONTROL_SPEED 0.8 // define drone movement speed for absolute control
#define CONTROL_MOVEMENT_SPEED 0.3 // define dronve movement speed for user control
#define ALTITUDE_SPEED 0.3 // define the drone altitude gain/loose speed
#define YAW_SPEED 0.3 // define the drone yaw speed
//#define RESET_TIMER 1 // define a reset timer for autonomous control
#define SEEN_TIMER 0.1 // define a timer to consider pattern as "seen"
#define WIDTH 640 // define window width
#define HEIGHT 360 // define window height
#define WEBCAM_HEIGHT 480 // define webcam window height

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
char* filename26 = "..\\..\\src\\resource\\26.png";//id=26
char* filename27 = "..\\..\\src\\resource\\27.png";//id=27
char* filename28 = "..\\..\\src\\resource\\28.png";//id=28
char* filename29 = "..\\..\\src\\resource\\29.png";//id=29
char* filename30 = "..\\..\\src\\resource\\30.png";//id=30
char* filename31 = "..\\..\\src\\resource\\x.png";//id=31 -> x
char* filename32 = "..\\..\\src\\resource\\c.png";//id=32 -> c
char* filename33 = "..\\..\\src\\resource\\p.png";//id=33 -> p

// --------------------------------------------------------------------------
// main(Number of arguments, Argument values)
// Description  : This is the entry point of the program.
// Return value : SUCCESS:0  ERROR:-1
// --------------------------------------------------------------------------
int main(int argc, char **argv) {

	// Initialize
	// If drone is not connected, initialise webcam
	if (!ardrone.open()) {
		printf("Drone failed to connect.\n");
		isDroneConnected = false;
	}
	else isDroneConnected = true;

	//// DEBUGGING
	//isDroneConnected = false;


	quitProgram = false;
	int patternCount = 0;
	visiblePattern = 0;
	//lastVisiblePattern = 0;
	absoluteControl = false;
	vx = 0.0, vy = 0.0, vz = 0.0, vr = 0.0;


	// Loading patterns
	LoadPattern(filename1, patternLibrary, patternCount);
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
	LoadPattern(filename26, patternLibrary, patternCount);
	LoadPattern(filename27, patternLibrary, patternCount);
	LoadPattern(filename28, patternLibrary, patternCount);
	LoadPattern(filename29, patternLibrary, patternCount);
	LoadPattern(filename30, patternLibrary, patternCount);
	LoadPattern(filename31, patternLibrary, patternCount);
	LoadPattern(filename32, patternLibrary, patternCount);
	LoadPattern(filename33, patternLibrary, patternCount);

	cout << patternCount << " patterns are loaded." << endl;


	// Pattern detector arguments
	int norm_pattern_size = PAT_SIZE;
	double fixed_thresh = 40;
	double adapt_thresh = 5;//non-used with FIXED_THRESHOLD mode
	int adapt_block_size = 45;//non-used with FIXED_THRESHOLD mode
	double confidenceThreshold = 0.35;
	int mode = 2;//1:FIXED_THRESHOLD, 2: ADAPTIVE_THRESHOLD

	// Set to vertical drone camera
	ardrone.setCamera(1);

	// Initialise PatternDetector
	PatternDetector myDetector(fixed_thresh, adapt_thresh, adapt_block_size, confidenceThreshold, norm_pattern_size, mode);

	vector<Point2f> coordinates;
	Point2f point;
	coordinates.push_back(point);
	coordinates.push_back(point);
	coordinates.push_back(point);
	coordinates.push_back(point);
	coordinates.push_back(point);
	for (int i = 0; i <= patternCount; i++) {
		// Initialise each pattern coordinates
		patternsCoordinates.push_back(coordinates);

		// Initialise each pattern timer
		int timer;
		timers.push_back(timer);
	}


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
		}
		else {
			// Capture webcam feed
			webcamCapture = cvCaptureFromCAM(0);

			// Get webcam image
			img = cvQueryFrame(webcamCapture);
		}



		// Create image matrix
		Mat imgMat = Mat(img);

		// Render the HUD
		Mat result;
		if (isDroneConnected)
			result = HUD(imgMat, WIDTH, HEIGHT);
		else
			result = HUD(imgMat, WIDTH, WEBCAM_HEIGHT);

		// Create a buffer of the image
		Mat buffer;
		result.copyTo(buffer);

		// Run the detector
		myDetector.detect(imgMat, cameraMatrix, distortions, patternLibrary, detectedPattern);

		// Augment the input frame (and print out the properties of pattern if you want)
		if (detectedPattern.size()) {
			for (unsigned int i = 0; i < detectedPattern.size(); i++) {

				// Get pattern id
				int id = detectedPattern[i].id;

				//// Start current pattern seen timer
				//int now = cvGetTickCount();
				//int passedSinceSeen = ((now - timers[detectedPattern[i].id]) / (cvGetTickFrequency() * 1000)) / 1000;

				//// Only set visible pattern if detected a pattern for more than 1 second 
				//if (passedSinceSeen > SEEN_TIMER) SetVisiblePattern(detectedPattern[i].id);
				SetVisiblePattern(id);


				// Drawing
				detectedPattern.at(i).draw(buffer, cameraMatrix, distortions);


				// Get pattern corner and centre coordinates
				Point2f ul, ur, lr, ll, centre;
				detectedPattern.at(i).getCoordinates(ul, ur, lr, ll, centre, cameraMatrix, distortions);

				// Store coordinates
				patternsCoordinates[detectedPattern[i].id][0] = ul;
				patternsCoordinates[detectedPattern[i].id][1] = ur;
				patternsCoordinates[detectedPattern[i].id][2] = lr;
				patternsCoordinates[detectedPattern[i].id][3] = ll;
				patternsCoordinates[detectedPattern[i].id][4] = centre;


				if (isDroneConnected)
					CheckGamePatterns(WIDTH, HEIGHT, id);
				else CheckGamePatterns(WIDTH, WEBCAM_HEIGHT, id);
			}
		}
		else {
			//for (int i = 0; i < patternCount; i++) {
			//	// reset pattern timers
			//	timers[i] = cvGetTickCount();
			//}

			// Reset visible pattern
			visiblePattern = 0;
		}

		//// Get elapsed time since last saw pattern
		//lastVPElapsed = cvGetTickCount();
		//int elapsedTime = ((lastVPElapsed - lastVPStart) / (cvGetTickFrequency() * 1000)) / 1000;

		//// Check if reset timer limit passed and reset last visible pattern
		//if (elapsedTime < 300)
		//	if (RESET_TIMER - elapsedTime < 0) lastVisiblePattern = 0;



		// Get key input
		int key = cvWaitKey(33);

		// Providing key controls
		KeyControls(key);


		// Autonomous drone control
		KeepGoodAltitude();
		AutoAdjustPosition(key);

		// Always go back to hovering if no user input, no pattern visible and too old last visible pattern (0)
		if (absoluteControl && key < 0) Hover();
		else if (key < 0 && visiblePattern == 0/* && lastVisiblePattern == 0*/) Hover();

		// Drone movement
		ardrone.move3D(vx, vy, vz, vr);


		// Combine buffer with original image + opacity
		double opacity = 0.2;
		addWeighted(buffer, opacity, result, 1 - opacity, 0, result);


		// Initialise window with OpenGL support
		namedWindow("AR.Drone", WINDOW_OPENGL);
		// Show the OpenCV image in a new window AR.Drone
		imshow("AR.Drone", result);


		// Give HighGUI time to process the draw requests
		cvWaitKey(1);


		// Clear pattern for next tick
		detectedPattern.clear();
	}

	// Stop processes before quitting
	Stop();
	return 0;
}


// Movement methods
void PitchBackwards(bool controlling) {
	if (absoluteControl)
		vx = -ABSOLUTE_CONTROL_SPEED;
	else if (controlling)
		vx = -CONTROL_MOVEMENT_SPEED;
	else vx = -MOVEMENT_SPEED;
	cout << "pitch backwards" << endl;
}

void PitchForwards(bool controlling) {
	if (absoluteControl)
		vx = ABSOLUTE_CONTROL_SPEED;
	else if (controlling)
		vx = CONTROL_MOVEMENT_SPEED;
	else vx = MOVEMENT_SPEED;
	cout << "pitch forwards" << endl;
}

void RollLeft(bool controlling) {
	if (absoluteControl)
		vy = ABSOLUTE_CONTROL_SPEED;
	else if (controlling)
		vy = CONTROL_MOVEMENT_SPEED;
	else vy = MOVEMENT_SPEED;
	cout << "roll left" << endl;
}

void RollRight(bool controlling) {
	if (absoluteControl)
		vy = -ABSOLUTE_CONTROL_SPEED;
	else if (controlling)
		vy = -CONTROL_MOVEMENT_SPEED;
	else vy = -MOVEMENT_SPEED;
	cout << "roll right" << endl;
}

void YawCClockwise() {
	vr = YAW_SPEED;
	cout << "yaw ccwise" << endl;
}

void YawClockwise() {
	vr = -YAW_SPEED;
	cout << "yaw cwise" << endl;
}

void GainAltitude() {
	vz = ALTITUDE_SPEED;
	cout << "gain alt" << endl;
}

void LooseAltitude() {
	vz = -ALTITUDE_SPEED;
	cout << "loose alt" << endl;
}

void Hover() {
	vx = 0.0, vy = 0.0, vz = 0.0, vr = 0.0;
	cout << "hover" << endl;
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

	// Start hovering
	// h key
	if (key == 'h') Hover();

	// Movement
	// r key
	// Gain altitude
	if (key == 'r')
		if (!IsTooHigh())
			GainAltitude();

	// Down arrow
	// f key
	// Loose altitude
	if (key == 'f')
		if (!IsTooLow())
			LooseAltitude();

	// q key
	// Yaw c-clockwise
	if (key == 'q')
		YawCClockwise();

	// e key
	// Yaw clockwise
	if (key == 'e')
		YawClockwise();

	// Game controls - need to be within bounds to opperate
	// Left arrow / a key
	// Roll left
	if (key == 0x250000 || key == 'a')
		if (IsWithinLeftBounds())
			RollLeft(true);
		else RollRight(false);

	// Right arrow / d key
	// Roll right
	if (key == 0x270000 || key == 'd')
		if (IsWithinRightBounds())
			RollRight(true);
		else RollLeft(false);

	// Up arrow // w key
	// Pitch forwards
	if (key == 0x260000 || key == 'w')
		if (IsWithinUpperBounds())
			PitchForwards(true);

	// Down arrow / s key
	// Pitch backwards
	if (key == 0x280000 || key == 's')
		if (IsWithinLowerBounds())
			PitchBackwards(true);

	// Toggle absolute control
	// o key
	if (key == 'o') {
		if (absoluteControl) absoluteControl = false;
		else absoluteControl = true;
	}
}

// Sets the state of the visible pattern
void SetVisiblePattern(int patterID) {
	switch (patterID) {
	case 1:
		visiblePattern = 1;
		//cout << "seing patter 1" << endl;
		break;
	case 2:
		//cout << "seing patter 2" << endl;
		visiblePattern = 2;
		break;
	case 3:
		//cout << "seing patter 3" << endl;
		visiblePattern = 3;
		break;
	case 4:
		//cout << "seing patter 4" << endl;
		visiblePattern = 4;
		break;
	case 5:
		//cout << "seing patter 5" << endl;
		visiblePattern = 5;
		break;
	case 6:
		//cout << "seing patter 6" << endl;
		visiblePattern = 6;
		break;
	case 7:
		//cout << "seing patter 7" << endl;
		visiblePattern = 7;
		break;
	case 8:
		//cout << "seing patter 8" << endl;
		visiblePattern = 8;
		break;
	case 9:
		//cout << "seing patter 9" << endl;
		visiblePattern = 9;
		break;
	case 10:
		//cout << "seing patter 10" << endl;
		visiblePattern = 10;
		break;
	case 11:
		//cout << "seing patter 11" << endl;
		visiblePattern = 11;
		break;
	case 12:
		//cout << "seing patter 12" << endl;
		visiblePattern = 12;
		break;
	case 13:
		//cout << "seing patter 13" << endl;
		visiblePattern = 13;
		break;
	case 14:
		//cout << "seing patter 14" << endl;
		visiblePattern = 14;
		break;
	case 15:
		//cout << "seing patter 15" << endl;
		visiblePattern = 15;
		break;
	case 16:
		//cout << "seing patter 16" << endl;
		visiblePattern = 16;
		break;
	case 17:
		//cout << "seing patter 17" << endl;
		visiblePattern = 17;
		break;
	case 18:
		//cout << "seing patter 18" << endl;
		visiblePattern = 18;
		break;
	case 19:
		//cout << "seing patter 19" << endl;
		visiblePattern = 19;
		break;
	case 20:
		//cout << "seing patter 20" << endl;
		visiblePattern = 20;
		break;
	case 21:
		//cout << "seing patter 21" << endl;
		visiblePattern = 21;
		break;
	case 22:
		//cout << "seing patter 22" << endl;
		visiblePattern = 22;
		break;
	case 23:
		//cout << "seing patter 23" << endl;
		visiblePattern = 23;
		break;
	case 24:
		//cout << "seing patter 24" << endl;
		visiblePattern = 24;
		break;
	case 25:
		//cout << "seing patter 25" << endl;
		visiblePattern = 25;
		break;
	case 26:
		//cout << "seing patter 26" << endl;
		visiblePattern = 26;
		break;
	case 27:
		//cout << "seing patter 27" << endl;
		visiblePattern = 27;
		break;
	case 28:
		//cout << "seing patter 28" << endl;
		visiblePattern = 28;
		break;
	case 29:
		//cout << "seing patter 29" << endl;
		visiblePattern = 29;
		break;
	case 30:
		//cout << "seing patter 30" << endl;
		visiblePattern = 30;
		break;
	case 31:
		//cout << "seing patter 31" << endl;
		visiblePattern = 31;
		break;
	case 32:
		//cout << "seing patter 32" << endl;
		visiblePattern = 32;
		break;
	case 33:
		//cout << "seing patter 33" << endl;
		visiblePattern = 33;
		break;
	default:
		break;
	}

	// store last visible pattern
	//lastVisiblePattern = visiblePattern;

	// Start reset timer since last saw pattern
	//lastVPStart = cvGetTickCount();
}


// Autonomous drone control
void AutoAdjustPosition(int key) {
	// Only auto-correct if user is not controlling drone and not in absolute control mode
	if (key < 0 && !absoluteControl) {

		// Switch variable
		int patternSwitch = 0;

		//// Check if there is no pattern visible, but there was one store previously
		//if (visiblePattern == 0 && lastVisiblePattern != 0) {
		//	// If so, execute action of last pattern
		//	patternSwitch = lastVisiblePattern;
		//// Else, execute current action
		//} else patternSwitch = visiblePattern;
		patternSwitch = visiblePattern;

		switch (patternSwitch) {

		case 1:
		case 2:
		case 3:
		case 30:
			PitchBackwards(false);
			RollRight(false);
			break;

		case 4:
		case 5:
		case 6:
		case 7:
			PitchBackwards(false);
			break;

		case 8:
		case 9:
		case 10:
		case 11:
			PitchBackwards(false);
			RollLeft(false);
			break;

		case 12:
		case 13:
		case 14:
			RollLeft(false);
			break;

		case 15:
		case 16:
		case 17:
		case 18:
			PitchForwards(false);
			RollLeft(false);
			break;

		case 19:
		case 20:
		case 21:
		case 22:
			PitchForwards(false);
			break;

		case 23:
		case 24:
		case 25:
		case 26:
			PitchForwards(false);
			RollRight(false);
			break;

		case 27:
		case 28:
		case 29:
			RollRight(false);
			break;

		case 31:
			Hover();
			break;
		}

		//cout << "last visible " << lastVisiblePattern << endl;
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

// Check if coords of the pattern are within the left edge bounds
bool IsWithinLeftBounds() {
	// Get 20% of frame width
	int width = WIDTH * 20 / 100;

	// Only check if not in absolute control mode
	if (!absoluteControl) {
		if (patternsCoordinates[visiblePattern].size()) {
			// Get upper right and lower right pattern corner coordinates
			Point2f ur = patternsCoordinates[visiblePattern][1];
			Point2f lr = patternsCoordinates[visiblePattern][2];

			// Left bound patterns
			if (visiblePattern == 1 || visiblePattern == 25 || visiblePattern == 26 || visiblePattern == 27 || visiblePattern == 28 || visiblePattern == 29 || visiblePattern == 30)
				// Return false if ur or lr pattern corner is in the right part of image
				if (ur.x > width || lr.x > width) {
					cout << "left bound!" << endl;
					return false;
				}
				else return true;

				// If not patterns are detected, consider drone is within bounds.
			else return true;
		}
		// If not patterns are detected, consider drone is within bounds.
		else return true;
	}
	// If not patterns are detected, consider drone is within bounds.
	else return true;
}

// Check if coords of the pattern are within the right edge bounds
bool IsWithinRightBounds() {
	// Get 80% of frame width
	int width = WIDTH * 80 / 100;

	// Only check if not in absolute control mode
	if (!absoluteControl) {
		if (patternsCoordinates[visiblePattern].size()) {
			// Get upper left and lower left pattern corner coordinates
			Point2f ul = patternsCoordinates[visiblePattern][0];
			Point2f ll = patternsCoordinates[visiblePattern][3];

			// Right bound patterns
			if (visiblePattern == 10 || visiblePattern == 11 || visiblePattern == 12 || visiblePattern == 13 || visiblePattern == 14 || visiblePattern == 15 || visiblePattern == 16)
				// Return false if ul or ll pattern corner is in the left part of image
				if (ul.x < width || ll.x < width) {
					cout << "right bound!" << endl;
					return false;
				}
				else return true;

				// If not patterns are detected, consider drone is within bounds.
			else return true;
		}
		// If not patterns are detected, consider drone is within bounds.
		else return true;
	}
	// If not patterns are detected, consider drone is within bounds.
	else return true;
}

// Check if coords of the pattern are within the lower edge bounds
bool IsWithinLowerBounds() {
	// Get 80% of frame height
	int height = HEIGHT * 80 / 100;

	// Only check if not in absolute control mode
	if (!absoluteControl) {
		if (patternsCoordinates[visiblePattern].size()) {
			// Get upper left and upper right pattern corner coordinates
			Point2f ul = patternsCoordinates[visiblePattern][0];
			Point2f ur = patternsCoordinates[visiblePattern][1];

			// Lower bounds patterns
			if (visiblePattern == 16 || visiblePattern == 17 || visiblePattern == 18 || visiblePattern == 19 || /*visiblePattern == 20 ||*/ visiblePattern == 21 || visiblePattern == 22 || visiblePattern == 23 || visiblePattern == 24 || visiblePattern == 25)
				// Return false if ul or ur pattern corner is in the upper part of image
				if (ul.y < height || ur.y < height) {
					cout << "lower bound!" << endl;
					return false;
				}
				else return true;

				// If not patterns are detected, consider drone is within bounds.
			else return true;
		}
		// If not patterns are detected, consider drone is within bounds.
		else return !true;
	}
	// If not patterns are detected, consider drone is within bounds.
	else return true;
}

// Check if coords of the pattern are within the upper edge bounds
bool IsWithinUpperBounds() {
	// Get 20% of frame height
	int height = HEIGHT * 20 / 100;

	// Only check if not in absolute control mode
	if (!absoluteControl) {
		if (patternsCoordinates[visiblePattern].size()) {
			// Get lower right and lower left pattern corner coordinates
			Point2f lr = patternsCoordinates[visiblePattern][2];
			Point2f ll = patternsCoordinates[visiblePattern][3];

			// Upper bounds patterns
			if (visiblePattern == 1 || visiblePattern == 2 || visiblePattern == 3 || visiblePattern == 4 || visiblePattern == 5 || visiblePattern == 6 || visiblePattern == 7 || visiblePattern == 8 || visiblePattern == 9 || visiblePattern == 1)
				// Return false if ll or lr pattern corner is in the lower part of image
				if (ll.y > height || lr.y > height) {
					cout << "upper bound!" << endl;
					return false;
				}
				else return true;

				// If not patterns are detected, consider drone is within bounds.
			else return true;
		}
		// If not patterns are detected, consider drone is within bounds.
		else return true;
	}
	// If not patterns are detected, consider drone is within bounds.
	else return true;
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

	// Create a buffer of the image
	Mat buffer;
	videoFeed.copyTo(buffer);

	// Resized HUD image
	Mat rImage;

	// Resize HUD to fit window
	resize(image, rImage, Size(sizex, sizey), 0, 0, INTER_CUBIC);

	// Create a matrix to mix the video feed with the HUD image
	Mat result;

	// Overlay the HUD over the video feed
	OverlayImage(videoFeed, rImage, result, Point(0, 0));


	// Draw the crosshair
	Point2f point(sizex / 2, sizey / 2);
	circle(buffer, point, 10, Scalar(255, 255, 0), -1);


	// Display info on to HUD
	ostringstream str; // string stream
	ostringstream str2; // string stream
	ostringstream str3; // string stream

	str << "Absolute control : " << absoluteControl;
	putText(result, str.str(), Point(10, 30), CV_FONT_HERSHEY_PLAIN, 1, CV_RGB(0, 250, 0));

	str2 << "Battery : " << ardrone.getBatteryPercentage();
	putText(result, str2.str(), Point(10, 60), CV_FONT_HERSHEY_PLAIN, 1, CV_RGB(0, 250, 0));

	str3 << "Altitude : " << ardrone.getAltitude();
	putText(result, str3.str(), Point(10, 90), CV_FONT_HERSHEY_PLAIN, 1, CV_RGB(0, 250, 0));

	// Combine buffer with original image + opacity
	double opacity = 0.2;
	addWeighted(buffer, opacity, result, 1 - opacity, 0, result);

	return result;
}

// Returns the distance between 2 patterns
static int DistanceBetween(int pat1, int pat2) {
	if (patternsCoordinates[pat1].size() && patternsCoordinates[pat2].size()) {
		// Get the distance between the upper left corners of the patterns
		return norm(patternsCoordinates[pat1][0] - patternsCoordinates[pat2][0]);
	}
	else return 0;
}


static void CheckGamePatterns(int sizex, int sizey, int patID) {

	// Get crosshair coordinate (centre of screen)
	Point2f crosshair(sizex / 2, sizey / 2);

	if (patternsCoordinates[patID].size()) {
		// Get distance from crosshair to pattern
		int dist = norm(patternsCoordinates[patID][0] - crosshair);

		cout << dist << endl;

		// Check distance and drone altitude
		if (dist < 150 && ardrone.getAltitude() < 0.7) {
			// Start current pattern seen timer
			int now = cvGetTickCount();
			int passedSinceSeen = ((now - patternTimer) / (cvGetTickFrequency() * 1000)) / 1000;

			// Check if pattern is within range for more than 2 seconds
			if (passedSinceSeen > 2) {
				switch (patID)
				{
				case 31:
					cout << "!!!! PLATFORM" << endl;

					break;
				case 32:
					cout << "!!!! PEOPLE" << endl;

					break;
				case 33:
					cout << "!!!! CRATE" << endl;

					break;
				default:
					break;
				}
			}
		} else patternTimer = cvGetTickCount();
	}
}