#include "main.h"

#define PAT_SIZE 64//equal to pattern_size variable (see below)
#define SAVE_VIDEO 0 //if true, it saves the video in "output.avi"
#define NUM_OF_PATTERNS 9// define the number of patterns you want to use
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

// --------------------------------------------------------------------------
// main(Number of arguments, Argument values)
// Description  : This is the entry point of the program.
// Return value : SUCCESS:0  ERROR:-1
// --------------------------------------------------------------------------
int main(int argc, char **argv)
{

	// Initialize
	if (!ardrone.open()) {
		printf("Drone failed to connect.\n");
		isDroneConnected = false;
		//return -1;
	} else isDroneConnected = true;


	quitProgram = false;
	int patternCount = 0;

	/*create patterns' library using rotated versions of patterns
	*/
	loadPattern(filename1, patternLibrary, patternCount);
#if (NUM_OF_PATTERNS==2)
	loadPattern(filename2, patternLibrary, patternCount);
#endif
#if (NUM_OF_PATTERNS==3)
	loadPattern(filename2, patternLibrary, patternCount);
	loadPattern(filename3, patternLibrary, patternCount);
#endif
#if (NUM_OF_PATTERNS==4)
	loadPattern(filename2, patternLibrary, patternCount);
	loadPattern(filename3, patternLibrary, patternCount);
	loadPattern(filename4, patternLibrary, patternCount);
#endif
#if (NUM_OF_PATTERNS==5)
	loadPattern(filename2, patternLibrary, patternCount);
	loadPattern(filename3, patternLibrary, patternCount);
	loadPattern(filename4, patternLibrary, patternCount);
	loadPattern(filename5, patternLibrary, patternCount);
#endif
#if (NUM_OF_PATTERNS==6)
	loadPattern(filename2, patternLibrary, patternCount);
	loadPattern(filename3, patternLibrary, patternCount);
	loadPattern(filename4, patternLibrary, patternCount);
	loadPattern(filename5, patternLibrary, patternCount);
	loadPattern(filename6, patternLibrary, patternCount);
#endif
#if (NUM_OF_PATTERNS==7)
	loadPattern(filename2, patternLibrary, patternCount);
	loadPattern(filename3, patternLibrary, patternCount);
	loadPattern(filename4, patternLibrary, patternCount);
	loadPattern(filename5, patternLibrary, patternCount);
	loadPattern(filename6, patternLibrary, patternCount);
	loadPattern(filename7, patternLibrary, patternCount);
#endif
#if (NUM_OF_PATTERNS==8)
	loadPattern(filename2, patternLibrary, patternCount);
	loadPattern(filename3, patternLibrary, patternCount);
	loadPattern(filename4, patternLibrary, patternCount);
	loadPattern(filename5, patternLibrary, patternCount);
	loadPattern(filename6, patternLibrary, patternCount);
	loadPattern(filename7, patternLibrary, patternCount);
	loadPattern(filename8, patternLibrary, patternCount);
#endif
#if (NUM_OF_PATTERNS==9)
	loadPattern(filename2, patternLibrary, patternCount);
	loadPattern(filename3, patternLibrary, patternCount);
	loadPattern(filename4, patternLibrary, patternCount);
	loadPattern(filename5, patternLibrary, patternCount);
	loadPattern(filename6, patternLibrary, patternCount);
	loadPattern(filename7, patternLibrary, patternCount);
	loadPattern(filename8, patternLibrary, patternCount);
	loadPattern(filename9, patternLibrary, patternCount);
#endif

	//#if (NUM_OF_PATTERNS > 1)
	//	if (NUM_OF_PATTERNS > 1) {
	//		for (int i = 2; i <= NUM_OF_PATTERNS; i++) {
	//			char buffer[32];
	//			memset(buffer, 0, sizeof(buffer));
	//			
	//			sprintf(buffer, "filename%i", i);
	//			loadPattern(buffer, patternLibrary, patternCount);
	//			//const char *filename;
	//			//filename = ("filename" + std::to_string(i)).c_str();
	//			//loadPattern(("filename" + std::to_string(i)).c_str(), patternLibrary, patternCount);
	//			//loadPattern(filename, patternLibrary, patternCount);
	//
	//			//loadPattern(("filename" + std::to_string(i)).c_str(), patternLibrary, patternCount);
	//		}
	//	}
	//#endif

	cout << patternCount << " patterns are loaded." << endl;


	int norm_pattern_size = PAT_SIZE;
	double fixed_thresh = 40;
	double adapt_thresh = 5;//non-used with FIXED_THRESHOLD mode
	int adapt_block_size = 45;//non-used with FIXED_THRESHOLD mode
	double confidenceThreshold = 0.35;
	int mode = 2;//1:FIXED_THRESHOLD, 2: ADAPTIVE_THRESHOLD

	PatternDetector myDetector(fixed_thresh, adapt_thresh, adapt_block_size, confidenceThreshold, norm_pattern_size, mode);

	// capture webcam feed
	webcamCapture = cvCaptureFromCAM(0);

#if (SAVE_VIDEO)
	CvVideoWriter *video_writer = cvCreateVideoWriter("output.avi", -1, 25, cvSize(640, 480));
#endif

	Mat imgMat;
	int k = 0;
	while (1){ //modify it for longer/shorter videos

		// check to terminate program
		if (quitProgram) {
			Stop();
			break;
		}

		IplImage *img;

		if (isDroneConnected) {
			// Get drone image
			printf("Initialising drone\n");
			img = ardrone.getImage();
		} else {
			// Get webcam image
			printf("Initialising webcam\n");
			img = cvQueryFrame(webcamCapture);
		}


		Mat imgMat = Mat(img);
		double tic = (double)cvGetTickCount();


		//run the detector
		myDetector.detect(imgMat, cameraMatrix, distortions, patternLibrary, detectedPattern);

		double toc = (double)cvGetTickCount();
		double detectionTime = (toc - tic) / ((double)cvGetTickFrequency() * 1000);
		cout << "Detected Patterns: " << detectedPattern.size() << endl;
		cout << "Detection time: " << detectionTime << endl;

		printf("Battery = %d%%\n", ardrone.getBatteryPercentage());


		//augment the input frame (and print out the properties of pattern if you want)
		for (unsigned int i = 0; i<detectedPattern.size(); i++){
			detectedPattern.at(i).showPattern();
			detectedPattern.at(i).draw(imgMat, cameraMatrix, distortions);
			DoIfSees(detectedPattern[i].id);
		}


		KeyControlls();
		KeepGoodAltitude();






#if (SAVE_VIDEO)
		cvWriteFrame(video_writer, &((IplImage)imgMat));
#endif
		imshow("result", imgMat);
		cvWaitKey(1);
		k++;

		detectedPattern.clear();
	}

#if (SAVE_VIDEO)
	cvReleaseVideoWriter(&video_writer);
#endif



	Stop();
	return 0;
}


int loadPattern(const char* filename, std::vector<cv::Mat>& library, int& patternCount){
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




// movement methods
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

void KeyControlls() {
	// Key input
	int key = cvWaitKey(33);

	// quit if ESC key
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
	// up arrow
	// gain altitude
	if (key == 0x260000 || key == 'r' && !IsTooHigh()) GainAltitude();
	// down arrow
	// loose altitude
	if (key == 0x280000 || key == 'f' && !IsTooLow()) LooseAltitude();

	if (IsWithinBounds()) {
		// left arrow | a key
		// roll left
		if (key == 0x250000 || key == 'a') RollLeft();
		// right arrow | d key
		// roll right
		if (key == 0x270000 || key == 'd') RollRight();
		// w key
		// pitch forwards
		if (key == 'w')      PitchForwards();
		// s key
		// pitch backwards
		if (key == 's')      PitchBackwards();
		// q key
		// yaw c-clockwise
		if (key == 'q')      YawCClockwise();
		// e key
		// yaw clockwise
		if (key == 'e')      YawClockwise();
	}
}

void DoIfSees(int patterID) {
	std::stringstream s;
	detectedPattern[patterID];
	switch (patterID) {
	case 1:
		// if sees pattern 1 do
		s << "seeing 1 " << '\n';
		OutputDebugString(s.str().c_str());
		// go diagonally forward right
		ardrone.move3D(0.1, 0.0, 0.0, 0.0);
		break;
	case 2:
		// if sees pattern 2 do
		s << "seeing 2" << '\n';
		OutputDebugString(s.str().c_str());
		// go straight right

		break;
	case 3:
		// if sees pattern 3 do
		s << "seeing 3" << '\n';
		OutputDebugString(s.str().c_str());
		// go diagonally backward right

		break;
	case 4:
		// if sees pattern 4 do
		s << "seeing 4" << '\n';
		OutputDebugString(s.str().c_str());
		// go straight forward

		break;
	case 5:
		// if sees pattern 5 do
		// do nothig > hover
		s << "seeing 5" << '\n';
		OutputDebugString(s.str().c_str());
		// hover

		break;
	case 6:
		// if sees pattern 6 do
		s << "seeing 6" << '\n';
		OutputDebugString(s.str().c_str());
		// go straight backward

		break;
	case 7:
		// if sees pattern 7 do
		s << "seeing 7" << '\n';
		OutputDebugString(s.str().c_str());
		// go diagonally forward left

		break;
	case 8:
		// if sees pattern 8 do
		s << "seeing 8" << '\n';
		OutputDebugString(s.str().c_str());
		// go straight left

		break;
	case 9:
		// if sees pattern 9 do
		s << "seeing 9" << '\n';
		OutputDebugString(s.str().c_str());
		// go diagonally backward left

		break;
	default:
		break;
	}
}

bool IsTooLow() {
	double altitude = ardrone.getAltitude();

	if (altitude < 0.5) return true;
	else return false;
}

bool IsTooHigh() {
	double altitude = ardrone.getAltitude();

	if (altitude > 1.5) return true;
	else return false;
}

void KeepGoodAltitude() {
	// lower the drone
	if (IsTooHigh()) LooseAltitude();

	// raise the drone
	if (IsTooLow()) GainAltitude();
}

bool IsWithinBounds() {
	return true;
}

void Stop() {
	cvReleaseCapture(&webcamCapture);
	// See you
	ardrone.close();
}