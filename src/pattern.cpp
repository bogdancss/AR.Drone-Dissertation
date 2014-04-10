#include "pattern.h"
#include <iostream>
using namespace cv;
using namespace std;

namespace ARma {

	Pattern::Pattern(double param1){
		id = -1;
		size = param1;
		orientation = -1;
		confidence = -1;

		rotVec = (Mat_<float>(3, 1) << 0, 0, 0);
		transVec = (Mat_<float>(3, 1) << 0, 0, 0);
		rotMat = Mat::eye(3, 3, CV_32F);
	}

	//convert rotation vector to rotation matrix (if you want to proceed with other libraries)
	void Pattern::rotationMatrix(const Mat& rotation_vector, Mat& rotation_matrix)
	{
		Rodrigues(rotation_vector, rotation_matrix);
	}

	void Pattern::showPattern()
	{
		cout << "Pattern ID: " << id << endl;
		cout << "Pattern Size: " << size << endl;
		cout << "Pattern Confedince Value: " << confidence << endl;
		cout << "Pattern Orientation: " << orientation << endl;
		rotationMatrix(rotVec, rotMat);
		cout << "Exterior Matrix (from pattern to camera): " << endl;
		for (int i = 0; i < 3; i++){
			cout << rotMat.at<float>(i, 0) << "\t" << rotMat.at<float>(i, 1) << "\t" << rotMat.at<float>(i, 2) << " |\t" << transVec.at<float>(i, 0) << endl;
		}
	}

	void Pattern::getExtrinsics(int patternSize, const Mat& cameraMatrix, const Mat& distortions)
	{

		CvMat objectPts;//header for 3D points of pat3Dpts
		CvMat imagePts;//header for 2D image points of pat2Dpts 
		CvMat intrinsics = cameraMatrix;
		CvMat distCoeff = distortions;
		CvMat rot = rotVec;
		CvMat tra = transVec;
		//		CvMat rotationMatrix = rotMat; // projectionMatrix = [rotMat tra];

		CvPoint2D32f pat2DPts[4];
		for (int i = 0; i < 4; i++){
			pat2DPts[i].x = this->vertices.at(i).x;
			pat2DPts[i].y = this->vertices.at(i).y;
		}

		//3D points in pattern coordinate system
		CvPoint3D32f pat3DPts[4];
		pat3DPts[0].x = 0.0;
		pat3DPts[0].y = 0.0;
		pat3DPts[0].z = 0.0;
		pat3DPts[1].x = patternSize;
		pat3DPts[1].y = 0.0;
		pat3DPts[1].z = 0.0;
		pat3DPts[2].x = patternSize;
		pat3DPts[2].y = patternSize;
		pat3DPts[2].z = 0.0;
		pat3DPts[3].x = 0.0;
		pat3DPts[3].y = patternSize;
		pat3DPts[3].z = 0.0;

		cvInitMatHeader(&objectPts, 4, 3, CV_32FC1, pat3DPts);
		cvInitMatHeader(&imagePts, 4, 2, CV_32FC1, pat2DPts);

		//find extrinsic parameters
		cvFindExtrinsicCameraParams2(&objectPts, &imagePts, &intrinsics, &distCoeff, &rot, &tra);
	}

	void Pattern::draw(Mat& frame, const Mat& camMatrix, const Mat& distMatrix)
	{
		//model 3D points: they must be projected to the image plane
		Mat modelPts = (Mat_<float>(8, 3) << 0, 0, 0, size, 0, 0, size, size, 0, 0, size, 0,
			0, 0, -size, size, 0, -size, size, size, -size, 0, size, -size);


		std::vector<cv::Point2f> model2ImagePts;
		/* project model 3D points to the image. Points through the transformation matrix
		(defined by rotVec and transVec) "are transfered" from the pattern CS to the
		camera CS, and then, points are projected using camera parameters
		(camera matrix, distortion matrix) from the camera 3D CS to its image plane
		*/
		projectPoints(modelPts, rotVec, transVec, camMatrix, distMatrix, model2ImagePts);



		// If pattern 1 draw the platform as a cube
		if (id == 1) {
			// Colour - green
			Scalar colour(0, 255, 0);

			// Draw lines between cube corners
			int i;
			for (i = 0; i < 4; i++){
				cv::line(frame, model2ImagePts.at(i % 4), model2ImagePts.at((i + 1) % 4), colour, 10);
			}
			for (i = 4; i < 7; i++){
				cv::line(frame, model2ImagePts.at(i % 8), model2ImagePts.at((i + 1) % 8), colour, 10);
			}
			cv::line(frame, model2ImagePts.at(7), model2ImagePts.at(4), colour, 10);
			for (i = 0; i < 4; i++){
				cv::line(frame, model2ImagePts.at(i), model2ImagePts.at(i + 4), colour, 10);
			}

			// Get centre coords
			Point2f centreCoord = getCentreCoords(model2ImagePts.at(0), model2ImagePts.at(1));
			// Calculate distance between corners
			int size = norm(model2ImagePts.at(0) - model2ImagePts.at(1));
			// Draw a circle to indicate the pickup are
			circle(frame, centreCoord, size * 1.5, colour, 5);
		}
		// If pattern 2 draw the people as spheres
		else if (id == 2) {
			// Get centre coords
			Point2f centreCoord = getCentreCoords(model2ImagePts.at(0), model2ImagePts.at(1));

			// Calculate distance between corners
			int size = norm(model2ImagePts.at(0) - model2ImagePts.at(1));

			// Colour - red
			Scalar colour(0, 0, 255);

			// Draw a circle at the middle of pattern
			circle(frame, centreCoord, size / 1.5, colour, -1);
			// Draw a circle to indicate the pickup are
			circle(frame, centreCoord, size * 1.5, colour, 5);
		}
		// If pattern 3 draw the crates as squares
		else if (id == 3) {
			// Colour - blue
			Scalar colour(255, 0, 0);

			// Draw lines between corners
			cv::line(frame, model2ImagePts.at(0), model2ImagePts.at(1), colour, 20);
			cv::line(frame, model2ImagePts.at(1), model2ImagePts.at(2), colour, 20);
			cv::line(frame, model2ImagePts.at(2), model2ImagePts.at(3), colour, 20);
			cv::line(frame, model2ImagePts.at(3), model2ImagePts.at(0), colour, 20);
				
			// Get centre coords
			Point2f centreCoord = getCentreCoords(model2ImagePts.at(0), model2ImagePts.at(1));
			// Calculate distance between corners
			int size = norm(model2ImagePts.at(0) - model2ImagePts.at(1));
			// Draw a circle to indicate the pickup are
			circle(frame, centreCoord, size * 1.5, colour, 5);
		}
		// For all other patterns draw circles at pattern corners
		else {
			// Colour - yellow
			Scalar colour(0, 255, 255);

			// Draw circles
			circle(frame, model2ImagePts.at(0), 10, colour, -1);
			circle(frame, model2ImagePts.at(1), 10, colour, -1);
			circle(frame, model2ImagePts.at(2), 10, colour, -1);
			circle(frame, model2ImagePts.at(3), 10, colour, -1);
		}


		model2ImagePts.clear();
	}

	void Pattern::getCoordinates(Point2f& ul, Point2f& ur, Point2f& lr, Point2f& ll, Point2f& centre, const Mat& camMatrix, const Mat& distMatrix) {

		//model 3D points: they must be projected to the image plane
		Mat modelPts = (Mat_<float>(8, 3) << 0, 0, 0, size, 0, 0, size, size, 0, 0, size, 0,
			0, 0, -size, size, 0, -size, size, size, -size, 0, size, -size);


		std::vector<cv::Point2f> model2ImagePts;
		/* project model 3D points to the image. Points through the transformation matrix
		(defined by rotVec and transVec) "are transfered" from the pattern CS to the
		camera CS, and then, points are projected using camera parameters
		(camera matrix, distortion matrix) from the camera 3D CS to its image plane
		*/
		projectPoints(modelPts, rotVec, transVec, camMatrix, distMatrix, model2ImagePts);

		// return centre coordinate
		centre = getCentreCoords(model2ImagePts.at(0), model2ImagePts.at(1));

		// return corner coordinates
		ul = model2ImagePts.at(0);
		ur = model2ImagePts.at(1);
		lr = model2ImagePts.at(2);
		ll = model2ImagePts.at(3);
	}


	Point2f Pattern::getCentreCoords(Point2f a, Point2f b) {
		// Create 2 points from coords
		Point2f point1(a.x, a.y);
		Point2f point2(b.x, b.y);

		// calculate distance between corners
		int dist = norm(point1 - point2);

		// Get coords of middle of pattern
		Point2f coord(a.x + (dist / 2), a.y + (dist / 2));
		return coord;
	}
}