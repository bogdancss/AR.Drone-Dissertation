#ifndef _ARMA_PATTERN_
#define _ARMA_PATTERN_
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
using namespace cv;

namespace ARma {

class Pattern
{
	public:
		vector<Point2f> vertices;
		int id;
		int orientation;//{0,1,2,3}
		float size; //in milimeters
		double confidence;//min: -1, max: 1
		Mat rotVec, transVec, rotMat;

		Pattern(double param1=80);

		~Pattern(){};
		
		//solves the exterior orientation problem between patten and camera
		void getExtrinsics(int patternSize, const Mat& cameraMatrix, const Mat& distortions);

		//augments image
		void Pattern::draw(Mat& frame, const Mat& camMatrix, const Mat& distMatrix);

		//computes the rotation matrix from the rotation vector using Rodrigues
		void Pattern::rotationMatrix(const Mat& rotation_vector, Mat& rotation_matrix);
		
		//prints the properties of the patten and its transformation matrix
		void showPattern(void);

		// returns the coordinates of the 4 corners of the pattern
		void getCoordinates(Point2f& ul, Point2f& ur, Point2f& ll, Point2f& lr, Point2f& centre, const Mat& camMatrix, const Mat& distMatrix);

		// returns the coordinates of the centre of the pattern
		Point2f getCentreCoords(Point2f a, Point2f b);
};
}
#endif
