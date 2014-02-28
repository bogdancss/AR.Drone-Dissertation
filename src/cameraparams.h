#ifndef _CAMERAPARAMS_H
#define _CAMERAPARAMS_H
#include <opencv2/core/core.hpp>
using namespace cv;

CvMat* intrinsic = (CvMat*)cvLoad("..\\..\\src\\resource\\intrinsic.xml");
	CvMat* distor = (CvMat*)cvLoad("..\\..\\src\\resource\\distortion.xml");

	Mat cameraMatrix = cvarrToMat(intrinsic);
	Mat distortions = cvarrToMat(distor);



#endif