#include "Utilities.h"
#include <opencv2/imgproc.hpp>


cv::Mat Utilities::rotateImage(const cv::Mat& srcImage,cv::Mat& dstImage, double angle, double scale) {
	//int minx = INFINITY, maxx = 0;
	//int miny = INFINITY, maxy = 0;
	//for (int y = 0; y < srcImage.size().height; ++y) {
	//	for (int x = 0; x < srcImage.size().width; ++x) {
	//		if ((int)srcImage.at<uchar>(cv::Point(x, y)) > 0) {
	//			if (x > maxx) {
	//				maxx = x;
	//			}
	//			if (x < minx) {
	//				minx = x;
	//			}
	//			if (y > maxy) {
	//				maxy = y;
	//			}
	//			if (y < miny) {
	//				miny = y;
	//			}
	//		}
	//	}
	//}	
	//cv::Point centrePoint = cv::Point((maxx - minx)/ 2, (maxy - miny) / 2);
	cv::Point centrePoint = getCenterOfMass(srcImage);
	cv::Mat rotMat = cv::getRotationMatrix2D(centrePoint, angle, scale);
	cv::warpAffine(srcImage, dstImage, rotMat, srcImage.size());
	return dstImage;
}

cv::Point Utilities::getCenterOfMass(const cv::Mat& partImage){
	cv::Moments moments;
	cv::Point centerOfMass;

	//Use the first order moments to calculate the 
	//center of mass of the object.
	moments = cv::moments(partImage, true);
	centerOfMass.x = moments.m10 / moments.m00;
	centerOfMass.y = moments.m01 / moments.m00;

	return centerOfMass;
}