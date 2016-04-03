#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <ObjectDetector.h>
#include <opencv2/imgproc/imgproc.hpp>

cv::Point3d getOrientation(std::vector<cv::Point> &pts, cv::Mat img)
{
	//Construct a buffer used by the pca analysis
	cv::Mat data_pts = cv::Mat(pts.size(), 2, CV_64FC1);
	for (int i = 0; i < data_pts.rows; ++i)
	{
		data_pts.at<double>(i, 0) = pts[i].x;
		data_pts.at<double>(i, 1) = pts[i].y;
	}

	//Perform PCA analysis
	cv::PCA pca_analysis(data_pts, cv::Mat(), CV_PCA_DATA_AS_ROW);

	//Store the position of the object
	cv::Point pos = cv::Point(pca_analysis.mean.at<double>(0, 0),
		pca_analysis.mean.at<double>(0, 1));

	//Store the eigenvalues and eigenvectors
	std::vector<cv::Point2d> eigen_vecs(2);
	std::vector<double> eigen_val(2);
	for (int i = 0; i < 2; ++i)
	{
		eigen_vecs[i] = cv::Point2d(pca_analysis.eigenvectors.at<double>(i, 0),
			pca_analysis.eigenvectors.at<double>(i, 1));

		eigen_val[i] = pca_analysis.eigenvalues.at<double>(0, i);
	}

	// Draw the principal components
	circle(img, pos, 3, cv::Scalar(128, 128, 128), 2);
	//line(img, pos, pos + 0.02 * cv::Point(eigen_vecs[0].x * eigen_val[0], eigen_vecs[0].y * eigen_val[0]), cv::Scalar(128, 128, 128));
	//line(img, pos, pos + 0.02 * cv::Point(eigen_vecs[1].x * eigen_val[1], eigen_vecs[1].y * eigen_val[1]), cv::Scalar(128, 128, 128));

	return cv::Point3d(pos.x, pos.y, atan2(eigen_vecs[0].y, eigen_vecs[0].x));
	//return atan2(eigen_vecs[0].y, eigen_vecs[0].x);
}

void checkParameters(int argc, char** argv)
{
	//The program expects two parameters from the user. Since C++ adds it's own first
	//runtime parameter to argc the total equals 3.
	if (argc != 3)
	{
		std::cerr << "Incorrect number of parameters entered, expected 2. Terminating program.";
		exit(EXIT_FAILURE);
	}
}

void preprocessImages(char** argv)
{
	//auto sideMatrix = cv::imread(argv[2], CV_LOAD_IMAGE_GRAYSCALE);
	auto topDownMatrix = cv::imread(argv[1], CV_LOAD_IMAGE_GRAYSCALE);

	std::vector<std::vector<cv::Point> > contours;
	std::vector<cv::Vec4i> hierarchy;

	auto connectedComponents = ObjectDetector::findConnectedComponents(topDownMatrix);
	auto objects = ObjectDetector::filterObjects(connectedComponents, topDownMatrix);
	auto objectImage = objects[0].objectImage;

	cv::Moments moments = cv::moments(objectImage, true);
	cv::Point objectCenter{ moments.m10 / moments.m00, moments.m01 / moments.m00 };

	findContours(objectImage.clone(), contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
	std::vector<std::vector<cv::Point>> hull(contours.size());
	for (size_t i = 0; i < contours.size(); i++)
	{
		convexHull(cv::Mat(contours[i]), hull[i], false);
	}

	//Draw stuff
	for (size_t i = 0; i< contours.size(); i++)
	{
		drawContours(objectImage, hull, i, cv::Scalar(128, 128, 128, 255));
	}
	cv::rectangle(objectImage, objectCenter - cv::Point(1, 1), objectCenter + cv::Point(1, 1), cv::Scalar(0, 0, 0, 255), 3);

	namedWindow("Output image", cv::WINDOW_AUTOSIZE);
	cv::imshow("Output image", objectImage);
	cv::waitKey(0);
}

int main(int argc, char** argv)
{
	//checkParameters(argc, argv);
	preprocessImages(argv);
    return EXIT_SUCCESS;
}
