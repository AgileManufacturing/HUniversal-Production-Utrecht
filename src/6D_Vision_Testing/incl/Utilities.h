#define _UTILITIES_H
#ifdef _UTILITIES_H
#include <opencv2/core/mat.hpp>

class VisionObject;

class Utilities
{
public:
	static cv::Mat rotateImage(const cv::Mat& image, cv::Mat& dstImage, double angle, double scale = 1.0);
	static cv::Point getCenterOfMass(const cv::Mat& partImage);
	static void exitWithMessage(std::string message);
	static int addDegrees(int rotation, int degrees);
	static std::vector<int> filterYaw(const std::vector<std::pair<int, int>>& matchList);

	//This function expects that the vector has enough room for another element
	template<typename T>
	static void insertMatch(std::vector <std::pair<int, T>>& matchList, const std::pair<int, T>& valuePair) 
	{
		for (int i = 0; i < matchList.size(); ++i) {
			if (valuePair.first >= matchList[i].first) {
				for (int j = matchList.size() - 2; j >= i; --j) {
					matchList[j + 1] = matchList[j];
				}
				matchList[i] = valuePair;
				break;
			}
		}
	}
	
	static cv::Point2i getPartOffsets(VisionObject part);
};

std::istream& operator>>(std::istream& input, cv::Point& point);
#endif