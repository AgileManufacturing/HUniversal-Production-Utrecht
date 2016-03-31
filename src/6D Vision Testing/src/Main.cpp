#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include "TestClass.h"

TestClass testObject;

int main( int argc, char** argv )
{
	if( argc != 2)
	{
		std::cout <<" Usage: display_image ImageToLoadAndDisplay" << std::endl;
		return -1;
	}
	cv::Mat image;
    image = cv::imread(argv[1], CV_LOAD_IMAGE_COLOR);			// Read the file
    if(! image.data )										// Check for invalid input
    {
	    std::cout <<  "Could not open or find the image" << std::endl ;
        return -1;
    }
	cv::namedWindow( "Display window", cv::WINDOW_AUTOSIZE );		// Create a window for display.
    imshow( "Display window", image );						// Show our image inside it.
	cv::waitKey(0);												// Wait for a keystroke in the window
    return 0;
}