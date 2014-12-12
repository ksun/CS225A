// Matlab style plot functions for OpenCV by Changbo (zoccob@gmail).

#include "cv.h"
#include "highgui.h"
#include "cvplot.h"


#define rowPtr(imagePtr, dataType, lineIndex) \
	    (dataType *)(imagePtr->imageData + (lineIndex) * imagePtr->widthStep)


int main(int argc, char* argv[])
{
	// load an image
	char *imagefile = "test.jpg";

	IplImage *image = cvLoadImage(imagefile);

	if (image == NULL)
	{
		std::cout << "image error: " << imagefile << std::endl << std::flush;
		return -1;
	}

	// show an image
	cvShowImage("original", image);

	// plot and label:
	//
	// template<typename T>
	// void plot(const string figure_name, const T* p, int count, int step = 1,
	//		     int R = -1, int G = -1, int B = -1);
	//
	// figure_name: required. multiple calls of this function with same figure_name
	//              plots multiple curves on a single graph.
	// p          : required. pointer to data.
	// count      : required. number of data.
	// step       : optional. step between data of two points, default 1.
	// R, G,B     : optional. assign a color to the curve.
	//              if not assigned, the curve will be assigned a unique color automatically.
	//
	// void label(string lbl):
	//
	// label the most recently added curve with lbl.
	//
	
	// specify a line to plot
	int the_line = 100;
	
	int key = -1;
	while (the_line < image->height)
	{
		unsigned char *pb = rowPtr(image, unsigned char, the_line);
		int width = image->width;
		
		CvPlot::plot("RGB", pb+0, width, 3);
		CvPlot::label("B");
		CvPlot::plot("RGB", pb+1, width, 3, 255, 0, 0);
		CvPlot::label("G");
		CvPlot::plot("RGB", pb+2, width, 3, 0, 0, 255);
		CvPlot::label("R");
		
		key = cvWaitKey(0);

		if (key == 32)
		{
			// plot the next line
			the_line++;
			// clear previous plots
			CvPlot::clear("RGB");
		}
		else
		{
			break;
		}
	}

	cvReleaseImage(&image);

	return 0;
}

