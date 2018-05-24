#pragma once
#include<opencv2/core/core.hpp>
#include<opencv2/video/video.hpp>
#include<opencv2/highgui/highgui.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <iostream>
#include <fstream>
using namespace std;
using namespace cv;

bool checkSize(Rect r, Mat frame)
{
	if (r.width * 1.0 / r.height > 1 && r.width * 1.0 / r.height < 2)
	{
		if (r.width * r.height > 5000 && r.width * r.height < frame.rows * frame.cols / 5)
			return true;
	}
	return false;
}

bool wayToSortX(Rect a, Rect b)
{
	return a.x < b.x;
}
bool wayToSortY(Rect a, Rect b)
{
	return a.y < b.y;
}


Mat maximizeContrast(cv::Mat &imgGrayscale) {
	cv::Mat imgTopHat;
	cv::Mat imgBlackHat;
	cv::Mat imgGrayscalePlusTopHat;
	cv::Mat imgGrayscalePlusTopHatMinusBlackHat;

	cv::Mat structuringElement = cv::getStructuringElement(CV_SHAPE_RECT, cv::Size(5, 5));

	cv::morphologyEx(imgGrayscale, imgTopHat, CV_MOP_TOPHAT, structuringElement);
	cv::morphologyEx(imgGrayscale, imgBlackHat, CV_MOP_BLACKHAT, structuringElement);

	imgGrayscalePlusTopHat = imgGrayscale + imgTopHat;
	imgGrayscalePlusTopHatMinusBlackHat = imgGrayscalePlusTopHat - imgBlackHat;

	return(imgGrayscalePlusTopHatMinusBlackHat);
}

bool CheckArea(Rect rect)
{
	int area = rect.width * rect.height;
	if ((area < 50 || area > 2000) && (rect.width * 1.0 / rect.height) > 1.0)
		return false;

	return true;
}

Mat FindPossiblePlate(Mat imgThresh)
{
	rectangle(imgThresh, Rect(0, 0, 150, 150), Scalar(255, 255, 255), 3);

	vector<vector<Point>> contours;
	vector<Rect> chars;
	int possibleChar = 0;
	findContours(imgThresh, contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);	
	Rect result;
	
	for (int i = 0; i < contours.size(); i++)
	{
		Rect r = boundingRect(contours[i]);
		if (CheckArea(r) && contours[i].size() > 100)
		{
			if (r.height > imgThresh.rows / 3 && r.height < imgThresh.rows / 1.8 && imgThresh.rows > 5 * r.width && r.width > 10)
			{
				rectangle(imgThresh, r, Scalar(0, 0, 0));
				chars.push_back(r);
			}
		}
	}

	if (chars.size() > 4)
	{
		sort(chars.begin(), chars.end(), wayToSortX);
		int xMin = chars[0].x;
		int xMax = chars[chars.size() - 1].x;
		int witdh = chars[chars.size() - 1].width;

		sort(chars.begin(), chars.end(), wayToSortY);
		int yMin = chars[0].y;
		int yMax = chars[chars.size() - 1].y;
		int height = chars[chars.size() - 1].height;

		if (xMin - (150 - (xMax + witdh)) > 10)
		{
			result = Rect(Point(150 - (xMax + witdh), yMin), Point(xMax + witdh, yMax + height));
		}
		else if (((150 - (xMax + witdh)) - xMin) > 10)
		{
			result = Rect(Point(xMin, yMin), Point(xMax + witdh, yMax + height));
		}
		rectangle(imgThresh, result, Scalar(0, 0, 0));
		imshow("result", imgThresh);
		return imgThresh(result);
	}
	else
		return Mat();
}

//list gray.
Mat FindBestMat(vector<Rect> list, Mat frame)
{
	Mat binary, clone;
	vector<vector<Point>> contours;
	bool bestPlate = false;
	for (auto rect : list)
	{
		clone = frame(rect).clone();
		resize(clone, clone, Size(150, 150));
		GaussianBlur(clone, clone, Size(3, 3), 0);

		adaptiveThreshold(clone, binary, 255, CV_ADAPTIVE_THRESH_GAUSSIAN_C, CV_THRESH_BINARY, 19, 9);

		Mat plate = FindPossiblePlate(binary);

		if (!plate.empty())
		{
			return plate;
		}

	}
	return Mat();
}

//input is gray image find square object
vector<Rect> FindLicensePlate(Mat frame)
{
	vector<Rect> list;

	Mat binary, tmp;

	tmp = frame.clone();

	//find local max
	dilate(tmp, tmp, Mat::ones(Size(5, 5), CV_8UC1));

	GaussianBlur(tmp, tmp, Size(5, 5), 0);

	//cal gradient
	tmp.convertTo(tmp, CV_32F, 1 / 255.0);

	// Calculate gradients gx, gy
	Mat gx, gy;
	Sobel(tmp, gx, CV_32F, 1, 0, 3);
	Sobel(tmp, gy, CV_32F, 0, 1, 3);
	Mat mag, angle;
	cartToPolar(gx, gy, mag, angle, 1);

	threshold(mag, binary, 0.1, 255, CV_THRESH_BINARY);

	vector<vector<Point>> contours;

	binary.convertTo(binary, CV_8UC1);

	findContours(binary, contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);

	for (int i = 0; i < contours.size(); i++)
	{
		Rect r = boundingRect(contours[i]);

		if (checkSize(r, frame))
		{
			list.push_back(r);
		}
	}

	return list;
}

Mat FindPlate(Mat frame, vector<Rect> &possibleChars)
{
	vector<Rect> list = FindLicensePlate(frame);
	Mat result = FindBestMat(list, frame);

	return result;
}
