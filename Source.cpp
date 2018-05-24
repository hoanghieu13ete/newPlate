#include "FindPlate.h"

int main()
{
	ifstream read;
	read.open("trainingdata/bg.txt");

	string line;

	//VideoCapture Cap("3_13_2018.avi");

	Mat result;
	vector<Rect> list;
	Mat frame, gray, plate;

	while (getline(read, line))
	//while (Cap.read(frame))
	{
		frame = imread("trainingdata/" + line, 1);
		resize(frame, frame, Size(480, 360));
		cvtColor(frame, gray, CV_RGB2GRAY);
		list = FindLicensePlate(gray);
		result = FindBestMat(list, gray);

		if (!result.empty())
		{
			imshow("whatever", result);
			imshow("imshow", frame);
			waitKey(0);
		}
	}
	return 0;
}