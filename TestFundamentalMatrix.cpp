#include "fundamentalMatrix.h"
#include <iostream>
#include <vector>
#include "RANSAC.h" 
#include <opencv2\features2d\features2d.hpp>
#include <opencv2\nonfree\features2d.hpp>
#include <opencv2\imgproc\imgproc.hpp>
#include <opencv2\highgui\highgui.hpp>
#include <opencv2\calib3d\calib3d.hpp>
#include "test.h"

int testFundamental(double& error,double& errorOpenCV,int& max,int& maxOpenCV)
{

#ifdef desk
	cv::Mat img1 = cv::imread("image\\left.png");
	cv::Mat img2 = cv::imread("image\\right.png");
#endif
#ifdef mydesk
	cv::Mat img1 = cv::imread("image\\left_mydesk.JPG");
	cv::Mat img2 = cv::imread("image\\right_mydesk.JPG");
	cv::resize(img1, img1, cv::Size(0, 0), 0.15, 0.15);
	cv::resize(img2, img2, cv::Size(0, 0), 0.15, 0.15);
#endif

#ifdef middleburry
	cv::Mat img1 = cv::imread("image\\left_test.png");
	cv::Mat img2 = cv::imread("image\\right_test.png");
#endif

	if (img1.empty() || img2.empty())
	{
		std::cout << "read image false!" << std::endl;
		exit(-1);
	}

	cv::SIFT detector(100);

	std::vector<cv::KeyPoint> keypointsOfimg1, keypointsOfimg2;
	cv::Mat featureValue1, featureValue2;

	detector.detect(img1, keypointsOfimg1);
	detector.detect(img2, keypointsOfimg2);
	detector.compute(img1, keypointsOfimg1, featureValue1);
	detector.compute(img2, keypointsOfimg2, featureValue2);

	cv::FlannBasedMatcher matcher;
	std::vector<cv::DMatch> matches;
	matcher.match(featureValue1, featureValue2, matches);

#ifdef SHOW
	cv::Mat matchImg;
	cv::drawMatches(img1, keypointsOfimg1, img2, keypointsOfimg2, matches, matchImg);
	cv::imshow("Raw keyPoints match", matchImg);
	cv::imwrite("result\\Raw.jpg", matchImg);
	cv::waitKey(10);
#endif
	std::vector<cv::Point2d> points1(matches.size()), points2(matches.size());

	for (int i = 0; i < matches.size(); i++)
	{
		points1[i] = keypointsOfimg1[matches[i].queryIdx].pt;
		points2[i] = keypointsOfimg2[matches[i].trainIdx].pt;
	}

	RANSAC<cv::Point2d, cv::Mat> ransac(points1, points2);
	double minVal, maxVal;
	cv::minMaxIdx(points1, &minVal, &maxVal);
	ransac.inlierRatio = 0.6;
	ransac.confidence = 0.99;
	ransac.minimalSetSize = 8;
	ransac.minimalInlierSize = 8;
#ifdef xFx
	ransac.inlierThreshold = 0.12;
#else
	ransac.inlierThreshold = 0.006 * maxVal*0.006 * maxVal;
#endif
	if (ransac.compute(computeFundamentalMatrix, epipolarConsistent) == 0)
	{
		std::cout << "Fundamental matrix RANSAC failed" << std::endl;
		return 0;
	}
	//std::cout << "FundamentalMatrix computed by RANSAC algorithm:"<<std::endl << ransac.result << std::endl;
	std::cout << "Max support number RANSAC:  "  << ransac.maxSupportNumber << std::endl;
	max = ransac.maxSupportNumber;
	std::vector<cv::DMatch> inlierMatches;
	inlierMatches.reserve(ransac.maxSupportNumber);


	double errorRANSAC = 0;

	for (int i = 0; i < ransac.inlierState.size(); i++)
	{
		if (ransac.inlierState[i])
		{
			inlierMatches.push_back(matches[i]);
			//Îó²î·ÖÎö
			cv::Mat point1(3, 1, CV_64F), point2(1, 3, CV_64F);
			point1.at<double>(0, 0) = points1[i].x; point1.at<double>(1, 0) = points1[i].y; point1.at<double>(2, 0) = 1;
			point2.at<double>(0, 0) = points2[i].x; point2.at<double>(0, 1) = points2[i].y; point2.at<double>(0, 2) = 1;
			cv::Mat l1 = ransac.result*point1, l2 = ransac.result.t()*point2.t();
			errorRANSAC += (cv::norm(point2*ransac.result*point1)) / std::sqrt((l1.at<double>(0, 0)*l1.at<double>(0, 0)) + (l1.at<double>(1, 0)*l1.at<double>(1, 0)))
							+
							(cv::norm(point1.t()*ransac.result.t()*point2.t())) / std::sqrt((l2.at<double>(0, 0)*l2.at<double>(0, 0)) + (l2.at<double>(1, 0)*l2.at<double>(1, 0)));
		}
			
	}

	errorRANSAC /= ransac.maxSupportNumber;
	//std::cout <<"errorRANSAC:  "<< errorRANSAC << std::endl;
	error = errorRANSAC;
#ifdef SHOW
	cv::Mat matchRANSACImg;
	cv::drawMatches(img1, keypointsOfimg1, img2, keypointsOfimg2, inlierMatches, matchRANSACImg);
	cv::imshow("keyPoints match result after RANSAC", matchRANSACImg);
	cv::imwrite("result\\RANSAC.jpg", matchRANSACImg);
#endif
	/*double minVal, maxVal;
	cv::minMaxIdx(points1, &minVal, &maxVal);*/
	std::vector<uchar> status;
	cv::Mat F = cv::findFundamentalMat(points1, points2, cv::FM_RANSAC,0.006 * maxVal, 0.99, status);
	//std::cout << "FundamentalMatrix computed by OpenCV:" << std::endl << F << std::endl;

	std::vector<cv::DMatch> inlierMatchesO;

	errorOpenCV = 0;
	for (int i = 0; i <status.size(); i++)
	{
		if (status[i])
		{
			inlierMatchesO.push_back(matches[i]);

			//Îó²î·ÖÎö
			cv::Mat point1(3, 1, CV_64F), point2(1, 3, CV_64F);
			point1.at<double>(0, 0) = points1[i].x; point1.at<double>(1, 0) = points1[i].y; point1.at<double>(2, 0) = 1;
			point2.at<double>(0, 0) = points2[i].x; point2.at<double>(0, 1) = points2[i].y; point2.at<double>(0, 2) = 1;
			cv::Mat l1 = F*point1,l2 = F.t()*point2.t();
			errorOpenCV += (cv::norm(point2*F*point1)) / std::sqrt((l1.at<double>(0, 0)*l1.at<double>(0, 0)) + (l1.at<double>(1, 0)*l1.at<double>(1, 0)))
							+
							(cv::norm(point1.t()*F.t()*point2.t())) / std::sqrt((l2.at<double>(0, 0)*l2.at<double>(0, 0)) + (l2.at<double>(1, 0)*l2.at<double>(1, 0)));
		}
	}

	errorOpenCV /= inlierMatchesO.size();
	//std::cout << "errorOpenCV:  " << errorOpenCV << std::endl;

#ifdef SHOW
	cv::Mat matchRANSACImgO;
	cv::drawMatches(img1, keypointsOfimg1, img2, keypointsOfimg2, inlierMatchesO, matchRANSACImgO);
	cv::imshow("keyPoints match result OpenCV", matchRANSACImgO);
	cv::imwrite("result\\openCV.jpg", matchRANSACImgO);
	cv::waitKey(0);
#endif
	maxOpenCV = inlierMatchesO.size();
	std::cout << "Max support number OpenCV:  "<< inlierMatchesO.size() << std::endl;
	
	return 1;
	
}












