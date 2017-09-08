#pragma once
#include <opencv2\core\core.hpp>
#include <vector>
#define RANSAC_INLIER_THRESHOLD 0.1


cv::Mat convertToLeastSquares(cv::Mat& a, cv::Mat& b);
cv::Mat vecToMat(cv::Mat& v);
cv::Mat computeFundamentalMatrix(const std::vector<cv::Point2d>& points1, const std::vector<cv::Point2d>& points2);
double epipolarConsistent(const cv::Mat& result, const cv::Point2d& _point1, const cv::Point2d& _point2);
cv::Mat getNormalizeMatrix(const std::vector<cv::Point2d>& points);