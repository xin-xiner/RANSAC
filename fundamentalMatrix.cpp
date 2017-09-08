#include "fundamentalMatrix.h"
#include <vector>
#include <opencv2\calib3d\calib3d.hpp>
#include <iostream>
#include "RANSAC.h" 
#include "test.h"



cv::Mat computeFundamentalMatrix(const std::vector<cv::Point2d>& points1,const std::vector<cv::Point2d>& points2)
	{
		//cv::Mat F = cv::findFundamentalMat(points1, points2, cv::FM_7POINT);
		//F = F(cv::Range(0, 3), cv::Range(0, 3));
		//if (F.at<double>(2, 2) == 0)F = cv::Mat::eye(3, 3,CV_64F);
		//return F;

		cv::Mat x1(3, points1.size(), CV_64F), x2(3, points2.size(), CV_64F);
		for (int i = 0; i < points1.size(); i++)
		{
			x1.at<double>(0, i) = points1[i].x;x1.at<double>(1, i) = points1[i].y;x1.at<double>(2, i) = 1;
			x2.at<double>(0, i) = points2[i].x;x2.at<double>(1, i) = points2[i].y;x2.at<double>(2, i) = 1;
		}
		//***********************normalize
#ifdef Normalized
		cv::Mat T1 = getNormalizeMatrix(points1);
		cv::Mat T2 = getNormalizeMatrix(points2);

		x1 = T1*x1;
		x2 = T2*x2;
#endif
		//**********************
		cv::Mat A = convertToLeastSquares(x1, x2);
		cv::Mat s, u, v;


		cv::SVD svd(A);
		svd.compute(A, s, u, v);

		v = v.t();
		v = v.col(v.cols - 1);

		cv::Mat F = vecToMat(v);
		//****************denormalize
#ifdef Normalized
		F = T2.t()*F*T1;
#endif

		//*********************singular constraint enforcement
#ifdef Constraint
		svd.compute(F, s, u, v);
		s.at<double>(2) = 0;
		F = u*cv::Mat::diag(s)*v;
#endif

		F /= F.at<double>(2, 2);

		return F;
	}


#ifdef xFx
double epipolarConsistent(const cv::Mat& result,const cv::Point2d& _point1, const cv::Point2d& _point2)
{
	cv::Mat point1(3, 1, CV_64F), point2(1, 3, CV_64F);
	point1.at<double>(0, 0) = _point1.x; point1.at<double>(1, 0) = _point1.y; point1.at<double>(2, 0) = 1;
	point2.at<double>(0, 0) = _point2.x; point2.at<double>(0, 1) = _point2.y; point2.at<double>(0, 2) = 1;
	
	//std::cout << "epipolar:" << point1*result*point2 << "    " << cv::norm(point1*result*point2) << std::endl;
	return cv::norm(point2*result*point1) + cv::norm(point1.t()*result.t()*point2.t());
}

#endif

#ifdef Sampson
double epipolarConsistent(const cv::Mat& result,const cv::Point2d& _point1, const cv::Point2d& _point2)
{
	cv::Mat point1(3, 1, CV_64F), point2(1, 3, CV_64F);
	point1.at<double>(0, 0) = _point1.x; point1.at<double>(1, 0) = _point1.y; point1.at<double>(2, 0) = 1;
	point2.at<double>(0, 0) = _point2.x; point2.at<double>(0, 1) = _point2.y; point2.at<double>(0, 2) = 1;
	double a, b, c, d1, d2, s1, s2;

	a = result.at<double>(0, 0) * _point1.x + result.at<double>(0, 1) * _point1.y + result.at<double>(0, 2);
	b = result.at<double>(1, 0) * _point1.x + result.at<double>(1, 1) * _point1.y + result.at<double>(1, 2);
	c = result.at<double>(2, 0) * _point1.x + result.at<double>(2, 1) * _point1.y + result.at<double>(2, 2);

	s2 = 1. / (a*a + b*b);
	d2 = _point2.x*a + _point2.y*b + c;

	a = result.at<double>(0, 0) * _point2.x + result.at<double>(1, 0) * _point2.y + result.at<double>(2, 0);
	b = result.at<double>(0, 1) * _point2.x + result.at<double>(1, 1) * _point2.y + result.at<double>(2, 1);
	c = result.at<double>(0, 2) * _point2.x + result.at<double>(1, 2) * _point2.y + result.at<double>(2, 2);

	s1 = 1. / (a*a + b*b);
	d1 = _point1.x*a + _point1.y*b + c;

	double err = (float)std::max(d1*d1*s1, d2*d2*s2);
	//double err = (float)std::max(d1, d2);
	return err;
}
#endif
#ifdef dxl
double epipolarConsistent(const cv::Mat& result, const cv::Point2d& _point1, const cv::Point2d& _point2)
{

	cv::Mat point1(3, 1, CV_64F), point2(1, 3, CV_64F);
	point1.at<double>(0, 0) = _point1.x; point1.at<double>(1, 0) = _point1.y; point1.at<double>(2, 0) = 1;
	point2.at<double>(0, 0) = _point2.x; point2.at<double>(0, 1) = _point2.y; point2.at<double>(0, 2) = 1;
	cv::Mat l1 = result*point1, l2 = result.t()*point2.t();
	double err = (cv::norm(point2*result*point1)) / std::sqrt((l1.at<double>(0, 0)*l1.at<double>(0, 0)) + (l1.at<double>(1, 0)*l1.at<double>(1, 0)))
		+
		(cv::norm(point1.t()*result.t()*point2.t())) / std::sqrt((l2.at<double>(0, 0)*l2.at<double>(0, 0)) + (l2.at<double>(1, 0)*l2.at<double>(1, 0)));


	return err;
}
#endif


cv::Mat getNormalizeMatrix(const std::vector<cv::Point2d>& points)
{
	double tx = 0, ty = 0, s = 0;

	for (int i = 0; i < points.size(); i++)
	{
		tx += points[i].x;
		ty += points[i].y;
	}
	tx /= points.size();
	ty /= points.size();

	for (int i = 0; i < points.size(); i++)
	{
		double x, y;
		x = points[i].x - tx;
		y = points[i].y - ty;
		s += std::sqrt(x*x + y*y);
	}
	s /= points.size();
	s = std::sqrt(2.0) / s;
	cv::Mat T = (cv::Mat_<double>(3, 3) <<
		s, 0, -s*tx,
		0, s, -s*ty,
		0, 0, 1);
	return T;
}


cv::Mat vecToMat(cv::Mat& v)
{

	int dimension = v.rows;
	cv::Mat m(sqrt(dimension), sqrt(dimension), CV_64F);
	for (int i = 0; i < m.rows; i++)
	{
		for (int j = 0; j < m.cols; j++)
			m.at<double>(i, j) = v.at<double>(i*m.cols + j, 0);
	}
	return m;
}

cv::Mat convertToLeastSquares(cv::Mat& a, cv::Mat& b)
{
	cv::Mat v1(a.rows*a.rows, a.cols, CV_64F);
	cv::Mat v2(b.rows*b.rows, b.cols, CV_64F);
	for (int i = 0; i < a.rows; i++)
	{
		for (int j = 0; j < a.rows; j++)
		{
			a.row(j).copyTo(v1.row(i*a.rows + j));
			b.row(i).copyTo(v2.row(i*a.rows + j));
		}
	}
	multiply(v1, v2, v1);
	return v1.t();
}