#pragma once
#include <vector>
#include <algorithm>
//There are too many parameters in the parameter list if design the RANSAC as a function, so I design it as a template class.
//Another advantage of designing as a class is that instead of having to declare many variables, such as result matrix and state vector, when you call a RANSAC function, you only need to declare a RANSAC class.



template<class Point,class Model>//Point and Model is class who have overriden "=" as assignment operator.
class RANSAC
{
public://Parameters of RANSAC algorithm
	int minimalSetSize;
	double inlierRatio;
	double confidence;
	double inlierThreshold;
	int minimalInlierSize;
public://Results of RANSAC algorithm
	Model result;
	std::vector<bool> inlierState;
	int maxSupportNumber;


private://Matched points set;
	std::vector<Point> points1;
	std::vector<Point> points2;
	int iterationNumber;
public:
	RANSAC(const std::vector<Point>& points1, const std::vector<Point>& points2);
	template<class FunctionUseMinimalSet, class EvaluateFunctionOfPointPair>
	int compute(FunctionUseMinimalSet& computeFunction, EvaluateFunctionOfPointPair& evaluateFunction);
private:
	template<class FunctionUseMinimalSet, class EvaluateFunctionOfPointPair>
	int RANSAConce(Model& currentResult, FunctionUseMinimalSet& computeFunction, EvaluateFunctionOfPointPair& evaluateFunction);
	template<class EvaluateFunctionOfPointPair>
	int evaluateCurrentResult(const Model& currentResult, EvaluateFunctionOfPointPair& evaluateFunction);
};

//some usefull funtion
std::vector<int> randomPick(int sum, int num);


//-----------------------------------------------------------implemention--------------------------------------------------------------------------
//As seperate compiling of template isn't support by most compiler, there is no choice but writing declaration and impemention all in header file. 


template<class Point, class Model> template<class FunctionUseMinimalSet, class EvaluateFunctionOfPointPair>
int RANSAC<Point, Model>::RANSAConce(Model& _currentResult, FunctionUseMinimalSet& computeFunction, EvaluateFunctionOfPointPair& evaluateFunction)
{
	//Make a minimal set of points randomly

	std::vector<int> pos = randomPick(points1.size(), minimalSetSize);
	if (pos.size() == 0) return 0;//Return false if can't make a set;
	std::vector<Point> minimalPointsSet1(minimalSetSize), minimalPointsSet2(minimalSetSize);
	for (int i = 0; i<minimalSetSize; i++)
	{
		minimalPointsSet1[i] = points1[pos[i]];
		minimalPointsSet2[i] = points2[pos[i]];
	}

	_currentResult = computeFunction(minimalPointsSet1, minimalPointsSet2);

	return evaluateCurrentResult(_currentResult, evaluateFunction);
}


template<class Point, class Model>
RANSAC<Point, Model>::RANSAC(const std::vector<Point>& _points1, const std::vector<Point>& _points2)
{
	minimalSetSize = 8;
	inlierRatio = 0.5;
	confidence = 0.98;
	inlierThreshold = 0.1;
	minimalInlierSize = 10;
	iterationNumber = std::round(std::log10(1 - confidence) / std::log10(1 - pow(inlierRatio, minimalSetSize)));
	points1 = _points1;
	points2 = _points2;
	inlierState.resize(points1.size(),false);
	maxSupportNumber = 0;
}

template<class Point, class Model> template<class FunctionUseMinimalSet, class EvaluateFunctionOfPointPair>
int RANSAC<Point, Model>::compute(FunctionUseMinimalSet& computeFunction, EvaluateFunctionOfPointPair& evaluateFunction)
{
	maxSupportNumber = 0;
	iterationNumber = std::round(std::log10(1 - confidence) / std::log10(1 - pow(inlierRatio, minimalSetSize)));
	inlierState.resize(points1.size(), false);
	Model currentResult;
	for (int i = 0; i<iterationNumber; i++)
	{
		int supportNumber = RANSAConce(currentResult, computeFunction, evaluateFunction);
		//std::cout << supportNumber << std::endl;
		if (supportNumber>maxSupportNumber)
		{
			maxSupportNumber = supportNumber;
			result = currentResult;
		}
	}

	if (maxSupportNumber < minimalInlierSize)
		return 0; 

	evaluateCurrentResult(result, evaluateFunction);

	std::vector<Point> inlierSet1(maxSupportNumber), inlierSet2(maxSupportNumber);
	for (int i = 0, j = 0; i < maxSupportNumber&&j<inlierState.size(); j++)
	{
		if (inlierState[j])
		{
			inlierSet1[i] = points1[j];
			inlierSet2[i] = points2[j];
			i++;
		}
	}
	result = computeFunction(inlierSet1, inlierSet2);
	return maxSupportNumber;
}


template<class Point, class Model> template<class EvaluateFunctionOfPointPair>
int RANSAC<Point, Model>::evaluateCurrentResult(const Model& currentResult, EvaluateFunctionOfPointPair& evaluateFunctionOfPointPair)
{
	int inlierCount = 0;
	inlierState.assign(inlierState.size(), false);
	for (int i = 0; i < points1.size(); i++)
	{
		//std::cout << evaluateFunctionOfPointPair(currentResult, points1[i], points2[i]) << std::endl;
		if (evaluateFunctionOfPointPair(currentResult, points1[i], points2[i]) < inlierThreshold)
		{
			inlierState[i] = true;
			inlierCount++;
		}
	}
	return inlierCount;
}
