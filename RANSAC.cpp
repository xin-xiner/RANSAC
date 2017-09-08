#include "RANSAC.h"
#include <iostream>
#include <vector>
#include <random>

std::vector<int> randomPick(int sum, int num)
{
	if (num > sum)return std::vector<int>();
	std::random_device rd;
	std::vector<int> list(sum), result(num);
	int curSize = sum;
	for (int i = 0; i<sum; i++)
	{
		list[i] = i;
	}
	for (int i = 0; i<num; i++)
	{
		int curPose = rd() % curSize;

		result[i] = list[curPose];
		std::swap(list[curSize - 1], list[curPose]);
		curSize--;
	}
	return result;
}
