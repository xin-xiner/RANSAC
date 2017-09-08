#include <iostream>
#include "test.h"
using namespace std;
//using namespace cv;



int main()
{
	double errorRANSAC = 0, errorOpenCV = 0; 
	int maxRANSAC = 0, maxOpenCV = 0;
	const int count = 1;
	int validCount = 0;
	for (int i = 0; i < count; i++)
	{
		std::cout << i << std::endl;
		double errorRANSACtemp = 0, errorOpenCVtemp = 0;
		int maxRANSACtemp = 0, maxOpenCVtemp = 0;
		
		if (testFundamental(errorRANSACtemp, errorOpenCVtemp, maxRANSACtemp, maxOpenCVtemp))
			validCount++;
		errorRANSAC += errorRANSACtemp;
		errorOpenCV += errorOpenCVtemp;
		maxRANSAC += maxRANSACtemp;
		maxOpenCV += maxOpenCVtemp;

	}
	if (validCount != 0)
	{
		errorRANSAC /= validCount;
		errorOpenCV /= validCount;
		maxRANSAC /= validCount;
		maxOpenCV /= validCount;
	}
	std::cout << "errorRANSAC:   " << errorRANSAC << "   max support number:  " << maxRANSAC << std::endl;
	std::cout << "errorOpenCV:   " << errorOpenCV << "   max support number:  " << maxOpenCV << std::endl;

	system("pause");
}




















//#include <windows.h>
//#include <stdio.h>
//#include <conio.h>
//#include <tchar.h>

//#define BUF_SIZE 256
//TCHAR szName[] = TEXT("Global\\MyFileMappingObject");
//TCHAR szMsg[] = TEXT("Message from first process.");
//
//int _tmain()
//{
//	HANDLE hMapFile;
//	LPCTSTR pBuf;
//
//	hMapFile = CreateFileMapping(
//		INVALID_HANDLE_VALUE,    // use paging file
//		NULL,                    // default security
//		PAGE_READWRITE,          // read/write access
//		0,                       // maximum object size (high-order DWORD)
//		BUF_SIZE,                // maximum object size (low-order DWORD)
//		szName);                 // name of mapping object
//
//	if (hMapFile == NULL)
//	{
//		_tprintf(TEXT("Could not create file mapping object (%d).\n"),
//			GetLastError());
//		return 1;
//	}
//	pBuf = (LPTSTR)MapViewOfFile(hMapFile,   // handle to map object
//		FILE_MAP_ALL_ACCESS, // read/write permission
//		0,
//		0,
//		BUF_SIZE);
//
//	if (pBuf == NULL)
//	{
//		_tprintf(TEXT("Could not map view of file (%d).\n"),
//			GetLastError());
//
//		CloseHandle(hMapFile);
//
//		return 1;
//	}
//
//
//	CopyMemory((PVOID)pBuf, szMsg, (_tcslen(szMsg) * sizeof(TCHAR)));
//	_getch();
//
//	UnmapViewOfFile(pBuf);
//
//	CloseHandle(hMapFile);
//
//	return 0;
//}
//
