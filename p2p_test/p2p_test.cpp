#pragma once
#include "stdafx.h"
#include "PtoP.h"
//#include <time.h>

using namespace std;

int main()
{
	Vector q0(0, 0, 0, 0, 0, 0);
	Vector qf(PI / 2, PI / 3, PI / 4, PI / 5, PI / 6, PI/2);

	Vector qd0(0, 0, 0, 0, 0, 0);
	Vector qdf(0, 0, 0, 0, 0, 0);
	PtoP P1(q0,qf,linear,time,2,qd0,qdf,1);
	//P1.realtime(0.4); 
	P1.offline();
	//double element = 1;
	//matrix mat_test(4,4);
	//for (int i = 1; i < 5; i++)
	//{
	//	for (int j = 1; j < 5; j++)
	//	{
	//		mat_test(i, j) = element;
	//		printf("this is %d row and %d col, data is %f\n\r", i, j, element);
	//		element++;
	//	}
	//}


	//printf("\n\r"); _wsystem(_T("pause"));

}