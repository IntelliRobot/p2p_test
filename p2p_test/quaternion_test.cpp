#include "stdafx.h"
#include <stdio.h>
#include <math.h>

#include "matrix.h"
#include "vector.h"
#include "Quaternion.h"
#include "global.h"
#include <time.h>
//#define PI 3.14159265358979323846
using namespace std;

int main()
{
	double theta1 = PI / 2;
	double theta2 = -PI ;
	double theta3 = PI / 3;
	Quaternion Q4(theta1, theta2, theta3);
	//Q4.Display();
	//EulerAngle ABC,abc;

	Vector P1(3, VERT);
	P1.pVector[0] = 0;
	P1.pVector[1] = 0;
	P1.pVector[2] = 1;

	//Vector P3(3, VERT);;
	//P3 = P1;

	Vector P2(3,VERT);
	P2.SetValue(1,1) ;
	P2.SetValue(2, 1);
	P2.SetValue(3, 1);
	
	Quaternion Q1(theta1, P1);
	Quaternion Q2(theta2, P2);
	Quaternion Q3(theta3, P1);

	
	//Q1.Display();
	//Q2.Display();
	//Q3.Display();

	//Q1 + Q2 + Q3;
	//Q1.Display();
	//Q2.Display();
	////Q3.Display();
	//Q1 + Q2;
	//printf("\n加了后\n");
	//Q1.Display();
	//Q2.Display();
	//Q1 - Q2;
	//printf("\n减了后\n");
	//Q1.Display();
	//Q2.Display();
	//Q1 * theta1;
	//printf("\n乘了后\n");
	//Q1.Display();
	//Q2.Display();
	//Q1 * Q2;
	//printf("\n叉乘了后\n");
	//Q1.Display();
	//Q2.Display();
	Axis_Angle AA=Q1.Angle_Axis_between(Q3);
	Vector P1_r(3, VERT);
	Vector P2_r(3, VERT);
	Vector P3_r(3, VERT);
	Vector P4_r(3, VERT);
	//Vector P2_r(3, VERT);
	Q1.Inverse();
	

	P1_r = Q1*P2;
	P2_r = Q2*P2;
	P3_r = Q3*P2;
	P4_r = Q4*P2;

	//matrix R = Q1.Rot();
	Q1.Display();
	Q2.Display();
	Q3.Display();
	Q4.Display();
	//ABC=Q1.ToEuler();
	//abc = Q2.ToEuler();
	Q3=Q2*Q2.Inverse();

	AA.Qvec.Display();
	//printf("\nQ1欧拉角是\n %4.4f,%4.4f,%4.4f \n", ABC.A, ABC.B, ABC.C);
	//printf("\nQ2欧拉角是\n %4.4f,%4.4f,%4.4f \n", abc.A, abc.B, abc.C);
	printf("\nQ1*P2;旋转前后的矢量是\n ");
	P2.Display();
	P1_r.Display();
	printf("\nQ2*P2;旋转前后的矢量是\n ");
	P2.Display();
	P2_r.Display();
	printf("\nQ3*P2;旋转前后的矢量是\n ");
	P2.Display();
	P3_r.Display();
	printf("\nQ4*P2;旋转前后的矢量是\n ");
	P2.Display();
	P4_r.Display();
	//printf("\n旋转矩阵是\n ");
	//R.Display();
	//printf("\n欧拉角转四元数\n ");
	//Q1.Euler2Qt(ABC.A, ABC.B, ABC.C);
	//Q1.Display();
	

	printf("\n\r"); _wsystem(_T("pause"));
}