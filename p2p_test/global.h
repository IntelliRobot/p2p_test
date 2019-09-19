#pragma once
#include "stdafx.h"
#include "vector.h"
//#include "matrix.h"
#include "Quaternion.h"
//#include "math_addition.h"

//class Quaternion;
//class Vector;
//class matrix;

#define PI 3.14159265358979323846

 extern const Vector ZeroVector3;
 extern const Vector ZeroVector6;
 extern const Vector ZeroVector1;
 extern const Vector ZeroVector2;
 extern const Vector OneVector6;
 extern const Vector Qd_max;
 extern const Vector Qdd_max;
 extern const double Epsilon;//误差阈值
 extern const double V_max;//最大线速度
 extern const double Vd_max;////最大线加速度
 extern const double W_max;//最大角速度
 extern const double Wd_max;////最大角加速度
 //extern const int Zero_int;
 //extern const double Qd_max_percent=0.9;
 //extern const double Qdd_max_percent=0.9;
 struct DH_PARA
 {
	 // 机器人DH参数,craig方法

	 double a1;
	 double a2;
	 double a3;
	 double d1;
	 double d3;
	 double d4;
	 double d7;

 };
 extern const DH_PARA robot_left_dh;
 extern const DH_PARA robot_right_dh; 
 extern const DH_PARA robot_efort;

 enum TIMEorSPEED { time, speed };
 enum P2Pmethod { cubic, halfcos, trapez ,linear};
 enum cartesian_traj_type {line,circle,cubic3,spline};
 //enum VECTOR_TYPE { HORZ, VERT };
//const extern Vector Qd_max_rad(6, VERT);


 struct Ksew
 {
	 // 奇异参数
	 double Ks;
	 double Ke;
	 double Kw;
 };

 extern const Ksew Ksew_limit;

 struct Cubic_Coef
 {
	 Vector A;
	 Vector B;
	 Vector C;
	 Vector D;
 };
 struct Halfcos_Coef
 {
	 Vector A1;
	 Vector B1;
	 Vector C1;
	 Vector D1;
	 Vector E1;
	 Vector A2;
	 Vector B2;
	 Vector C2;
	 Vector D2;
	 Vector E2;
	 Vector Tc;
	 Vector Tl;
	 Vector S0C;
	 Vector VC;
	 Vector Vf;
	 Vector S;
	 double Tf;
 };

 struct Trapez_Coef
 {
	 Vector Tc;
	 Vector Tl;
	 Vector Stc;
	 Vector Stl;
	 Vector A1;
	 Vector A2;
	 Vector V0;
	 Vector Vc;
	 Vector S;
	 Vector Vf;
	 double Tf;
 };
 struct Axis_Angle
 {
	 double Theta;
	 Vector Qvec;
 };

 struct Posi_Pose
 {
	 Vector Posi;
	 Quaternion Pose;
 };

 struct halfcos_blend
 {
	 Vector A;
	 Vector B;
	 Vector C;
	 Vector E;
 };
