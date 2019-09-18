//Author:NingHC
//Last modified:2018-12-19
#pragma once
#include "stdafx.h"
#include "global.h"
//#include "vector.h"
#include "matrix.h"
#include "math_addition.h"
//#include "Quaternion.h"


const Vector ZeroVector3(0.0,0.0,0.0);
const Vector ZeroVector1(0.0);
const Vector ZeroVector2(0.0,0.0);

const Vector ZeroVector6(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

const Vector OneVector6(1, 1, 1, 1, 1, 1);

const Vector Qd_max(170*PI/180, 165*PI/180, 170*PI/180, 360*PI/180, 360*PI/180, 600*PI/180);

const Vector Qdd_max = 0.5*Qd_max;

const DH_PARA robot_left_dh = { 200,850,165,400,0,800,205 };
const DH_PARA robot_right_dh = { 200,850,165,400,0,800,205 };
const DH_PARA robot_efort = { 168.8375,781.2877,140.4575,504,-0.2077,760.5119,268 };
const double Epsilon = 0.000001;

const double V_max=2000;//最大线速度,mm/s
const double Vd_max=2*V_max;////最大线加速度mm/s2
const double W_max= 180 * PI / 180;//最大角速度
const double Wd_max=W_max;////最大角加速度
//const int Zero_int = 0;



