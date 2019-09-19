#pragma once

#include "vector.h"
#include "matrix.h"
#include "FunctionReturnMessage.h"

#define  pi 3.1415926535897932384626

struct HALFCOS_COEF_RESULT
{
	//return message
	FunctionReturnMessage ReturnMessage;
	
	//平均速度和运行时间
	double vc;
	double tf;

	//hal_coef
	double hal_Aa1;
	double hal_A2;
	double hal_B1;
	double hal_B2;
	double hal_C1;
	double hal_C2;
	double hal_D1;
	double hal_D2;
	double hal_E1;
	double hal_E2;
	double hal_tf;
	double hal_tc;
	double hal_tl;
	double hal_s0c;
	double hal_vc;
};

struct QUATER_CUBIC_COEF_RESULT
{
	//return message
	FunctionReturnMessage ReturnMessage;

	//quater_coef
	double cubic_A;
	double cubic_B;
	double cubic_C;
	double cubic_D;

	//dalpha
	double dalpha;
};

struct LINE_TRAJ_PARA_RESULT
{
	//插值步数
	unsigned int Na;

	//line_para
	double p0_start;
	double Q0_start;
	double Qf_end;
	double p2pL;
	double uvec;
	double dalpha;
	double hal_coef;
	double quater_coef;
	double Qvec;
};

class robot
{
public:
	//Q1为4元数表示的旋转矩阵
	vector quater_inv(const vector Q1);
	
	//s0为起始位置，sf为重点位置，t运行时间，v0起始速度，vf终点速度，vc平均速度
	HALFCOS_COEF_RESULT halfcos_coef(double s0, double sf, double t, double v0 = 0, double vf = 0, double vc = 0);

	QUATER_CUBIC_COEF_RESULT quater_cubic_coef(vector Q0, vector Qf, double W0, double Wf, double tf);

	//将旋转矩阵转换为四元数向量，T可以是齐次矩阵或旋转矩阵
	vector quater_r2q(matrix T);
	
	//四元数向量的叉积
	vector quater_cross(vector Q1, vector Q2);

	//直线轨迹规划参数计算，四元数版
	//T1、T2为齐次矩阵，TVc为直线时间和速度参数，V_type为时间模式或速度模式
	unsigned int line_traj_para(matrix T1, matrix T2, vector TVc, double t_interval,bool V_type);
};
