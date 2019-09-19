#pragma once
#include "stdafx.h"
#include "Cartesian_traj.h"
#include "Posture_Interp.h"
//#include "Vector.h"
//#include "Quaternion.h"
//#include "global.h"
//#include "math_addition.h"
//#include <math.h>


class Cartesian_Line : public Cartesian_traj
{
public:
	Posi_Pose PP_0;
	Posi_Pose PP_f;
	Vector Uvec = ZeroVector3;
	//Vector Quvec = ZeroVector3;
	Vector Vt=ZeroVector3;
	Vector At = ZeroVector3;

	Vector st_2 = ZeroVector2;
	Vector vt_2 = ZeroVector2;
	Vector at_2 = ZeroVector2;

	Quaternion Qt;
	Vector Wt = ZeroVector3;
	Vector Wdt = ZeroVector3;
	Posture_Interp Posture_line;

	Vector V_traj_length_Dalpha = ZeroVector2;
	Vector V_Vc_Wc = ZeroVector2;
	//Vector V_Vdc_Wdc = ZeroVector2;
	Vector V_Vmax_Wmax = ZeroVector2;
	Vector V_Vdmax_Wdmax = ZeroVector2;
	Vector V_V_W_mintf = ZeroVector2;
	Vector Scale_Tf;
	double Dalpha;
	double tf;
	double Vc;//直线段的线速度
	double Wc;//直线段的角速度
	double Vdmax;//直线段的线加速度
	double Wdmax;//直线段的角加速度
	double tf_min_limit;
	Cubic_Coef Cubic_coef;
	//Cubic_Coef Q_cubic_coef;
	Halfcos_Coef Half_coef;
	//Halfcos_Coef Q_half_coef;
	Trapez_Coef Trapez_coefs;
	//Trapez_Coef Q_trapz_coef;
	P2Pmethod cht;
	double acce_percent = 0.8;
	double speed_percent = 0.8;
	int tf_max_index = 0;

	//Vector V_0;
	//Vector V_f;

	double V_0_abs = 0;//初始线速度，标量
	double V_f_abs = 0;//终点线速度，标量
	double tf_speedp = 0.1;

public:

	Cartesian_Line(Posi_Pose PP0, Posi_Pose PPf,  enum P2Pmethod cht = halfcos, enum TIMEorSPEED method = speed, 
		double tf_lspeedp = 0.1,  double acce_p = 0.8, double t_interval_ = 0.01);

	//void cubic_coef_speed();
	//void cubic_coef_time();
	~Cartesian_Line();
	void cubic_coef();
	void halfcos_coef();
	void linear_coef();
	void trapez_coefs();
	void realtime_PVA();
	void realtime(int N);
};