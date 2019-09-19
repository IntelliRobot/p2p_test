#pragma once
#include "stdafx.h"
//#include "Vector.h"
#include "global.h"
//#include "math_addition.h"
//#include "Posture_Interp.h"
//#include "Cartesian_Line.h"

//#include <math.h>

class Cartesian_traj
{
public:
	Posi_Pose PP_n;//当前位置

	Vector V_n;//当前末端速度

	double t_n;

	enum cartesian_traj_type traj_type;
	
	Vector A_n;//当前末端加速度
	//Vector A_0;
	//Vector A_f;
	int Na;//插值步数
	int N_now=0;

	enum TIMEorSPEED method = time;
	//double tf_vcaverage = 0.1;
	double t_interval = 0.01;

	double traj_length;

public:
	Cartesian_traj(Posi_Pose PP0, Posi_Pose PPf, enum P2Pmethod cht = halfcos, enum TIMEorSPEED method = speed, enum cartesian_traj_type traj_type=line,
		double tf_lspeedp = 0.1, double acce_p = 0.8, double t_interval_ = 0.01);
	Cartesian_traj();
	~Cartesian_traj();
	
	void realtime(int N);


	

//private:

};