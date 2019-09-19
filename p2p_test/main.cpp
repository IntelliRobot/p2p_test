#pragma once
#include "stdafx.h"
#include "robot_phr.h"
using namespace std;

int main(void)
{
	robot_phr R_efort(robot_efort);

	
	//double pp1[] = { 1,0,0,1400, 0,-1,0,-500, 0,0,-1,1000, 0,0,0,1 };
	//double pp2[] = { 0,0,1,1400, 0,-1,0,500, 1,0,0,1000, 0,0,0,1 };

	//matrix ppm1(4, 4, pp1);
	//matrix ppm2(4, 4, pp2);
	Posi_Pose PP1;//= Matrix2PP(ppm1);
	Posi_Pose PP2;//= Matrix2PP(ppm2);
	Posi_Pose PP1_t6;
	PP1.Posi = Vector(1400, -500, 1000);
	PP2.Posi = Vector(1400, 500, 1000);
	PP1.Pose = Quaternion(0, 1, 0, 0);
	PP2.Pose = Quaternion(0.707097942370197, 0.00353551917456039, 0.707097942370197, 0.00353551917456039);
	int cfg = 1;

	R_efort.mat_t60 = R_efort.Ikine_Tcp2t6(PP1);

	matrix qIK8 = R_efort.Ikine_matrix(R_efort.mat_t60);

	R_efort.q_n = R_efort.Ikine_cfg(qIK8, cfg);

}