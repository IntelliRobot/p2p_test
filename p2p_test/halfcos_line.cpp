#include "stdafx.h"
#include "halfcos_line.h"
#include <iostream>
#include <fstream> 

halfcos_line::halfcos_line(Posi_Pose PP0, Posi_Pose PPf, enum TIMEorSPEED method , double tf_vcaverage , double t_interval, double V0_1 , double Vf_1 )
{
	traj_plan(PP0,PPf,method,tf_vcaverage,t_interval);

	 V_0_abs = 0;//初始线速度，标量
	 V_f_abs = 0;

}

halfcos_line::~halfcos_line()
{
}


void halfcos_line::realtime(double t)
{
}