#pragma once

#include "stdafx.h"
 
#include "vector.h"

#include "matrix.h"

#include "global.h"

#include "PtoP.h"

#include "Quaternion.h"

#include "math_addition.h"

#include "FunctionReturnMessage.h"

#include "Cartesian_line.h"



struct TOOL_PARA

{

	//工具参数

	Vector tcp; //tcp坐标系偏移xyz,以6轴坐标位基准，姿态不变

	Vector tool_mass_center;//重心位置

	matrix tool_mass;//惯量矩阵
	

};



struct GEAR_RATIO

{

	//各轴减速比

	double gr1;

	double gr2;

	double gr3;

	double gr4;

	double gr5;

	double gr6;

};




class robot_phr

{

public:
	Vector q_zerop= (0.0, -0.5*PI, 0.0, 0.0, 0.0, 0.0);//机器人零位
	
	Vector q_n;//当前关节角度

	Vector q_L;//前一个关节角度

	Vector qd_n;//当前关节角速度

	Vector qdd_n;//当前关节角加速度

	matrix mat_t60;

	//Posi_Pose PP_t60;

	matrix mat_Tcp;

	Vector vec_qn_e;
		
	double qd_percent;//最大角速度百分比

	double t_interval = 0.01;
	
	
	Vector q_singular_start;
		
	Vector q_singular_end;
		
	Vector qd_singular_start;
		
	Vector qd_singular_end;

		
	
	DH_PARA DH;//DH参数
		
	Ksew Ksew_n;//当前关节奇异值
		
	Ksew Ksew_L;//前一个关节奇异值

	Ksew Ksew_temp;//关节奇异值临时值
		
	//Ksew Ksew_limit;
		
	int Singular_flag = 0;

	int Jspeed_exceed_flag = 0;

	int CFG=0;

	PtoP Singular_cubic;

	//int CFG_temp = 0;
		
	//traj_plan robot_traj;

	
public:

	robot_phr(const DH_PARA dh);


	~robot_phr();

	//void P2P(Posi_Pose PP0, Posi_Pose PPf, enum P2Pmethod cht = halfcos, enum TIMEorSPEED method = time, double tf_s = 0.1, Vector V0_1= ZeroVector6, Vector Vf_1= ZeroVector6 ,  double qddp = 0.8,double t_interval_=0.01);
	
	void K_sew(const Vector &q_n);
	
	int Qcfg_K(const Vector &q_n);

	//从q中选出与q0构型cfg一样的一组解；返回解和K值
	Vector Qcfg_M(const matrix &QM);

	Vector Q_samecfg(const matrix &Qm, const Vector &q0);

	Vector Q_samecfg(const matrix &Qm, const int &cfg);
	

	
	int Singularity_jump(const int index, Cartesian_Line &robot_traj);

	void Singularity_cubic_rt(int N);
	
	double Ksew_select(const Ksew &Ksew, int Singular_flag);
	
	void Singularity_monitor();//奇异监控

	void Joint_speed_monitor();
	
	Vector Ikine_nearest(const matrix &Tn, Vector &q_n);//matlab 对应 ik_Nearest_cfg

	matrix Ikine_matrix(const matrix &Tn);//matlab 对应 ikine_num_sub

	Vector  Ikine_cfg(const matrix &Tn, int cfg=-1);

	matrix Ikine_Tcp2t6(const Posi_Pose &Tcp);

	//matrix Ikine_Tcp2t6(const matrix &Tcp);

	matrix* fkine_num_d3(const Vector &q);
	matrix* Fkine_T6toTcp( matrix &T6);
	matrix Qn2Qn_efort( matrix &qn);
	matrix Qn_e2Qmotor_efort( matrix &qn_e);
	Vector Qn_efort2Qn(const Vector &qn_e);
	matrix jacob_num_sub(const Vector &q);
	matrix invJ_num_sub_11(const Vector &q);
	matrix invJ_num_sub_22(const Vector &q);
	Vector JJdot_qdot_num(const Vector &q, const Vector &qdot);
	Vector qd2v(const Vector &q, const Vector &qd);
	Vector v2qd(const Vector &v, const Vector &q);
	Vector qdd2a(const Vector &q, const Vector &qd, const Vector &qdd);
	Vector a2qdd(const Vector &a, const Vector &q, const Vector &qd);
};