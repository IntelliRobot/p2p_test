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

	//���߲���

	Vector tcp; //tcp����ϵƫ��xyz,��6������λ��׼����̬����

	Vector tool_mass_center;//����λ��

	matrix tool_mass;//��������
	

};



struct GEAR_RATIO

{

	//������ٱ�

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
	Vector q_zerop= (0.0, -0.5*PI, 0.0, 0.0, 0.0, 0.0);//��������λ
	
	Vector q_n;//��ǰ�ؽڽǶ�

	Vector q_L;//ǰһ���ؽڽǶ�

	Vector qd_n;//��ǰ�ؽڽ��ٶ�

	Vector qdd_n;//��ǰ�ؽڽǼ��ٶ�

	matrix mat_t60;

	//Posi_Pose PP_t60;

	matrix mat_Tcp;

	Vector vec_qn_e;
		
	double qd_percent;//�����ٶȰٷֱ�

	double t_interval = 0.01;
	
	
	Vector q_singular_start;
		
	Vector q_singular_end;
		
	Vector qd_singular_start;
		
	Vector qd_singular_end;

		
	
	DH_PARA DH;//DH����
		
	Ksew Ksew_n;//��ǰ�ؽ�����ֵ
		
	Ksew Ksew_L;//ǰһ���ؽ�����ֵ

	Ksew Ksew_temp;//�ؽ�����ֵ��ʱֵ
		
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

	//��q��ѡ����q0����cfgһ����һ��⣻���ؽ��Kֵ
	Vector Qcfg_M(const matrix &QM);

	Vector Q_samecfg(const matrix &Qm, const Vector &q0);

	Vector Q_samecfg(const matrix &Qm, const int &cfg);
	

	
	int Singularity_jump(const int index, Cartesian_Line &robot_traj);

	void Singularity_cubic_rt(int N);
	
	double Ksew_select(const Ksew &Ksew, int Singular_flag);
	
	void Singularity_monitor();//������

	void Joint_speed_monitor();
	
	Vector Ikine_nearest(const matrix &Tn, Vector &q_n);//matlab ��Ӧ ik_Nearest_cfg

	matrix Ikine_matrix(const matrix &Tn);//matlab ��Ӧ ikine_num_sub

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