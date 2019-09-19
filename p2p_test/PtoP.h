#pragma once
#include "stdafx.h"
//#include "Vector.h"
//#include "Quaternion.h"
#include "global.h"
#include "math_addition.h"
//#include <math.h>
#include <iostream>
#include <fstream> 
using namespace std;
//#include <algorithm>

//struct P_Q
//{
//	Vector P;
//	Quaternion Q;
//};

class Vector;



class PtoP
{
public:

	double	t_interval = 0.01;
	int Nw = 6;
	int Na = 0;
	int N_now = 0;
	Vector q0 = ZeroVector6;
	Vector qf = ZeroVector6;
	Vector qd0 = ZeroVector6;
	Vector qdf = ZeroVector6;
	Vector delta_q=ZeroVector6;
	Vector delta_q_abs = ZeroVector6;
	Vector V_qd_mintf = ZeroVector6;

	int tf_max_index= 0; //tf被改变标志，=1 代表向量第一个元素需要时间最长

	Cubic_Coef Cubic_coef;
	Halfcos_Coef Half_coef;

	Trapez_Coef Trapez_coefs;
	
	//Vector A = ZeroVector6;
	//Vector B = ZeroVector6;
	//Vector C = ZeroVector6;
	//Vector D = ZeroVector6;
	
	double tf;
	double tf_min_limit;

	double qd_percent=0.5;
	Vector qd= ZeroVector6;
	double qdd_percent = 0.8;
	Vector qdd = ZeroVector6;
	Vector qd_const = ZeroVector6;

	double t;
	Vector qt= ZeroVector6;
	Vector qdt= ZeroVector6;
	Vector qddt= ZeroVector6;
	enum TIMEorSPEED method ;
	enum P2Pmethod cht;

	//matrix qN;


public:
	PtoP();

	PtoP(Vector q0_1, Vector qf_1, enum P2Pmethod cht= halfcos, enum TIMEorSPEED method=speed, double tf_s=0.1,Vector qd0_1=ZeroVector6, Vector qdf_1=ZeroVector6, double qddp=0.8,double t_interval_=0.01);

	~PtoP();

	void cubic_coef();
	void halfcos_coef();
	void linear_coef();
	void trapez_coefs();
			
	void realtime(int N);
	void offline(double interval_out=0.01);


private:

};

