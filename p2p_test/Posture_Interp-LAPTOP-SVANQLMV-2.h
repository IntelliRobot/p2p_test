#pragma once
#include "stdafx.h"
#include "vector.h"
#include "matrix.h"
#include "Quaternion.h"
#include "math_addition.h"

#include <math.h>
#include <vector>
#include <iostream>
#include <fstream> 
using namespace std;

//class Vector;
//class matrix;
//class Quaternion;

// 姿态得两点或多点插值;
//已经测试通过得功能包括，两点slerp、两点mspline、三点mslpine、五点spline;
//其中mspline方法，可以设定起始角速度W0和终点角速度Wf;
//未测试多点squad，暂时不用;
//输出值：可以是每循环输出姿态Qt，角速度Wt，角加速度Wtdot
//		  也可以是一次输出全部点得csv文件
//输入参数：看具体函数


struct Quarternion_Mspline_coef
{
	matrix A3r2;
	matrix A3r3;
	matrix A3r4;

	matrix A2r1;
	matrix A2r2;
	matrix A2r3;

	matrix  A1r1;
	matrix  A1r2;
};

enum Quaternion_method {slerp,squad,mspline};

//struct Posture_Data
//{
//	Quaternion Q;
//	vector W;
//	vector Wdot;
//};


class Posture_Interp
{
public:

	int Point_N;
	Quaternion Q0;
	Quaternion Qf;
	Quaternion Qt;
	Quaternion Qdt;
	Quaternion Qddt;
	
	matrix Qt_matrix;
	Quaternion DeltaQ;
	Vector Qvec;
	vector<Quaternion> Qsn; //vector 区别大小写，大写是向量，小写是数据类型，容器
	vector<Quaternion> Sin; //vector 区别大小写，大写是向量，小写是数据类型，容器
	
	Vector W0;
	Vector W0dot = ZeroVector3;
	Vector Wf;
	Vector Wfdot = ZeroVector3;
	Vector Wt;
	matrix Wt_matrix;
	Vector Wtdot;
	matrix Wtdot_matrix;
	Vector Tsn;
	Vector Tsn_percent;
	double Slerp_costheta;
	double Slerp_theta;
	double Slerp_sintheta;
	double Slerp_invsintheta;
	double Tf;
	int Na;
	double t_interval=0.01;
	enum Quaternion_method method;
	int max_iter = 20;
	int iters;
	//vector < Vector > ev;
	//vector<Vector> Wk;
	matrix ev;//= matrix(Point_N-1,3);
	matrix Wk;//= matrix(Point_N,3 );
	Vector dalpha;
	Vector dt;
	Quarternion_Mspline_coef Ms_Coef;
	//Quarternion_Slerp_coef S_Coef;
	//Quarternion_Squad_coef Sq_Coef;

public:

	Posture_Interp();
	// 多点之间姿态插值<多点三次样条，squad>
	Posture_Interp(vector<Quaternion> Qsn, Vector Tsn, enum Quaternion_method method, Vector W0 = ZeroVector3,Vector Wf=ZeroVector3, double t_interval = 0.01);
		
	//两点之间姿态slerp
	Posture_Interp(const Quaternion &Q0, const Quaternion &Qf, enum Quaternion_method method, double t_interval = 0.01);
	
	//两点之间三次插值
	Posture_Interp(const Quaternion &Q0, const Quaternion & Qf, Vector W0_, Vector Wf_, double Tf,enum Quaternion_method method, double t_interval = 0.01);

	//析构
	~Posture_Interp();

	//slerp系数初始化
	void Slerp_ini(const Quaternion &Q0, const Quaternion &Qf);

	// squad系数初始化
	void Squad_ini();

	//多点三次样条初始化
	void Mspline_ini();

	////多点三次样条系数
	void Mspline_coef();

public:

	// slerp插值
	Quaternion Slerp(double t);

	void Squad();

	void Mspline();
	


private:

	//以下为多点三次样条过程计算函数
	void Omega_matrix_coef_abc( Vector &a_coef,  Vector  &b_coef,  Vector &c_coef);

	matrix Omega_matrix_coef_dk(matrix &omega_prev, const Vector &a_coef, const Vector &b_coef, const Vector &c_coef, const Vector &Wi, const Vector &Wf);

	Vector Rvec_wf(const Vector &ev, const double &dtheta, Vector &wf);

	Vector B_coef_vec(const Vector &ev, const double &dtheta, const Vector &xvec);


	Vector B_coef_inv(const Vector &ev, const double &dtheta, const Vector &xvec);

	matrix quater_Mspline_alpha(double dti_frac,int i);

	void quater_multi_spline_Wk(const Vector &Wi, const Vector &Wf);

};


