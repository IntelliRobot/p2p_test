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

// ��̬����������ֵ;
//�Ѿ�����ͨ���ù��ܰ���������slerp������mspline������mslpine�����spline;
//����mspline�����������趨��ʼ���ٶ�W0���յ���ٶ�Wf;
//δ���Զ��squad����ʱ����;
//���ֵ��������ÿѭ�������̬Qt�����ٶ�Wt���Ǽ��ٶ�Wtdot
//		  Ҳ������һ�����ȫ�����csv�ļ�
//��������������庯��


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
	vector<Quaternion> Qsn; //vector �����Сд����д��������Сд���������ͣ�����
	vector<Quaternion> Sin; //vector �����Сд����д��������Сд���������ͣ�����
	
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
	// ���֮����̬��ֵ<�������������squad>
	Posture_Interp(vector<Quaternion> Qsn, Vector Tsn, enum Quaternion_method method, Vector W0 = ZeroVector3,Vector Wf=ZeroVector3, double t_interval = 0.01);
		
	//����֮����̬slerp
	Posture_Interp(const Quaternion &Q0, const Quaternion &Qf, enum Quaternion_method method, double t_interval = 0.01);
	
	//����֮�����β�ֵ
	Posture_Interp(const Quaternion &Q0, const Quaternion & Qf, Vector W0_, Vector Wf_, double Tf,enum Quaternion_method method, double t_interval = 0.01);

	//����
	~Posture_Interp();

	//slerpϵ����ʼ��
	void Slerp_ini(const Quaternion &Q0, const Quaternion &Qf);

	// squadϵ����ʼ��
	void Squad_ini();

	//�������������ʼ��
	void Mspline_ini();

	////�����������ϵ��
	void Mspline_coef();

public:

	// slerp��ֵ
	Quaternion Slerp(double t);

	void Squad();

	void Mspline();
	


private:

	//����Ϊ��������������̼��㺯��
	void Omega_matrix_coef_abc( Vector &a_coef,  Vector  &b_coef,  Vector &c_coef);

	matrix Omega_matrix_coef_dk(matrix &omega_prev, const Vector &a_coef, const Vector &b_coef, const Vector &c_coef, const Vector &Wi, const Vector &Wf);

	Vector Rvec_wf(const Vector &ev, const double &dtheta, Vector &wf);

	Vector B_coef_vec(const Vector &ev, const double &dtheta, const Vector &xvec);


	Vector B_coef_inv(const Vector &ev, const double &dtheta, const Vector &xvec);

	matrix quater_Mspline_alpha(double dti_frac,int i);

	void quater_multi_spline_Wk(const Vector &Wi, const Vector &Wf);

};


