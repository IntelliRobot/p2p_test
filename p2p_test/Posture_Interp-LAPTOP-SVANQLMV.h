#pragma once
#include "stdafx.h"
#include "Quaternion.h"
#include "matrix.h"
#include <math.h>
#include <vector>
#include <iostream>
#include "Vector.h"
#include "math_addition.h"
using namespace std;

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

struct Quarternion_Slerp_coef
{};
struct Quarternion_Squad_coef 
{};
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
	Quaternion DeltaQ;
	Vector Qvec;
	vector<Quaternion> Qsn; //vector 区别大小写，大写是向量，小写是数据类型，容器
	vector<Quaternion> Sin; //vector 区别大小写，大写是向量，小写是数据类型，容器
	Vector W0;
	Vector W0dot;
	Vector Wf;
	Vector Wfdot;
	Vector Wt;
	Vector Wtdot;
	Vector Tsn;
	Vector Tsn_percent;
	double Slerp_costheta;
	double Slerp_theta;
	double Slerp_invsintheta;
	double Tf;
	int Na;
	double t_interval=0.01;
	enum Quaternion_method method;
	matrix ev = matrix(3, Point_N-1);
	Vector dtheta;
	Vector dt;
	Quarternion_Mspline_coef Ms_Coef;
	//Quarternion_Slerp_coef S_Coef;
	Quarternion_Squad_coef Sq_Coef;

public:


	// 构造函数，四元数三次样条插值,求插值参数
	//Posture_Interp();

	Posture_Interp(vector<Quaternion> Qsn, Vector Tsn, enum Quaternion_method method, Vector W0 = ZeroVector3,Vector Wf=ZeroVector3, double t_interval = 0.01);

	Posture_Interp(const Quaternion &Q0, const Quaternion &Qf, enum Quaternion_method method, double t_interval = 0.01);
	Posture_Interp(const Quaternion &Q0, const Quaternion &Qf, Vector W0_, Vector Wf_, enum Quaternion_method method, double t_interval = 0.01);

	~Posture_Interp();

	void Slerp_ini(const Quaternion &Q0, const Quaternion &Qf);

	void Squad_ini();

	void Mspline_ini();

	void Mspline_coef();

public:

	Quaternion Slerp(double t);

	void Squad();

	void Mspline(double t);
	


private:

	Vector B_coef_inv(const Vector &ev,  Vector &dtheta, const Vector &xvec);

};


