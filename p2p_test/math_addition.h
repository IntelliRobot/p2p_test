#pragma once
#include <math.h>
#include "vector.h"
#include "matrix.h"
#include "global.h"
class Quaternion;



//数值与向量相乘
Vector operator *(double value, const Vector & theVector);

//向量与向量相＋
Vector operator +(const Vector &V1, const Vector &V2);

//向量与向量相-
Vector operator -(const Vector &V1, const Vector &V2);


//向量与向量相*
Vector Vmultiply(const Vector &V1, const Vector &V2);

//向量与向量相/
Vector Vdevide(const Vector &V1, const Vector &V2);
//实数与向量相/
Vector Vdevide(const double &V1, const Vector &V2);

//数值与矩阵相乘
matrix operator * (double value, matrix & theMatrix);

//数值与向量相加
Vector operator +(double value, Vector & theVector);

//数值与向量相减
Vector operator -(double value, Vector & theVector);

//数值与矩阵相加
matrix operator + (double value, matrix & theMatrix);

//数值与矩阵相减
matrix operator - (double value, matrix & theMatrix);

Vector Vsin(const Vector &X);
Vector Vcos(const Vector &X);


//向量的点积
double dot(const Vector  &a, const Vector  &b);
//向量的叉积
Vector cross(Vector const &a, Vector const &b);

//四元数与四元数相*
Quaternion cross(const Quaternion &Q1, const Quaternion &Q2);

// Vector to Axis_Angle

Axis_Angle Vector2AA(const Vector &V1);

Quaternion Vector2Q(const Vector &V1);

Posi_Pose Matrix2PP( matrix &M1);

int Sign(double x);

double rad2deg(const double r);

double deg2rad(const double & deg);

Vector deg2rad(const Vector  &deg);

void Cubic_spline_coef(const Vector &q0, const Vector &qf, const Vector &qd0, const Vector &qdf, double tf, Cubic_Coef &CC);


double Cubic_max_tf(const Vector & s, const Vector & qd0, const Vector & qdf, const Vector & qdmax, const Vector & qddmax);

void Cubic_spline_rt(Vector &qt, Vector &qdt, Vector &qddt, const Cubic_Coef &CC, double &t);

Vector Halfcos_Tf(const Vector & s, const Vector & qd0, const Vector & qdf, const Vector & qdmax, const Vector & qddmax);

void Halfcos_coef(const Vector &q0, const Vector &qf, const Vector &qd0, const Vector &qdf, enum TIMEorSPEED method,double tf,
	const Vector &V_Vc_Wc, const Vector &qddmax, Halfcos_Coef &HC, const int &tf_max_index);

void Halfcos_rt(Vector &qt, Vector &qdt, Vector &qddt,const Halfcos_Coef &HC, double &t);

Vector Trapez_Tf(const Vector & s, const Vector & qd0, const Vector & qdf, const Vector & qdmax, const Vector & qddmax);

void Trapez_coef(const Vector &q0, const Vector &qf, const Vector &qd0, const Vector &qdf, enum TIMEorSPEED method, double tf, 
	const Vector &qdc, const Vector &qddmax, Trapez_Coef &TC,const int &tf_max_index);

void Trapez_rt(Vector &qt, Vector &qdt, Vector &qddt, Trapez_Coef &TC, double &t);