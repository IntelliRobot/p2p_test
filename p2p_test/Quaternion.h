#pragma once
#include "stdafx.h"
//#include "global.h"
#include <math.h>

#include <iostream>
#include "FunctionReturnMessage.h"
//https://www.cnblogs.com/lvdongjie/p/5384747.html
//https://blog.csdn.net/qq_35936174/article/details/76640318
//https://blog.csdn.net/u013236946/article/details/72829663
//https://www.cnblogs.com/21207-iHome/p/6894128.html
class matrix;
class Vector;
struct Axis_Angle;

struct EulerAngle
{

	//zyx 欧拉角
	double A; //绕Z轴 yaw
	double B; //绕y轴 pitch
	double C; //绕x轴 roll

};



class Quaternion
{
public:
	//四元数4个变量
	 double w;
	 double x;
	 double y;
	 double z;
	//Quarternion_Cubic_coef Cubic_coef;

public:
	//
	Quaternion();
	
	//构造函数
	Quaternion(double w, double x, double y, double z);

	//拷贝构造函数
	Quaternion(const Quaternion & theother);

	// 将旋转矩阵转换为四元数向量，T可以是齐次矩阵或旋转矩阵
	Quaternion( matrix T);

	// 通过单位旋转轴和转角定义,Exp() e^AA
	Quaternion(Axis_Angle &AA);

	//构造函数，将向量转为四元数
	Quaternion(const Vector & P1);

	//ZYX欧拉角构造四元数
	Quaternion(double yaw, double pitch, double roll);


	~Quaternion();
	
	//四元数点积
	double Dot(const Quaternion &Q1)const;
		
	//Q1为4元数表示的旋转矩阵，四元数的逆
	Quaternion Inverse()const;

	//四元数向量的叉积
	 Quaternion  Cross(const Quaternion &Q1)const;

	 	
	
	//两个四元数之间的夹角,theta 和旋转轴
	Axis_Angle Log()const;


	//四元数的范数
	double Norm() const;

	//四元数转为旋转矩阵
	matrix Rot() const;
	
	//ZYX欧拉角转为四元数
	//Quaternion Euler2Qt(double yaw, double pitch, double roll);

	//四元数转为ZYX欧拉角,也就是RPY
	EulerAngle ToEuler() const;

	const Vector ToVector( int p) const;

	const matrix  Mat_cross()const;

	// 显示四元数
	void Display();


public:

	Quaternion  operator+(const Quaternion &Q1) const;

	Quaternion  operator*(double s) const;

	Quaternion operator-(const Quaternion &Q1) const;

	//两个四元数叉乘
	 Quaternion  operator * (const Quaternion &Q1)const;

	//四元数赋值
	const Quaternion & operator  =(const Quaternion &Q1);

	// 四元数与向量相乘，等同于旋转矩阵与向量相乘，对向量进行变换
	Vector operator *( Vector &P1)const;

	

private:

};

