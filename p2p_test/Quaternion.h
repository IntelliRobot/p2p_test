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

	//zyx ŷ����
	double A; //��Z�� yaw
	double B; //��y�� pitch
	double C; //��x�� roll

};



class Quaternion
{
public:
	//��Ԫ��4������
	 double w;
	 double x;
	 double y;
	 double z;
	//Quarternion_Cubic_coef Cubic_coef;

public:
	//
	Quaternion();
	
	//���캯��
	Quaternion(double w, double x, double y, double z);

	//�������캯��
	Quaternion(const Quaternion & theother);

	// ����ת����ת��Ϊ��Ԫ��������T��������ξ������ת����
	Quaternion( matrix T);

	// ͨ����λ��ת���ת�Ƕ���,Exp() e^AA
	Quaternion(Axis_Angle &AA);

	//���캯����������תΪ��Ԫ��
	Quaternion(const Vector & P1);

	//ZYXŷ���ǹ�����Ԫ��
	Quaternion(double yaw, double pitch, double roll);


	~Quaternion();
	
	//��Ԫ�����
	double Dot(const Quaternion &Q1)const;
		
	//Q1Ϊ4Ԫ����ʾ����ת������Ԫ������
	Quaternion Inverse()const;

	//��Ԫ�������Ĳ��
	 Quaternion  Cross(const Quaternion &Q1)const;

	 	
	
	//������Ԫ��֮��ļн�,theta ����ת��
	Axis_Angle Log()const;


	//��Ԫ���ķ���
	double Norm() const;

	//��Ԫ��תΪ��ת����
	matrix Rot() const;
	
	//ZYXŷ����תΪ��Ԫ��
	//Quaternion Euler2Qt(double yaw, double pitch, double roll);

	//��Ԫ��תΪZYXŷ����,Ҳ����RPY
	EulerAngle ToEuler() const;

	const Vector ToVector( int p) const;

	const matrix  Mat_cross()const;

	// ��ʾ��Ԫ��
	void Display();


public:

	Quaternion  operator+(const Quaternion &Q1) const;

	Quaternion  operator*(double s) const;

	Quaternion operator-(const Quaternion &Q1) const;

	//������Ԫ�����
	 Quaternion  operator * (const Quaternion &Q1)const;

	//��Ԫ����ֵ
	const Quaternion & operator  =(const Quaternion &Q1);

	// ��Ԫ����������ˣ���ͬ����ת������������ˣ����������б任
	Vector operator *( Vector &P1)const;

	

private:

};

