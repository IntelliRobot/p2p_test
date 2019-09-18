#pragma once
#include "stdafx.h"
#include "Quaternion.h"
#include "math_addition.h"
#include "matrix.h"

using namespace std;
Quaternion::Quaternion()
{
}
//直接定义四元数
 Quaternion::Quaternion(double w1, double x1, double y1, double z1)

{
	//double mag = sqrt(w1*w1 + x1*x1 + y1*y1 + z1*z1 );
	
	//w = w1/mag;

	//x = x1/ mag;
	//	
	//y = y1/ mag;

	//z = z1/ mag;

	w = w1;// mag;

	x = x1;// mag;

	y = y1;// mag;

	z = z1;// mag;
	
}

//拷贝构造函数
Quaternion::Quaternion(const Quaternion & theother) 
{
	 w = theother.w;
	 x = theother.x;
	 y = theother.y;
	 z = theother.z;
	//cout << "copy construct" << endl;
}

//析构函数

Quaternion::~Quaternion()
{
	//cout << "deconstruct" << endl;
}

//构造函数，旋转矩阵转为四元数
Quaternion::Quaternion(  matrix T)
{
	//提取旋转矩阵
	 matrix r=T.SubMatrix(1, 1, 3, 3);

	//计算qs kx xy kz
	double qs = sqrt(r.Trace() + 1) / 2.0;
	double kx = r(3, 2) - r(2, 3);  //Oz - Ay
	double ky = r(1, 3) - r(3, 1);  //Ax - Nz
	double kz = r(2, 1) - r(1, 2);  //Ny - Ox

	//计算符号
	int sign_kx = kx >= 0 ? 1 : -1;
	int sign_ky = ky >= 0 ? 1 : -1;
	int sign_kz = kz >= 0 ? 1 : -1;

	//计算kx1 ky1 kz1
	//对于一个旋转矩阵，kx1、ky1、kz1都是大于等于0的数
	double kx1 = r(1, 1) - r(2, 2) - r(3, 3) + 1;
	double ky1 = r(2, 2) - r(1, 1) - r(3, 3) + 1;
	double kz1 = r(3, 3) - r(1, 1) - r(2, 2) + 1;

	w = qs;
	x = 0.5 * sign_kx * sqrt(kx1);
	y = 0.5 * sign_ky * sqrt(ky1);
	z = 0.5 * sign_kz * sqrt(kz1);

}
//构造函数，通过单位旋转轴和转角定义四元数,相当于EXP函数
Quaternion::Quaternion(Axis_Angle &AA)

{
	double theta = AA.Theta;
	 w = cos(theta);
	 x = sin(theta)*AA.Qvec.pVector[0];
	 y = sin(theta)*AA.Qvec.pVector[1];
	 z = sin(theta)*AA.Qvec.pVector[2];

	//double mag = sqrt(w1*w1 + x1*x1 + y1*y1 + z1*z1);
	//
	//w = w1 / mag;

	//x = x1 / mag;

	//y = y1 / mag;

	//z = z1 / mag;

}

//构造函数，将向量转为四元数
Quaternion::Quaternion(const  Vector & P1)
{   
	w = 0;

	x = P1.pVector[0]; 

	y = P1.pVector[1]; 

	z = P1.pVector[2]; 

}


//构造函数，ZYX欧拉角构造四元数
Quaternion::Quaternion(double yaw, double pitch, double roll) //ZYX角度输入，也即ABC输入，角度输入
{
	double  angle;
	double  sinRoll, sinPitch, sinYaw, cosRoll, cosPitch, cosYaw;

	angle = deg2rad(yaw * 0.5);
	sinYaw = sin(angle);
	cosYaw = cos(angle);

	angle = deg2rad(pitch * 0.5);
	sinPitch = sin(angle);
	cosPitch = cos(angle);

	angle = deg2rad(roll * 0.5);
	sinRoll = sin(angle);
	cosRoll = cos(angle);

	double _w = cosRoll*cosPitch*cosYaw + sinRoll*sinPitch*sinYaw;
	double _z = cosRoll*cosPitch*sinYaw - sinRoll*sinPitch*cosYaw;
	double _y = cosRoll*sinPitch*cosYaw + sinRoll*cosPitch*sinYaw;
	double _x = sinRoll*cosPitch*cosYaw - cosRoll*sinPitch*sinYaw;


	double mag = sqrt(_x *_x + _y*_y + _z *_z + _w*_w);
	x = _x / mag;
	y = _y / mag;
	z = _z / mag;
	w = _w / mag;

}

//Q1为4元数表示的旋转矩阵，四元数的逆
Quaternion Quaternion::Inverse()const
{
	
	//前一位不变
	double wq = w;
	//后三位取反
	double xq = -1 * x;
	double yq = -1 * y;
	double zq = -1 * z;
	return Quaternion (wq,xq,yq,zq);
	
	//return Q_result;
}

//四元数点积
double Quaternion::Dot(const Quaternion &Q1) const
{
	return Q1.w * w + Q1.x * x + Q1.y * y + Q1.z * z;
}


//四元数叉乘
 Quaternion Quaternion::Cross(const Quaternion & Q1) const
{
	double w0;
	double x0;
	double y0;
	double z0;
	w0 = Q1.w * w - Q1.x * x - Q1.y * y - Q1.z * z;
	x0 = w * Q1.x + Q1.w * x + Q1.z * y - Q1.y * z;
	y0 = w * Q1.y + Q1.w * y + Q1.x * z - Q1.z * x;
	z0 = w * Q1.z + Q1.w * z + Q1.y * x - Q1.x * y;

	//Quaternion Q_result(w0,x0,y0,z0);
	//w = w0;
	//x = x0;
	//y = y0;
	//z = z0;

	//return *this;
	return Quaternion(w0, x0, y0, z0);
}

//四元数2范数
double Quaternion::Norm() const
{
	return w*w + x*x + y*y + z*z;
}

matrix Quaternion::Rot() const
{
	matrix result(3, 3);

	result(1, 1) = 1 - 2 * (y * y + z * z); result(1, 2) = 2 * (x*y - w*z);         result(1, 3) = 2 * (x*z + w*y);
	result(2, 1) = 2 * (x*y + w*z);         result(2, 2) = 1 - 2 * (x *x + z * z); result(2, 3) = 2 * (y*z - w*x);
	result(3, 1) = 2 * (x*z - w*y);         result(3, 2) = 2 * (y*z + w*x);         result(3, 3) = 1 - 2 * (x *x + y * y);

		
	return result;
}



EulerAngle Quaternion::ToEuler() const
{
	
	//Z-Y-X欧拉角
	const double Epsilon = 0.001; //奇异区域阈值 
	const double Threshold_plus = 0.5 + Epsilon;//w*y - x*z=0.5时奇异
	const double Threshold_minus = 0.5 - Epsilon;

	double TEST = w*y - x*z;

	 EulerAngle euler_result;

	if (TEST < Threshold_plus && TEST > Threshold_minus) // 
	{
		int sign = Sign(TEST);

		euler_result.A= 0; // yaw Z轴

		euler_result.B = sign * (PI / 2.0); // pitch Y轴

		euler_result.C = -2 * sign * (double)atan2(x, w); // roll X轴

	}
	else
	{
		euler_result.A = atan2(x*y + w*z, 0.5 - y*y - z*z);
		euler_result.B = asin(-2 * (x*z - w*y));
		euler_result.C = atan2(y*z + w*x, 0.5- x*x - y*y );
	}

	euler_result.A = rad2deg(euler_result.A);
	euler_result.B = rad2deg(euler_result.B);
	euler_result.C = rad2deg(euler_result.C);
	return euler_result;
		
}

const Vector Quaternion::ToVector(int p) const
{
	if (p==3)
	{
		return Vector( x, y, z);
	}
	
	else
	{
		return Vector(w, x, y, z);
	}

}

const matrix  Quaternion::Mat_cross() const
{
	double temp[12] = {-2*x,2 * w,-2 * z,2 * y,-2 * y,2 * z,2 * w,-2 * x,-2 * z,-2 * y,2 * x,2 * w};

	matrix M_temp(3, 4, temp);
	
	return M_temp;
	
}

void Quaternion::Display()
{
	printf("四元数是\n %4.4f,%4.4f,%4.4f,%4.4f \n", w,x,y,z);

}



//+号重载
Quaternion  Quaternion::operator+(const Quaternion & Q1) const
{
		
	/*w += Q1.w;
	x += Q1.x;
	y += Q1.y;
	z += Q1.z;
	return *this;*/
	
	double wq = Q1.w+w;
	double xq = Q1.x+x;
	double yq = Q1.y+y;
	double zq = Q1.z+z;

	return Quaternion(wq, xq, yq, zq);
}

//实数缩放
Quaternion  Quaternion::operator*(double s) const
{
	double wq, xq, yq, zq;
	 wq = w*s;
	 xq = x*s;
	 yq = y*s;
	 zq = z*s;
	 //return *this;
	;
	return Quaternion (wq, xq, yq, zq);
}

//-号重载
Quaternion Quaternion::operator-(const Quaternion & Q1) const
{
	
	double wq = w-Q1.w;
	double xq = x-Q1.x;
	double yq = y-Q1.y;
	double zq = z-Q1.z;
	return Quaternion (wq, xq, yq, zq);
	//return Q_result;
	//return *this;
}

//*重载为叉乘
 Quaternion Quaternion::operator*(const Quaternion & Q1)const 
{
	return Cross(Q1);
	 //return *this;
}

//=重载
const Quaternion & Quaternion::operator=( const Quaternion & Q1)
{
	w = Q1.w;
	x = Q1.x;
	y = Q1.y;
	z = Q1.z;
	return Q1;
}

//四元数与向量相乘，相当于四元数代表的旋转矩阵与向量相乘，对向量进行旋转
Vector Quaternion::operator*( Vector & P1) const
{
	//double	x1 = P1(1); 
	//double	y1 = P1(2);
	//double  z1 = P1(3);
	Quaternion Q1(P1); 
	//Q1 = Q1*P1.Norm();
	Quaternion Q2 = *this*Q1*Inverse();
	Vector P2(3, VERT);
	P2.pVector[0] = Q2.x;
	P2.pVector[1] = Q2.y;
	P2.pVector[2] = Q2.z;
	//P2.SetValue(1, Q2.x);
	//P2.SetValue(2, Q2.y);
	//P2.SetValue(3, Q2.z);
	return P2;
}

//Log运算，提取,theta,单位弧度;旋转轴单位向量
Axis_Angle Quaternion::Log( ) const
{	
	Vector Axis(3, VERT);
	double Theta,Sintheta;
	double Norm;
	double w1, x1, y1, z1;
	w1 = w;
	x1 = x;
	y1 = y;
	z1 = z;
	Norm= sqrt(w*w + x*x + y*y + z*z);
	if (Norm >Epsilon)
	{
		w1 = w/Norm;
		x1 = x/Norm;
		y1 = y/Norm;
		z1 = z/Norm;
	}
	
	double cosTheta = w1;
	if (cosTheta < 0)
	{
		cosTheta = -1 * cosTheta;
		Axis = Axis*(-1);
	}
	Theta = acos(cosTheta);
	Sintheta = sin(Theta);

	if (fabs(Sintheta)>Epsilon)
	{
		x1 = x1 / Sintheta;
		y1 = y1 / Sintheta;
		z1 = z1 / Sintheta;
	}
	
	Axis.SetValue(1, x1);
	Axis.SetValue(2, y1);
	Axis.SetValue(3, z1);

	Axis_Angle result = { Theta,Axis };
			
	return result;


}