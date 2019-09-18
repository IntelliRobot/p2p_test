#pragma once

#include "stdafx.h"

//matrix类的声明
class matrix;

enum VECTOR_TYPE { VERT , HORZ,};

class Vector
{
public:
	//向量存储位置
	double * pVector = NULL;

	//向量维数
	unsigned int COUNT = 0;

	//向量类型
	enum VECTOR_TYPE TYPE;

public:
	Vector();

	//构造函数，分配内存
	Vector(unsigned int uiCount, enum VECTOR_TYPE enumType = HORZ);

	//拷贝构造函数
	Vector(const Vector & theVector);

	//构造3维向量
	Vector(double x, double y, double z);

	//构造4维向量
	Vector(double w, double x, double y, double z);

	//构造1维向量
	Vector(double x);

	//构造2维向量
	Vector(double x,double y);

	//构造复制向量
	Vector(double x, int N);   


	//拼接
	Vector(const Vector &A, const Vector &B);

	//构造6维向量
	Vector(double u, double v, double w, double x, double y, double z);

	//析构函数，释放内存呢
	~Vector();

public:
	//()运算符重载
	double & operator () (unsigned int Index) const;

	//赋值运算符重载
	const Vector & operator = (const Vector & theVector);

	//加减运算符重载
	Vector operator + ( const Vector & adder);

	Vector operator - (const Vector & adder);

	Vector operator +(const double theValue);

	Vector operator -(const double theValue);


	//乘法运算符重载
	const Vector operator * (const double value);
	matrix operator * (const Vector & theVector);

	//触发运算符重载
	Vector operator / (const double value);

public:
	//根据脚标获取向量的元素
	 double GetValue(unsigned int Index) const;

	//根据脚标设置向量的元素
	 void SetValue(unsigned int Index, double Value);

	//向量的转置
	Vector Transposition() const;

	//提取子向量
	Vector SubVector(unsigned int StartIndex, unsigned int EndIndex);

	//将向量转换成矩阵
	matrix AsMatrix() const;

	//向量的模
	double Norm();

	//向量元素最大值,及索引
	double Max( int &p);

	//向量元素最大值
	double Max();

	//向量元素最小值
	double Min();

	//向量元素的平方根
	Vector Sqrt();

	//向量元素的绝对值
	Vector Abs();

	//向量元素符号
	Vector Sign();

	//将向量输出至屏幕
	void Display();

	//改变向量维数
	void ChangeDimension(unsigned int uiCount);


	//判断向量是否为0向量
	bool IsZero();



	//获取向量最小值位置索引
	unsigned int IndexOfMinValue();

	
	//获取向量最大值位置索引
	unsigned int IndexOfMaxValue();

	//向量对应元素相乘
	Vector Vmultiply(Vector theVector);

	//向量对应元素相除
	Vector Vdivide(Vector theVector);

	//判断向量是否全部>=0
	bool IsPositive();

	//正向舍入
	Vector ceil();
};