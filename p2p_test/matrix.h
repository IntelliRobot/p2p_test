#pragma once

#include "stdafx.h"

//Vector类的声明
class Vector;
class Quaternion;
//矩阵类
class matrix
{
public:
	//矩阵存储位置
	double * pMatrix = NULL;

	//行列数
	unsigned int ROW = 0;
	unsigned int COL = 0;
	Vector RVector;

public:
	
	matrix();
	//构造函数，分配内存
	matrix(unsigned int m_row, unsigned int n_col);

	//数组构造成矩阵
	matrix(unsigned int m_row, unsigned int n_col, double *ar);

	//拷贝构造函数
	matrix(const matrix & theMatrix);

	matrix(const matrix & Rot,const Vector &P);


	//析构函数，收回内存
	~matrix();

	//()运算符重载
	double & operator () (unsigned int m_row, unsigned int n_col);

	//赋值运算符重载
	const matrix & operator = (const matrix & theMatrix);

	//加减运算符重载
	matrix operator + (const matrix & adder);
	matrix operator - (const matrix & adder);

	//乘法运算符重载
	matrix operator * (const double value);
	matrix operator * (const matrix & value);
	Vector operator * (const Vector & value);

	//除法运算符重载
	matrix operator / (const double value);

public:
	//根据脚标提取矩阵元素
    inline double GetValue(unsigned int m_row, unsigned int n_col) const;

	//根据脚标设置矩阵元素
	inline void SetValue(unsigned int m_row, unsigned int n_col, double value);

	//根据向量矩阵元素
	 void SetValue(unsigned int i, const Vector  &value, enum VECTOR_TYPE vh);

	//根据四元数元素
	 void SetValue(unsigned int i, const Quaternion &value, enum VECTOR_TYPE vh);

	//提取子矩阵
	const matrix SubMatrix(unsigned int start_row, unsigned int start_col, unsigned int end_row, unsigned int end_col)const;

	//提取子向量,复制
	Vector SubVector(unsigned int rc, enum VECTOR_TYPE vh)const;

	//提取子行向量,引用
	Vector RowVector(unsigned int rc);
	
	//将只有一行或一列的矩阵转换成向量
	Vector AsVector() const;
	
	//求矩阵的转置矩阵
	matrix Transposition() const;

	//求矩阵的行列式
	double Determinant();

	//求矩阵的逆矩阵
	matrix Inverse();

	//对角线元素之和
	double Trace();



	//将矩阵元素输出到屏幕
	void Display();


	//加减运算符
	matrix operator +(const double theValue);
	matrix operator -(const double theValue);

	//改变矩阵维数
	void ChangeDimension(unsigned int uiRow, unsigned int uiCol);

	//判断矩阵是否是0矩阵
	bool IsZero();

private:
	//求去除行m_row和列n_col剩下的矩阵
	matrix Remainder(unsigned int m_row, unsigned int n_col);

	//求-1的n次幂
    inline int NegtiveOnePower(unsigned int n);
};


