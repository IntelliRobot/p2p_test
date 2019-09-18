//Author:NingHC
//Last modified:2018-12-19

#include "stdafx.h"
#include "Vector.h"
#include "matrix.h"
#include <math.h>
#include <iostream>
using namespace std;

Vector::Vector()
{
}

//构造函数，默认为行向量
Vector::Vector(unsigned int uiCount, enum VECTOR_TYPE enumType)
{
	//设定向量参数
	COUNT = uiCount;
	TYPE = enumType;

	//分配内存
	pVector = new double[uiCount];

	//向量初始化
	memset(pVector, 0, uiCount * sizeof(double));
}


Vector::Vector(double x, double y, double z)
{ //定义三维向量

	COUNT = 3;
	TYPE = VERT;
	pVector = new double[COUNT];
	pVector[0] = x;
	pVector[1] = y;
	pVector[2] = z;
}

Vector::Vector(double w,double x, double y, double z)
{ //定义三维向量

	COUNT = 4;
	TYPE = VERT;
	pVector = new double[COUNT];
	pVector[0] = w;
	pVector[1] = x;
	pVector[2] = y;
	pVector[3] = z;

}

Vector::Vector(double x)
{ //定义1维向量

	COUNT = 1;
	TYPE = VERT;
	pVector = new double[COUNT];
	pVector[0] = x;
}

Vector::Vector(double x,double y)
{ //定义2维向量

	COUNT = 2;
	TYPE = VERT;
	pVector = new double[COUNT];
	pVector[0] = x;
	pVector[1] = y;
}

Vector::Vector(const Vector &A, const Vector &B)
{
	COUNT = A.COUNT + B.COUNT;

	pVector = new double[COUNT];
	
	
	TYPE = VERT;
	for (unsigned int i = 0; i < A.COUNT; i++)
	{
		pVector[i] = A.pVector[i];
	}

	for (unsigned int i = 0; i < B.COUNT; i++)
	{
		pVector[i+A.COUNT] = B.pVector[i];
	}



}

Vector::Vector(double u, double v, double w, double x, double y, double z)
{ //定义6维向量

	COUNT = 6;
	TYPE = VERT;
	pVector = new double[COUNT];
	pVector[0] = u;
	pVector[1] = v;
	pVector[2] = w;
	pVector[3] = x;
	pVector[4] = y;
	pVector[5] = z;
}

// 构造复制向量
Vector::Vector(double x,int N)
{
	COUNT = N;
	TYPE = VERT;
	pVector = new double[COUNT];

	for (int i = 0; i < COUNT; i++)
	{
		pVector[i] = x;
	}


}

//拷贝构造函数
Vector::Vector(const Vector & theVector)
{
	//设定向量参数
	COUNT = theVector.COUNT;
	TYPE = theVector.TYPE;

	//分配内存
	pVector = new double[COUNT];

	//拷贝赋值
	memcpy(pVector, theVector.pVector, COUNT * sizeof(double));
}

//析构函数
Vector::~Vector()
{
	if (pVector != NULL) 
	{ //cout << "vector deconstruct" << endl;
		delete pVector; 
		pVector = NULL;
	}
	//delete[] pVector;
}

//利用脚标提取特定数值，Index=[1,...,n]
double & Vector::operator () (unsigned int Index) const
{
	return pVector[Index - 1];
}

//赋值操作，如果向量长度不同，以最小长度作为赋值依据
//计算条件：向量类型必须相同
const Vector & Vector::operator = (const Vector & theVector)
{
	//校验
	if ((pVector == NULL)||(COUNT<theVector.COUNT))
	{
		COUNT = theVector.COUNT;
		pVector = new double[COUNT];
		TYPE = theVector.TYPE;
		memcpy(pVector, theVector.pVector, COUNT*sizeof(double));
		
	}
	
	else
	{
		TYPE = theVector.TYPE;
		COUNT = theVector.COUNT;

		//unsigned uiCount = COUNT > theVector.COUNT ? theVector.COUNT : COUNT;

		//赋值
		memcpy(pVector, theVector.pVector, COUNT *sizeof(double));
	}
	//取最小的COUNT


	//返回
	return theVector;
}

//向量与向量相加
//计算条件：向量维数相同
Vector Vector::operator + (const Vector & adder)
{
	//进行校验
	if (COUNT != adder.COUNT) { throw("不满足向量相加条件"); }

	//定义返回结果
	Vector result(COUNT, TYPE);

	//进行加法计算
	for (unsigned int i = 1; i <= COUNT; i++)
	{
		result(i) = GetValue(i) + adder.GetValue(i);
	}

	//返回结果
	return result;
}

//向量与向量相减
//计算条件：向量维数相同
Vector Vector::operator - (const Vector & adder)
{
	//进行校验
	if (COUNT != adder.COUNT) { throw("不满足向量相加条件"); }

	//定义返回结果
	Vector result(COUNT, TYPE);

	//进行减法计算
	for (unsigned int i = 1; i <= COUNT; i++)
	{
		result(i) = GetValue(i) - adder.GetValue(i);
	}

	//返回结果
	return result;
}


//向量与数值相加
Vector Vector::operator +(const double theValue)
{
	//定义返回结果
	Vector result(COUNT, TYPE);

	//进行减法计算
	for (unsigned int i = 1; i <= COUNT; i++)
	{
		result(i) = GetValue(i) + theValue;
	}

	//返回结果
	return result;
}

//向量与数值相减
Vector Vector::operator -(const double theValue)
{
	//定义返回结果
	Vector result(COUNT, TYPE);

	//进行减法计算
	for (unsigned int i = 1; i <= COUNT; i++)
	{
		result(i) = GetValue(i) - theValue;
	}

	//返回结果
	return result;
}

//向量与数相乘
const Vector Vector::operator * (const double value)
{
	//定义结果
	Vector result(COUNT, TYPE);

	//进行计算
	for (unsigned int i = 1; i <= COUNT; i++)
	{
		result(i) = value * GetValue(i);
	}

	//返回结果
	return result;
}

//向量与向量相乘
//计算条件：向量维数相同且类型不同
matrix Vector::operator * (const Vector & theVector)
{
	//将向量转变成矩阵
	matrix a(AsMatrix());
	matrix b(theVector.AsMatrix());

	//定义返回结果
	matrix result(a.ROW, b.COL);

	//验证
	if (a.COL != b.ROW) { throw("不满足相乘条件"); return result; }

	//进行计算
	result = a*b;

	//返回结果
	return result;
}

//向量与数进行除法
Vector Vector::operator / (const double value)
{
	//验证
	if (value == 0) { throw("除数不能为0"); }

	//定义返回值
	Vector result(COUNT,TYPE);

	//每个元素进行除法
	for (unsigned int i = 1; i <= COUNT; i++)
	{
		result(i) = GetValue(i) / value;
	}

	//返回
	return result;
}

//根据脚标获取向量元素，Index=[1,...,n]
 double Vector::GetValue(unsigned int Index) const
{
	unsigned int index_p = Index - 1;
	return pVector[index_p];
}

//根据脚标设置向量元素，Index=[1,...,n]
  void Vector::SetValue(unsigned int Index, double Value)
{
	 unsigned int index_p = Index - 1;
	 pVector[index_p] = Value;
}

//向量的转置
Vector Vector::Transposition() const
{
	//定义向量
	Vector result(COUNT, TYPE == HORZ ? VERT : HORZ);

	//向量赋值
	memcpy(result.pVector, pVector, COUNT*sizeof(double));

	//返回结果
	return result;
}

//提取子向量
//计算条件1<=StartIndex<=EndIndex<=COUNT
Vector Vector::SubVector(unsigned int StartIndex, unsigned int EndIndex)
{
	//校验
	if (StartIndex > EndIndex || StartIndex == 0 || EndIndex > COUNT) { throw("不满足提取子集的条件"); }

	//定义向量
	Vector result(EndIndex - StartIndex + 1, TYPE);

	//变量赋值
	memcpy(result.pVector, &pVector[StartIndex - 1], result.COUNT * sizeof(double));

	//返回结果
	return result;
}

//向量的模
double Vector::Norm()
{
	//定义返回结果
	double SquareSum = 0;

	//计算平方和
	for (unsigned int i = 1; i <= COUNT; i++)
	{
		SquareSum += GetValue(i) * GetValue(i);
	}

	//返回平方根
	return sqrt(SquareSum);
}

//向量元素最大值

double Vector::Max(int &p)
{
	double ele_max = pVector[0];

	p = 1;

	for (unsigned int i = 1; i < COUNT; i++)
	{
		if (ele_max<pVector[i])
		{
			ele_max = pVector[i];
			p = i + 1;
		}
	}

	return ele_max;
}

double Vector::Max()
{
	double ele_max = pVector[0];



	for (unsigned int i = 1; i < COUNT; i++)
	{
		if (ele_max<pVector[i])
		{
			ele_max = pVector[i];
		
		}
	}

	return ele_max;
}

//向量元素最小值
double Vector::Min()
{
	double ele_min = pVector[0];

	for (unsigned int i = 1; i < COUNT; i++)
	{
		if (ele_min>pVector[i])
		{
			ele_min= pVector[i];
		}
	}

	return ele_min;
}

Vector Vector::Sqrt()
{
	Vector result(COUNT, TYPE);
	//计算平方和
	for (unsigned int i = 1; i <= COUNT; i++)
	{
		if (pVector[i - 1]>=0)
		{
			result.pVector[i - 1] = sqrt(pVector[i - 1]);
		}
		else
		{
			throw("错误，负数开方"); 
		}
		
	}

	return result;
}



//向量元素的绝对值
Vector Vector::Abs()
{
	Vector result(COUNT, TYPE);
	//计算平方和
	for (unsigned int i = 1; i <= COUNT; i++)
	{
		result.pVector[i-1]= fabs(pVector[i - 1]);
	}

	return result;
}


//向量元素的符号
Vector Vector::Sign()
{
	Vector result(COUNT, TYPE);
	//计算平方和
	for (unsigned int i = 1; i <= COUNT; i++)
	{
		if (pVector[i - 1]>=0)
		{
			result.pVector[i - 1] = 1;
		}
		else
		{
			result.pVector[i - 1] = -1;
		}
	}

	return result;
}

//转换成矩阵
matrix Vector::AsMatrix() const
{
	//定义返回值
	matrix result(1, COUNT);

	//赋值
	memcpy(result.pMatrix, pVector, COUNT*sizeof(double));

	//返回结果
	if (TYPE == HORZ) { return result; }
	else { return result.Transposition(); }
}

//输出向量元素到显示屏
void Vector::Display()
{
	for (unsigned int i = 1; i <= COUNT; i++)
	{
		printf("%4.4f", GetValue(i));

		if (TYPE == HORZ) { printf(" "); }
		else { printf("\r\n"); }
	}
}

//改变向量维数
void Vector::ChangeDimension(unsigned int uiCount)
{
	//获取最小维数
	unsigned int MinCount = uiCount <= COUNT ? uiCount : COUNT;

	//重新分配内存空间
	double * pTemp = new double[uiCount];

	//使内存空间为0
	memset(pTemp, 0, sizeof(double)*uiCount);

	//进行内存空间的复制
	memcpy(pTemp, pVector, sizeof(double)*MinCount);

	//释放pVector
	delete pVector;

	//pVector重新赋值
	pVector = pTemp;

	//重新设置COUNT,类型不变
	COUNT = uiCount;
}

//判断向量是否为0向量
bool Vector::IsZero()
{
	//碰到非零值，返回false
	for (unsigned int i = 0; i < COUNT; i++)
	{
		if (pVector[i] != 0) { return false; }
	}

	//返回true
	return true;
}

//向量对应元素相乘:忽略向量类型，结果类型与this一致
Vector Vector::Vmultiply(Vector theVector)
{
	//变量定义
	Vector result(COUNT, TYPE);

	//如果不满足条件，返回
	if (COUNT != theVector.COUNT) { return result; }

	//进行运算
	for (unsigned int i = 1; i < COUNT; i++)
	{
		result(i) = GetValue(i)*theVector.GetValue(i);
	}

	//返回
	return result;
}

//向量对应元素相除:忽略向量类型，结果类型与this一致
Vector Vector::Vdivide(Vector theVector)
{
	//变量定义
	Vector result(COUNT, TYPE);

	//如果不满足条件，返回
	if (COUNT != theVector.COUNT) { return result; }

	//进行运算
	for (unsigned int i = 1; i < COUNT; i++)
	{
		result(i) = GetValue(i) / theVector.GetValue(i);
	}

	//返回
	return result;
}

//判断向量是否全部>=0
bool Vector::IsPositive()
{
	//碰到小于0的元素，返回false
	for (unsigned int i = 0; i < COUNT; i++)
	{
		if (pVector[i] < 0) { return false; }
	}

	//返回true
	return true;
}



//正向舍入
Vector Vector::ceil()
{
	//变量定义
	Vector result(COUNT, TYPE);

	//计算
	for (unsigned int i = 1; i <= COUNT; i++)
	{
		if (pVector[i - 1] <= 0)
		{
			result(i) = (int)pVector[i - 1];
		}
		else
		{
			result(i) = ((int)pVector[i - 1] == pVector[i - 1]) ? (int)pVector[i - 1] : (int)pVector[i - 1] + 1;
		}
	}

	//返回
	return result;
}


//获取向量(第一个)最小值位置索引
unsigned int Vector::IndexOfMinValue()
{
	//定义变量
	unsigned int result(1); double MinValue(pVector[0]);

	//搜索最小值
	for (unsigned int i = 1; i < COUNT; i++)
	{
		if (MinValue > pVector[i])
		{
			MinValue = pVector[i];
			result = i + 1;
		}
	}

	//返回结果
	return result;
}



//获取向量(第一个)最大值位置索引
unsigned int Vector::IndexOfMaxValue()
{
	//定义变量
	unsigned int result(1); double MaxValue(pVector[0]);

	//搜索最小值
	for (unsigned int i = 0; i < COUNT; i++)
	{
		if (MaxValue < pVector[i])
		{
			MaxValue = pVector[i];
			result = i + 1;
		}
	}

	//返回结果
	return result;
}