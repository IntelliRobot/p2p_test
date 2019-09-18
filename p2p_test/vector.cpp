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

//���캯����Ĭ��Ϊ������
Vector::Vector(unsigned int uiCount, enum VECTOR_TYPE enumType)
{
	//�趨��������
	COUNT = uiCount;
	TYPE = enumType;

	//�����ڴ�
	pVector = new double[uiCount];

	//������ʼ��
	memset(pVector, 0, uiCount * sizeof(double));
}


Vector::Vector(double x, double y, double z)
{ //������ά����

	COUNT = 3;
	TYPE = VERT;
	pVector = new double[COUNT];
	pVector[0] = x;
	pVector[1] = y;
	pVector[2] = z;
}

Vector::Vector(double w,double x, double y, double z)
{ //������ά����

	COUNT = 4;
	TYPE = VERT;
	pVector = new double[COUNT];
	pVector[0] = w;
	pVector[1] = x;
	pVector[2] = y;
	pVector[3] = z;

}

Vector::Vector(double x)
{ //����1ά����

	COUNT = 1;
	TYPE = VERT;
	pVector = new double[COUNT];
	pVector[0] = x;
}

Vector::Vector(double x,double y)
{ //����2ά����

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
{ //����6ά����

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

// ���츴������
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

//�������캯��
Vector::Vector(const Vector & theVector)
{
	//�趨��������
	COUNT = theVector.COUNT;
	TYPE = theVector.TYPE;

	//�����ڴ�
	pVector = new double[COUNT];

	//������ֵ
	memcpy(pVector, theVector.pVector, COUNT * sizeof(double));
}

//��������
Vector::~Vector()
{
	if (pVector != NULL) 
	{ //cout << "vector deconstruct" << endl;
		delete pVector; 
		pVector = NULL;
	}
	//delete[] pVector;
}

//���ýű���ȡ�ض���ֵ��Index=[1,...,n]
double & Vector::operator () (unsigned int Index) const
{
	return pVector[Index - 1];
}

//��ֵ����������������Ȳ�ͬ������С������Ϊ��ֵ����
//�����������������ͱ�����ͬ
const Vector & Vector::operator = (const Vector & theVector)
{
	//У��
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

		//��ֵ
		memcpy(pVector, theVector.pVector, COUNT *sizeof(double));
	}
	//ȡ��С��COUNT


	//����
	return theVector;
}

//�������������
//��������������ά����ͬ
Vector Vector::operator + (const Vector & adder)
{
	//����У��
	if (COUNT != adder.COUNT) { throw("�����������������"); }

	//���巵�ؽ��
	Vector result(COUNT, TYPE);

	//���мӷ�����
	for (unsigned int i = 1; i <= COUNT; i++)
	{
		result(i) = GetValue(i) + adder.GetValue(i);
	}

	//���ؽ��
	return result;
}

//�������������
//��������������ά����ͬ
Vector Vector::operator - (const Vector & adder)
{
	//����У��
	if (COUNT != adder.COUNT) { throw("�����������������"); }

	//���巵�ؽ��
	Vector result(COUNT, TYPE);

	//���м�������
	for (unsigned int i = 1; i <= COUNT; i++)
	{
		result(i) = GetValue(i) - adder.GetValue(i);
	}

	//���ؽ��
	return result;
}


//��������ֵ���
Vector Vector::operator +(const double theValue)
{
	//���巵�ؽ��
	Vector result(COUNT, TYPE);

	//���м�������
	for (unsigned int i = 1; i <= COUNT; i++)
	{
		result(i) = GetValue(i) + theValue;
	}

	//���ؽ��
	return result;
}

//��������ֵ���
Vector Vector::operator -(const double theValue)
{
	//���巵�ؽ��
	Vector result(COUNT, TYPE);

	//���м�������
	for (unsigned int i = 1; i <= COUNT; i++)
	{
		result(i) = GetValue(i) - theValue;
	}

	//���ؽ��
	return result;
}

//�����������
const Vector Vector::operator * (const double value)
{
	//������
	Vector result(COUNT, TYPE);

	//���м���
	for (unsigned int i = 1; i <= COUNT; i++)
	{
		result(i) = value * GetValue(i);
	}

	//���ؽ��
	return result;
}

//�������������
//��������������ά����ͬ�����Ͳ�ͬ
matrix Vector::operator * (const Vector & theVector)
{
	//������ת��ɾ���
	matrix a(AsMatrix());
	matrix b(theVector.AsMatrix());

	//���巵�ؽ��
	matrix result(a.ROW, b.COL);

	//��֤
	if (a.COL != b.ROW) { throw("�������������"); return result; }

	//���м���
	result = a*b;

	//���ؽ��
	return result;
}

//�����������г���
Vector Vector::operator / (const double value)
{
	//��֤
	if (value == 0) { throw("��������Ϊ0"); }

	//���巵��ֵ
	Vector result(COUNT,TYPE);

	//ÿ��Ԫ�ؽ��г���
	for (unsigned int i = 1; i <= COUNT; i++)
	{
		result(i) = GetValue(i) / value;
	}

	//����
	return result;
}

//���ݽű��ȡ����Ԫ�أ�Index=[1,...,n]
 double Vector::GetValue(unsigned int Index) const
{
	unsigned int index_p = Index - 1;
	return pVector[index_p];
}

//���ݽű���������Ԫ�أ�Index=[1,...,n]
  void Vector::SetValue(unsigned int Index, double Value)
{
	 unsigned int index_p = Index - 1;
	 pVector[index_p] = Value;
}

//������ת��
Vector Vector::Transposition() const
{
	//��������
	Vector result(COUNT, TYPE == HORZ ? VERT : HORZ);

	//������ֵ
	memcpy(result.pVector, pVector, COUNT*sizeof(double));

	//���ؽ��
	return result;
}

//��ȡ������
//��������1<=StartIndex<=EndIndex<=COUNT
Vector Vector::SubVector(unsigned int StartIndex, unsigned int EndIndex)
{
	//У��
	if (StartIndex > EndIndex || StartIndex == 0 || EndIndex > COUNT) { throw("��������ȡ�Ӽ�������"); }

	//��������
	Vector result(EndIndex - StartIndex + 1, TYPE);

	//������ֵ
	memcpy(result.pVector, &pVector[StartIndex - 1], result.COUNT * sizeof(double));

	//���ؽ��
	return result;
}

//������ģ
double Vector::Norm()
{
	//���巵�ؽ��
	double SquareSum = 0;

	//����ƽ����
	for (unsigned int i = 1; i <= COUNT; i++)
	{
		SquareSum += GetValue(i) * GetValue(i);
	}

	//����ƽ����
	return sqrt(SquareSum);
}

//����Ԫ�����ֵ

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

//����Ԫ����Сֵ
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
	//����ƽ����
	for (unsigned int i = 1; i <= COUNT; i++)
	{
		if (pVector[i - 1]>=0)
		{
			result.pVector[i - 1] = sqrt(pVector[i - 1]);
		}
		else
		{
			throw("���󣬸�������"); 
		}
		
	}

	return result;
}



//����Ԫ�صľ���ֵ
Vector Vector::Abs()
{
	Vector result(COUNT, TYPE);
	//����ƽ����
	for (unsigned int i = 1; i <= COUNT; i++)
	{
		result.pVector[i-1]= fabs(pVector[i - 1]);
	}

	return result;
}


//����Ԫ�صķ���
Vector Vector::Sign()
{
	Vector result(COUNT, TYPE);
	//����ƽ����
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

//ת���ɾ���
matrix Vector::AsMatrix() const
{
	//���巵��ֵ
	matrix result(1, COUNT);

	//��ֵ
	memcpy(result.pMatrix, pVector, COUNT*sizeof(double));

	//���ؽ��
	if (TYPE == HORZ) { return result; }
	else { return result.Transposition(); }
}

//�������Ԫ�ص���ʾ��
void Vector::Display()
{
	for (unsigned int i = 1; i <= COUNT; i++)
	{
		printf("%4.4f", GetValue(i));

		if (TYPE == HORZ) { printf(" "); }
		else { printf("\r\n"); }
	}
}

//�ı�����ά��
void Vector::ChangeDimension(unsigned int uiCount)
{
	//��ȡ��Сά��
	unsigned int MinCount = uiCount <= COUNT ? uiCount : COUNT;

	//���·����ڴ�ռ�
	double * pTemp = new double[uiCount];

	//ʹ�ڴ�ռ�Ϊ0
	memset(pTemp, 0, sizeof(double)*uiCount);

	//�����ڴ�ռ�ĸ���
	memcpy(pTemp, pVector, sizeof(double)*MinCount);

	//�ͷ�pVector
	delete pVector;

	//pVector���¸�ֵ
	pVector = pTemp;

	//��������COUNT,���Ͳ���
	COUNT = uiCount;
}

//�ж������Ƿ�Ϊ0����
bool Vector::IsZero()
{
	//��������ֵ������false
	for (unsigned int i = 0; i < COUNT; i++)
	{
		if (pVector[i] != 0) { return false; }
	}

	//����true
	return true;
}

//������ӦԪ�����:�����������ͣ����������thisһ��
Vector Vector::Vmultiply(Vector theVector)
{
	//��������
	Vector result(COUNT, TYPE);

	//�������������������
	if (COUNT != theVector.COUNT) { return result; }

	//��������
	for (unsigned int i = 1; i < COUNT; i++)
	{
		result(i) = GetValue(i)*theVector.GetValue(i);
	}

	//����
	return result;
}

//������ӦԪ�����:�����������ͣ����������thisһ��
Vector Vector::Vdivide(Vector theVector)
{
	//��������
	Vector result(COUNT, TYPE);

	//�������������������
	if (COUNT != theVector.COUNT) { return result; }

	//��������
	for (unsigned int i = 1; i < COUNT; i++)
	{
		result(i) = GetValue(i) / theVector.GetValue(i);
	}

	//����
	return result;
}

//�ж������Ƿ�ȫ��>=0
bool Vector::IsPositive()
{
	//����С��0��Ԫ�أ�����false
	for (unsigned int i = 0; i < COUNT; i++)
	{
		if (pVector[i] < 0) { return false; }
	}

	//����true
	return true;
}



//��������
Vector Vector::ceil()
{
	//��������
	Vector result(COUNT, TYPE);

	//����
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

	//����
	return result;
}


//��ȡ����(��һ��)��Сֵλ������
unsigned int Vector::IndexOfMinValue()
{
	//�������
	unsigned int result(1); double MinValue(pVector[0]);

	//������Сֵ
	for (unsigned int i = 1; i < COUNT; i++)
	{
		if (MinValue > pVector[i])
		{
			MinValue = pVector[i];
			result = i + 1;
		}
	}

	//���ؽ��
	return result;
}



//��ȡ����(��һ��)���ֵλ������
unsigned int Vector::IndexOfMaxValue()
{
	//�������
	unsigned int result(1); double MaxValue(pVector[0]);

	//������Сֵ
	for (unsigned int i = 0; i < COUNT; i++)
	{
		if (MaxValue < pVector[i])
		{
			MaxValue = pVector[i];
			result = i + 1;
		}
	}

	//���ؽ��
	return result;
}