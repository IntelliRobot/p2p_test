#pragma once

#include "stdafx.h"

//matrix�������
class matrix;

enum VECTOR_TYPE { VERT , HORZ,};

class Vector
{
public:
	//�����洢λ��
	double * pVector = NULL;

	//����ά��
	unsigned int COUNT = 0;

	//��������
	enum VECTOR_TYPE TYPE;

public:
	Vector();

	//���캯���������ڴ�
	Vector(unsigned int uiCount, enum VECTOR_TYPE enumType = HORZ);

	//�������캯��
	Vector(const Vector & theVector);

	//����3ά����
	Vector(double x, double y, double z);

	//����4ά����
	Vector(double w, double x, double y, double z);

	//����1ά����
	Vector(double x);

	//����2ά����
	Vector(double x,double y);

	//���츴������
	Vector(double x, int N);   


	//ƴ��
	Vector(const Vector &A, const Vector &B);

	//����6ά����
	Vector(double u, double v, double w, double x, double y, double z);

	//�����������ͷ��ڴ���
	~Vector();

public:
	//()���������
	double & operator () (unsigned int Index) const;

	//��ֵ���������
	const Vector & operator = (const Vector & theVector);

	//�Ӽ����������
	Vector operator + ( const Vector & adder);

	Vector operator - (const Vector & adder);

	Vector operator +(const double theValue);

	Vector operator -(const double theValue);


	//�˷����������
	const Vector operator * (const double value);
	matrix operator * (const Vector & theVector);

	//�������������
	Vector operator / (const double value);

public:
	//���ݽű��ȡ������Ԫ��
	 double GetValue(unsigned int Index) const;

	//���ݽű�����������Ԫ��
	 void SetValue(unsigned int Index, double Value);

	//������ת��
	Vector Transposition() const;

	//��ȡ������
	Vector SubVector(unsigned int StartIndex, unsigned int EndIndex);

	//������ת���ɾ���
	matrix AsMatrix() const;

	//������ģ
	double Norm();

	//����Ԫ�����ֵ,������
	double Max( int &p);

	//����Ԫ�����ֵ
	double Max();

	//����Ԫ����Сֵ
	double Min();

	//����Ԫ�ص�ƽ����
	Vector Sqrt();

	//����Ԫ�صľ���ֵ
	Vector Abs();

	//����Ԫ�ط���
	Vector Sign();

	//�������������Ļ
	void Display();

	//�ı�����ά��
	void ChangeDimension(unsigned int uiCount);


	//�ж������Ƿ�Ϊ0����
	bool IsZero();



	//��ȡ������Сֵλ������
	unsigned int IndexOfMinValue();

	
	//��ȡ�������ֵλ������
	unsigned int IndexOfMaxValue();

	//������ӦԪ�����
	Vector Vmultiply(Vector theVector);

	//������ӦԪ�����
	Vector Vdivide(Vector theVector);

	//�ж������Ƿ�ȫ��>=0
	bool IsPositive();

	//��������
	Vector ceil();
};