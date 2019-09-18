#pragma once

#include "stdafx.h"

//Vector�������
class Vector;
class Quaternion;
//������
class matrix
{
public:
	//����洢λ��
	double * pMatrix = NULL;

	//������
	unsigned int ROW = 0;
	unsigned int COL = 0;
	Vector RVector;

public:
	
	matrix();
	//���캯���������ڴ�
	matrix(unsigned int m_row, unsigned int n_col);

	//���鹹��ɾ���
	matrix(unsigned int m_row, unsigned int n_col, double *ar);

	//�������캯��
	matrix(const matrix & theMatrix);

	matrix(const matrix & Rot,const Vector &P);


	//�����������ջ��ڴ�
	~matrix();

	//()���������
	double & operator () (unsigned int m_row, unsigned int n_col);

	//��ֵ���������
	const matrix & operator = (const matrix & theMatrix);

	//�Ӽ����������
	matrix operator + (const matrix & adder);
	matrix operator - (const matrix & adder);

	//�˷����������
	matrix operator * (const double value);
	matrix operator * (const matrix & value);
	Vector operator * (const Vector & value);

	//�������������
	matrix operator / (const double value);

public:
	//���ݽű���ȡ����Ԫ��
    inline double GetValue(unsigned int m_row, unsigned int n_col) const;

	//���ݽű����þ���Ԫ��
	inline void SetValue(unsigned int m_row, unsigned int n_col, double value);

	//������������Ԫ��
	 void SetValue(unsigned int i, const Vector  &value, enum VECTOR_TYPE vh);

	//������Ԫ��Ԫ��
	 void SetValue(unsigned int i, const Quaternion &value, enum VECTOR_TYPE vh);

	//��ȡ�Ӿ���
	const matrix SubMatrix(unsigned int start_row, unsigned int start_col, unsigned int end_row, unsigned int end_col)const;

	//��ȡ������,����
	Vector SubVector(unsigned int rc, enum VECTOR_TYPE vh)const;

	//��ȡ��������,����
	Vector RowVector(unsigned int rc);
	
	//��ֻ��һ�л�һ�еľ���ת��������
	Vector AsVector() const;
	
	//������ת�þ���
	matrix Transposition() const;

	//����������ʽ
	double Determinant();

	//�����������
	matrix Inverse();

	//�Խ���Ԫ��֮��
	double Trace();



	//������Ԫ���������Ļ
	void Display();


	//�Ӽ������
	matrix operator +(const double theValue);
	matrix operator -(const double theValue);

	//�ı����ά��
	void ChangeDimension(unsigned int uiRow, unsigned int uiCol);

	//�жϾ����Ƿ���0����
	bool IsZero();

private:
	//��ȥ����m_row����n_colʣ�µľ���
	matrix Remainder(unsigned int m_row, unsigned int n_col);

	//��-1��n����
    inline int NegtiveOnePower(unsigned int n);
};


