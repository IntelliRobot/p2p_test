//Author:NingHC
//Last modified:2018-12-19

#include "stdafx.h"
#include "Vector.h"
#include "Quaternion.h"
#include "matrix.h"
#include "global.h"
using namespace std;

//���캯���������ڴ�
//m_row=[1...m]
//n_col=[1...n]
matrix::matrix()
{
}
matrix::matrix(unsigned int m_row, unsigned int n_col)
{
	//�趨���Д�
	ROW = m_row;
	COL = n_col;

	//�����ڴ�
	pMatrix = new  double[m_row * n_col];

	//����ֵΪ0
	memset(pMatrix, 0, sizeof(double) * m_row * n_col);
}

//���鹹��ɾ���
matrix::matrix(unsigned int m_row, unsigned int n_col, double *ar)
{
	//�趨���Д�
	ROW = m_row;
	COL = n_col;
	pMatrix = new  double[m_row * n_col];

	memcpy(pMatrix, ar, sizeof(double) * ROW * COL);
}

//�������캯��
matrix::matrix(const matrix & theMatrix)
{
	//�趨���Д�
	ROW = theMatrix.ROW;
	COL = theMatrix.COL;

	//�����ڴ�
	pMatrix = new double[ROW * COL];

	//��������
	memcpy(pMatrix, theMatrix.pMatrix, sizeof(double) * ROW * COL);
}

matrix::matrix(const matrix & Rot, const Vector & P)
{
	ROW = 4;
	COL = 4;
	pMatrix = new  double[ROW * COL];

	if ((Rot.COL>=3)&&(Rot.ROW>=3)&&(P.COUNT>=3))
	{
		for (unsigned int i = 1; i <= 3; i++)
		{
			for (unsigned int j = 1; j <= 3; j++)
			{
				pMatrix[(i-1) * COL + (j-1)] = Rot.GetValue(i,j);
			}
		}

		pMatrix[3 * COL + 0] = 0;
		pMatrix[3 * COL + 1] = 0;
		pMatrix[3 * COL + 2] = 0;
		pMatrix[3 * COL + 3] = 1;

		pMatrix[0 * COL + 3] = P.GetValue(1);
		pMatrix[1 * COL + 3] = P.GetValue(2);
		pMatrix[2 * COL + 3] = P.GetValue(3);

	}

	else
	{
		throw("���������ά������");;

	}


}

//���������������ڌ\
matrix::~matrix()
{
	
	if (pMatrix != NULL) 
	{ 
//		delete[] RVector.pVector;
		//RVector.~Vector(); 
		cout << "matrix deconstruct" << endl;
		delete[] pMatrix; 
		RVector.pVector = NULL;//�ͷ������ռ䣬�������������

	}
	//delete []pMatrix;
}

//�����������������кŻ�ȡ����Ă�
//����ʱע�⣺ǧ��Ҫ�����߽磬��������һ��δ֪���ڴ�λ��
//m_row=[1...m]
//n_col=[1...n]
inline double matrix::GetValue(unsigned int m_row, unsigned int n_col) const
{
	return pMatrix[(m_row - 1) * COL + (n_col - 1)];
}

//�����������������кŸ����󸳂�
//����ʱע�⣺ǧ��Ҫ�����߽磬��������һ��δ֪���ڴ�λ��
//m_row=[1...m]
//n_col=[1...n]
void matrix::SetValue(unsigned int m_row, unsigned int n_col, double value)
{
	pMatrix[(m_row - 1) * COL + (n_col - 1)] = value;
}

 void matrix::SetValue(unsigned int i, const Vector  &value, enum VECTOR_TYPE vh_)
{
	if ( vh_ == VERT)
	{
		for (unsigned int p = 1; p <= value.COUNT; p++)
		{
			pMatrix[(p - 1) * COL + (i - 1)] = value.pVector[p - 1];
		}
	}
	//////
	if ( vh_ == HORZ)
	{
		for (unsigned int p = 1; p <= value.COUNT; p++)
		{
			pMatrix[(i - 1) * COL + (p - 1)] = value.pVector[p - 1];
		}
	}
}
 void matrix::SetValue(unsigned int i, const Quaternion & value, enum VECTOR_TYPE vh_)
{
	if (vh_ == VERT)
	{
			pMatrix[0 * COL + (i - 1)] = value.w;
			pMatrix[1 * COL + (i - 1)] = value.x;
			pMatrix[2 * COL + (i - 1)] = value.y;
			pMatrix[3 * COL + (i - 1)] = value.z;
		
	}
	//////
	if (vh_ == HORZ)
	{
			pMatrix[(i - 1) * COL + 0] = value.w;
			pMatrix[(i - 1) * COL + 1] = value.x;
			pMatrix[(i - 1) * COL + 2] = value.y;
			pMatrix[(i - 1) * COL + 3] = value.z;
	}
}


//���ýű���ȡ�ض�����
//����ʱע�⣺ǧ��Ҫ�����߽磬��������һ��δ֪���ڴ�λ��
//m_row=[1...m]
//n_col=[1...n]
double & matrix::operator () (unsigned int m_row, unsigned int n_col)
{
	//ֱ�ӷ���
	return pMatrix[(m_row - 1) * COL + (n_col - 1)];
}

//����ĸ�ֵ��ʹ
//�淶����ʱ��Ҫ����������ά����ͬ
//���淶������������ԭ����
//��һ��Ԫ�ض��룬��Mdest(1,1)=Msrc(1,1)
//�еĸ�ֵ�߽�ȡ��С��
//�еĸ�ֵ�߽�ȡ��С��
//����Ԫ�ز�������
const matrix & matrix::operator = (const matrix & theMatrix)
{
	//ȡ���к�С��
	if (pMatrix==NULL)
	{
		ROW = theMatrix.ROW;
		COL = theMatrix.COL;

		//�����ڴ�
		pMatrix = new  double[ROW*COL];
		memcpy(pMatrix, theMatrix.pMatrix, sizeof(double) * ROW * COL);
	}
	
	else if ((ROW ==theMatrix.ROW)&&(COL = theMatrix.COL))
	{
		memcpy(pMatrix, theMatrix.pMatrix, sizeof(double) * ROW * COL);
	}
	else
	{
		unsigned int row = ROW > theMatrix.ROW ? theMatrix.ROW : ROW;
		unsigned int col = COL > theMatrix.COL ? theMatrix.COL : COL;

		//��ֵ��ʹ
		for (unsigned int i = 1; i <= row; i++)
		{
			for (unsigned int j = 1; j <= col; j++)
			{
				SetValue(i, j, theMatrix.GetValue(i, j));
			}
		}
	}


	//���ؽ��
	return theMatrix;

}

//����ӷ�����
//Ҫ����������ά��������ͬ
matrix matrix::operator + (const matrix & adder)
{
	//���巵�ؽ��
	matrix result(ROW, COL);

	//���мӷ�����
	for (unsigned int i = 0; i < ROW * COL; i++)
	{
		result.pMatrix[i] = pMatrix[i] + adder.pMatrix[i];
	}

	//���ؽ��
	return result;
}

//����ļ����ƹ�
//Ҫ����������ά��������ͬ
matrix matrix::operator - (const matrix & suber)
{
	//���巵�ؽ��
	matrix result(ROW, COL);

	//���м�������
	for (unsigned int i = 0; i < ROW * COL; i++)
	{
		result.pMatrix[i] = pMatrix[i] - suber.pMatrix[i];
	}

	//���ؽ��
	return result;
}

//�����������г˷�����
matrix matrix::operator * (const double value)
{
	//���巵�ؽ��
	matrix result(ROW, COL);

	//���г˷�����
	for (unsigned int i = 0; i < ROW*COL; i++)
	{
		result.pMatrix[i] = pMatrix[i] * value;
	}

	//���ؽ��
	return result;
}

//�����������г�������
matrix matrix::operator / (const double value)
{
	//��������
	if (value == 0) { throw("��������د"); }

	//���巵�ؽ��
	matrix result(ROW, COL);

	//���г˷�����
	for (unsigned int i = 0; i < ROW*COL; i++)
	{
		result.pMatrix[i] = pMatrix[i] / value;
	}

	//���ؽ��
	return result;
}

//������������v
//������˵���������һ�����������=�ڶ������������
//�����������������˵��������򷵻�һ��Ԫ��ȫ����0�ľذ�
matrix matrix::operator * (const matrix & MultMatrix)
{
	//���巵�ؽ��
	matrix result(ROW, MultMatrix.COL);

	//����У��
	if (COL != MultMatrix.ROW) {
		throw("��������������˵�����");
		return result;
	}

	//���м���
	for (unsigned int i = 1; i <= ROW; i++)
	{
		for (unsigned int j = 1; j <= MultMatrix.COL; j++)
		{
			for (unsigned int k = 1; k <= COL; k++)
			{
				result(i, j) += GetValue(i, k) * MultMatrix.GetValue(k, j);
			}
		}
	}

	//���ؽ��
	return result;
}

//�������������v
//������������˵����������������=������ά��
//�ӳ�����Ϊ������
//���ؽ��Ϊ������
Vector matrix::operator * (const Vector & value)
{
	//�������
	Vector result(ROW, VERT);

	//��֤
	if (COL != value.COUNT) { throw("����������������˵�������"); return result; }

	//���м���
	for (unsigned int i = 1; i <= ROW; i++)
	{
		for (unsigned int j = 1; j <= COL; j++)
		{
			result(i) += GetValue(i, j) * value.pVector[j - 1];
		}
	}

	//���ؽ��
	return result;
}

//������ֹ���к���ȡ�Ӿ���
//��ȷ��ȡ������
//end_row >= start_row
//end_col >= start_col
//���򣬷��ؾ���[0]
const matrix  matrix::SubMatrix(unsigned int start_row, unsigned int start_col, unsigned int end_row, unsigned int end_col) const
{
	//�����������ʼ��
	unsigned int result_row = end_row - start_row + 1;
	unsigned int result_col = end_col - start_col + 1;

	//����У��
	if (result_row == 0) result_row = 1;
	if (result_col == 0) result_col = 1;

	//���巵�ؽ��
	 matrix result(result_row, result_col);

	//���ؽ������
	for (unsigned int i = start_row; i <= end_row; i++)
	{
		for (unsigned int j = start_col; j <= end_col; j++)
		{
			result(i - start_row + 1, j - start_col + 1) = GetValue(i, j);
		}
	}

	//���ؽ��
	return result;
}
//��ȡ������
Vector matrix::SubVector(unsigned int rc, enum VECTOR_TYPE vh)const
{
	
	if (vh == VERT && rc<=COL)
	{
		Vector result(ROW, VERT);

		for (unsigned int i = 1; i <= ROW; i++)
		{
			result.pVector[i-1]= GetValue(i, rc);
		}

		return result;

	}

	if (vh==HORZ && rc<=ROW)
	{
		Vector result(COL, HORZ);
		for (unsigned int i = 1; i <= COL; i++)
		{
			result.pVector[i - 1] = GetValue(rc, i);
		}
		return result;
	}

	else
	{
		return ZeroVector3;
	}

}

Vector matrix:: RowVector(unsigned int rc)
{
	//Vector RVector(COL,HORZ);
	RVector.TYPE = HORZ;
	RVector.COUNT = COL;
	if (rc <= ROW)
	{
		//for (int i = 0; i < RVector.COUNT; i++)
		//{
		//	RVector.pVector[i] = pMatrix[(rc - 1)*COL + i];
		//}
		
		RVector.pVector = &pMatrix[(rc - 1)*COL];
		
	}
	else
	{
		//for (int i = 0; i < RVector.COUNT; i++)
		//{
			//RVector.pVector[i] = pMatrix[(ROW - 1)*COL + i];
		RVector.pVector = &pMatrix[(ROW - 1)*COL];
		//}
	}

	return RVector;
}

//��õ�ǰ�����ת�þذ�
matrix matrix::Transposition() const
{
	//���巵�ؽ��
	matrix result(COL, ROW);

	//���󸳂�
	for (unsigned int i = 1; i <= COL; i++)
	{
		for (unsigned int j = 1; j <= ROW; j++)
		{
			result.SetValue(i, j, GetValue(j, i));
		}
	}

	//���ؽ��
	return result;
}

//���������Џ�
//������������������Ƿ��󣬼���m=n
double matrix::Determinant()
{
	//����У��
	if (ROW != COL) {
		throw("�������������");
		return 0.0;
	}

	//��������ά��Ϊ1�����ص�һ��Ԫ�H
	if (ROW == 1) { return GetValue(1, 1); }

	//���巵�ؽ��
	double result = 0.0;

	//���ݴ�������ʽ������ӄ
	for (unsigned int j = 1; j <= COL; j++)
	{
		result += NegtiveOnePower(1 + j) * GetValue(1, j) * Remainder(1, j).Determinant();
	}

	//���ؽ��
	return result;
}

//��������ذ�
//���������ߡ���������Ƿ���ߡ�������󲻿��棬����[0]����
matrix matrix::Inverse()
{
	//��֤��������
	if (ROW != COL) { throw("��ȷ����������Ƿ���"); }

	//���巵�ؾ���
	matrix result(ROW, COL);

	//�����Ķ�
	double MatDet = Determinant();

	//����������=0������[0]����
	//�������Ӣ��rank
	if (MatDet == 0) { return result; }

	//����MatStart
	for (unsigned int i = 1; i <= ROW; i++)
	{
		for (unsigned int j = 1; j <= COL; j++)
		{
			result(j, i) = NegtiveOnePower(i + j)*Remainder(i, j).Determinant();
		}
	}

	//���ؽ��
	return result / MatDet;
}

//��Խ���Ԫ��֮��
//�����������������Ϊ���󣬼���m=n
double matrix::Trace()
{
	//��֤
	if (ROW != COL) { throw("��ȷ���������Ϊ����"); return 0.0; }

	//���巵�ؽ��
	double result = 0;

	//���м���
	for (unsigned int i = 1; i <= ROW; i++)
	{
		result += GetValue(i, i);
	}

	//���ؽ��
	return result;
}

//��ʾ����Ԫ�ص�����
void matrix::Display()
{
	//�������н������
	for (unsigned int i = 1; i <= ROW; i++)
	{
		for (unsigned int j = 1; j <= COL; j++)
		{
			printf("%4.4f  ", GetValue(i, j));
		}
		printf("\r\n");
	}
}


//�������
matrix matrix::operator +(const double theValue)
{
	//���巵�ؽ��
	matrix result(ROW, COL);

	//���м�������
	for (unsigned int i = 0; i < ROW * COL; i++)
	{
		result.pMatrix[i] = pMatrix[i] + theValue;
	}

	//���ؽ��
	return result;
}

//�������
matrix matrix::operator -(const double theValue)
{
	//���巵�ؽ��
	matrix result(ROW, COL);

	//���м�������
	for (unsigned int i = 0; i < ROW * COL; i++)
	{
		result.pMatrix[i] = pMatrix[i] - theValue;
	}

	//���ؽ��
	return result;
}

//�ı����ά��
//uiRow=[1...m]
//uiCol=[1...n]
void matrix::ChangeDimension(unsigned int uiRow, unsigned int uiCol)
{
	//���·����ڴ�
	double * pTemp = new double[uiRow*uiCol];

	//�ڴ渴λ
	memset(pTemp, 0, sizeof(double)*uiRow*uiCol);

	//��ȡ��С��
	unsigned int MinRow = uiRow < ROW ? uiRow : ROW;

	//��ȡ��С��
	unsigned int MinCol = uiCol < COL ? uiCol : COL;

	//���ڴ��ʼ��
	for (unsigned int i = 1; i <= MinRow; i++)
	{
		for (unsigned int j = 1; j <= MinCol; j++)
		{
			pTemp[(i - 1)*uiCol + (j - 1)] = pMatrix[(i - 1)*COL + (j - 1)];
		}
	}

	//�ͷ�ԭ���ڴ�
	delete pMatrix;

	//���¸�ֵ
	pMatrix = pTemp;

	//�ı�ά��
	ROW = uiRow;
	COL = uiCol;
}

//�жϾ����Ƿ�Ϊ0���󣬼�����Ԫ�ض�Ϊ0�ľ���
bool matrix::IsZero()
{
	//������0��ֵ������false
	for (unsigned int i = 0; i < ROW*COL; i++)
	{
		if (pMatrix[i] != 0) { return false; }
	}

	//����true
	return true;
}

//�����л��еľ�����Ϊ��������
//����������m=1 ��n=1
//���о���ת���������������о���ת���������Y
Vector  matrix::AsVector() const
{
	//��֤��������
	if (ROW != 1 && COL != 1) {
		throw("������ת��������������");
	}

	//���巵�ؽ������
	Vector result((ROW >= COL) ? ROW : COL, (ROW >= COL) ? VERT : HORZ);

	//������ʼ��
	memcpy(result.pVector, pMatrix, result.COUNT * sizeof(double));

	//��������
	return result;
}

//��ȡȥ����m_row����n_col��ʣ��Ԫ����ɵľ���
//���������ߡ�ROW>1 ����COL>1��?<=m_row<=ROW ���� 1<=n_col<=COL
// ����ʽӢ��minors���Ƿ������
matrix matrix::Remainder(unsigned int m_row, unsigned int n_col)
{
	//��֤��������
	if (ROW <= 1 || COL <= 1 || m_row < 1 || m_row > ROW || n_col < 1 || n_col > COL) {
		throw("�������������");
	}

	//���巵�ؾ���
	matrix result(ROW - 1, COL - 1);

	//����Ԫ�ظ���
	for (unsigned int i = 1, m = 1; i <= ROW; i++)
	{
		//�����m_row�е�Ԫ��,��������
		if (i == m_row) { continue; }

		//���з��ؽ���ĸ�ֵ��ʹ
		for (unsigned int j = 1, n = 1; j <= COL; j++)
		{
			//�����n_col�е�Ԫ�أ���������
			if (j == n_col) { continue; }

			//����
			result(m, n++) = GetValue(i, j);
		}

		//�к�����
		m++;
	}

	//����
	return result;
}

//����-1��n����
inline int matrix::NegtiveOnePower(unsigned int n)
{
	return n % 2 == 0 ? 1 : -1;
}

