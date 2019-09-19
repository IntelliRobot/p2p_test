//Author:NingHC
//Last modified:2018-12-19

#include "stdafx.h"
#include "Vector.h"
#include "Quaternion.h"
#include "matrix.h"
#include "global.h"
using namespace std;

//构造函数，分配内存
//m_row=[1...m]
//n_col=[1...n]
matrix::matrix()
{
}
matrix::matrix(unsigned int m_row, unsigned int n_col)
{
	//设定行列敿
	ROW = m_row;
	COL = n_col;

	//分配内存
	pMatrix = new  double[m_row * n_col];

	//赋初值为0
	memset(pMatrix, 0, sizeof(double) * m_row * n_col);
}

//数组构造成矩阵
matrix::matrix(unsigned int m_row, unsigned int n_col, double *ar)
{
	//设定行列敿
	ROW = m_row;
	COL = n_col;
	pMatrix = new  double[m_row * n_col];

	memcpy(pMatrix, ar, sizeof(double) * ROW * COL);
}

//拷贝构造函敿
matrix::matrix(const matrix & theMatrix)
{
	//设定行列敿
	ROW = theMatrix.ROW;
	COL = theMatrix.COL;

	//分配内存
	pMatrix = new double[ROW * COL];

	//拷贝赋倿
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
		throw("矩阵或向量维数不够");;

	}


}

//析构函数，回收内孿
matrix::~matrix()
{
	
	if (pMatrix != NULL) 
	{ 
//		delete[] RVector.pVector;
		//RVector.~Vector(); 
		//cout << "matrix deconstruct" << endl;
		delete[] pMatrix; 
		RVector.pVector = NULL;//释放向量空间，以免二次析构；

	}
	//delete []pMatrix;
}

//内联函数，根据行列号获取矩阵的倿
//利用时注意：千万不要超出边界，否则引用一个未知的内存位置
//m_row=[1...m]
//n_col=[1...n]
inline double matrix::GetValue(unsigned int m_row, unsigned int n_col) const
{
	return pMatrix[(m_row - 1) * COL + (n_col - 1)];
}

//内联函数，根据行列号给矩阵赋倿
//利用时注意：千万不要超出边界，否则引用一个未知的内存位置
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


//利用脚标提取特定数倿
//利用时注意：千万不要超出边界，否则引用一个未知的内存位置
//m_row=[1...m]
//n_col=[1...n]
double & matrix::operator () (unsigned int m_row, unsigned int n_col)
{
	//直接返回
	return pMatrix[(m_row - 1) * COL + (n_col - 1)];
}

//矩阵的赋值操使
//规范操作时：要求两个矩阵维数相同
//不规范操作，按以下原则处理：
//第一个元素对齐，即Mdest(1,1)=Msrc(1,1)
//行的赋值边界取最小倿
//列的赋值边界取最小倿
//其余元素不做处理
const matrix & matrix::operator = (const matrix & theMatrix)
{
	//取行列号小的
	if (pMatrix==NULL)
	{
		ROW = theMatrix.ROW;
		COL = theMatrix.COL;

		//分配内存
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

		//赋值操使
		for (unsigned int i = 1; i <= row; i++)
		{
			for (unsigned int j = 1; j <= col; j++)
			{
				SetValue(i, j, theMatrix.GetValue(i, j));
			}
		}
	}


	//返回结果
	return theMatrix;

}

//矩阵加法运算
//要求两个矩阵维数必须相同
matrix matrix::operator + (const matrix & adder)
{
	//定义返回结果
	matrix result(ROW, COL);

	//进行加法运算
	for (unsigned int i = 0; i < ROW * COL; i++)
	{
		result.pMatrix[i] = pMatrix[i] + adder.pMatrix[i];
	}

	//返回结果
	return result;
}

//矩阵的减法计箿
//要求两个矩阵维数必须相同
matrix matrix::operator - (const matrix & suber)
{
	//定义返回结果
	matrix result(ROW, COL);

	//进行减法运算
	for (unsigned int i = 0; i < ROW * COL; i++)
	{
		result.pMatrix[i] = pMatrix[i] - suber.pMatrix[i];
	}

	//返回结果
	return result;
}

//矩阵与数进行乘法运算
matrix matrix::operator * (const double value)
{
	//定义返回结果
	matrix result(ROW, COL);

	//进行乘法运算
	for (unsigned int i = 0; i < ROW*COL; i++)
	{
		result.pMatrix[i] = pMatrix[i] * value;
	}

	//返回结果
	return result;
}

//矩阵与数进行除法运算
matrix matrix::operator / (const double value)
{
	//除数验算
	if (value == 0) { throw("除数不能丿"); }

	//定义返回结果
	matrix result(ROW, COL);

	//进行乘法运算
	for (unsigned int i = 0; i < ROW*COL; i++)
	{
		result.pMatrix[i] = pMatrix[i] / value;
	}

	//返回结果
	return result;
}

//矩阵与矩阵相乿
//矩阵相乘的条件：第一个矩阵的列数=第二个矩阵的行数
//如果两个矩阵不满足相乘的条件，则返回一个元素全部是0的矩阿
matrix matrix::operator * (const matrix & MultMatrix)
{
	//定义返回结果
	matrix result(ROW, MultMatrix.COL);

	//规则校验
	if (COL != MultMatrix.ROW) {
		throw("两个矩阵不满足相乘的条件");
		return result;
	}

	//进行计算
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

	//返回结果
	return result;
}

//矩阵与向量相乿
//矩阵与向量相乘的条件：矩阵的列数=向量的维敿
//视乘向量为列向量
//返回结果为列向量
Vector matrix::operator * (const Vector & value)
{
	//定义变量
	Vector result(ROW, VERT);

	//验证
	if (COL != value.COUNT) { throw("不满足矩阵和向量相乘的条件！"); return result; }

	//进行计算
	for (unsigned int i = 1; i <= ROW; i++)
	{
		for (unsigned int j = 1; j <= COL; j++)
		{
			result(i) += GetValue(i, j) * value.pVector[j - 1];
		}
	}

	//返回结果
	return result;
}

//根据起止行列号提取子矩阵
//正确提取条件＿
//end_row >= start_row
//end_col >= start_col
//否则，返回矩阵[0]
const matrix  matrix::SubMatrix(unsigned int start_row, unsigned int start_col, unsigned int end_row, unsigned int end_col) const
{
	//定义变量并初始化
	unsigned int result_row = end_row - start_row + 1;
	unsigned int result_col = end_col - start_col + 1;

	//计算校验
	if (result_row == 0) result_row = 1;
	if (result_col == 0) result_col = 1;

	//定义返回结果
	 matrix result(result_row, result_col);

	//返回结果赋倿
	for (unsigned int i = start_row; i <= end_row; i++)
	{
		for (unsigned int j = start_col; j <= end_col; j++)
		{
			result(i - start_row + 1, j - start_col + 1) = GetValue(i, j);
		}
	}

	//返回结果
	return result;
}
//提取子向量
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

//获得当前矩阵的转置矩阿
matrix matrix::Transposition() const
{
	//定义返回结果
	matrix result(COL, ROW);

	//矩阵赋倿
	for (unsigned int i = 1; i <= COL; i++)
	{
		for (unsigned int j = 1; j <= ROW; j++)
		{
			result.SetValue(i, j, GetValue(j, i));
		}
	}

	//返回结果
	return result;
}

//求矩阵的行列弿
//计算条件：矩阵必须是方阵，即：m=n
double matrix::Determinant()
{
	//计算校验
	if (ROW != COL) {
		throw("不满足计算条仿");
		return 0.0;
	}

	//如果方阵的维数为1，返回第一个元紿
	if (ROW == 1) { return GetValue(1, 1); }

	//定义返回结果
	double result = 0.0;

	//根据代数余子式进行求觿
	for (unsigned int j = 1; j <= COL; j++)
	{
		result += NegtiveOnePower(1 + j) * GetValue(1, j) * Remainder(1, j).Determinant();
	}

	//返回结果
	return result;
}

//求矩阵的逆矩阿
//计算条件＿、矩阵必须是方阵＿、如果矩阵不可逆，返回[0]矩阵
matrix matrix::Inverse()
{
	//验证计算条件
	if (ROW != COL) { throw("请确保矩阵必须是方阵"); }

	//定义返回矩阵
	matrix result(ROW, COL);

	//求矩阵的秿
	double MatDet = Determinant();

	//如果矩阵的秩=0，返回[0]矩阵
	//矩阵的秩英文rank
	if (MatDet == 0) { return result; }

	//计算MatStart
	for (unsigned int i = 1; i <= ROW; i++)
	{
		for (unsigned int j = 1; j <= COL; j++)
		{
			result(j, i) = NegtiveOnePower(i + j)*Remainder(i, j).Determinant();
		}
	}

	//返回结果
	return result / MatDet;
}

//求对角线元素之和
//计算条件：矩阵必须为方阵，即：m=n
double matrix::Trace()
{
	//验证
	if (ROW != COL) { throw("请确保矩阵必须为方阵"); return 0.0; }

	//定义返回结果
	double result = 0;

	//进行计算
	for (unsigned int i = 1; i <= ROW; i++)
	{
		result += GetValue(i, i);
	}

	//返回结果
	return result;
}

//显示矩阵元素到屏广
void matrix::Display()
{
	//根据行列进行输出
	for (unsigned int i = 1; i <= ROW; i++)
	{
		for (unsigned int j = 1; j <= COL; j++)
		{
			printf("%4.4f  ", GetValue(i, j));
		}
		printf("\r\n");
	}
}


//加运算符
matrix matrix::operator +(const double theValue)
{
	//定义返回结果
	matrix result(ROW, COL);

	//进行减法运算
	for (unsigned int i = 0; i < ROW * COL; i++)
	{
		result.pMatrix[i] = pMatrix[i] + theValue;
	}

	//返回结果
	return result;
}

//减运算符
matrix matrix::operator -(const double theValue)
{
	//定义返回结果
	matrix result(ROW, COL);

	//进行减法运算
	for (unsigned int i = 0; i < ROW * COL; i++)
	{
		result.pMatrix[i] = pMatrix[i] - theValue;
	}

	//返回结果
	return result;
}

//改变矩阵维数
//uiRow=[1...m]
//uiCol=[1...n]
void matrix::ChangeDimension(unsigned int uiRow, unsigned int uiCol)
{
	//重新分配内存
	double * pTemp = new double[uiRow*uiCol];

	//内存复位
	memset(pTemp, 0, sizeof(double)*uiRow*uiCol);

	//获取最小行
	unsigned int MinRow = uiRow < ROW ? uiRow : ROW;

	//获取最小列
	unsigned int MinCol = uiCol < COL ? uiCol : COL;

	//新内存初始化
	for (unsigned int i = 1; i <= MinRow; i++)
	{
		for (unsigned int j = 1; j <= MinCol; j++)
		{
			pTemp[(i - 1)*uiCol + (j - 1)] = pMatrix[(i - 1)*COL + (j - 1)];
		}
	}

	//释放原来内存
	delete pMatrix;

	//重新赋值
	pMatrix = pTemp;

	//改变维数
	ROW = uiRow;
	COL = uiCol;
}

//判断矩阵是否为0矩阵，即所有元素都为0的矩阵
bool matrix::IsZero()
{
	//碰到非0数值，返回false
	for (unsigned int i = 0; i < ROW*COL; i++)
	{
		if (pMatrix[i] != 0) { return false; }
	}

	//返回true
	return true;
}

//将单列或单行的矩阵作为向量返囿
//计算条件：m=1 房n=1
//单行矩阵转换成行向量，单列矩阵转换成列向釿
Vector  matrix::AsVector() const
{
	//验证计算条件
	if (ROW != 1 && COL != 1) {
		throw("不满足转换成向量的条仿");
	}

	//定义返回结果向量
	Vector result((ROW >= COL) ? ROW : COL, (ROW >= COL) ? VERT : HORZ);

	//变量初始匿
	memcpy(result.pVector, pMatrix, result.COUNT * sizeof(double));

	//变量返回
	return result;
}

//求取去除行m_row和列n_col后剩余元素组成的矩阵
//计算条件＿、ROW>1 并且COL>1＿?<=m_row<=ROW 并且 1<=n_col<=COL
// 余子式英文minors，是否改名？
matrix matrix::Remainder(unsigned int m_row, unsigned int n_col)
{
	//验证计算条件
	if (ROW <= 1 || COL <= 1 || m_row < 1 || m_row > ROW || n_col < 1 || n_col > COL) {
		throw("不满足计算条仿");
	}

	//定义返回矩阵
	matrix result(ROW - 1, COL - 1);

	//矩阵元素赋倿
	for (unsigned int i = 1, m = 1; i <= ROW; i++)
	{
		//如果是m_row行的元素,跳过继续
		if (i == m_row) { continue; }

		//进行返回结果的赋值操使
		for (unsigned int j = 1, n = 1; j <= COL; j++)
		{
			//如果是n_col列的元素，跳过继绿
			if (j == n_col) { continue; }

			//赋倿
			result(m, n++) = GetValue(i, j);
		}

		//行号自增
		m++;
	}

	//返回
	return result;
}

//计算-1的n次幂
inline int matrix::NegtiveOnePower(unsigned int n)
{
	return n % 2 == 0 ? 1 : -1;
}

