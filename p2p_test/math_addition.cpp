#include "stdafx.h"
#include "math_addition.h"

//数值与矩阵相乘
matrix operator * (double value, matrix & theMatrix)
{
	//定义返回结果
	matrix result(theMatrix.ROW, theMatrix.COL);

	//进行乘法运算
	for (unsigned int i = 0; i < theMatrix.ROW*theMatrix.COL; i++)
	{
		result.pMatrix[i] = theMatrix.pMatrix[i] * value;
	}

	//返回结果
	return result;
}

//数值与向量相乘
Vector operator * (double value, const Vector & theVector)
{
	//定义返回结果
	Vector result(theVector.COUNT, theVector.TYPE);

	//进行计算
	for (unsigned int i = 0; i < theVector.COUNT; i++)
	{
		result.pVector[i] = value * theVector.pVector[i];
	}

	//返回
	return result;
}

//向量与向量相＋
Vector operator +(const Vector &V1, const Vector &V2)
{

	//定义返回结果
	Vector result(V1.COUNT, V1.TYPE);

	//进行计算
	for (unsigned int i = 0; i < V1.COUNT; i++)
	{
		result.pVector[i] = V1.pVector[i] + V2.pVector[i];
	}

	//返回
	return result;
}


//向量与向量相-
Vector operator -(const Vector &V1, const Vector &V2)
{

	//定义返回结果
	Vector result(V1.COUNT, V1.TYPE);

	//进行计算
	for (unsigned int i = 0; i < V1.COUNT; i++)
	{
		result.pVector[i] = V1.pVector[i] - V2.pVector[i];
	}

	//返回
	return result;
}

//向量与向量元素相乘
Vector Vmultiply(const Vector &V1, const Vector &V2)
{

	//定义返回结果
	Vector result(V1.COUNT, V1.TYPE);

	//进行计算
	for (unsigned int i = 0; i < V1.COUNT; i++)
	{

		result.pVector[i] = V1.pVector[i] * V2.pVector[i];
	}

	//返回
	return result;
}

//向量与向量元素相除
Vector Vdevide(const Vector &V1, const Vector &V2)
{

	//定义返回结果
	Vector result(V1.COUNT, V1.TYPE);
	double v2;

	//进行计算
	for (unsigned int i = 0; i < V1.COUNT; i++)
	{
		v2 = V2.pVector[i];
		if (v2 == 0)
		{
			
		result.pVector[i] = 0;

		}
		else
		{
			result.pVector[i] = V1.pVector[i] / v2;
		}
		
	}

	//返回
	return result;
}

Vector Vdevide(const double &V1, const Vector &V2)
{

	//定义返回结果
	Vector result(V2.COUNT, V2.TYPE);
	double v2;

	//进行计算
	for (unsigned int i = 0; i < V2.COUNT; i++)
	{
		v2 = V2.pVector[i];
		if (v2 == 0)
		{
			v2 = 1;
		}
		result.pVector[i] = V1 / v2;
	}

	//返回
	return result;
}


//数值与向量相加
Vector operator +(double value, Vector & theVector)
{
	//定义返回结果
	Vector result(theVector.COUNT, theVector.TYPE);

	//进行计算
	for (unsigned int i = 0; i < theVector.COUNT; i++)
	{
		result.pVector[i] = value + theVector.pVector[i];
	}

	//返回
	return result;
}

//数值与向量相减
Vector operator -(double value, Vector & theVector)
{
	//定义返回结果
	Vector result(theVector.COUNT, theVector.TYPE);

	//进行计算
	for (unsigned int i = 0; i < theVector.COUNT; i++)
	{
		result.pVector[i] = value - theVector.pVector[i];
	}

	//返回
	return result;
}

//数值与矩阵相加
matrix operator + (double value, matrix & theMatrix)
{
	//定义返回结果
	matrix result(theMatrix.ROW, theMatrix.COL);

	//进行乘法运算
	for (unsigned int i = 0; i < theMatrix.ROW*theMatrix.COL; i++)
	{
		result.pMatrix[i] = value + theMatrix.pMatrix[i];
	}

	//返回结果
	return result;
}

//数值与矩阵相减
matrix operator - (double value, matrix & theMatrix)
{
	//定义返回结果
	matrix result(theMatrix.ROW, theMatrix.COL);

	//进行乘法运算
	for (unsigned int i = 0; i < theMatrix.ROW*theMatrix.COL; i++)
	{
		result.pMatrix[i] = value - theMatrix.pMatrix[i];
	}

	//返回结果
	return result;
}

Vector Vsin(const Vector &X)
{
	Vector result(X.COUNT, X.TYPE);
	for (unsigned int i = 0; i < X.COUNT; i++)
	{
		result.pVector[i] = sin(X.pVector[i]);
	}

	return result;
}

Vector Vcos(const Vector &X)
{
	Vector result(X.COUNT, X.TYPE);
	for (unsigned int i = 0; i < X.COUNT; i++)
	{
		result.pVector[i] = cos(X.pVector[i]);
	}

	return result;
}



//向量的点积
double dot(const Vector &a, const Vector &b)
{
	//定义返回结果
	double result = 0;

	//进行计算
	for (unsigned int i = 1; i <= a.COUNT; i++)
	{
		result += a(i)*b(i);
	}

	//返回结果
	return result;
}

//向量的叉积,计算条件：a和b都为3维向量
Vector cross(Vector const &a, Vector const &b)
{
	//校验
	if (a.COUNT != 3 || b.COUNT != 3) { throw("不满足叉积运算条件"); }

	//定义返回结果
	Vector result(3, VERT);

	//进行计算
	result(1) = a(2)*b(3) - a(3)*b(2);
	result(2) = a(3)*b(1) - a(1)*b(3);
	result(3) = a(1)*b(2) - a(2)*b(1);

	//返回结果
	return result;
}


//四元数与四元数相*
Quaternion cross(const Quaternion &Q1, const Quaternion &Q2)
{
	double w0;
	double x0;
	double y0;
	double z0;
	w0 = Q2.w * Q1.w - Q2.x * Q1.x - Q2.y * Q1.y - Q2.z * Q1.z;
	x0 = Q1.w * Q2.x + Q2.w * Q1.x + Q2.z * Q1.y - Q2.y * Q1.z;
	y0 = Q1.w * Q2.y + Q2.w * Q1.y + Q2.x * Q1.z - Q2.z * Q1.x;
	z0 = Q1.w * Q2.z + Q2.w * Q1.z + Q2.y * Q1.x - Q2.x * Q1.y;

	//Quaternion Q_result(w0,x0,y0,z0);
	//w = w0;
	//x = x0;
	//y = y0;
	//z = z0;

	//return *this;
	return Quaternion(w0, x0, y0, z0);

}


Axis_Angle Vector2AA(const Vector & V1)
{
	Axis_Angle result;

	result.Theta = sqrt(V1.pVector[0] * V1.pVector[0] + V1.pVector[1] * V1.pVector[1] + V1.pVector[2] * V1.pVector[2]);



	if (result.Theta > Epsilon)
	{
		result.Qvec.pVector[0] = V1.pVector[0] / result.Theta;
		result.Qvec.pVector[1] = V1.pVector[1] / result.Theta;
		result.Qvec.pVector[2] = V1.pVector[2] / result.Theta;
	}

	else
	{
		result.Qvec = Vector(0, 0, 0);
	}

	return result;
}


Quaternion Vector2Q(const Vector & V1)
{
	//Quaternion result;

	double alpha = sqrt(V1.pVector[0] * V1.pVector[0] + V1.pVector[1] * V1.pVector[1] + V1.pVector[2] * V1.pVector[2]);
	double w, x, y, z;


	if (alpha > Epsilon)
	{
		w = cos(alpha /2);
		x = sin(alpha /2)*V1.pVector[0]/alpha;
		y = sin(alpha /2)*V1.pVector[1]/alpha;
		z = sin(alpha /2)*V1.pVector[2]/alpha;
	}

	else
	{
		w =1;
		x = 0;
		y = 0;
		z = 0;;
	}

	return Quaternion(w,x,y,z);
}


Posi_Pose Matrix2PP( matrix &M1)
{
	Posi_Pose result;
	result.Pose = Quaternion(M1);
	result.Posi = ZeroVector3;
	result.Posi.pVector[0] = M1.GetValue(1, 4);
	result.Posi.pVector[1]= M1.GetValue(2, 4);
	result.Posi.pVector[2]= M1.GetValue(3, 4);

	return result;

}


int Sign(double x)
{
	int sign;
	if (x >= 0.0)
	{
		sign = 1;
	}
	else
	{
		sign = -1;

	}
	return sign;
}

double rad2deg(const double r)
{
	return r * 180 / PI;

}

double deg2rad(const double &deg)
{
	return deg * PI / 180;

}

Vector deg2rad(const Vector  &deg)
{
	//定义返回结果
	Vector result(deg.COUNT, deg.TYPE);

	//进行计算
	for (unsigned int i = 0; i < deg.COUNT; i++)
	{
		result.pVector[i] =  PI* deg.pVector[i]/180;
	}

	//返回
	return result;
}

void Cubic_spline_coef(const Vector &q0, const Vector &qf, const Vector &qd0, const Vector &qdf, double tf, Cubic_Coef &CC)
{
	int N = q0.COUNT;

	for (int i = 1; i <= N; i++)
	{
		CC.A.SetValue(i, (-2 * (qf.GetValue(i) - q0.GetValue(i)) + (qdf.GetValue(i) + qd0.GetValue(i))*tf) / (tf*tf*tf));
		CC.B.SetValue(i, (3 * (qf.GetValue(i) - q0.GetValue(i)) - 2 * qd0.GetValue(i)*tf - qdf.GetValue(i)*tf) / (tf*tf));
		CC.C.SetValue(i, qd0.GetValue(i));
		CC.D.SetValue(i, q0.GetValue(i));

	}
}

void Halfcos_coef(const Vector &q0, const Vector &qf, const Vector &qd0, const Vector &qdf, enum TIMEorSPEED method,double tf,  const Vector &V_Vc_Wc,
	 const Vector &qddmax,Halfcos_Coef &HC, const int &tf_max_index)
{
	
	int N = qd0.COUNT;
	//int Max_tf_index=0;
	////Vector scale = Vdevide(Tf,Vector(tf, N));
	//double tf=Tf.Max(Max_tf_index);
	
	Vector s= qf - q0;
	Vector dvc0(N, qd0.TYPE);
	Vector dvcf(N, qd0.TYPE);
	Vector dtc0(N, qd0.TYPE);
	Vector dtcf(N, qd0.TYPE);
	Vector s0c(N, qd0.TYPE);
	Vector slf(N, qd0.TYPE);
	Vector scl(N, qd0.TYPE);
	Vector tcl(N, qd0.TYPE);
	Vector tl(N, qd0.TYPE);
	Vector qdc(N, qd0.TYPE);
	Vector qddc(N, qd0.TYPE);

	//Vector V_V_W_mintf(N, qd0.TYPE);



	
	for (int i = 0; i < N; i++)
	{
	
		if ((tf_max_index==i+1)&&(method==speed))
		{
			qddc.pVector[i] = qddmax.pVector[i];
			qdc.pVector[i] = V_Vc_Wc(tf_max_index);
			continue;
		}
		if ((tf_max_index == i + 1) && (method == time))
		{
			qddc.pVector[i] = qddmax.pVector[i];
			qdc.pVector[i] = sqrt(2.0*qddmax.pVector[i] * fabs(s.pVector[i])/PI);
			continue;
		}

		if (s.pVector[i] < 0.1*Epsilon)
		{
			qddc.pVector[i] = 0;
			qdc.pVector[i] = 0;
			continue;
		}
		
		if (tf>0)
		{
						
			int K =2; //余弦形状调节器
			qdc.pVector[i] = 2.0 * K / (2.0 * K - 1.0)*s.pVector[i] / tf;
			double qdc0 = qd0.pVector[i] - qdc.pVector[i];
			double qdcf = qdc.pVector[i] - qdf.pVector[i];
			double temp = ((fabs(qdc0)*qdc0 - (fabs(qdcf)*qdcf)));
			double temp2 = (1.0 / (2.0*K - 1.0)*s.pVector[i]);
			qddc.pVector[i] = -1.0 / 4.0*PI*temp / temp2;
			//qddc.pVector[i] = -1.0/4.0*PI*((fabs(qdc0)*qdc0  - (fabs(qdcf)*qdcf) )) / ( 1.0/(2.0*K-1.0)*s.pVector[i]);
		}



	}

	
		

	dvc0 = qdc - qd0;
	dvcf = qdf - qdc;
	HC.D1 = 0.5*(qdc + qd0);
	HC.D2 = 0.5*(qdc + qdf);
	
	
	dtc0 = Vdevide(dvc0.Abs(), qddc);
	dtc0 = PI*0.5*dtc0;
	dtcf = Vdevide(dvcf.Abs(), qddc);
	dtcf = PI*0.5*dtcf;


	s0c = Vmultiply(dtc0, HC.D1);
	slf = Vmultiply(dtcf, HC.D2);
	scl = s - s0c - slf;

	if (scl.Min()<0)
	{
		 printf("tf时间太短或平均速度太快");
		 throw("tf时间太短");
		
	}
	
	tcl = Vdevide(scl, qdc);
	HC.Tl = dtc0 + tcl;
	HC.Tc = dtc0;
	HC.S0C = s0c;

	HC.C1 = Vector(0.0,N);
	HC.C2 = Vector(0.0,N);
	HC.B1 = Vdevide(PI, dtc0);
	HC.A1 = Vdevide(dvc0, -2.0*HC.B1);
	HC.E1 = q0;

	HC.B2 = Vdevide(PI, dtcf);
	HC.A2 = Vdevide(dvcf, -2.0*HC.B2);
	HC.E2 = q0 + s0c + scl;

	HC.VC = qdc;
	HC.Vf = qdf;

	HC.S = s;
	HC.Tf = tf;
	
}

//void Halfcos_coef(const Vector &q0, const Vector &qf, const Vector &qd0, const Vector &qdf, enum TIMEorSPEED method, double tf, Vector &qdc, Halfcos_Coef &HC)
//{

Vector Halfcos_Tf(const Vector & s, const Vector & qd0, const Vector & qdf, const Vector & qdmax, const Vector & qddmax)
{
	int Max_tf_index = 0;
	Vector qddc = qddmax;
	Vector qdc = qdmax;
	Vector dvc0 = qdc - qd0;
	Vector dvcf = qdf - qdc;
	Vector HCD1 = 0.5*(qdc + qd0);
	Vector HCD2 = 0.5*(qdc + qdf);
	Vector dtc0 = PI*0.5*Vdevide(dvc0.Abs(), qddc);
	Vector dtcf = PI*0.5*Vdevide(dvcf.Abs(), qddc);
	Vector s0c = Vmultiply(dtc0, HCD1);
	Vector slf = Vmultiply(dtcf, HCD2);
	Vector scl = s - s0c - slf;
	Vector tcl = Vdevide(scl, qdc);
	Vector tl = dtc0 + tcl;
	Vector Tf = tl + dtcf;

	double tf = Tf.Max(Max_tf_index);//求出最长时间

	if ((tf <= 0) || scl(Max_tf_index)<0)
	{
		printf("速度太快或时间太短");
		throw("tf时间太短");
	}

	return Tf;

}

void Trapez_coef(const Vector &q0, const Vector &qf, const Vector &qd0, const Vector &qdf, enum TIMEorSPEED method, double tf, const Vector &V_Vc_Wc,
	const Vector &qddmax, Trapez_Coef &TC, const int &tf_max_index)
{
	int N = qd0.COUNT;
	//int Max_tf_index = 0;
	//Vector scale = Vdevide(Tf,Vector(tf, N));
	//double tf_min_limit=Tf.Max(Max_tf_index);

	Vector s = qf - q0;
	Vector dvc0(N, qd0.TYPE);
	Vector dvcf(N, qd0.TYPE);
	Vector dtc0(N, qd0.TYPE);
	Vector dtcf(N, qd0.TYPE);
	Vector s0c(N, qd0.TYPE);
	Vector slf(N, qd0.TYPE);
	Vector scl(N, qd0.TYPE);
	Vector tcl(N, qd0.TYPE);
	Vector tl(N, qd0.TYPE);
	Vector qdc(N, qd0.TYPE);
	Vector qddc(N, qd0.TYPE);

	//Vector V_V_W_mintf(N, qd0.TYPE);




	for (int i = 0; i < N; i++)
	{

		if ((tf_max_index == i + 1) && (method == speed))
		{
			qddc.pVector[i] = qddmax.pVector[i];
			qdc.pVector[i] = V_Vc_Wc(tf_max_index);
			continue;
		}
		if ((tf_max_index == i + 1) && (method == time))
		{
			qddc.pVector[i] = qddmax.pVector[i];
			qdc.pVector[i] = sqrt(qddmax.pVector[i] * fabs(s.pVector[i]));
			continue;
		}

		if (s.pVector[i] < 0.1*Epsilon)
		{
			qddc.pVector[i] = 0;
			qdc.pVector[i] = 0;
			continue;
		}


		if (tf > 0)
		{
			int K = 2; //余弦形状调节器
			qdc.pVector[i] = 2.0 * K / (2.0 * K - 1.0)*s.pVector[i] / tf;
			double qdc0 = qd0.pVector[i] - qdc.pVector[i];
			double qdcf = qdc.pVector[i] - qdf.pVector[i];
			double temp = ((fabs(qdc0)*qdc0 - (fabs(qdcf)*qdcf)));
			double temp2 = (1.0 / (2.0*K - 1.0)*s.pVector[i]);
			qddc.pVector[i] = -1.0 / 2.0*temp / temp2;
			//qddc.pVector[i] = -1.0/4.0*PI*((fabs(qdc0)*qdc0  - (fabs(qdcf)*qdcf) )) / ( 1.0/(2.0*K-1.0)*s.pVector[i]);
		}
	}

	dvc0 = qdc - qd0;
	dvcf = qdf - qdc;
	
	TC.Tc = Vdevide(dvc0.Abs(), qddc);

	TC.Tl =  Vector(tf,N) - Vdevide(dvcf.Abs(), qddc);
	TC.A1 = Vmultiply(dvc0.Sign(), qddc);
	TC.A2 = Vmultiply(dvcf.Sign(), qddc);
	Vector vvc0 = Vmultiply(qdc, qdc) - Vmultiply(qd0, qd0);
	TC.Stc = Vdevide(vvc0, 2.0*TC.A1);
	Vector vvcf = Vmultiply(qdf, qdf) - Vmultiply(qdc, qdc);
	TC.Stl = s-Vdevide(vvcf, 2.0*TC.A2);
	TC.V0 = qd0;
	TC.Vc=qdc;
	TC.Vf = qdf;
	TC.S = s;
	TC.Tf = tf;
	//TC.Stl = TC.Stc + Vmultiply(qdc, TC.Tl - TC.Tc);
	
}

Vector Trapez_Tf(const Vector & s, const Vector & qd0, const Vector & qdf, const Vector & qdmax, const Vector & qddmax)
{
	int Max_tf_index = 0;
	Vector qddc = qddmax;
	Vector qdc = qdmax;
	Vector dvc0 = qdc - qd0;
	Vector dvcf = qdf - qdc;
	Vector Tc0 = Vdevide(dvc0.Abs(), qddc);
	Vector Tlf = Vdevide(dvcf.Abs(), qddc);
	Vector A1 = Vmultiply(dvc0.Sign(), qddc);
	Vector A2 = Vmultiply(dvcf.Sign(), qddc);
	Vector vvc0 = Vmultiply(qdc, qdc) - Vmultiply(qd0, qd0);
	Vector Stc = Vdevide(vvc0, 2.0*A1);
	Vector vvcf = Vmultiply(qdf, qdf) - Vmultiply(qdc, qdc);
	Vector Stl = s - Vdevide(vvcf, 2.0*A2);
	Vector Scl = Stl - Stc;
	Vector tcl = Vdevide(Scl, qdc);
	Vector tl = Tc0 + tcl;
	Vector Tf = tl + Tlf;



	double tf = Tf.Max(Max_tf_index);//求出最长时间

	if ((tf <= 0) || Scl(Max_tf_index)<0)
	{
		printf("速度太快或时间太短");
		throw("tf时间太短");
	}

	return Tf;

}


double Cubic_max_tf(const Vector & delta_q, const Vector & qd0, const Vector & qdf, const Vector & qdmax, const Vector & qddmax)
{
	int N = delta_q.COUNT;
	Vector F1(N, VERT);
	Vector F2(N, VERT);
	double tf_max = 0;

	for (int i = 1; i <= N; i++) // 计算最小
	{
		double v1_temp = (qd0(i) + qdmax(i))*(qdf(i) + qdmax(i));
		double v2_temp = (qd0(i) - qdmax(i))*(qdf(i) - qdmax(i));
		double qd0f = qd0(i) + qdf(i);
		double tf_acce_max=0;
		if (v1_temp<0 || v2_temp<0) { printf("时间太短，速度太快"); }
		else
		{
			F1.SetValue(i, (qd0f - qdmax(i) - sqrt(v1_temp)) / 3);
			F2.SetValue(i, (qd0f + qdmax(i) + sqrt(v2_temp)) / 3);

			double tf1 = delta_q(i) / F1(i);
			double tf2 = delta_q(i) / F2(i);

			double tf_temp = tf1 > tf2 ? tf1 : tf2;

			tf_max = tf_temp> tf_max ? tf_temp : tf_max;
		}

		if ((qd0(i)==0) &&(qdf(i)==0))
		{
			tf_acce_max = fabs(6 * delta_q(i) / qddmax(i));
			tf_acce_max = sqrt(tf_acce_max);
		}
		tf_max = tf_acce_max> tf_max ? tf_acce_max : tf_max;

	}
	
	return tf_max;
}


void Cubic_spline_rt(Vector &qt, Vector &qdt, Vector &qddt, const Cubic_Coef &CC, double &t)
{
	double t_2 = t*t;
	double t_3 = t*t*t;
	qt = t_3*CC.A+ t_2*CC.B + t*CC.C+ CC.D;
	qdt = (t_2 * 3)*CC.A+ (2 * t)*CC.B+ CC.C;
	qddt = (6 * t)*CC.A + 2*CC.B ;

}



void Halfcos_rt(Vector &qt, Vector &qdt, Vector &qddt,const Halfcos_Coef &HC, double &t)
{
	for (unsigned int i = 0; i < qt.COUNT; i++)
	{
		if (t<HC.Tc.pVector[i])
		{
			qt.pVector[i] = HC.A1.pVector[i] * sin(HC.B1.pVector[i] * t + HC.C1.pVector[i]) + HC.D1.pVector[i] * t + HC.E1.pVector[i];
			qdt.pVector[i] = HC.A1.pVector[i] * HC.B1.pVector[i] * cos(HC.B1.pVector[i] * t + HC.C1.pVector[i]) + HC.D1.pVector[i];
			qddt.pVector[i] = -HC.A1.pVector[i] * HC.B1.pVector[i] * HC.B1.pVector[i] * sin(HC.B1.pVector[i] * t + HC.C1.pVector[i]);
		}
		else if (t<HC.Tl.pVector[i])
		{

			qt.pVector[i] = HC.S0C.pVector[i] + HC.VC.pVector[i] * (t - HC.Tc.pVector[i]);
			qdt.pVector[i] = HC.VC.pVector[i];
			qddt.pVector[i] = 0;
		}
		else if (t <= HC.Tf)
		{
			double dtl = t - HC.Tl.pVector[i];
			qt.pVector[i] = HC.A2.pVector[i] * sin(HC.B2.pVector[i] * dtl + HC.C2.pVector[i]) + HC.D2.pVector[i] * dtl + HC.E2.pVector[i];
			qdt.pVector[i] = HC.A2.pVector[i] * HC.B2.pVector[i] * cos(HC.B2.pVector[i] * dtl + HC.C2.pVector[i]) + HC.D2.pVector[i];
			qddt.pVector[i] = -HC.A2.pVector[i] * HC.B2.pVector[i] * HC.B2.pVector[i] * sin(HC.B2.pVector[i] * dtl + HC.C2.pVector[i]);
		}

		else
		{
			qt.pVector[i] = HC.S.pVector[i];
			qdt.pVector[i] = HC.Vf.pVector[i];
			qddt.pVector[i] = 0;
		}

	}


}

void Trapez_rt( Vector &qt, Vector &qdt, Vector &qddt, Trapez_Coef &TC,double &t)
{
	

	for (int i = 0; i < qt.COUNT; i++)
	{
		if (t < TC.Tc.pVector[i])  //第1段
		{	
			qt.pVector[i] = 0.5*TC.A1.pVector[i]*t*t+t*TC.V0.pVector[i]; // 距离
			qdt.pVector[i] = TC.A1.pVector[i]*t+TC.V0.pVector[i]; // 线速度
			qddt.pVector[i] = TC.A1.pVector[i]; // 线加速度 
		}


		else if (t < TC.Tl.pVector[i]) //线性段
		{
			qt.pVector[i] = TC.Stc.pVector[i]+ TC.Vc.pVector[i]*(t - TC.Tc.pVector[i]);
			qdt.pVector[i] = TC.Vc.pVector[i];
			qddt.pVector[i] = 0;
		}

		else if(t<=TC.Tf) //第2段
		{
			double dtl = t - TC.Tl.pVector[i];

			qt.pVector[i] = 0.5*TC.A2.pVector[i] * dtl*dtl + dtl*TC.Vc.pVector[i]+TC.Stl.pVector[i]; // 距离
			qdt.pVector[i] = TC.A2.pVector[i] * dtl + TC.Vc.pVector[i]; // 线速度
			qddt.pVector[i] = TC.A2.pVector[i]; // 线加速度 
		}

		else
		{
			qt.pVector[i] = TC.S.pVector[i];
			qdt.pVector[i] = TC.Vf.pVector[i];
			qddt.pVector[i] = TC.A2.pVector[i];
		}

	}

}