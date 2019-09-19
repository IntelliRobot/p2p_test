#include "stdafx.h"
#include <math.h>
#include <string.h>
#include "robot.h"
#include "global.h"

//Q1为四元数向量表示的旋转矩阵，返回值为四元素表示的旋转矩阵
vector robot::quater_inv(vector Q1)
{
	//定义返回结果
	vector result(4, VERT);
	
	//前一位不变
	result(1) = Q1(1);

	//后三位取反
	result(2) = -1 * Q1(2);
	result(3) = -1 * Q1(3);
	result(4) = -1 * Q1(4);

	//返回结果
	return result;
}

HALFCOS_COEF_RESULT robot::halfcos_coef(double s0, double sf, double t, double v0, double vf, double vc)
{
	//输入参数个数
	unsigned int margin = 3;
	
	if (v0 != 0) { margin += 1; }
	if (vf != 0) { margin += 1; }
	if (vc != 0) { margin += 1; }

	//定义返回结果并初始化
	HALFCOS_COEF_RESULT result;

	double tf = t;

	//定义平均速度系数，系数越大平均速度越小；模式1用
	double K = 2;

	//计算运行距离
	double s = sf - s0;

	//最大加速度
	double amax = 0;

	//计算平均速度
	if (margin == 5)
	{
		if (vc == 0) { vc = 2 * K / (2 * K - 1) * s / tf; }
		if (vc > 1000) 
		{ 
			result.ReturnMessage.SetMessage("time is too short.");
			return result; 
		}

		//计算最大加速度
		amax = (pi*((fabs(v0 - vc)*(v0 - vc)) / 2 - (fabs(vc - vf)*(vc - vf)) / 2)) / (2 * s - 2 * tf * vc);
		if (amax > 500) 
		{ 
			result.ReturnMessage.SetMessage("time is too short.");
			return result; 
		}
	}
	if (margin == 6) { amax = 500; }

	double dvc0 = vc - v0;//首段半波速度变化幅值
	double dvcf = vf - vc;//末段半波速度变化幅值
	double dtc0 = pi*fabs(dvc0) / (2 * amax);//首段加速度用的时间，半周期；pi*abs(dvc0) / amax 为首段周期
	double dtcf = pi*fabs(dvcf) / (2 * amax);//末段加速度用的时间，半周期；pi*abs(dvcf) / amax 为末段周期
	double D1 = (vc + v0) / 2;//余弦函数偏移值
	double D2 = (vc + vf) / 2;//余弦函数偏移值

	double s0c, slf;

	if (margin >= 6)
	{
		if (dtc0 < 1) { dtc0 = 1; }
		if (dtcf < 1) { dtcf = 1; }

		s0c = D1*dtc0;
		slf = D2*dtcf;

		if (s0c > 0.5*s || slf>0.5*s)
		{
			result.ReturnMessage.SetMessage("speed vc is too high.");
			return result;
		}
	}

	double C1 = 0;
	double C2 = 0;

	double B1 = pi / dtc0;
	double A1 = dvc0 / (2 * B1);
	double E1 = s0;

	double B2 = pi / dtcf;
	double A2 = -dvc0 / (2 * B2);

	double scl = s - s0c - slf;
	double tcl = scl / vc;

	double tc = dtc0;
	double tl=dtc0 + tcl;
	double sl = s0 + s0c + scl;
	double E2 = sl;

	result.hal_A1 = A1;
	result.hal_A2 = A2;
	result.hal_B1 = B1;
	result.hal_B2 = B2;
	result.hal_C1 = C1;
	result.hal_C2 = C2;
	result.hal_D1 = D1;
	result.hal_D2 = D2;
	result.hal_E1 = E1;
	result.hal_E2 = E2;
	result.hal_tf = tf;
	result.hal_tc = tc;
	result.hal_tl = tl;
	result.hal_s0c = s0c;
	result.hal_vc = vc;

	double hal_tf = 0;
	if (margin > 5)
	{
		tf = tl + dtcf;
		hal_tf = fabs(tf);
	}

	result.vc = vc;
	result.tf = hal_tf;
	result.hal_tf = hal_tf;

	result.ReturnMessage.bNoError = true;

	return result;
}

//姿态三次插值系数计算
//Q0起点单位四元数-向量
//Qf终点单位四元数-向量
QUATER_CUBIC_COEF_RESULT robot::quater_cubic_coef(vector Q0, vector Qf, double W0, double Wf, double tf)
{
	//定义变量
	QUATER_CUBIC_COEF_RESULT result;
	
	//定义Q0列向量
	vector Q0_V(Q0);
	Q0_V.TYPE = VERT;
	

	//定义Qf列向量
	vector Qf_V(Qf);
	Qf_V.TYPE = VERT;

	//首末姿态绕固定轴转过的角度的余弦
	double cosTheta = dot(Q0_V, Qf_V) / (Q0_V.Norm() * Qf_V.Norm());

	//选择优弧
	if (cosTheta < 0)
	{
		Q0_V = -1 * Q0_V;
		cosTheta = -1 * cosTheta;
	}

	//参数计算
	double dalpha = 2 * acos(cosTheta);
	double q0 = 0;
	double qf = dalpha;

	//返回变量赋值
	result.cubic_A = (-2 * (qf - q0) + (Wf - W0) * tf) / (tf * tf * tf);
	result.cubic_B = (3 * (qf - q0) - 2 * W0*tf - Wf*tf) / (tf*tf);
	result.cubic_C = W0;
	result.cubic_D = q0;

	result.dalpha = dalpha;

	//无错误指示
	result.ReturnMessage.bNoError = true;
	
	//返回结果
	return result;
}

//将齐次矩阵或旋转矩阵转换成四元素向量
//T:可以是齐次矩阵，也可以是旋转矩阵
//Last Modified:2018-12-20
vector robot::quater_r2q(matrix T)
{
	//提取旋转矩阵
	matrix r = T.SubMatrix(1, 1, 3, 3);

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

	//定义result
	vector result(4, VERT);

	//计算result
	result(1) = qs;
	result(2) = 0.5 * sign_kx * sqrt(kx1);
	result(3) = 0.5 * sign_ky * sqrt(ky1);
	result(4) = 0.5 * sign_kz * sqrt(kz1);

	//返回结果
	return result;
}

//两个四元数向量的叉积
vector robot::quater_cross(vector Q1, vector Q2)
{
	//转换成列向量
	vector Q1_V(Q1);
	Q1_V.TYPE = VERT;

	//转换成列向量
	vector Q2_V(Q2);
	Q2_V.TYPE = VERT;

	//将四元数向量分成两部分
	double s1_q1 = Q1_V(1);
	double s1_q2 = Q2_V(1);

	vector svec_q1(Q1_V.SubVector(2, 4));
	vector svec_q2(Q2_V.SubVector(2, 4));

	//分别计算
	double s1_qq = s1_q1*s1_q2 - dot(svec_q1, svec_q2);
	vector svec_qq(s1_q1*svec_q2+s1_q2*svec_q1+cross(svec_q1,svec_q2));

	//定义返回结果
	vector result(4, VERT);

	//向量赋值
	result(1) = s1_qq;
	memcpy(&result.pVector[1], svec_qq.pVector, svec_qq.COUNT * sizeof(double));

	//返回结果
	return result;
}

//直线轨迹规划参数计算，四元数版
unsigned int robot::line_traj_para(matrix T1, matrix T2, vector TVc, double t_interval, bool V_type)
{
	//获取轨迹运行时间和平均速度
	double Tz = TVc(1);
	double Vc = TVc(2);

	//验证
	if (Vc < 0) { throw("Vc不能是负数"); }

	//提取位置向量
	vector p0_start = T1.SubMatrix(1, 4, 3, 4).AsVector(); //起点位置向量
	vector pf_end = T2.SubMatrix(1, 4, 3, 4).AsVector();   //终点位置向量

	//提取旋转矩阵
	matrix Rq0(T1.SubMatrix(1, 1, 3, 3)); //起点旋转矩阵
	matrix Rqf(T2.SubMatrix(1, 1, 3, 3)); //重点旋转矩阵

	//旋转矩阵转换成四元素表示
	vector Q0_start(quater_r2q(Rq0));
	vector Qf_end(quater_r2q(Rqf));

	//求直线长度
	double p2pL = (pf_end - p0_start).Norm();

	//直线的单位矢量
	vector uvec((pf_end-p0_start)/p2pL);

	//获取转轴单位矢量
	vector QvecTemp(quater_cross(quater_inv(Q0_start), Qf_end));
	vector QvecSub(QvecTemp.SubVector(2, 4));

	vector Qvec(QvecSub / QvecSub.Norm());

	//计算半波正弦系数
	HALFCOS_COEF_RESULT HalfCosCoefResult;
	if (V_type)
	{
		HalfCosCoefResult = halfcos_coef(0, p2pL, Tz, 0, 0, Vc);
	}
	else
	{
		HalfCosCoefResult = halfcos_coef(0, p2pL, Tz, 0, 0);
	}

	//获取插值步数
	unsigned int Na = (unsigned int)(HalfCosCoefResult.hal_tf / t_interval);

	//姿态的三次插值
	QUATER_CUBIC_COEF_RESULT QuaterCubicCoefResult=quater_cubic_coef(Q0_start, Qf_end, 0.0, 0.0, HalfCosCoefResult.hal_tf);

	//准备result
	LINE_TRAJ_PARA_RESULT LineTrajParaResult;

	return Na;
}
