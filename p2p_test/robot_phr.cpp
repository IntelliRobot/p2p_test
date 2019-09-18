#pragma once
#include "stdafx.h"
#include "robot_phr.h"
#include "math.h"


robot_phr::robot_phr(const DH_PARA dh)
	: mat_t60(4, 4), mat_Tcp(4, 4), vec_qn_e(6, HORZ)
{
	DH = dh;
	
	 q_n=q_zerop;

	 q_L = q_n;

	 qd_n=ZeroVector6;//当前关节角速度

	qdd_n=ZeroVector6;//当前关节角加速度

	 qd_percent=0.0;//最大角速度百分比


	//Vector q_singular_start;

	//Vector q_singular_end;

	//Vector qd_singular_start;

	//Vector qd_singular_end


	//Ksew Ksew_n;//当前关节奇异值

	//Ksew Ksew_L;//前一个关节奇异值

	//Ksew Ksew_temp;//关节奇异值临时值

	//Ksew Ksew_limit;

	int Singular_flag = 0;

	int CFG = 0;
}


robot_phr::~robot_phr()
{
}


//void robot_phr::P2P(Posi_Pose PP0, Posi_Pose PPf, enum P2Pmethod cht , enum TIMEorSPEED method , double tf_s , Vector V0, Vector Vf , double qddp, double t_interval_)
//{
//	Vector q0 = Ikine_cfg(PP0);
//	Vector qf = Ikine_cfg(PPf);
//	Vector qd0 = V2qd(V0,q0);
//	Vector qdf = V2qd(Vf,qf);
//	P2P_ P12(q0, qf, cht, method, tf_s, qd0, qdf , qddp, t_interval_);
//	P12.offline();
//}


void robot_phr::K_sew(const Vector &q_n)
{	
	//Ksew_n = Ksew_temp;
	Ksew_temp.Ks = DH.a1 + DH.a3*cos(q_n(2) + q_n(3)) - DH.d4*sin(q_n(2) + q_n(3)) + DH.a2*cos(q_n(2));//q2,q3
	Ksew_temp.Ke = DH.d4*cos(q_n(3)) + DH.a3*sin(q_n(3));//q3
	Ksew_temp.Kw = -sin(q_n(5));//q5
}



int robot_phr::Qcfg_K(const Vector &q_n)
{
	bool lg[3];

	K_sew(q_n);

	if (Ksew_temp.Ks < 0)	{lg[0] = true;}
	else {lg[0] = false;}

	if (Ksew_temp.Ke < 0)	{lg[1] = true;}
	else {lg[1] = false;}

	if (Ksew_temp.Kw < 0)	{lg[2] = true;}
	else {lg[2] = false;}
	
	int cfg = lg[0] * 4 + lg[1] * 2 + lg[2];

	return	cfg;

}



//返回8个CFG
Vector robot_phr::Qcfg_M(const matrix &QM)
{
	Vector result(QM.COL, HORZ);

	for (int i = 1; i <=QM.ROW; i++)
	{
		result.pVector[i - 1] = Qcfg_K(QM.SubVector(i, HORZ));
	}

	return result;
}



//从q中选出与q0构型cfg一样的一组解；返回解和K值
Vector robot_phr::Q_samecfg(const matrix &QM, const Vector &q0)
{
	//求取q的cfg=[cfg,K]
	int cfg = Qcfg_K(q0);

	return Q_samecfg(QM, cfg);

}


Vector robot_phr::Q_samecfg(const matrix &QM, const int &cfg)
{
	int cfg_qi;
	Vector result = ZeroVector6;//定义返回值q
	result.TYPE = HORZ;

	for (int i = 1; i <= QM.ROW; i++)
	{
		cfg_qi = Qcfg_K(QM.SubVector(i,HORZ));

		if (cfg == cfg_qi)
		{
			//Ksew_n = Ksew_temp;
			result = QM.SubVector(i,HORZ);
			break;
		}
	}

	return result;
}



//matlab 对应 ik_Nearest_cfg
Vector robot_phr::Ikine_nearest(const matrix &Tn, Vector &q_n)
{
	//先求8组逆解
	//matrix qIK8(Ikine_matrix(PPn));
	//寻找与q0的cfg相同的一组解
	//Vector qIK_N1(Q_samecfg(qIK8, q_n));
	Vector qIK_N1(Ikine_cfg(Tn, CFG));
	Ksew_L = Ksew_n;
	//CFG = Qcfg_K(q_n);
	Ksew_n = Ksew_temp;


	///////////////////////////////////////////////
	//----------------------------------
	// |    |    |    | q41 | q5 | q61 | ----原来的
	// |    |    |    | q41 | q5 | q63 |
	// | q1 | q3 | q2 |-----------------
	// |    |    |    | q43 | q5 | q61 |
	// |    |    |    | q43 | q5 | q63 |
	//----------------------------------
	//由于theta4和theta6实际范围内存在周期解
	//所以将qIK_N1的一行扩展成4行
	///////////////////////////////////////////////

	//提取theta4、theta5、theta6
	double q41 = qIK_N1(4);
	double q5plus = qIK_N1(5);
	double q61 = qIK_N1(6);

	//求q43、q63
	double q43 = (q41 >= 0) ? (q41 - 2 * PI) : (q41 + 2 * PI);
	double q63 = (q61 >= 0) ? (q61 - 2 * PI) : (q61 + 2 * PI);

	//[qIK_N4]变量定义
	matrix qIK_N4(4, 6);

	//[qIK_N4]部分赋值，theta1、theta2、theta3保持不变
	for (int i = 1; i <= 4; i++)
	{
		for (int j = 1; j <= 3; j++)
		{
			qIK_N4(i, j) = qIK_N1(j);
		}
	}

	//[qIK_N4]部分赋值将theta4和theta6 按上表排列到矩阵中
	qIK_N4(1, 4) = q41;
	qIK_N4(2, 4) = q41;
	qIK_N4(3, 4) = q43;
	qIK_N4(4, 4) = q43;
	qIK_N4(1, 5) = q5plus;
	qIK_N4(2, 5) = q5plus;
	qIK_N4(3, 5) = q5plus;
	qIK_N4(4, 5) = q5plus;
	qIK_N4(1, 6) = q61;
	qIK_N4(2, 6) = q63;
	qIK_N4(3, 6) = q61;
	qIK_N4(4, 6) = q63;

	//变量定义
	Vector e4(4,VERT);
	Vector e6(2,VERT);

	//求e4
	for (int i = 1; i <= 4; i++)
	{
		e4(i) = fabs(qIK_N4(i, 4) - q_n(4));
	}

	//求ed4
	//ed4 = e4.Abs();

	//在4组解中求出theta4误差最小的2组
	unsigned int I = e4.IndexOfMinValue();
	matrix qIK_N2(qIK_N4.SubMatrix(I, 1, I + 1, qIK_N4.COL));

	//求解e6
	for (int i = 1; i <= 2; i++)
	{
		e6(i) = fabs(qIK_N2(i, 6) - q_n(6));
	}

	//求解ed6
	//ed6 = e6.Abs();

	//在2组解中求出theta6误差最小的1组解
	I = e6.IndexOfMinValue();

	Vector qIK(qIK_N2.SubMatrix(I, 1, I, qIK_N2.COL).AsVector());
	q_L = q_n;
	q_n = qIK;

	return q_n;

}

matrix robot_phr::Ikine_matrix(const matrix &T)
{
	//matrix tend(PPn.Pose.Rot());
	
	
	//取值		
	double nx = T.GetValue(1, 1);
	double ny = T.GetValue(2, 1);
	double nz = T.GetValue(3, 1);

	double ox = T.GetValue(1, 2);
	double oy = T.GetValue(2, 2);
	double oz = T.GetValue(3, 2);

	double ax = T.GetValue(1, 3);
	double ay = T.GetValue(2, 3);
	double az = T.GetValue(3, 3);

	double px = T.GetValue(1,4);
	double py = T.GetValue(2,4);
	double pz = T.GetValue(3,4);

	double k3_temp;

	//定义结果矩阵
	matrix theta(8, 6);

	//[m0]计算m0_2
	double m0_2 = px*px + py*py - DH.d3*DH.d3;

	//[m0]如果m0_2<0，则无解，报错返回
	if (m0_2 < 0) { throw("point not reachable"); return theta; }

	//[m0]计算m0
	double m0 = sqrt(m0_2);

	//[theta1]定义theta1
	Vector theta1(2, VERT);

	//[theta1]计算theta1的值
	theta1(1) = atan2(py, px) - atan2(DH.d3, m0);
	if (theta1(1) >= 0)
	{
		theta1(2) = theta1(1) - PI;
	}
	else
	{
		theta1(2) = theta1(1) + PI;
	}

	//[theta1]归一化处理，保证theta1的值位于区间[-pi,pi]。
	for (int i = 1; i <= 2; i++)
	{
		if (theta1(i) < -PI) { theta1(i) += 2 * PI; }
	}

	//变量定义
	Vector m1(2, VERT), m2(2, VERT), m3(2, VERT);

	//求取m1
	m1(1) = cos(theta1(1));
	m1(2) = cos(theta1(2));

	//求取m2
	m2(1) = sin(theta1(1));
	m2(2) = sin(theta1(2));

	//求取m3
	m3 = px*m1 + py*m2;

	//[theta3]定义theta3
	Vector theta3(4, VERT), K3(2, VERT), m4(2, VERT);

	//[theta3]计算K3
	//K3 = ( pz *pz- 2 * DH.d1*pz + DH.a1*DH.a1 - DH.a2*DH.a2 - DH.a3 *DH.a3 + DH.d1 *DH.d1 - DH.d4*DH.d4 + Vmultiply(m3, m3) - 2 * DH.a1*m3) / (2 * DH.a2);
	k3_temp = pz * pz - 2 * DH.d1 * pz + DH.a1 * DH.a1 - DH.a2 * DH.a2 - DH.a3 * DH.a3 + DH.d1 * DH.d1 - DH.d4 * DH.d4;
	K3 = (Vector(k3_temp, 2) + Vmultiply(m3, m3) - 2 * DH.a1 * m3) / (2 * DH.a2);

	//[theta3]计算m4
	m4 = Vector((DH.d4*DH.d4 + DH.a3*DH.a3), 2) - Vmultiply(K3, K3);

	//定义变量
	Vector ML(m4.Sign());

	//[theta3]判断解的个数
	if (ML(1) < 0 && ML(2) < 0)/*m4两个值都小于0，则无解*/
	{
		return theta;
	}
	else if (ML(1) < 0 && ML(2) >= 0)
	{
		m4(1) = 0;//NaN
	}
	else if (ML(1) >= 0 && ML(2) < 0)
	{
		m4(2) = 0;//NaN
	}
	else {}

	//[theta3]计算theta3
	double atan2_a3d4 = atan2(DH.a3, DH.d4);
	theta3(1) = atan2_a3d4 - atan2(K3(1), sqrt(m4(1)));
	theta3(2) = atan2_a3d4 - atan2(K3(2), sqrt(m4(2)));
	theta3(3) = atan2_a3d4 - atan2(K3(1), -sqrt(m4(1)));
	theta3(4) = atan2_a3d4 - atan2(K3(2), -sqrt(m4(2)));

	//[theta3]归一化处理，保证theta3的值位于区间[-3pi/2,pi/2]。
	for (int i = 1; i <= 4; i++)
	{
		if (theta3(i) > PI / 2) { theta3(i) -= 2 * PI; }
	}

	//变量定义
	Vector m5(4, VERT), m6(4, VERT);

	//计算m5
	m5(1) = sin(theta3(1));
	m5(2) = sin(theta3(2));
	m5(3) = sin(theta3(3));
	m5(4) = sin(theta3(4));

	//计算m6
	m6(1) = cos(theta3(1));
	m6(2) = cos(theta3(2));
	m6(3) = cos(theta3(3));
	m6(4) = cos(theta3(4));

	//扩展theta1
	theta1.ChangeDimension(4);

	theta1(3) = theta1(1);
	theta1(4) = theta1(2);

	//扩展m1
	m1.ChangeDimension(4);

	m1(3) = m1(1);
	m1(4) = m1(2);

	//扩展m2
	m2.ChangeDimension(4);

	m2(3) = m2(1);
	m2(4) = m2(2);

	//扩展m3
	m3.ChangeDimension(4);

	m3(3) = m3(1);
	m3(4) = m3(2);

	//[theta2]变量定义
	Vector S23(4, VERT), C23(4, VERT), theta23(4, VERT), theta2(4, VERT);

	//[theta2]计算S23、C23
	//S23 = (DH.a3*DH.d1 + DH.a1*DH.d4 - DH.a3*pz - DH.a2*pz*m6 - DH.a1*DH.a2*m5 - DH.d4*m3 + DH.a2*DH.d1*m6 + DH.a2*Vmultiply(m3,m5));
	S23 = Vector((DH.a3*DH.d1 + DH.a1*DH.d4 - DH.a3*pz), 4) - DH.a2*pz*m6 - DH.a1*DH.a2*m5 - DH.d4*m3 + DH.a2*DH.d1*m6 + DH.a2*Vmultiply(m3, m5);
	//C23 = (DH.d1*DH.d4 - DH.a1*DH.a3 - DH.d4*pz + DH.a3*m3 - DH.a2*DH.d1*m5 + DH.a2*pz*m5 - DH.a1*DH.a2*m6 + DH.a2*Vmultiply(m3,m6));
	C23 = Vector((DH.d1*DH.d4 - DH.a1*DH.a3 - DH.d4*pz), 4) + DH.a3*m3 - DH.a2*DH.d1*m5 + DH.a2*pz*m5 - DH.a1*DH.a2*m6 + DH.a2*Vmultiply(m3, m6);

	//[theta2]计算theta23
	theta23(1) = atan2(S23(1), C23(1));
	theta23(2) = atan2(S23(2), C23(2));
	theta23(3) = atan2(S23(3), C23(3));
	theta23(4) = atan2(S23(4), C23(4));

	//[theta2]计算theta2
	theta2 = theta23 - theta3;

	//[theta2]归一化处理，保证theta2的值位于区间[-pi,pi/2]。
	for (int i = 1; i <= 4; i++)
	{
		if (theta2(i) > PI)
		{
			theta2(i) -= 2 * PI;
		}
		else if (theta2(i)<-PI)
		{
			theta2(i) += 2 * PI;
		}
		else {}
	}

	//变量定义
	Vector m8(4, VERT), m9(4, VERT), m10(4, VERT);

	//计算m8
	m8(1) = cos(theta23(1));
	m8(2) = cos(theta23(2));
	m8(3) = cos(theta23(3));
	m8(4) = cos(theta23(4));

	//计算m9
	m9(1) = sin(theta23(1));
	m9(2) = sin(theta23(2));
	m9(3) = sin(theta23(3));
	m9(4) = sin(theta23(4));

	//计算m10
	m10 = ax*m1 + ay*m2;

	//[theta4]变量定义
	Vector C4(4, VERT), S4(4, VERT), theta4(4, VERT);

	//[theta4]计算C4、S4
	C4 = Vmultiply(-1.0*m8,m10) + az*m9;
	S4 = ay*m1 - ax*m2;

	//[theta4]计算theta4
	theta4(1) = atan2(S4(1), C4(1));
	theta4(2) = atan2(S4(2), C4(2));
	theta4(3) = atan2(S4(3), C4(3));
	theta4(4) = atan2(S4(4), C4(4));

	//变量定义
	Vector m11(4, VERT), m12(4, VERT);

	//计算m11
	m11(1) = cos(theta4(1));
	m11(2) = cos(theta4(2));
	m11(3) = cos(theta4(3));
	m11(4) = cos(theta4(4));

	//计算m12
	m12(1) = sin(theta4(1));
	m12(2) = sin(theta4(2));
	m12(3) = sin(theta4(3));
	m12(4) = sin(theta4(4));

	//[theta5]变量定义
	Vector C5(4, VERT), S5(4, VERT), theta5(4, VERT);

	//[theta5]计算C5、S5
	S5 = -1 * Vmultiply(Vmultiply(m8,m11),m10) + az*Vmultiply(m9,m11) + Vmultiply(m12,S4);
	C5 = -1 * az*m8 - Vmultiply(m9,m10);

	//[theta5]计算theta5
	theta5(1) = atan2(S5(1), C5(1));
	theta5(2) = atan2(S5(2), C5(2));
	theta5(3) = atan2(S5(3), C5(3));
	theta5(4) = atan2(S5(4), C5(4));

	//变量定义
	Vector m13(4, VERT), m14(4, VERT), m15(4, VERT), m16(4, VERT);

	//计算m13
	m13(1) = sin(theta5(1));
	m13(2) = sin(theta5(2));
	m13(3) = sin(theta5(3));
	m13(4) = sin(theta5(4));

	//计算m14
	m14(1) = cos(theta5(1));
	m14(2) = cos(theta5(2));
	m14(3) = cos(theta5(3));
	m14(4) = cos(theta5(4));

	//计算m15、m16
	m15 = nx*m1 + ny*m2;
	m16 = ny*m1 - nx*m2;

	//[theta6]变量定义
	Vector C6(4, VERT), S6(4, VERT), theta6(4, VERT);

	//[theta6]计算C6、S6
	S6 = -1 * Vmultiply(m11,m16) + nz*Vmultiply(m9,m12) - Vmultiply(Vmultiply(m8,m12),m15);
	C6 = Vmultiply(Vmultiply(m8,m11),Vmultiply(m14,m15)) - Vmultiply(Vmultiply(m9,m13),m15) - Vmultiply(Vmultiply(m14,m12),m16) - nz*Vmultiply(Vmultiply(m9,m11),m14) - nz*Vmultiply(m8,m13);

	//[theta6]计算theta6
	theta6(1) = atan2(S6(1), C6(1));
	theta6(2) = atan2(S6(2), C6(2));
	theta6(3) = atan2(S6(3), C6(3));
	theta6(4) = atan2(S6(4), C6(4));

	//< 矩阵调整 >//////////////////////////////////////////
	// maximum ik resolution num is 8：
	// ------------------------------------------
	// |    |    |    | q4      |  q5 | q6      | --- case1
	// |    | q3 | q2 |--------------------------
	// |    | +  |    | q4 + pi | -q5 | q6 + pi | --- case2
	// | q1 |---------|--------------------------
	// | 1  |    |    | q4      |  q5 | q6      | --- case3
	// |    | q3 | q2 |--------------------------
	// |    | -  |    | q4 + pi | -q5 | q6 + pi | --- case4
	// ------------------------------------------
	// |    |    |    | q4      |  q5 | q6      | --- case5
	// |    | q3 | q2 |--------------------------
	// |    | +  |    | q4 + pi | -q5 | q6 + pi | --- case6
	// | q1 |---------|--------------------------
	// | 2  |    |    | q4      |  q5 | q6      | --- case7
	// |    | q3 | q2 |--------------------------
	// |    | -  |    | q4 + pi | -q5 | q6 + pi | --- case8
	// ------------------------------------------
	////////////////////////////////////////////////////////

	//定义theta_t
	matrix theta_t(4, 6);

	//theta_t赋值
	for (int i = 1; i <= 4; i++)
	{
		theta_t(i, 1) = theta1(i);
		theta_t(i, 2) = theta2(i);
		theta_t(i, 3) = theta3(i);
		theta_t(i, 4) = theta4(i);
		theta_t(i, 5) = theta5(i);
		theta_t(i, 6) = theta6(i);
	}

	//定义变量
	Vector theta4_(4, VERT), theta5_(4, VERT), theta6_(4, VERT);

	//计算另外4组theta4、5、6的值
	for (int i = 1; i <= 4; i++)
	{
		//计算theta4_
		if (theta_t(i, 4) > 0)
		{
			theta4_(i) = theta_t(i, 4) - PI;
		}
		else
		{
			theta4_(i) = theta_t(i, 4) + PI;
		}

		//计算theta5_
		theta5_(i) = -1 * theta_t(i, 5);

		//计算theta6_
		if (theta_t(i, 6) > 0)
		{
			theta6_(i) = theta_t(i, 6) - PI;
		}
		else
		{
			theta6_(i) = theta_t(i, 6) + PI;
		}
	}

	//对theta的输出顺序进行拼接
	if (ML(1) >= 0 && ML(2) < 0)
	{
		//ROW1 = case1
		theta(1, 1) = theta_t(1, 1);
		theta(1, 2) = theta_t(1, 2);
		theta(1, 3) = theta_t(1, 3);
		theta(1, 4) = theta_t(1, 4);
		theta(1, 5) = theta_t(1, 5);
		theta(1, 6) = theta_t(1, 6);

		//ROW2 = case2
		theta(2, 1) = theta_t(1, 1);
		theta(2, 2) = theta_t(1, 2);
		theta(2, 3) = theta_t(1, 3);
		theta(2, 4) = theta4_(1);
		theta(2, 5) = theta5_(1);
		theta(2, 6) = theta6_(1);

		//ROW3 = case3
		theta(3, 1) = theta_t(3, 1);
		theta(3, 2) = theta_t(3, 2);
		theta(3, 3) = theta_t(3, 3);
		theta(3, 4) = theta_t(3, 4);
		theta(3, 5) = theta_t(3, 5);
		theta(3, 6) = theta_t(3, 6);

		//ROW4 = case4
		theta(4, 1) = theta_t(3, 1);
		theta(4, 2) = theta_t(3, 2);
		theta(4, 3) = theta_t(3, 3);
		theta(4, 4) = theta4_(3);
		theta(4, 5) = theta5_(3);
		theta(4, 6) = theta6_(3);

		//改变矩阵维数
		theta.ChangeDimension(4, 6);
	}
	else if (ML(2) >= 0 && ML(1) < 0)
	{
		//ROW1 = case5
		theta(1, 1) = theta_t(2, 1);
		theta(1, 2) = theta_t(2, 2);
		theta(1, 3) = theta_t(2, 3);
		theta(1, 4) = theta_t(2, 4);
		theta(1, 5) = theta_t(2, 5);
		theta(1, 6) = theta_t(2, 6);

		//ROW2 = case6
		theta(2, 1) = theta_t(2, 1);
		theta(2, 2) = theta_t(2, 2);
		theta(2, 3) = theta_t(2, 3);
		theta(2, 4) = theta4_(2);
		theta(2, 5) = theta5_(2);
		theta(2, 6) = theta6_(2);

		//ROW3 = case7
		theta(3, 1) = theta_t(4, 1);
		theta(3, 2) = theta_t(4, 2);
		theta(3, 3) = theta_t(4, 3);
		theta(3, 4) = theta_t(4, 4);
		theta(3, 5) = theta_t(4, 5);
		theta(3, 6) = theta_t(4, 6);

		//ROW4 = case8
		theta(4, 1) = theta_t(4, 1);
		theta(4, 2) = theta_t(4, 2);
		theta(4, 3) = theta_t(4, 3);
		theta(4, 4) = theta4_(4);
		theta(4, 5) = theta5_(4);
		theta(4, 6) = theta6_(4);

		//改变矩阵维数
		theta.ChangeDimension(4, 6);
	}
	else
	{
		//ROW1 = case1
		theta(1, 1) = theta_t(1, 1);
		theta(1, 2) = theta_t(1, 2);
		theta(1, 3) = theta_t(1, 3);
		theta(1, 4) = theta_t(1, 4);
		theta(1, 5) = theta_t(1, 5);
		theta(1, 6) = theta_t(1, 6);

		//ROW2 = case2
		theta(2, 1) = theta_t(1, 1);
		theta(2, 2) = theta_t(1, 2);
		theta(2, 3) = theta_t(1, 3);
		theta(2, 4) = theta4_(1);
		theta(2, 5) = theta5_(1);
		theta(2, 6) = theta6_(1);

		//ROW3 = case3
		theta(3, 1) = theta_t(3, 1);
		theta(3, 2) = theta_t(3, 2);
		theta(3, 3) = theta_t(3, 3);
		theta(3, 4) = theta_t(3, 4);
		theta(3, 5) = theta_t(3, 5);
		theta(3, 6) = theta_t(3, 6);

		//ROW4 = case4
		theta(4, 1) = theta_t(3, 1);
		theta(4, 2) = theta_t(3, 2);
		theta(4, 3) = theta_t(3, 3);
		theta(4, 4) = theta4_(3);
		theta(4, 5) = theta5_(3);
		theta(4, 6) = theta6_(3);

		//ROW5 = case5
		theta(5, 1) = theta_t(2, 1);
		theta(5, 2) = theta_t(2, 2);
		theta(5, 3) = theta_t(2, 3);
		theta(5, 4) = theta_t(2, 4);
		theta(5, 5) = theta_t(2, 5);
		theta(5, 6) = theta_t(2, 6);

		//ROW6 = case6
		theta(6, 1) = theta_t(2, 1);
		theta(6, 2) = theta_t(2, 2);
		theta(6, 3) = theta_t(2, 3);
		theta(6, 4) = theta4_(2);
		theta(6, 5) = theta5_(2);
		theta(6, 6) = theta6_(2);

		//ROW7 = case7
		theta(7, 1) = theta_t(4, 1);
		theta(7, 2) = theta_t(4, 2);
		theta(7, 3) = theta_t(4, 3);
		theta(7, 4) = theta_t(4, 4);
		theta(7, 5) = theta_t(4, 5);
		theta(7, 6) = theta_t(4, 6);

		//ROW8 = case8
		theta(8, 1) = theta_t(4, 1);
		theta(8, 2) = theta_t(4, 2);
		theta(8, 3) = theta_t(4, 3);
		theta(8, 4) = theta4_(4);
		theta(8, 5) = theta5_(4);
		theta(8, 6) = theta6_(4);
	}

	//返回结果矩阵
	return theta;
}

//Posi_Pose robot_phr::Ikine_Tcp2t6(const Posi_Pose &Tcp)
//{
//
//	Posi_Pose result;
//	Vector P76(0, 0, -DH.d7);
//	result.Pose = Tcp.Pose;
//	result.Posi = Tcp.Pose.Rot()*P76+Tcp.Posi;
//	return result;
//}

matrix robot_phr::Ikine_Tcp2t6(const Posi_Pose &Tcp)
{

	
	Vector P76(0, 0, -DH.d7);

	matrix Rot(Tcp.Pose.Rot());

	matrix result(Rot, Rot*P76+Tcp.Posi);

	return result;
}



Vector robot_phr::Ikine_cfg(const matrix &qIK8, int cfg)
{
	//matrix qIK8(Ikine_matrix(PPn));
	//寻找与q0的cfg相同的一组解

	if (cfg>=0&&cfg<=7)
	{
		CFG = cfg;
	}

	//Vector qIK_N1(Q_samecfg(qIK8, CFG));

	return Q_samecfg(qIK8, CFG);
}
double robot_phr::Ksew_select(const Ksew &Ksew, int Singular_flag)
{
	double K_select;
	switch (Singular_flag)
	{
	case 1:
	{
		K_select = Ksew.Ks;
		break;
	}
		
	case 2:
	{
		K_select = Ksew.Ke;
		break;
	}
	case 3:
	{
		K_select = Ksew.Kw;
		break;
	}
	return K_select;
	
	}

}


void robot_phr::Singularity_monitor()



{
	Singular_flag = 0;
	
	if (Ksew_n.Ks < Ksew_limit.Ks)
	{
		Singular_flag = 1;
	}
	
	if (Ksew_n.Ke < Ksew_limit.Ke)
	{		
		Singular_flag = 2;		
	}
	
	if (Ksew_n.Kw < Ksew_limit.Kw)
	{
		Singular_flag = 3;
	}

}



//int robot_phr::Singularity_jump(const int index,  Cartesian_traj &robot_traj)
//{
//	double K_last = Ksew_select(Ksew_L, Singular_flag);
//	double K_now = Ksew_select(Ksew_n, Singular_flag);;
//	double K_limit = Ksew_select(Ksew_limit, Singular_flag);
//	double K_slope = K_last - K_now;
//	double K_steps_temp = 2 * ceil(K_now / K_slope);
//	int K_adjuststeps = 0;
//	double K_steps = K_steps_temp + K_adjuststeps;
//	int j = index + K_steps + 1;
//	while (fabs(K_now)<K_limit)
//	{
//		K_steps = K_steps + K_adjuststeps;
//		j = index + K_steps + 1; //保险起见加1
//		double tt = j*robot_traj.t_interval;
//		robot_traj.realtime(tt);
//		matrix T_n = Ikine_Tcp2t6(robot_traj.PP_n);
//		Ikine_nearest(T_n,q_n);
//		K_now = Ksew_select(Ksew_n, Singular_flag);
//		q_singular_end = q_n;
//		qd_singular_end = qd_n;
//		double delta_K = K_limit - fabs(K_now);
//		K_adjuststeps = ceil(delta_K / abs(K_slope)); //计算补充步数 j=index+k_jumpsteps+k_adjuststeps;
//		if (j>robot_traj.Na)
//		{
//			j = robot_traj.Na;
//			break;
//		}
//	}
//	return j;
//
//}





matrix* robot_phr::fkine_num_d3(const Vector &q)
{
	double m1;
	double m2;
	double m3;
	double m4;
	double m5;
	double m6;
	double m7;
	double m8;
	double m9;
	double m10;
	double m11;
	double m12;
	double m13;
	double m14;
	double m15;
	double m16;
	double m17;
	double m18;
	double m19;
	double m20;
	double m21;
	double m22;
	double m23;
	double m24;
	double m25;

	double a1, a2, a3, d1, d3, d4;
	a1 = DH.a1;
	a2 = DH.a2;
	a3 = DH.a3;
	d1 = DH.d1;
	d3 = DH.d3;
	d4 = DH.d4;

	m1 = cos(q(1));
	m2 = sin(q(1));
	m3 = cos(q(2));
	m4 = sin(q(2));
	m5 = cos(q(3));
	m6 = sin(q(3));
	m7 = cos(q(4));
	m8 = sin(q(4));
	m9 = cos(q(5));
	m10 = sin(q(5));
	m11 = sin(q(6));
	m12 = cos(q(6));
	m13 = sin(q(2) + q(3));
	m14 = m1*m3*m5 - m1*m4*m6;
	m15 = m2*m7 - m8*m14;
	m16 = m2*m8 + m7*m14;
	m17 = m1*m3*m6 + m1*m5*m4;
	m18 = m9*m16 - m10*m17;
	m19 = m2*m3*m5 - m2*m4*m6;
	m20 = m1*m7 + m8*m19;
	m21 = m1*m8 - m7*m19;
	m22 = m2*m3*m6 + m2*m5*m4;
	m23 = m9*m21 + m10*m22;
	m24 = m3*m5 - m4*m6;
	m25 = m10*m24 + m7*m9*m13;

	mat_t60(1, 1) = m15*m11 + m12*m18;
	mat_t60(2, 1) = -m11*m20 - m12*m23;
	mat_t60(3, 1) = -m12*m25 + m8*m11*m13;
	mat_t60(4, 1) = 0;
	mat_t60(1, 2) = m12*m15 - m11*m18;
	mat_t60(2, 2) = -m12*m20 + m11*m23;
	mat_t60(3, 2) = m11*m25 + m8*m12*m13;
	mat_t60(4, 2) = 0;
	mat_t60(1, 3) = -m9*m17 - m16*m10;
	mat_t60(2, 3) = -m9*m22 + m10*m21;
	mat_t60(3, 3) = -m9*m24 + m7*m10*m13;
	mat_t60(4, 3) = 0;
	mat_t60(1, 4) = m1*(a2*m3 + a3*m3*m5 - d4*m3*m6 - d4*m5*m4 - a3*m4*m6 + a1) - d3*m2;
	mat_t60(2, 4) = m2*(a2*m3 + a3*m3*m5 - d4*m3*m6 - d4*m5*m4 - a3*m4*m6 + a1) + d3*m1;
	mat_t60(3, 4) = d4*m4*m6 - d4*m3*m5 - a3*m3*m6 - a3*m5*m4 - a2*m4 + d1;
	mat_t60(4, 4) = 1;

	return &mat_t60;
}

matrix* robot_phr::Fkine_T6toTcp(matrix &T6)
{
	double d7;
	matrix T67(4, 4);
	d7 = DH.d7;
	for (int i = 1; i < 5; i++)
	{
		for (int j = 1; j < 5; j++)
		{
			if (i == j)
			{
				T67(i, j) = 1;
			}
			else
			{
				T67(i, j) = 0;
			}
		}
	}
	T67(3, 4) = d7;
	mat_Tcp = T6 * T67;
	return &mat_Tcp;
}

matrix robot_phr::Qn2Qn_efort( matrix &qn)
{
	matrix mx(qn);
	for (unsigned int m = 1; m <= mx.ROW; m++)
	{
		mx(m, 2) = qn(m, 2) + PI / 2;
	}
	for (unsigned int i = 1; i <= mx.ROW; i++)
	{
		mx(i, 3) = -qn(i, 3);
	}
	for (unsigned int j = 1; j <= mx.ROW; j++)
	{
		mx(j, 5) = -qn(j, 5);
	}
	return mx;
}

matrix robot_phr::Qn_e2Qmotor_efort( matrix &qn_e)
{
	matrix mx(qn_e);
	Vector vx(qn_e.ROW, VERT);
	const double i6 = 51;

	for (unsigned int i = 1; i <= mx.ROW; i++)
	{
		for (unsigned int j = 1; j <= mx.COL; j++)
		{
			mx(i, j) = rad2deg(qn_e(i, j));
		}
	}

	for (unsigned int k = 2; k <= mx.ROW; k++)
	{
		vx(k) = mx(k, 5) - mx(k - 1, 5);
	}
	vx(1) = 0;

	for (unsigned int m = 1; m <= mx.ROW; m++)
	{
		mx(m, 6) = mx(m, 6) - vx(m) / i6;
	}

	return mx;
}

Vector robot_phr::Qn_efort2Qn(const Vector &qn_e)
{
	Vector vx(qn_e);
	vx(2) = qn_e(2) - PI / 2;
	vx(3) = -qn_e(3);
	vx(5) = -qn_e(5);
	return vx;
}

matrix robot_phr::jacob_num_sub(const Vector &q)
{
	matrix mx(6, 6);

	double a1, a2, a3, d1, d3, d4;
	a1 = DH.a1;
	a2 = DH.a2;
	a3 = DH.a3;
	d1 = DH.d1;
	d3 = DH.d3;
	d4 = DH.d4;

	//J1
	mx(1, 1) = -sin(q(1))*(a1 + a3*cos(q(2) + q(3)) - d4*sin(q(2) + q(3)) + a2*cos(q(2)));
	mx(1, 2) = cos(q(1))*(a1 + a3*cos(q(2) + q(3)) - d4*sin(q(2) + q(3)) + a2*cos(q(2)));
	mx(1, 3) = 0;
	mx(1, 4) = 0;
	mx(1, 5) = 0;
	mx(1, 6) = 1;
	//J2
	mx(2, 1) = -cos(q(1))*(d4*cos(q(2) + q(3)) + a3*sin(q(2) + q(3)) + a2*sin(q(2)));
	mx(2, 2) = -sin(q(1))*(d4*cos(q(2) + q(3)) + a3*sin(q(2) + q(3)) + a2*sin(q(2)));
	mx(2, 3) = d4*sin(q(2) + q(3)) - a3*cos(q(2) + q(3)) - a2*cos(q(2));
	mx(2, 4) = -sin(q(1));
	mx(2, 5) = cos(q(1));
	mx(2, 6) = 0;
	//J3
	mx(3, 1) = -cos(q(1))*(d4*cos(q(2) + q(3)) + a3*sin(q(2) + q(3)));
	mx(3, 2) = -sin(q(1))*(d4*cos(q(2) + q(3)) + a3*sin(q(2) + q(3)));
	mx(3, 3) = d4*sin(q(2) + q(3)) - a3*cos(q(2) + q(3));
	mx(3, 4) = -sin(q(1));
	mx(3, 5) = cos(q(1));
	mx(3, 6) = 0;
	//J4
	mx(4, 1) = 0;
	mx(4, 2) = 0;
	mx(4, 3) = 0;
	mx(4, 4) = -sin(q(2) + q(3))*cos(q(1));
	mx(4, 5) = -sin(q(2) + q(3))*sin(q(1));
	mx(4, 6) = -cos(q(2) + q(3));
	//J5
	mx(5, 1) = 0;
	mx(5, 2) = 0;
	mx(5, 3) = 0;
	mx(5, 4) = -cos(q(4))*sin(q(1)) + sin(q(4))*cos(q(2) + q(3))*cos(q(1));
	mx(5, 5) = cos(q(1))*cos(q(4)) + sin(q(4))*cos(q(2) + q(3))*sin(q(1));
	mx(5, 6) = -sin(q(2) + q(3))*sin(q(4));
	//J6
	mx(6, 1) = 0;
	mx(6, 2) = 0;
	mx(6, 3) = 0;
	mx(6, 4) = -sin(q(5))*(sin(q(1))*sin(q(4)) + cos(q(4))*cos(q(2) + q(3))*cos(q(1))) - cos(q(5))*sin(q(2) + q(3))*cos(q(1));
	mx(6, 5) = sin(q(5))*(cos(q(1))*sin(q(4)) - cos(q(4))*cos(q(2) + q(3))*sin(q(1))) - cos(q(5))*sin(q(2) + q(3))*sin(q(1));
	mx(6, 6) = sin(q(2) + q(3))*cos(q(4))*sin(q(5)) - cos(q(2) + q(3))*cos(q(5));

	return mx;
}

matrix robot_phr::invJ_num_sub_11(const Vector &q)
{
	matrix mx(3, 3);
	double deltaj11;
	double a1, a2, a3, d1, d3, d4;
	a1 = DH.a1;
	a2 = DH.a2;
	a3 = DH.a3;
	d1 = DH.d1;
	d3 = DH.d3;
	d4 = DH.d4;

	deltaj11 = a2*(d4*cos(q(3)) + a3*sin(q(3)))*(a1 + a3*cos(q(2) + q(3)) - d4*sin(q(2) + q(3)) + a2*cos(q(2)));
	mx(1, 1) = -a2*sin(q(1))*(d4*cos(q(3)) + a3*sin(q(3)));
	mx(1, 2) = a2*cos(q(1))*(d4*cos(q(3)) + a3*sin(q(3)));
	mx(1, 3) = 0;
	mx(2, 1) = cos(q(1))*(a3*cos(q(2) + q(3)) - d4*sin(q(2) + q(3)))*(a3*cos(q(2) + q(3)) - d4*sin(q(2) + q(3)) + a2*cos(q(2)) + a1);
	mx(2, 2) = sin(q(1))*(a3*cos(q(2) + q(3)) - d4*sin(q(2) + q(3)))*(a3*cos(q(2) + q(3)) - d4*sin(q(2) + q(3)) + a2*cos(q(2)) + a1);
	mx(2, 3) = -(d4*cos(q(2) + q(3)) + a3*sin(q(2) + q(3)))*(a3*cos(q(2) + q(3)) - d4*sin(q(2) + q(3)) + a2*cos(q(2)) + a1);
	mx(3, 1) = -cos(q(1))*(a3*cos(q(2) + q(3)) - d4*sin(q(2) + q(3)) + a2*cos(q(2)))*(a3*cos(q(2) + q(3)) - d4*sin(q(2) + q(3)) + a2*cos(q(2)) + a1);
	mx(3, 2) = -sin(q(1))*(a3*cos(q(2) + q(3)) - d4*sin(q(2) + q(3)) + a2*cos(q(2)))*(a3*cos(q(2) + q(3)) - d4*sin(q(2) + q(3)) + a2*cos(q(2)) + a1);
	mx(3, 3) = (d4*cos(q(2) + q(3)) + a3*sin(q(2) + q(3)) + a2*sin(q(2)))*(a3*cos(q(2) + q(3)) - d4*sin(q(2) + q(3)) + a2*cos(q(2)) + a1);

	for (unsigned int i = 1; i <= mx.ROW; i++)
	{
		for (unsigned int j = 1; j <= mx.COL; j++)
		{
			mx(i, j) = mx(i, j) / deltaj11;
		}
	}
	return mx;
}

matrix robot_phr::invJ_num_sub_22(const Vector &q)
{
	matrix mx(3, 3);
	double deltaj22;
	double a1, a2, a3, d1, d3, d4;
	a1 = DH.a1;
	a2 = DH.a2;
	a3 = DH.a3;
	d1 = DH.d1;
	d3 = DH.d3;
	d4 = DH.d4;

	deltaj22 = -sin(q(5));
	mx(1, 1) = sin(q(2) + q(3))*cos(q(1))*sin(q(5)) - cos(q(5))*sin(q(1))*sin(q(4)) - cos(q(2) + q(3))*cos(q(1))*cos(q(4))*cos(q(5));
	mx(1, 2) = sin(q(2) + q(3))*sin(q(1))*sin(q(5)) + cos(q(1))*cos(q(5))*sin(q(4)) - cos(q(2) + q(3))*cos(q(4))*cos(q(5))*sin(q(1));
	mx(1, 3) = cos(q(2) + q(3))*sin(q(5)) + sin(q(2) + q(3))*cos(q(4))*cos(q(5));
	mx(2, 1) = sin(q(5))*(cos(q(4))*sin(q(1)) - cos(q(2) + q(3))*cos(q(1))*sin(q(4)));
	mx(2, 2) = -sin(q(5))*(cos(q(1))*cos(q(4)) + cos(q(2) + q(3))*sin(q(1))*sin(q(4)));
	mx(2, 3) = sin(q(2) + q(3))*sin(q(4))*sin(q(5));
	mx(3, 1) = sin(q(1))*sin(q(4)) + cos(q(2) + q(3))*cos(q(1))*cos(q(4));
	mx(3, 2) = cos(q(2) + q(3))*cos(q(4))*sin(q(1)) - cos(q(1))*sin(q(4));
	mx(3, 3) = -sin(q(2) + q(3))*cos(q(4));

	for (unsigned int i = 1; i <= mx.ROW; i++)
	{
		for (unsigned int j = 1; j <= mx.COL; j++)
		{
			mx(i, j) = mx(i, j) / deltaj22;
		}
	}
	return mx;
}

Vector robot_phr::JJdot_qdot_num(const Vector &q, const Vector &qdot)
{
	Vector Jdot_qdot(6, VERT);
	double a1, a2, a3, d1, d3, d4;
	a1 = DH.a1;
	a2 = DH.a2;
	a3 = DH.a3;
	d1 = DH.d1;
	d3 = DH.d3;
	d4 = DH.d4;

	double q1, q2, q3, q4, q5, q6;
	double qdot1, qdot2, qdot3, qdot4, qdot5, qdot6;

	q1 = q(1);
	q2 = q(2);
	q3 = q(3);
	q4 = q(4);
	q5 = q(5);
	q6 = q(6);

	qdot1 = qdot(1);
	qdot2 = qdot(2);
	qdot3 = qdot(3);
	qdot4 = qdot(4);
	qdot5 = qdot(5);
	qdot6 = qdot(6);

	Jdot_qdot(1) = qdot1 * 2 * a2*qdot2*sin(q1)*sin(q2) - a2*cos(q1)*cos(q2)*(qdot1 * qdot1 + qdot2 * qdot2) + 2 * qdot1*sin(q1)*(d4*cos(q2 + q3) + a3*sin(q2 + q3))*(qdot2 + qdot3) - cos(q1)*(a3*cos(q2 + q3) - d4*sin(q2 + q3))*(qdot1 * qdot1 + (qdot2 + qdot3)*(qdot2 + qdot3)) - a1*qdot1 * qdot1 * cos(q1);


	return Jdot_qdot;
}

Vector robot_phr::qd2v(const Vector &qd, const Vector &q)
{
	Vector vxqd(6, VERT);
	Vector vxq(6, VERT);
	matrix mx(6, 6);
	Vector vx(6, VERT);
	if (qd.TYPE == HORZ)
	{
		vxqd = qd.Transposition();
	}
	else
	{
		vxqd = qd;
	}
	if (q.TYPE == HORZ)
	{
		vxq = q.Transposition();
	}
	else
	{
		vxq = q;
	}
	mx = jacob_num_sub(vxq);
	vx = mx * vxqd;
	return vx;
}

Vector robot_phr::V2qd(const Vector &v, const Vector &q)
{
	Vector vxv(6, VERT);
	Vector vxq(6, VERT);
	Vector vxqd(6, VERT);
	matrix jnj11(3, 3), invj22(3, 3);
	matrix mxJ21(6, 6);
	matrix mxJ21part(3, 3);
	Vector vxtemp(3, VERT);
	Vector qdv(3, VERT), qdw(3, VERT);
	if (v.TYPE == HORZ)
	{
		vxv = v.Transposition();
	}
	else
	{
		vxv = v;
	}
	if (q.TYPE == HORZ)
	{
		vxq = q.Transposition();
	}
	else
	{
		vxq = q;
	}

	if (v(1) == 0 && v(2) == 0 && v(3) == 0 && v(4) == 0 && v(5) == 0 && v(6) == 0)
	{
		for (unsigned int i = 1; i <= 6; i++)
		{
			vxqd(i) = 0;
		}
	}
	else
	{
		jnj11 = invJ_num_sub_11(vxq);
		invj22 = invJ_num_sub_22(vxq);
		mxJ21 = jacob_num_sub(vxq);
		mxJ21part = mxJ21.SubMatrix(4, 1, 6, 3);
		vxtemp = vxv.SubVector(1, 3);
		qdv = jnj11 * vxtemp;
		vxtemp = vxv.SubVector(4, 6);
		vxtemp = vxtemp - mxJ21part * qdv;
		qdw = invj22 * vxtemp;
		for (unsigned int j = 1; j <= 6; j++)
		{
			if (j <= 3)
			{
				vxqd(j) = qdv(j);
			}
			else
			{
				vxqd(j) = qdw(j - 3);
			}
		}
	}
	return vxqd;
}