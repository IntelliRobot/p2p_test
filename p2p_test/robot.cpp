#include "stdafx.h"
#include <math.h>
#include <string.h>
#include "robot.h"
#include "global.h"

//Q1Ϊ��Ԫ��������ʾ����ת���󣬷���ֵΪ��Ԫ�ر�ʾ����ת����
vector robot::quater_inv(vector Q1)
{
	//���巵�ؽ��
	vector result(4, VERT);
	
	//ǰһλ����
	result(1) = Q1(1);

	//����λȡ��
	result(2) = -1 * Q1(2);
	result(3) = -1 * Q1(3);
	result(4) = -1 * Q1(4);

	//���ؽ��
	return result;
}

HALFCOS_COEF_RESULT robot::halfcos_coef(double s0, double sf, double t, double v0, double vf, double vc)
{
	//�����������
	unsigned int margin = 3;
	
	if (v0 != 0) { margin += 1; }
	if (vf != 0) { margin += 1; }
	if (vc != 0) { margin += 1; }

	//���巵�ؽ������ʼ��
	HALFCOS_COEF_RESULT result;

	double tf = t;

	//����ƽ���ٶ�ϵ����ϵ��Խ��ƽ���ٶ�ԽС��ģʽ1��
	double K = 2;

	//�������о���
	double s = sf - s0;

	//�����ٶ�
	double amax = 0;

	//����ƽ���ٶ�
	if (margin == 5)
	{
		if (vc == 0) { vc = 2 * K / (2 * K - 1) * s / tf; }
		if (vc > 1000) 
		{ 
			result.ReturnMessage.SetMessage("time is too short.");
			return result; 
		}

		//���������ٶ�
		amax = (pi*((fabs(v0 - vc)*(v0 - vc)) / 2 - (fabs(vc - vf)*(vc - vf)) / 2)) / (2 * s - 2 * tf * vc);
		if (amax > 500) 
		{ 
			result.ReturnMessage.SetMessage("time is too short.");
			return result; 
		}
	}
	if (margin == 6) { amax = 500; }

	double dvc0 = vc - v0;//�׶ΰ벨�ٶȱ仯��ֵ
	double dvcf = vf - vc;//ĩ�ΰ벨�ٶȱ仯��ֵ
	double dtc0 = pi*fabs(dvc0) / (2 * amax);//�׶μ��ٶ��õ�ʱ�䣬�����ڣ�pi*abs(dvc0) / amax Ϊ�׶�����
	double dtcf = pi*fabs(dvcf) / (2 * amax);//ĩ�μ��ٶ��õ�ʱ�䣬�����ڣ�pi*abs(dvcf) / amax Ϊĩ������
	double D1 = (vc + v0) / 2;//���Һ���ƫ��ֵ
	double D2 = (vc + vf) / 2;//���Һ���ƫ��ֵ

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

//��̬���β�ֵϵ������
//Q0��㵥λ��Ԫ��-����
//Qf�յ㵥λ��Ԫ��-����
QUATER_CUBIC_COEF_RESULT robot::quater_cubic_coef(vector Q0, vector Qf, double W0, double Wf, double tf)
{
	//�������
	QUATER_CUBIC_COEF_RESULT result;
	
	//����Q0������
	vector Q0_V(Q0);
	Q0_V.TYPE = VERT;
	

	//����Qf������
	vector Qf_V(Qf);
	Qf_V.TYPE = VERT;

	//��ĩ��̬�ƹ̶���ת���ĽǶȵ�����
	double cosTheta = dot(Q0_V, Qf_V) / (Q0_V.Norm() * Qf_V.Norm());

	//ѡ���Ż�
	if (cosTheta < 0)
	{
		Q0_V = -1 * Q0_V;
		cosTheta = -1 * cosTheta;
	}

	//��������
	double dalpha = 2 * acos(cosTheta);
	double q0 = 0;
	double qf = dalpha;

	//���ر�����ֵ
	result.cubic_A = (-2 * (qf - q0) + (Wf - W0) * tf) / (tf * tf * tf);
	result.cubic_B = (3 * (qf - q0) - 2 * W0*tf - Wf*tf) / (tf*tf);
	result.cubic_C = W0;
	result.cubic_D = q0;

	result.dalpha = dalpha;

	//�޴���ָʾ
	result.ReturnMessage.bNoError = true;
	
	//���ؽ��
	return result;
}

//����ξ������ת����ת������Ԫ������
//T:��������ξ���Ҳ��������ת����
//Last Modified:2018-12-20
vector robot::quater_r2q(matrix T)
{
	//��ȡ��ת����
	matrix r = T.SubMatrix(1, 1, 3, 3);

	//����qs kx xy kz
	double qs = sqrt(r.Trace() + 1) / 2.0;
	double kx = r(3, 2) - r(2, 3);  //Oz - Ay
	double ky = r(1, 3) - r(3, 1);  //Ax - Nz
	double kz = r(2, 1) - r(1, 2);  //Ny - Ox

	//�������
	int sign_kx = kx >= 0 ? 1 : -1;
	int sign_ky = ky >= 0 ? 1 : -1;
	int sign_kz = kz >= 0 ? 1 : -1;

	//����kx1 ky1 kz1
	//����һ����ת����kx1��ky1��kz1���Ǵ��ڵ���0����
	double kx1 = r(1, 1) - r(2, 2) - r(3, 3) + 1;
	double ky1 = r(2, 2) - r(1, 1) - r(3, 3) + 1;
	double kz1 = r(3, 3) - r(1, 1) - r(2, 2) + 1;

	//����result
	vector result(4, VERT);

	//����result
	result(1) = qs;
	result(2) = 0.5 * sign_kx * sqrt(kx1);
	result(3) = 0.5 * sign_ky * sqrt(ky1);
	result(4) = 0.5 * sign_kz * sqrt(kz1);

	//���ؽ��
	return result;
}

//������Ԫ�������Ĳ��
vector robot::quater_cross(vector Q1, vector Q2)
{
	//ת����������
	vector Q1_V(Q1);
	Q1_V.TYPE = VERT;

	//ת����������
	vector Q2_V(Q2);
	Q2_V.TYPE = VERT;

	//����Ԫ�������ֳ�������
	double s1_q1 = Q1_V(1);
	double s1_q2 = Q2_V(1);

	vector svec_q1(Q1_V.SubVector(2, 4));
	vector svec_q2(Q2_V.SubVector(2, 4));

	//�ֱ����
	double s1_qq = s1_q1*s1_q2 - dot(svec_q1, svec_q2);
	vector svec_qq(s1_q1*svec_q2+s1_q2*svec_q1+cross(svec_q1,svec_q2));

	//���巵�ؽ��
	vector result(4, VERT);

	//������ֵ
	result(1) = s1_qq;
	memcpy(&result.pVector[1], svec_qq.pVector, svec_qq.COUNT * sizeof(double));

	//���ؽ��
	return result;
}

//ֱ�߹켣�滮�������㣬��Ԫ����
unsigned int robot::line_traj_para(matrix T1, matrix T2, vector TVc, double t_interval, bool V_type)
{
	//��ȡ�켣����ʱ���ƽ���ٶ�
	double Tz = TVc(1);
	double Vc = TVc(2);

	//��֤
	if (Vc < 0) { throw("Vc�����Ǹ���"); }

	//��ȡλ������
	vector p0_start = T1.SubMatrix(1, 4, 3, 4).AsVector(); //���λ������
	vector pf_end = T2.SubMatrix(1, 4, 3, 4).AsVector();   //�յ�λ������

	//��ȡ��ת����
	matrix Rq0(T1.SubMatrix(1, 1, 3, 3)); //�����ת����
	matrix Rqf(T2.SubMatrix(1, 1, 3, 3)); //�ص���ת����

	//��ת����ת������Ԫ�ر�ʾ
	vector Q0_start(quater_r2q(Rq0));
	vector Qf_end(quater_r2q(Rqf));

	//��ֱ�߳���
	double p2pL = (pf_end - p0_start).Norm();

	//ֱ�ߵĵ�λʸ��
	vector uvec((pf_end-p0_start)/p2pL);

	//��ȡת�ᵥλʸ��
	vector QvecTemp(quater_cross(quater_inv(Q0_start), Qf_end));
	vector QvecSub(QvecTemp.SubVector(2, 4));

	vector Qvec(QvecSub / QvecSub.Norm());

	//����벨����ϵ��
	HALFCOS_COEF_RESULT HalfCosCoefResult;
	if (V_type)
	{
		HalfCosCoefResult = halfcos_coef(0, p2pL, Tz, 0, 0, Vc);
	}
	else
	{
		HalfCosCoefResult = halfcos_coef(0, p2pL, Tz, 0, 0);
	}

	//��ȡ��ֵ����
	unsigned int Na = (unsigned int)(HalfCosCoefResult.hal_tf / t_interval);

	//��̬�����β�ֵ
	QUATER_CUBIC_COEF_RESULT QuaterCubicCoefResult=quater_cubic_coef(Q0_start, Qf_end, 0.0, 0.0, HalfCosCoefResult.hal_tf);

	//׼��result
	LINE_TRAJ_PARA_RESULT LineTrajParaResult;

	return Na;
}
