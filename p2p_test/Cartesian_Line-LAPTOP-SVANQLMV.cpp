#include "stdafx.h"
#include "Cartesian_line.h"


Cartesian_Line::Cartesian_Line(Posi_Pose PP0, Posi_Pose PPf, enum P2Pmethod cht, enum TIMEorSPEED method_,
	double tf_speedp_, double acce_p, double t_interval_)
{
	PP_0 = PP0;
	PP_f = PPf;
	method = method_;
	tf_speedp = tf_speedp_;
	t_interval = t_interval_;
	acce_percent = acce_p;
		
	Uvec = PPf.Posi - PP0.Posi;
	traj_length = Uvec.Norm();
	Uvec = Uvec / traj_length;

	Vdc = acce_percent*Vd_max;
	Wdc = acce_percent*Wd_max;

	vector<Quaternion> Qsn_in = { PP0.Pose,PPf.Pose };
	
	Vector Tsn_in(tf_speedp);


	Posture_Interp Posture_line( Qsn_in, Tsn_in, slerp, ZeroVector1, ZeroVector1);

	Dalpha = 2*Posture_line.Slerp_theta;

	V_traj_length_Dalpha=(traj_length, Dalpha);
	V_Vdc_Wdc=(Vdc, Wdc);

	double V_0_abs = 0;//��ʼ���ٶȣ�����
	double V_f_abs = 0;

	if (0 < tf_speedp  && tf_speedp <= 1 && method == speed)
	{
		speed_percent = tf_speedp_;
		Vc = speed_percent *V_max;
		Wc = speed_percent *W_max;
		V_Vc_Wc = (Vc, Wc);
	}
	
	if (tf_speedp > 0 && method == time)
	{
		tf = tf_speedp;
		V_Vc_Wc = (V_max, W_max);
	}

	
	switch (cht)
	{
	case cubic:
		Cubic_coef = { ZeroVector2,ZeroVector2,ZeroVector2,ZeroVector2 };
		cubic_coef();
		break;
	case halfcos:
		Half_coef = { ZeroVector2,ZeroVector2,ZeroVector2,ZeroVector2,ZeroVector2,ZeroVector2,ZeroVector2,ZeroVector2,
			ZeroVector2,ZeroVector2,ZeroVector2,ZeroVector2,ZeroVector2,ZeroVector2 };
		halfcos_coef();
		break;
	case trapez:
		Trapez_coefs = { ZeroVector2,ZeroVector2,ZeroVector2,ZeroVector2,ZeroVector2,ZeroVector2 };
		trapez_coefs();
		break;
	case linear:
		linear_coef();
		break;
	default:
		break;
	}
}



void Cartesian_Line::cubic_coef()
{

	double tf_min_limit =Cubic_max_tf(V_traj_length_Dalpha, ZeroVector2, ZeroVector2, V_Vc_Wc);

	if (method == time)
	{
		if (tf < tf_min_limit)
		{
			tf = tf_min_limit;
		}
	}

	else
	{
		tf = tf_min_limit;
	}

	if (t_interval > 0)
	{
		Na = floor(tf / t_interval) + 1;
	}

	//�����ٶȺͽ��ٶ�ƴ�ӳ������ķ���


	if (speed_percent > 0)
	{
		Cubic_spline_coef(ZeroVector2, V_traj_length_Dalpha, ZeroVector2, ZeroVector2, tf, Cubic_coef);

	}
		
}




void Cartesian_Line::linear_coef()
{

	double tf_min_limit = Vdevide(V_traj_length_Dalpha, V_Vc_Wc).Max();
		

	if (method == time)
	{
		if (tf < tf_min_limit)
		{
			tf = tf_min_limit;
		}
	}

	else
	{
		tf = tf_min_limit;
	}

	if (t_interval > 0)
	{
		Na = floor(tf / t_interval) + 1;
	}

}


void Cartesian_Line::halfcos_coef()
{

	if (speed_percent > 0)
	{
		Halfcos_coef(ZeroVector2, V_traj_length_Dalpha, ZeroVector2, ZeroVector2, tf, method, V_Vc_Wc, V_Vdc_Wdc, Half_coef);
	}
	
	if (t_interval > 0)
	{
		Na = floor(tf / t_interval) + 1;
	}

}

void Cartesian_Line::trapez_coefs()
{

	if (speed_percent > 0)
	{
		Trapez_coef(ZeroVector2, V_traj_length_Dalpha, ZeroVector2, ZeroVector2, method, tf, V_Vc_Wc, V_Vdc_Wdc, Trapez_coefs);

	}

	if (t_interval > 0)
	{
		Na = floor(tf / t_interval) + 1;
	}

}



void Cartesian_Line::realtime_PVA()
{
	PP_n.Posi = PP_0.Posi + Uvec*st_2(1);
	double t_p = st_2(2) / Dalpha;
	Posture_line.Slerp(t_p);
	Qt = Posture_line.Qt;
	PP_n.Pose = Qt;
	Wt = vt_2(2)*Posture_line.Qvec;
	Wdt = at_2(2)*Posture_line.Qvec;
	Wt = Qt*Wt;
	Wdt = Qt*Wdt;
   //Wt = Posture_line.Wt;
	//Wdt = Posture_line.Wtdot;
	Vt = vt_2(1)*Uvec;
	At = at_2(1)*Uvec;
	V_n = (Vt, Wt);
	A_n = (At, Wdt);

}

void Cartesian_Line::realtime(double t_now)
{
	t_n = t_now;//��������
			  //if (t >= 0 && t <= 1 && method == speed)

			  //{
			  //	t = t*tf;

			  //}

	if (t_n <= tf)
	{
		switch (cht)
		{
		case cubic:
			Cubic_spline_rt(st_2, vt_2, at_2, Cubic_coef, t_n);
			realtime_PVA();
			break;
		case halfcos:
			Halfcos_rt(st_2, vt_2, at_2, Half_coef, t_n);
			realtime_PVA();
			break;
		case trapez:
			Trapez_rt(st_2, vt_2, at_2, Trapez_coefs, t_n);
			realtime_PVA();
			//Trapez_rt(qt, qdt, qddt, qd0, qd, trapez_coef, t_n);
			break;
		case linear:
			double tpercent = t_n / tf;
			st_2 = tpercent*V_traj_length_Dalpha;
			vt_2 = V_Vc_Wc;
			at_2 = ZeroVector2;
			realtime_PVA();
		default:
			break;
		}
		

	}

}