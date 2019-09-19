#pragma once
#include "stdafx.h"
#include "Cartesian_line.h"


Cartesian_Line::Cartesian_Line(Posi_Pose PP0, Posi_Pose PPf, enum P2Pmethod cht_, enum TIMEorSPEED method_,
	double tf_speedp_, double acce_p, double t_interval_)
{
	PP_0 = PP0;
	PP_f = PPf;
	cht = cht_;
	traj_type = line;
	method = method_;
	tf_speedp = tf_speedp_;
	t_interval = t_interval_;
	acce_percent = acce_p;
		
	Uvec = PP_f.Posi - PP_0.Posi;
	traj_length = Uvec.Norm();
	if (traj_length<Epsilon)
	{
		throw("除数不能为0");
	}
	Uvec = Uvec / traj_length;

	Vdmax = acce_percent*Vd_max;
	Wdmax = acce_percent*Wd_max;

	V_n = ZeroVector6;
	A_n = ZeroVector6;


	//Vector Tsn_in(tf_speedp);


	Posture_line= Posture_Interp(PP_0.Pose, PP_f.Pose, slerp, t_interval);

	Dalpha = 2*Posture_line.Slerp_theta;

	V_traj_length_Dalpha=Vector(traj_length, Dalpha);

	V_Vdmax_Wdmax = Vector(Vdmax, Wdmax);
	V_Vmax_Wmax = Vector(V_max, W_max);

	double V_mintf;
	double W_mintf;

	if (cht==halfcos)
	{
		V_mintf = sqrt(2 * Vdmax*traj_length / PI);// 两端速度为零，取此速度时，tf最小
		W_mintf = sqrt(2 * Wdmax*Dalpha / PI);//
		V_V_W_mintf = Vector(V_mintf, W_mintf);

	}

	if (cht == trapez)
	{
		V_mintf = sqrt( Vdmax*traj_length );// 两端速度为零，取此速度时，tf最小
		W_mintf = sqrt( Wdmax*Dalpha );//
		V_V_W_mintf = Vector(V_mintf, W_mintf);
	}


	

	double V_0_abs = 0;//初始线速度，标量
	double V_f_abs = 0;


	if (( tf_speedp>=0)  && (tf_speedp <= 1) && (method == speed))
	{
		speed_percent = tf_speedp_;
		Vc = speed_percent *V_max;
		Wc = speed_percent *W_max;
		V_Vc_Wc = Vector(Vc, Wc);
		
		if ((cht == trapez) || (cht == halfcos))
		{
			Vector max_speed_p = Vdevide(V_Vc_Wc, V_Vmax_Wmax );

			double min_speed_p = max_speed_p.Max();

			if (speed_percent>min_speed_p)
			{
				printf("%f \n", min_speed_p);
				printf("速度百分比需小于上述值");
				throw("速度百分比需小于上述值");

			}
		}
	}
	
	if ((tf_speedp >=0 )&& (method == time))
	{
		tf = tf_speedp;
		//V_Vc_Wc = Vector(V_max, W_max);
	}

	if ((tf_speedp < 0) ||(tf_speedp >1) && (method == speed))
	{
		throw("速度百分比需在0~1");
	}

	
	switch (cht)
	{
	case cubic:
		Cubic_coef = { ZeroVector2,ZeroVector2,ZeroVector2,ZeroVector2 };
		cubic_coef();
		break;
	case halfcos:
		Half_coef = { ZeroVector2,ZeroVector2,ZeroVector2,ZeroVector2,ZeroVector2,ZeroVector2,ZeroVector2,ZeroVector2,
			ZeroVector2,ZeroVector2,ZeroVector2,ZeroVector2,ZeroVector2,ZeroVector2,ZeroVector2,ZeroVector2,0};
		halfcos_coef();
		break;
	case trapez:
		Trapez_coefs = { ZeroVector2,ZeroVector2,ZeroVector2,ZeroVector2,ZeroVector2,ZeroVector2,ZeroVector2,ZeroVector2 ,ZeroVector2,ZeroVector2,0};
		trapez_coefs();
		break;
	case linear:
		linear_coef();
		break;
	default:
		break;
	}
}

Cartesian_Line::~Cartesian_Line()
{
}

void Cartesian_Line::cubic_coef()
{

	 tf_min_limit =Cubic_max_tf(V_traj_length_Dalpha, ZeroVector2, ZeroVector2, V_Vmax_Wmax, V_Vdmax_Wdmax);

	if (method == time)
	{
		 
		if (tf < tf_min_limit)
		{
			tf = tf_min_limit;
		}
	}

	else
	{
		tf = Cubic_max_tf(V_traj_length_Dalpha, ZeroVector2, ZeroVector2, V_Vc_Wc, V_Vdmax_Wdmax);
	}

	if (t_interval > 0)
	{
		Na = floor(tf / t_interval) ;

		tf = Na*t_interval;
	}

	//将线速度和角速度拼接成向量的方法


	if (speed_percent > 0)
	{
		Cubic_spline_coef(ZeroVector2, V_traj_length_Dalpha, ZeroVector2, ZeroVector2, tf, Cubic_coef);

	}
		
}




void Cartesian_Line::linear_coef()
{

	 tf_min_limit = Vdevide(V_traj_length_Dalpha, V_Vc_Wc).Max();
		

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
		Na = floor(tf / t_interval) ;
	}

}


void Cartesian_Line::halfcos_coef()
{

	
	Vector Tf = Halfcos_Tf(V_traj_length_Dalpha, ZeroVector2, ZeroVector2, V_V_W_mintf, V_Vdmax_Wdmax);
	
	int max_index = 0;
	tf_min_limit = Tf.Max(max_index);



	if (method == time)
	{
	
		if (tf < tf_min_limit)
		{
			tf = tf_min_limit;
		}

		Tf(max_index) = tf;
		//Scale_Tf = Vdevide(Vector(tf, int(Tf.COUNT)), Tf); //>=1 tf是实际要严格执行的时间，Tf是最小时间
	}

	else
	{
	
		Tf= Halfcos_Tf(V_traj_length_Dalpha, ZeroVector2, ZeroVector2, V_Vc_Wc, V_Vdmax_Wdmax);
		tf = Tf.Max();
				
	}

	if (t_interval > 0)
	{
		Na = floor(tf / t_interval) ;

		tf = Na*t_interval;
	}

		Halfcos_coef(ZeroVector2, V_traj_length_Dalpha, ZeroVector2, ZeroVector2,method,tf, V_Vc_Wc, V_Vdmax_Wdmax, Half_coef, tf_max_index);


}

void Cartesian_Line::trapez_coefs()
{

	Vector Tf = Trapez_Tf(V_traj_length_Dalpha, ZeroVector2, ZeroVector2, V_V_W_mintf, V_Vdmax_Wdmax);

	int max_index = 0;
	tf_min_limit = Tf.Max(max_index);



	if (method == time)
	{

		if (tf < tf_min_limit)
		{
			tf = tf_min_limit;
		}

		Tf(max_index) = tf;
		//Scale_Tf = Vdevide(Vector(tf, int(Tf.COUNT)), Tf); //>=1 tf是实际要严格执行的时间，Tf是最小时间
	}

	else
	{

		Tf = Trapez_Tf(V_traj_length_Dalpha, ZeroVector2, ZeroVector2, V_Vc_Wc, V_Vdmax_Wdmax);
		tf = Tf.Max();

	}

	if (t_interval > 0)
	{
		Na = floor(tf / t_interval) ;

		tf = Na*t_interval;
	}

Trapez_coef(ZeroVector2, V_traj_length_Dalpha, ZeroVector2, ZeroVector2, method, tf, V_Vc_Wc, V_Vdmax_Wdmax, Trapez_coefs, tf_max_index);

}



void Cartesian_Line::realtime_PVA()
{
	PP_n.Posi = PP_0.Posi + Uvec*st_2(1);
	double t_p;

	if (Dalpha==0)
	{
		t_p = 0;
	}
	else
	{
		t_p = st_2(2) / Dalpha;
	}
	Posture_line.Slerp(t_p);
	Qt = Posture_line.Qt;
	PP_n.Pose = Qt;
	Wt = vt_2(2)*Posture_line.Qvec;
	Wdt = at_2(2)*Posture_line.Qvec;
	Wt = Qt*Wt;
	Wdt = Qt*Wdt;
	Vt = vt_2(1)*Uvec;
	At = at_2(1)*Uvec;
	Vector V_n_temp(Vt, Wt);
	Vector A_n_temp(At, Wdt);
	V_n = V_n_temp;
	A_n = A_n_temp;

}

void Cartesian_Line::realtime(int N)
{
	
	N_now = N;
	t_n = N*t_interval;//分数比例
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
		//default:

		}
		

	}

}