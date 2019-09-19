#include "stdafx.h"
#include "PtoP.h"
#include "vector.h"


PtoP::PtoP()
{}

PtoP::PtoP(Vector q0_1, Vector qf_1, enum P2Pmethod cht_1,enum TIMEorSPEED method_1 , double tf_s , Vector qd0_1 , Vector qdf_1 ,double qddp,double t_interval_)
{
	//成员变量赋值
	q0 = q0_1;
	qf = qf_1;
	qd0 = qd0_1;
	qdf = qdf_1;
	delta_q = qf - q0;
	delta_q_abs = delta_q.Abs();
	cht = cht_1;
	method = method_1;
	t_interval = t_interval_;
	qdd_percent = qddp;
	qdd = qdd_percent*Qdd_max;
	Nw = q0.COUNT;

	
	


	if (cht == halfcos)
	{
		for (int i = 0; i < Nw; i++)
		{
			V_qd_mintf.pVector[i] = sqrt(2 * Qdd_max.pVector[i] * delta_q_abs.pVector[i] / PI);// 两端速度为零，取此速度时，tf最小，此时速度最大
		}


	}

	if (cht == trapez)
	{
		for (int i = 0; i < Nw; i++)
		{
			V_qd_mintf.pVector[i] = sqrt( Qdd_max.pVector[i] * delta_q_abs.pVector[i] );// 两端速度为零，取此速度时，tf最小
		}

	}




	//double V_0_abs = 0;//初始线速度，标量
	//double V_f_abs = 0;


	if ((tf_s >= 0) && (tf_s <= 1) && (method == speed))
	{
		qd_percent = tf_s;
		qd = qd_percent *Qd_max;

		if ((cht == trapez) || (cht == halfcos))
		{
			Vector max_speed_p = Vdevide(V_qd_mintf, Qd_max);

			double min_speed_p = max_speed_p.Max();

			if (qd_percent>min_speed_p)
			{
				printf("%f \n", min_speed_p);
				printf("速度百分比需小于上述值");
				throw("速度百分比需小于上述值");

			}
		}

	}

	if ((tf_s >= 0) && (method == time))
	{
		tf = tf_s;
		//qd = Vector(V_max, W_max);
	}

	if ((tf_s < 0) || (tf_s >1) && (method == speed))
	{
		
			throw("速度百分比需在0~1");
	}


	switch (cht)
	{
	case cubic:
		Cubic_coef = { ZeroVector6,ZeroVector6,ZeroVector6,ZeroVector6 };
		cubic_coef();
		break;
	case halfcos:
		Half_coef = { ZeroVector6,ZeroVector6,ZeroVector6,ZeroVector6,ZeroVector6,ZeroVector6,ZeroVector6,ZeroVector6,
			ZeroVector6,ZeroVector6,ZeroVector6,ZeroVector6,ZeroVector6,ZeroVector6,ZeroVector6,ZeroVector6,0 };
		halfcos_coef();
		break;
	case trapez:
		Trapez_coefs = { ZeroVector6,ZeroVector6,ZeroVector6,ZeroVector6,ZeroVector6,ZeroVector6,ZeroVector6,ZeroVector6 ,ZeroVector6,ZeroVector6,0 };
		trapez_coefs();
		break;
	case linear:
		linear_coef();
		break;
	default:
		break;
	}

	
}

PtoP::~PtoP()
{}

void PtoP::cubic_coef()
{

	tf_min_limit = Cubic_max_tf(delta_q_abs, qd0, qdf, Qd_max, qdd);

	if (method == time)
	{

		if (tf < tf_min_limit)
		{
			tf = tf_min_limit;
		}
	}

	else
	{
		tf = Cubic_max_tf(delta_q_abs, qd0, qdf, qd, qdd);
	}

	if (t_interval > 0)
	{
		Na = floor(tf / t_interval);

		tf = Na*t_interval;
	}

	//将线速度和角速度拼接成向量的方法


	if (qd_percent > 0)
	{
		Cubic_spline_coef(q0, qf, qd0, qdf, tf, Cubic_coef);

	}

}




void PtoP::linear_coef()
{

	tf_min_limit = Vdevide(delta_q_abs, Qd_max).Max();


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
		Na = floor(tf / t_interval);
	}

}


 void PtoP::halfcos_coef()
{


	Vector Tf = Halfcos_Tf(delta_q_abs, qd0, qdf, V_qd_mintf, Qdd_max);

	int max_index = 0;
	tf_min_limit = Tf.Max(max_index);



	if (method == time)
	{

		if (tf < tf_min_limit)
		{
			tf = tf_min_limit;
			tf_max_index = max_index;
		}

		
		//Scale_Tf = Vdevide(Vector(tf, int(Tf.COUNT)), Tf); //>=1 tf是实际要严格执行的时间，Tf是最小时间
	}

	else
	{

		Tf = Halfcos_Tf(delta_q_abs, qd0, qdf, qd, Qdd_max);
		tf = Tf.Max(tf_max_index);

	}

	if (t_interval > 0)
	{
		Na = floor(tf / t_interval);

		tf = Na*t_interval;
	}

	Halfcos_coef(q0, qf, qd0, qdf, method, tf, qd, Qdd_max, Half_coef, tf_max_index);


}

void PtoP::trapez_coefs()
{

	Vector Tf = Trapez_Tf(delta_q_abs, qd0, qdf, V_qd_mintf, Qdd_max);
		
	int index = 0;
	tf_min_limit = Tf.Max(index);



	if (method == time)
	{

		if (tf < tf_min_limit)
		{
			tf = tf_min_limit;
			tf_max_index = index;
	
		}

		//Tf(max_index) = tf;
	
	}

	else
	{
		Tf = Trapez_Tf(delta_q_abs, qd0, qdf, qd, Qdd_max);
		tf = Tf.Max(tf_max_index);
		
	}

	if (t_interval > 0)
	{
		Na = floor(tf / t_interval);

		tf = Na*t_interval;
	}

	Trapez_coef(q0, qf, qd0, qdf, method, tf, qd, Qdd_max, Trapez_coefs, tf_max_index);

}



void PtoP::realtime(int N)
{
	N_now = N;
	t = N*t_interval;//分数比例
	//if (t >= 0 && t <= 1 && method == speed)

	//{
	//	t = t*tf;

	//}

	if (t<=tf)
	{
		switch (cht)
		{
		case cubic:
			Cubic_spline_rt(qt, qdt, qddt, Cubic_coef, t);
			break;
		case halfcos:
			Halfcos_rt(qt, qdt, qddt, Half_coef, t);
			break;
		case trapez:
			Trapez_rt(qt, qdt, qddt, Trapez_coefs, t);
			break;
		case linear:
			double tpercent = t / tf;
			qt = tpercent*delta_q;
			qdt = delta_q/tf;
			qddt = ZeroVector6;
			break;
		//default:
			//break;
		}
		
		
		
		

	
	}

}



void PtoP::offline(double interval_out)
{
	t_interval = interval_out;
	if (t_interval > 0)
	{
		Na = floor(tf / t_interval) + 1;
	}

	//qN(N, 6);

	ofstream outFile; //输出文件流(输出到文件)

	outFile.open("D:\\qN.csv", ios::out);//打开模式可省略

	for (int i = 0; i < Na; i++)

	{
		double tt = i*t_interval;
		realtime(tt);

		outFile << i << ',' << qt(1) << ',' << qt(2) << ',' << qt(3) << ',' << qt(4)
			<< ',' << qt(5) << ',' << qt(6) << endl;
	}
	outFile.close();

	outFile.open("D:\\qddN.csv", ios::out);//打开模式可省略

	for (int i = 0; i < Na; i++)

	{
		double tt = i*t_interval;
		realtime(tt);

		outFile << i << ',' << qddt(1) << ',' << qddt(2) << ',' << qddt(3) << ',' << qddt(4) 
			<< ',' << qddt(5) << ',' << qddt(6) << endl;
	}
	outFile.close();

	outFile.open("D:\\qdN.csv", ios::out);//打开模式可省略

	for (int i = 0; i < Na; i++)

	{
		double tt = i*t_interval;
		realtime(tt);

		outFile << i << ',' << qdt(1) << ',' << qdt(2) << ',' << qdt(3) << ',' << qdt(4)
			<< ',' << qdt(5) << ',' << qdt(6) << endl;
	}
	outFile.close();


}