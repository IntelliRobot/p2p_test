#pragma once
#include "stdafx.h"
#include "Posture_Interp.h"
///#include "Quaternion.h"
//#include "matrix.h"
///#include "vector.h"
///#include "math_addition.h"


Posture_Interp::Posture_Interp()
{}

Posture_Interp::Posture_Interp(vector<Quaternion> Qsn_in, Vector Tsn_in, enum Quaternion_method method_in, Vector W0_ , Vector Wf_, double t_interval )
{
	
	Point_N = Qsn_in.size();//点的个数
	method = method_in;
	W0 = W0_;
	Wf = Wf_;
	Q0 = Qsn_in[0];
	Qf = Qsn_in[Point_N-1];

	Qvec = cross(Q0.Inverse(), Qf).ToVector(3);
	double Qvec_norm = Qvec.Norm();
	if (Qvec_norm>Epsilon)
	{
		Qvec = Qvec / Qvec_norm;

	}
	else
	{
		Qvec = ZeroVector3;
	}

	Qsn = Qsn_in;
	Tsn = Tsn_in;
	Tf = Tsn_in.Max();

	if (Tf>0)
	{
		Tsn_percent = Vector(Point_N, HORZ);//Tsn / Tf;

	}

	//Sin.size = Point_N;
	//dalpha = Vector(Point_N - 1, HORZ);



	//if (Point_N == 2 & method == slerp) { Slerp_ini(Q0,Qf); } //两点间线性插值，初始化
	if ((Point_N >= 3) && (method == squad)) { Squad_ini(); } //多点线性插值，初始化
	if ((Point_N >= 3) && (method == mspline)) { Mspline_ini(); }//两点或多点三次样条插值，初始化

	
}

Posture_Interp::Posture_Interp(const Quaternion &Q0, const Quaternion &Qf, enum Quaternion_method method, double t_interval )
{
	Point_N = 2;
	if ( method == slerp) 
	{ 
		Slerp_ini(Q0, Qf); 
	}
}

Posture_Interp::Posture_Interp(const Quaternion &Q0_, const Quaternion & Qf_, Vector W0_, Vector Wf_,double Tf_,enum Quaternion_method method, double t_interval)
{
	Point_N = 2;
	Tf = Tf_;
	Tsn = Vector(Point_N, HORZ);
	Tsn.pVector[0] = 0;
	Tsn.pVector[1] = Tf;


	
	if (method == mspline) 
	{
		W0 = W0_;
		Wf = Wf_;
		Q0 = Q0_;
		Qf = Qf_;
		vector<Quaternion> Qsn_ = { Q0,Qf };

		Qsn = Qsn_;

		dt = Vector(Point_N, HORZ);

		dt.pVector[0] = 0;
		dt.pVector[1] = Tf;

		dalpha = Vector(Point_N , HORZ);
		ev = matrix(Point_N , 3);

		Quaternion delta_Q = cross(Qf, Q0.Inverse());
		Axis_Angle delta_Log = delta_Q.Log();

		dalpha.pVector[1] = 2*delta_Log.Theta;
		ev.SetValue(2, delta_Log.Qvec,HORZ);

		Wk = matrix(Point_N, 3);
		Wk.SetValue(1, W0, HORZ);
		Wk.SetValue(2, Wf, HORZ);
		
		Mspline_ini(); 

	}

}

Posture_Interp::~Posture_Interp()
{
	cout << "posture deconstruct" << endl;
}



void  Posture_Interp::Slerp_ini(const Quaternion &Q0_,const Quaternion &Qf_)
{
	W0 = ZeroVector3;
	Wf = ZeroVector3;
	Q0 = Q0_;
	Qf = Qf_;

	Qvec = cross(Q0.Inverse(), Qf).ToVector(3);
	double Qvec_norm = Qvec.Norm();
	if (Qvec_norm>Epsilon)
	{
		Qvec = Qvec / Qvec_norm;

	}
	else
	{
		Qvec = ZeroVector3;
	}

	DeltaQ = cross(Q0.Inverse(), Qf);

	Slerp_theta= DeltaQ.Log().Theta;
	Slerp_sintheta = sin(Slerp_theta);
	
	if (Slerp_sintheta>Epsilon)
	{
		Slerp_invsintheta = 1 / Slerp_sintheta;
	}

}

void  Posture_Interp::Squad_ini()
{
	if (Tf>0)
	{
		Tsn_percent = Tsn / Tf;

	}

	Na =ceil( Tf / t_interval);
	
	for ( int i = 0; i < Point_N; i++)
	{
		Quaternion Qsnim1, Qsnip1;
		if (i==0)
		{
			Qsnim1 = Q0;
			Qsnip1 = Qsn[1];
		}

		else if (i==Point_N-1)
		{
			Qsnim1 = Qsn[Point_N-2];
			Qsnip1 = Qf;
		}

		else
		{
			Qsnim1 = Qsn[i - 1];
			Qsnip1 = Qsn[i + 1];

		}
		Quaternion Qiim1 = cross(Qsn[i].Inverse(), Qsnim1);
		Quaternion Qiip1 = cross(Qsn[i].Inverse(), Qsnip1);
		Axis_Angle Qiii = Vector2AA(-1*(Qiim1.Log().Theta*Qiim1.Log().Qvec + Qiip1.Log().Theta*Qiip1.Log().Qvec) / 4);
		Quaternion Si = Quaternion(Qiii);
		Sin[i] = cross(Qsn[i], Si);
	}
	
	//return  Sq_Coef;
}

void  Posture_Interp::Mspline_ini()
{
	
	
	if (Point_N >2)
	{
		dalpha = Vector(Point_N , HORZ);
		ev = matrix(Point_N , 3);

		dt = Vector(Point_N, HORZ);
		dt.pVector[0] = 0;

		for (int i = 1; i <= Point_N-1; i++)
		{
			dt.pVector[i] = Tsn.pVector[i] - Tsn.pVector[i-1];
			Quaternion delta_Q = cross(Qsn[i], Qsn[i - 1].Inverse());
			Axis_Angle deltaq_log = delta_Q.Log();
			dalpha.pVector[i] = 2 * deltaq_log.Theta;
			ev.SetValue(i + 1, deltaq_log.Qvec, HORZ);
		}
		
		quater_multi_spline_Wk(W0, Wf);

	}
	
	Na = ceil(Tf / t_interval);
	Mspline_coef();


}

void Posture_Interp::Mspline_coef()
{
	Ms_Coef.A1r1 = matrix(Point_N,3);
	Ms_Coef.A1r2 = matrix(Point_N, 3);
	Ms_Coef.A2r1 = matrix(Point_N, 3);
	Ms_Coef.A2r2 = matrix(Point_N, 3);
	Ms_Coef.A2r3 = matrix(Point_N, 3);
	Ms_Coef.A3r2 = matrix(Point_N, 3);
	Ms_Coef.A3r3 = matrix(Point_N, 3);
	Ms_Coef.A3r4 = matrix(Point_N, 3);

	for (int i = 2; i <= Point_N; i++)
	{
		//ev.RowVector(i);
		//Wk.RowVector(i);

		Vector Binvwf = B_coef_inv(ev.RowVector(i), dalpha(i), Wk.RowVector(i));
		Vector dtheta_v = dalpha(i)*ev.RowVector(i);

		//Wk.RowVector(i-1);

		Ms_Coef.A3r2.SetValue(i-1, dt(i)*Wk.RowVector(i - 1),HORZ);
	
		Ms_Coef.A3r3.SetValue(i-1, dt(i)*Binvwf - 3 * dtheta_v, HORZ);
				
		Ms_Coef.A3r4.SetValue(i-1, dtheta_v, HORZ);
		
		
		Ms_Coef.A2r1.SetValue(i-1, Wk.RowVector(i - 1), HORZ);

		Vector A2r2_temp = 2 * Wk.RowVector(i - 1) + 2 * Binvwf - 6 / dt(i)*dtheta_v;
				
		Ms_Coef.A2r2.SetValue(i-1, A2r2_temp, HORZ);
		
		Ms_Coef.A2r3.SetValue(i-1, Binvwf, HORZ);
		
		
		Ms_Coef.A1r1.SetValue(i-1, (A2r2_temp + 2 * Wk.RowVector(i - 1)) / dt(i), HORZ);
			
		Ms_Coef.A1r2.SetValue(i-1, A2r2_temp + 2 * Binvwf, HORZ);

	}

}





Quaternion  Posture_Interp::Slerp(double t)
{
	double k0;
	double kf;
	//double k0_w;
//	double kf_w;
	double t_theta = t*Slerp_theta;
	if (Slerp_sintheta<Epsilon)
	{
		k0 = 1.0 - t;
		kf = t;
		//k0_w = cos(Slerp_theta - t_theta);
		//kf_w = cos(t_theta);

	}
	else
	{
	    k0 = sin(Slerp_theta - t_theta)*Slerp_invsintheta;	
		kf = sin(t_theta)*Slerp_invsintheta;
	//	k0_w = (cos(Slerp_theta - t_theta)*Slerp_invsintheta)*Slerp_theta;
		//kf_w = (cos(t_theta)*Slerp_invsintheta)*Slerp_theta;

	}


	Qt = Q0*k0 + Qf*kf;
	//Qdt= Q0*k0_w + Qf*kf_w;
	//Qddt= Qt*Slerp_theta*Slerp_theta*(-1);

	//matrix Qt_mat_cross= Qt.Mat_cross();
	
//	Wt = t_interval*(Qt_mat_cross*Qdt.ToVector());
	
	//Wtdot = ZeroVector3;
	//Wtdot=t_interval*t_interval*(Qt_mat_cross*Qddt.ToVector());
	

	return Qt;

}

void  Posture_Interp::Squad()
{
	
	double t;
	int i = 0;

	for (int p = 0; p < Na; p++)
	{
		t = p*t_interval;

		if (t>=Tsn.pVector[i] && t<Tsn.pVector[i+1])
		{
			double deltat = Tsn.pVector[i + 1] - Tsn.pVector[i];
			double k;
			if (deltat>0)
			{
				 k = (t - Tsn.pVector[i]) / deltat;
			}
			
			Posture_Interp qq(Qsn[i], Qsn[i + 1],slerp);
			Posture_Interp ss(Sin[i], Sin[i + 1], slerp);
			Posture_Interp squad_i(qq.Slerp(k), ss.Slerp(k),slerp);
			Qt = squad_i.Slerp(2 * k*(1 - k));
			
		}

		if (t>Tsn.pVector[i + 1] && t<Tf)
		{
			i++;
		}

		if (t==Tf)
		{
			Qt = Qf;
		}
	}
	
	//return Qt;
}

void  Posture_Interp::Mspline()
{
	//return Qt;
	int pointer = 1;
	Wt_matrix=matrix(Na+1, 3);
	Wtdot_matrix=matrix(Na+1, 3);
	Qt_matrix=matrix(Na+1, 4);
	Vector u_vec=ZeroVector3;
	Vector w_vec = ZeroVector3;
	Vector wu_vec = ZeroVector3;
	Vector wdot_vec = ZeroVector3;
	//ofstream outFile;
	//outFile.open("D:\\Qt.csv", ios::out);
	for (int i = 0; i <= Na; i++)//进入插值循环
	{
		double tt = i*t_interval;
		if (tt > Tsn(pointer + 1))// 进入下一段
		{
			pointer = pointer + 1;
		}
		double dti = tt - Tsn(pointer); // 相对于每段起始点的增量时间
		//Mspline_coef_i = Mspline_coef(:, : , pointer);

		double dti_frac = dti / abs(dt(pointer + 1));

		matrix alpha3v=quater_Mspline_alpha(dti_frac, pointer);
		Quaternion dQi = Vector2Q(alpha3v.RowVector(1));//alpha_v
				
		double theta_norm = alpha3v.RowVector(1).Norm();
		if (theta_norm < Epsilon)
		{
			Vector u_vec = ZeroVector3;
			Wt= alpha3v.RowVector(2);//alphad_v;
			
			Wtdot= alpha3v.RowVector(2);//alphadd_v

			Wt_matrix.SetValue(i + 1, Wt, HORZ);

			Wtdot_matrix.SetValue(i + 1, Wt, HORZ);
		}
		else
		{
			
			u_vec = alpha3v.RowVector(1) / theta_norm;
			w_vec = cross(u_vec, alpha3v.RowVector(2)) / theta_norm;
			wu_vec = cross(w_vec, u_vec);
			double thetad_norm = dot(u_vec, alpha3v.RowVector(2));
			double thetadd_morm = dot(u_vec, alpha3v.RowVector(3)) + dot(wu_vec, alpha3v.RowVector(2));
			wdot_vec = (cross(alpha3v.RowVector(1), alpha3v.RowVector(3)) - 2 * dot(alpha3v.RowVector(1), alpha3v.RowVector(2))*w_vec) / (theta_norm *theta_norm);
				
			Wt = thetad_norm*u_vec + sin(theta_norm)*wu_vec - (1 - cos(theta_norm))*w_vec;
			Wt_matrix.SetValue(i + 1, Wt, HORZ);

			Wtdot = thetadd_morm*u_vec + sin(theta_norm)*cross(wdot_vec, u_vec) - (1 - cos(theta_norm))*wdot_vec + thetad_norm*wu_vec + cross(Wt, (thetad_norm*u_vec - w_vec));

			Wtdot_matrix.SetValue(i + 1, Wt, HORZ);
		}
		

			Qt = cross(dQi, Qsn[pointer-1]);

			Qt_matrix.SetValue(i + 1, Qt, HORZ);
			//outFile << i+1 << ',' << Qt.w << ',' << Qt.x<< ',' << Qt.y<<',' << Qt.z << endl;
	}
	//outFile.close();
}

void Posture_Interp::Omega_matrix_coef_abc(  Vector &a_coef,  Vector  &b_coef,  Vector &c_coef)
{

	a_coef.pVector[0] = 0;
	//c_coef.pVector[0] = 0;
	for (int i = 1; i <= Point_N -2; i++)
	{
		a_coef.pVector[i] = 2 / dt.pVector[i];; //长度N - 2, a索引2到N - 1，a(1), a(N)无效，有效长度N - 2
		c_coef.pVector[i-1] = 2 / dt.pVector[i];   //长度N - 1个, c索引2到N, c(1)无效，有效长度N - 2
			
	}
	c_coef.pVector[Point_N - 2] = 2/dt.pVector[Point_N - 1];

	b_coef = 2*(a_coef + c_coef);//N-1个 pVector[0~N-2]

	if (Point_N >3)
	{
		for (int i = 3; i < Point_N; i++)
		{
			double ab_temp = a_coef(i)/ b_coef(i - 1);
			b_coef(i) = b_coef(i) - c_coef(i - 1)*ab_temp;

		}

	}
	

}

Vector Posture_Interp::Rvec_wf(const Vector &ev, const double &dalpha, Vector &wf)
{
	Vector result(wf.COUNT, wf.TYPE);
	if (dalpha == 0)
	{
		result = ZeroVector3;
	}
	else
	{
		double evDotwf = dot(ev, wf);
		Vector evXwf = cross(ev, wf);
		double sindtheta = sin(dalpha);
		double cosdtheta = cos(dalpha);
		double r0 = 0.5*(dalpha - sindtheta) / (1 - cosdtheta);
		double r1 = (dalpha*sindtheta - 2 * (1 - cosdtheta)) / (dalpha*(1 - cosdtheta));
		result = r0*(wf.Norm()*wf.Norm() - evDotwf*evDotwf)*ev + r1*evDotwf*cross(evXwf, ev);
	}
	return result;
}

Vector Posture_Interp::B_coef_vec(const Vector &ev, const double &dalpha, const Vector &xvec)
{
	Vector result(xvec.COUNT, xvec.TYPE);
	if (dalpha == 0)
	{
		result = xvec;
	}

	else
	{
		//Vector tmp1 = dot(xvec, ev)*ev;
		//Vector tmp2 = sin(dalpha) / dalpha*(cross(cross(ev, xvec), ev));
		//Vector tmp3 = (1 - cos(dalpha)) / dalpha*cross(ev, xvec);
		//result = tmp2- tmp3;
		result = dot(xvec, ev)*ev + sin(dalpha) / dalpha*(cross(cross(ev, xvec), ev)) - (1 - cos(dalpha)) / dalpha*cross(ev, xvec);
	}
	return result;
}
Vector Posture_Interp::B_coef_inv(const Vector &ev, const double &dalpha, const Vector &xvec)
{
	Vector result(xvec.COUNT, xvec.TYPE);
	if (dalpha==0)
	{
		result = xvec;
	}

	else
	{
		result= dot(ev, xvec)*ev + sin(dalpha)*dalpha / (2 * (1 - cos(dalpha)))*(cross(cross(ev, xvec), ev)) + dalpha / 2 * cross(ev, xvec);
	}
	return result;
}

matrix  Posture_Interp::Omega_matrix_coef_dk(  matrix &omega_prev, const Vector &a_coef, const Vector &b_coef, const Vector &c_coef, const Vector &W0, const Vector &Wf)
{
	matrix d_vec(Point_N - 1,3);
	if (Point_N > 3)
	{
		for (int k = 2; k <=Point_N-1; k++)
		{
			Vector d_vec_k = 6 * (dalpha(k) / (dt(k)*dt(k))*ev.RowVector(k) + dalpha(k + 1) / (dt(k + 1)*dt(k + 1))*ev.RowVector(k + 1)) - Rvec_wf(ev.RowVector( k), dalpha(k), omega_prev.RowVector(k));
			d_vec.SetValue(k,d_vec_k, HORZ);
		}
		Vector temp = a_coef(2)*B_coef_vec(ev.RowVector(2), dalpha(2), W0);
		Vector d_vec_2 = d_vec.RowVector(2)-temp;//- a_coef(2)*B_coef_vec(ev.RowVector(2), dalpha(2), W0);
		d_vec.SetValue(2, d_vec_2, HORZ);
		Vector d_vec_N_1 = d_vec.RowVector(Point_N - 1) - c_coef(Point_N - 1)*B_coef_inv(ev.RowVector(Point_N), dalpha(Point_N), Wf);
		d_vec.SetValue(Point_N - 1, d_vec_N_1, HORZ);

		for (int i = 3; i < Point_N; i++)
		{
			double ab_temp = a_coef(i)/ b_coef(i - 1);
			Vector Bd_vec = B_coef_vec(ev.RowVector(i), dalpha(i), d_vec.RowVector(i - 1));
			d_vec.SetValue(i,(d_vec.RowVector(i) - ab_temp*Bd_vec),HORZ);
		}
			
		
	}
	else if(Point_N == 3)
	{
		Vector d_veci = a_coef(2)*B_coef_vec(ev.RowVector(2), dalpha(2), W0);
		Vector d_vecf = c_coef(Point_N - 1)*B_coef_inv(ev.RowVector(Point_N), dalpha(Point_N), Wf);
		int k = 2;
		Vector d_vec_k = 6 * (dalpha(k) / (dt(k)*dt(k))*ev.RowVector(k) + dalpha(k + 1) / (dt(k + 1)*dt(k + 1))*ev.RowVector(k + 1)) - Rvec_wf(ev.RowVector(k), dalpha(k), omega_prev.RowVector(k));
		d_vec.SetValue( k,(d_vec_k - d_veci - d_vecf),HORZ);
		
	}
	return d_vec;


}

matrix Posture_Interp::quater_Mspline_alpha(double dti_frac,int i)
{
	double x = dti_frac;
	double x_1 = dti_frac - 1;

	Vector alpha_v = x*x_1*x_1 * Ms_Coef.A3r2.RowVector(i) + x*x * x_1*Ms_Coef.A3r3.RowVector(i) + x*x*x * Ms_Coef.A3r4.RowVector(i);
	Vector alphad_v = x_1*x_1 * Ms_Coef.A2r1.RowVector(i) + x*x_1*Ms_Coef.A2r2.RowVector(i) + x*x * Ms_Coef.A2r3.RowVector(i);
	Vector alphadd_v = x_1*Ms_Coef.A1r1.RowVector(i) + x*Ms_Coef.A1r2.RowVector(i);
	matrix alpha3v(3, 3);
	alpha3v.SetValue(1, alpha_v, HORZ);
	alpha3v.SetValue(2, alphad_v, HORZ);
	alpha3v.SetValue(3, alphadd_v, HORZ);
	return alpha3v;
}

void Posture_Interp::quater_multi_spline_Wk(const Vector &W0, const Vector &Wf)
{

	Wk= matrix(Point_N, 3);
	Wk.SetValue(1, W0, HORZ);
	Wk.SetValue(Point_N, Wf, HORZ);

	iters = 0;
	double domega_norm_sum = Epsilon + 1;

	//求系数a，b, c
	Vector a_coef(Point_N - 1, HORZ);
	Vector b_coef(Point_N - 1, HORZ);
	Vector c_coef(Point_N - 1, HORZ);
	matrix omega(Point_N - 1, 3);
	Omega_matrix_coef_abc(a_coef, b_coef, c_coef);
	Vector domega_norm(Point_N - 1, HORZ);

	//求d_vec


		//迭代求wk
	while (iters<max_iter && domega_norm_sum >Epsilon)
		// 求矩阵方程系数
{	
	matrix d_vec=Omega_matrix_coef_dk(Wk, a_coef, b_coef, c_coef, W0, Wf);
		
	omega.SetValue(Point_N - 1, 1 / b_coef(Point_N - 1)*d_vec.RowVector(Point_N - 1), HORZ);

	if (Point_N> 3)
	{
		for (int i = Point_N - 2; i >= 2; i--)
		{
			Vector omegaVi = (d_vec.RowVector(i) - c_coef(i)*B_coef_inv(ev.RowVector( i + 1), dalpha(i + 1), omega.RowVector(i + 1))) / b_coef(i);
			omega.SetValue(i, omegaVi, HORZ);
		}
		
	}
	for (int i = 2; i <= Point_N - 1;i++)
	{
		
		domega_norm(i) = (omega.RowVector(i) - Wk.RowVector(i)).Norm();
	}
	
	domega_norm_sum = domega_norm.Norm();

	for (int i = 2; i <= Point_N-1; i++)
	{
		Wk.SetValue(i, omega.RowVector(i), HORZ);
	}
	
	iters = iters + 1;
}
		//Wk = Wk;


}
