#include "ikine_robot_traj.h"

void main()
{
	Posi_Pose PP0 ;
	Posi_Pose PPf;
	Posi_Pose *PP_n;
	enum P2Pmethod cht = halfcos;
	enum TIMEorSPEED method = speed;
	enum cartesian_traj_type traj_type = line;
	double tf_lspeedp = 0.1;
	double acce_p = 0.8;
	double t_interval_ = 0.01;
	double	 qddp = 0.8;
	Cartesian_traj traj1;

	robot_phr Robot_phr(robot_left_dh);//或者robot_right_dh


	if (traj_type == line)
	{

		Cartesian_Line traj1(PP0, PPf, cht, method, tf_lspeedp, acce_p, t_interval_);
				
	};

	//if (traj_type == circle)
	//{

	//	Cartesian_Circle traj1(PP0, PPf, cht, method, tf_lspeedp, acce_p, t_interval_);

	//};
	//if (traj_type == cubic3)
	//{

	//	Cartesian_Cubic traj1(PP0, PPf, cht, method, tf_lspeedp, acce_p, t_interval_);

	//};
	//if (traj_type == spline)
	//{

	//	Cartesian_Spline traj1(PP0, PPf, cht, method, tf_lspeedp, acce_p, t_interval_);

	//};


	ofstream outFile; //输出文件流(输出到文件)

	outFile.open("D:\\qN_traj1.csv", ios::out);//打开模式可省略

	int point_i = 1;

	for (int i = 0; i < traj1.Na; i++)
	{
		double tt = i* traj1.t_interval;
		traj1.realtime(tt);
		Robot_phr.Ikine_nearest(traj1.PP_n, Robot_phr.q_n);//求出K和qn
		outFile << point_i << ',' << Robot_phr.q_n(1) << ',' << Robot_phr.q_n(2) << ',' << Robot_phr.q_n(3) << ',' << Robot_phr.q_n(4)
			<< ',' << Robot_phr.q_n(5) << ',' << Robot_phr.q_n(6) << endl;
		point_i++;
		Robot_phr.Singularity_monitor();//奇异监控
		if (Robot_phr.Singular_flag > 0) //进入奇异处理
		{
			Robot_phr.q_singular_start = Robot_phr.q_n;
			Robot_phr.qd_singular_start = Robot_phr.qd_n;
			Robot_phr.Singularity_jump(i,traj1);// 计算跳转步数k_jumpsteps

			P2P_ P2P(Robot_phr.q_singular_start, Robot_phr.q_singular_end, cubic,speed, 0.9, Robot_phr.qd_singular_start, Robot_phr.qd_singular_end,qddp, t_interval_);//关节空间三次插值
			
			for (int i = 0; i < P2P.Na; i++)
			{
				double tt = i*P2P.t_interval; //分数比例
				P2P.realtime(tt);
				outFile << point_i << ',' << P2P.qt(1) << ',' << P2P.qt(2) << ',' << P2P.qt(3) << ',' << P2P.qt(4)
					<< ',' << P2P.qt(5) << ',' << P2P.qt(6) << endl;
				point_i++;
			}
		}
	}
	outFile.close();

};