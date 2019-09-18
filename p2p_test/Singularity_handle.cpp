#include "Singularity_handle.h"
int Singularity_jump(const Ksew &K_last_,const Ksew &K_now_,int flag)
{
	double K_last;
	double K_now;
	
	switch (flag)
	{
	case 1:
	{
		K_last = K_last_.Ks;
		K_now = K_now_.Ks;
		break;
	}

	case 2:
	{
		K_last = K_last_.Ke;
		K_now = K_now_.Ke;
		break;
	}

	case 3:
	{
		K_last = K_last_.Kw;
		K_now = K_now_.Kw;
		break;
	}

	
	}

	double K_slope = K_last - K_now;

	return 2 * ceil(K_now / K_slope);
}