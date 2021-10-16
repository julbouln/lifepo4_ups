// basic gauge
#include <stdint.h>

#include "gauge.h"

extern float cur_bat_v;
extern float cur_bat_a;
extern float cur_inp_a;

static float lifepo4_discharge_profile[100] = {
	3.450, 3.340, 3.320, 3.310, 3.300,
	3.299, 3.297, 3.296, 3.295, 3.293, // linear
	3.292, 3.291, 3.289, 3.288, 3.287, 3.285, 3.284, 3.283, 3.281, 3.280,
	3.279, 3.277, 3.276, 3.275, 3.273, 3.272, 3.271, 3.269, 3.268, 3.267,
	3.265, 3.264, 3.263, 3.261, 3.260, 3.259, 3.257, 3.256, 3.255, 3.253,
	3.252, 3.251, 3.249, 3.248, 3.247, 3.245, 3.244, 3.243, 3.241, 3.240,
	3.239, 3.237, 3.236, 3.235, 3.233, 3.232, 3.231, 3.229, 3.228, 3.227,
	3.225, 3.224, 3.223, 3.221, 3.220, 3.219, 3.217, 3.216, 3.215, 3.213,
	3.212, 3.211, 3.209, 3.208, 3.207, 3.205, 3.204, 3.203, 3.201, 3.200,
	3.190, 3.180, 3.169, 3.158, 3.146, 3.133, 3.120, 3.105, 3.089, 3.072, // log
	3.053, 3.033, 3.010, 2.983, 2.953, 2.917, 2.873, 2.816, 2.737, 2.600};

float gauge_bat_v()
{
	return (((cur_bat_v)*2.5) / 0.32);
}

float gauge_bat_a()
{
	return (((cur_bat_a)*2.5) / 0.5);
}

float gauge_sys_a()
{
	return (((cur_inp_a)*2.5) / 0.32);
}

uint8_t gauge_bat_percent(float bat_v)
{
	uint8_t i;
	for (i = 0; i < 100; i++)
	{
		if (lifepo4_discharge_profile[i] <= bat_v)
		{
			return (100 - i);
		}
	}
	return 0;
}