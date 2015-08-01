#ifndef __CDSP_PREFERENCE_7023_H__
#define __CDSP_PREFERENCE_7023_H__
#include "mach/gp_cdsp.h"

gp_cdsp_user_preference_t cdsp_user_preference =
{
	.ae_target = 40,
	.ae_target_night = 18,
	.y_scale_day = 34,
	.y_offset_day = -8,	
	.y_scale_night = 36,//c:36,//b:34  //a:32
	.y_offset_night = -2,//c:0, //b:4
	.u_offset_day = 1,
	.v_offset_day = 2,
	.u_offset_night = 1,
	.v_offset_night = 2,
	.u_scale_day = 36,//40//a:28,
	.v_scale_day = 36,//40//a:28,
	.u_scale_night = 26,
	.v_scale_night = 26,
	.edge_day = 2,
	.edge_night = 2,
	.wb_offset_day = 0,
	.wb_offset_night = 0,
	.max_lum = 64-15,//e:27/25 for F2.4//d:34 for F1.8//c:22, //a:24
};
#endif
