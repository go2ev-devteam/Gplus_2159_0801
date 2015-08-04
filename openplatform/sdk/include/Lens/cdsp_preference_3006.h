#ifndef __CDSP_PREFERENCE_3006_H__
#define __CDSP_PREFERENCE_3006_H__
#include "mach/gp_cdsp.h"

gp_cdsp_user_preference_t cdsp_user_preference =
{
	.ae_target = 36, 
	.ae_target_night = 16, //1225b:16//1225:18 
	.y_scale_day = 33,//35, 
	.y_offset_day = -2,//-4, 
	.y_scale_night = 34,//1225b:33//1225:34, //c:36,//b:34  //a:32 
	.y_offset_night = 0,//1225b:-8//1225:0, c:0, //b:4 
	.u_offset_day = 0, 
	.v_offset_day = 0, 
	.u_offset_night = 0, 
	.v_offset_night = 0, 
	.u_scale_day = 40,//a:28, 
	.v_scale_day = 40,//a:28, 
	.u_scale_night = 28, 
	.v_scale_night = 28, 
	.edge_day = 2, 
	.edge_night = 0, 
	.wb_offset_day = 0, 
	.wb_offset_night = 0, 
	.max_lum = 64-16,//e:27/25 for F2.4//d:34 for F1.8//c:22, //a:24 
};
#if 0
gp_cdsp_user_preference_t cdsp_user_preference =
{
	.ae_target = 36, 
	.ae_target_night = 18, //1225b:16//1225:18 
	.y_scale_day = 32, 
	.y_offset_day = -2, 
	.y_scale_night = 34,//1225b:33//1225:34, //c:36,//b:34  //a:32 
	.y_offset_night = 4,//1225b:-8//1225:0, c:0, //b:4 
	.u_offset_day = 0, 
	.v_offset_day = 0, 
	.u_offset_night = 0, 
	.v_offset_night = 0, 
	.u_scale_day = 38,//a:28, 
	.v_scale_day = 38,//a:28, 
	.u_scale_night = 24, 
	.v_scale_night = 24, 
	.edge_day = 2, 
	.edge_night = 1, 
	.wb_offset_day = 0, 
	.wb_offset_night = -2, 
	.max_lum = 64-20,//e:27/25 for F2.4//d:34 for F1.8//c:22, //a:24 
};
#endif
#endif
