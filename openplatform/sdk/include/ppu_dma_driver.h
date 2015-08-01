#include <mach/gp_ppu.h>

#define COLOR_RGB565			0
#define COLOR_VYUY				1
#define COLOR_YVYU				2
#define COLOR_UYVY				3
#define COLOR_YUYV				4
#define COLOR_BGRG				5
#define COLOR_GBGR				6
#define COLOR_RGBG				7
#define COLOR_GRGB				8
#define COLOR_RGBA				9

typedef struct ppu_buffer_info_s{
	int buffer_color_mode;
	int transparent_color;
	unsigned int transparent_enable;
	unsigned int transparent_mode;	
	unsigned int t_width;
	unsigned int t_height;
	unsigned int s_width;
	unsigned int s_height;	
	unsigned int t_addr;
	unsigned int s_addr;
} ppu_buffer_info_t;

//PPU DMA Mode
extern int frame_buffer_ppu_update_init(void);
extern int frame_buffer_ppu_update_uninit(void);
extern int frame_buffer_ppu_update_info(ppu_buffer_info_t *ppu_info);
extern int frame_buffer_ppu_update_go(unsigned int buffer_start,unsigned int buffer_x_offset,unsigned int buffer_y_offset);
extern int frame_buffer_ppu_update_state_get(void);


