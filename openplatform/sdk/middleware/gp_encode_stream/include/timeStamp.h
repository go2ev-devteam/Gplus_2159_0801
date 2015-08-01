#define TS_X_OFFSET	500
#define TS_Y_OFFSET	80
#define TS_X_4x3_OFFSET 740

int timeStamp_Init();
void timeStamp_Uninit();
int timeStamp_draw(char* frameBuf, int frameWidth, int frameHeight, int date_only, int format);