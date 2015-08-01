#include "stdio.h"

/**************************************************************************/
// QRcode Decoder Head File
// v1003(150203)
/**************************************************************************/

//========================================================
// Function Name :  QR_GetErrorNum
// Syntax : long QR_GetErrorNum(unsigned char *qr_working_memory);
// Purpose :  get number of error code 
// Parameters : unsigned char *qr_working_memory: working memory pointer
// Return : error code number
//========================================================
long QR_GetErrorNum(unsigned char *qr_working_memory);

//========================================================
// Function Name :  QR_GetVersion
// Syntax : long QR_GetVersion(unsigned char *qr_working_memory);
// Purpose :  get version of QRcode-pattern
// Parameters : unsigned char *qr_working_memory: working memory pointer
// Return : version of QRcode-pattern
//========================================================
long QR_GetVersion(unsigned char *qr_working_memory);	//Version of QRcode

//========================================================
// Function Name :  QR_GetECCLevel
// Syntax : long QR_GetECCLevel(unsigned char *qr_working_memory);
// Purpose :  get Ecc-level of QRcode-pattern
// Parameters : unsigned char *qr_working_memory: working memory pointer
// Return : ECC-level of QRcode-pattern
//========================================================
long QR_GetECCLevel(unsigned char *qr_working_memory);

//========================================================
// Function Name :  QR_GetWorkingMemorySize
// Syntax : long QR_GetWorkingMemorySize(void);
// Purpose :  get working memory size
// Parameters : none
// Return : size of working memory
//========================================================
long QR_GetWorkingMemorySize(void);

//========================================================
// Function Name :  QR_Init
// Syntax : long QR_Init(unsigned char *qr_working_memory,long width, long height,unsigned char// image, unsigned char image_type, unsigned char* resultstream);
// Purpose : kernel initial process
// Parameters : unsigned char *qr_working_memory : working memory pointer
//              long width : width of image(pixels)
//              long height : height of image(pixels)
//              unsigned char* image : image data pointer
//              unsigned char image_type : bit[1]: YUYV 
//										   bit[2]: for swap Y image(Y1Y0 Y3Y2..)(for Y only image)
//                                         bit[4]: for sequential Y image(Y0Y1Y2..)(for Y only image)
//                                         bit[8]: for Black/White Color Inverse Pattern
//                                         (reference "Image Type")
//              unsigned char* resultstream : decode result
// Return : 0 is ok; other vaule is failed. (Check error code)
//========================================================
long QR_Init(unsigned char *qr_working_memory,long width, long height,unsigned char* image, unsigned char image_type, unsigned char* resultstream);

//========================================================
// Function Name :  QR_Decode
// Syntax : unsigned char* QR_Decode(unsigned char *qr_working_memory);
// Purpose : main decode process of QRcode
// Parameters : unsigned char *qr_working_memory: working memory pointer
// Return : decode result
//========================================================
unsigned char* QR_Decode(unsigned char *qr_working_memory);

//========================================================
// Function Name :  QR_GetVersion
// Syntax : const char* QR_Version(void);
// Purpose :  get library version
// Parameters : none
// Return : version of QRcode decoder library
//========================================================
const char* QR_Version(void);							//version of Decoder 

//========================================================*/
// Error Code Number //
#define QR_OK					0
#define QR_ERR_FINDERPATTERN	-1
#define QR_ERR_ALIGNPATTERN		-2
#define QR_ERR_IMAGE_SIZE       -3
#define QR_ERR_IMAGE_BPP        -4
#define QR_ERR_MEMORY           -5
#define QR_ERR_VERSION          -6
#define QR_ECC_FAILUR			-7
#define QR_ERR_TOO_MUCH_LINE    -8
#define QR_REDO_FINDERPATTERN	-9			//add 130715

// Image Type //
#define IMAGETYPE_YUYV 			0x01	//for YUYV
#define IMAGETYPE_UYVY 			0x02	//for UYVY
#define IMAGETYPE_Y0Y1			0x04	//for (Y0Y1 Y2Y3..)(Y only image)
#define IMAGETYPE_Y1Y0			0x08	//for (Y1Y0 Y3Y2..)(Y only image)
#define IMAGETYPE_COLORINVERSE	0x10	//for white/black inverse pattern
