#include "hal_mcu.h"
#include "hal_defs.h"
#include "hal_types.h"

#include "userTYPES.h"




#define RECALIB_TIMEOUT     12000 // Every 2 minutes
#define MSG_PING           0x00
#define MSG_SEND           0x10
#define MSG_RECIVE         0x20
#define MSG_NEEDSEND       0x30


#define LCD_CS  	P0_7
#define LCD_MOSI  	P1_6
#define LCD_CLK  	P1_5
#define LCD_RS  	P0_6
#define LCD_RESET  	P1_4
#define LCD_POWER  	P0_1


#define	HIGH		1
#define	LOW		0



void delaylcd (uint16 x);
void Lcdwritecom(uint8 com);
void Lcdwritedata(uint8 dat);
void Prog_Reset(void);
void Resetchip(void);
void SetRamAddr (uint8 Page, uint8 Col);
void SetContrast(uint8 Gain, uint8 Step);
void LcdPortInit(void);
void InitLcd(void);
void Printn(uint8 xx ,uint8 yy , uint32 no,uint8 yn,uint8 le);
void Printn8(uint8 xx ,uint8 yy , uint32 no,uint8 yn,uint8 le);
void Print6(uint8 xx, uint8 yy, uint8 ch1[], uint8 yn);
void Print8(uint16 y,uint16 x, uint8 ch[],uint16 yn);
void Print16(uint16 y,uint16 x,uint8 ch[],uint16 yn);
void Print(uint8 y, uint8 x, uint8 ch[], uint16 yn);
void Rectangle(uint8 x1,uint8 y1,uint8 x2,uint8 y2);
void Rectangle_x(uint8 x1,uint8 y1,uint8 x2,uint8 y2);
void ClearScreen(void);
void TurnOnDisp(void);
void TurnShowInterface(uint8 mode);
