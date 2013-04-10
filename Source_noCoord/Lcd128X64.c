#include <string.h>
#include <Font.h>
#include <Lcd128X64.h>


uint8 ContrastValue = 0x20;//灰度调整

void ClearScreen(void);
/*******************************************************************************
//函数名：void delaylcd (uint16 x)
//功能：廷时
//输入：时间
//输出：无
********************************************************************************/
void delaylcd (uint16 x)
{
	uint8 j;
	while (x--)
	{
		for (j=0;j<115;j++);
	}
}

/*
unsigned char SPIRead(unsigned char wd)
{ unsigned char rd,i;
        rd=0;
        for(i=0;i<8;i++){
             spisck_0();   
             if(wd & (1<<(7-i))){
               spimo_1();      
             }
             else{
               spimo_0();      
             }
             spisck_1();
             if(spimi()){
                  rd |=( 1 << (7 - i));
             }
        }
        return rd;
}
*/



/*******************************************************************************
//函数名：void Lcdwritecom(uint8 com)
//功能：lcd写指令
//输入：com指令
//输出：无
********************************************************************************/
void Lcdwritecom(uint8 com)
{
	uint8 i, temp;
	
	LCD_CS  =  LOW;
	LCD_CLK =  LOW;
	LCD_RS  =  LOW;

	temp = com;

	for(i=0; i<8; i++)
	{
		if(temp & 0x80)
		{
                        LCD_MOSI = HIGH;
		}
                else {
                        LCD_MOSI =  LOW;
                }
		temp <<= 1;
		LCD_CLK =  HIGH;
		asm("nop");
		asm("nop");
		LCD_CLK =  LOW;
	}
        LCD_CS = HIGH;
}



/*******************************************************************************
//函数名：void Lcdwritedata(uint8 dat)
//功能：lcd写数据
//输入：dat数据
//输出：无
********************************************************************************/
void Lcdwritedata(uint8 dat)
{
	uint8 i, temp;
	
	LCD_CLK =  LOW;
	LCD_CS  =  LOW;
	LCD_RS  =  HIGH;
	temp = dat;

	for(i=0; i<8; i++)
	{
		if(temp & 0x80)
		{
			LCD_MOSI = HIGH;
		}
		else    LCD_MOSI = LOW;
		temp <<= 1;
		LCD_CLK = HIGH;
		asm("nop");
		asm("nop");
		LCD_CLK = LOW;
	}
	LCD_CS = HIGH;
}



/*******************************************************************************
//函数名：void Prog_Reset(void)
//功能：lcd复位
//输入：无
//输出：无
********************************************************************************/
void Prog_Reset(void)
{
	LCD_RESET =  LOW;
	delaylcd(100);
	LCD_RESET =  HIGH;
}
/*******************************************************************************
//函数名：void Resetchip(void)
//功能：lcd软件复位
//输入：无
//输出：无
********************************************************************************/
void Resetchip(void)
{
	Prog_Reset();
}

/*******************************************************************************
//函数名：void SetRamAddr (uint8 Page, uint8 Col)
//功能：lcd位置选择
//输入：Page-页，Col-列
//输出：无
********************************************************************************/
void SetRamAddr (uint8 Page, uint8 Col)
{
	Lcdwritecom(0xB0 + Page);
	Lcdwritecom(Col & 0x0f); //Set lower column address
	Lcdwritecom(0x10 | ((Col & 0xf0) >> 4)); //Set higher column address
}



/*******************************************************************************
//函数名：void SetContrast(uint8 Gain, uint8 Step)
//功能：lcd对比度设定
//输入：Page-页，Col-列
//输出：无
********************************************************************************/
void SetContrast(uint8 Gain, uint8 Step)
{
	Lcdwritecom(0x81);
	Lcdwritecom(Step);
}

void LcdPortInit(void)
{
          //LCD_CS
	  P0SEL &= ~(1<<7);    
          P0DIR |= (1<<7);  
          P0    |= (1<<7);  

          //LCD_MOSI
          P1SEL &= ~(1<<6);  
          P1DIR |=  (1<<6);  
          P1    |=  (1<<6);  

          //LCD_CLK
          P1SEL &= ~(1<<5);  
          P1DIR |=  (1<<5);  
          P1    |=  (1<<5);  

          //LCD_RS
          P0SEL &= ~(1<<6);  
          P0DIR |=  (1<<6);  
          P0    |=  (1<<6);  

          //LCD_RESET
          P1SEL &= ~(1<<4);  
          P1DIR |=  (1<<4);  
          P1    |=  (1<<4);  
          
          //LCD_POWER
          P0SEL &= ~(1<<0);  
          P0DIR |=  (1<<0);  
          P0    |=  (1<<0);  
          
}


/*******************************************************************************
//函数名：void InitLcd(void)
//功能：lcd初始化
//输入：无
//输出：无
********************************************************************************/
void InitLcd(void)
{
	LcdPortInit();
        LCD_POWER =  LOW;    //OLED POWER ON     //背光
        
	Resetchip();
	Lcdwritecom(0xAE);
	Lcdwritecom(0xAD);	//dc-dc off
	Lcdwritecom(0x8a);
	//DelayMs(100);
        delaylcd(1);
	Lcdwritecom(0x00);
	Lcdwritecom(0x10);
	Lcdwritecom(0x40);
	Lcdwritecom(0x81);
	Lcdwritecom(ContrastValue);
	Lcdwritecom(0xA0);
	Lcdwritecom(0xA4);
	Lcdwritecom(0xA6);
	Lcdwritecom(0xA8);
	Lcdwritecom(0x3f);
	Lcdwritecom(0xD3);
	Lcdwritecom(0x00);
	Lcdwritecom(0xD5);
	Lcdwritecom(0x20);
	Lcdwritecom(0xD8);
	Lcdwritecom(0x00);
	Lcdwritecom(0xDA);
	Lcdwritecom(0x12);
	Lcdwritecom(0xDB);
	Lcdwritecom(0x00);
	Lcdwritecom(0xD9);
	Lcdwritecom(0x22);
	Lcdwritecom(0xc8);
	Lcdwritecom(0xAF);
	ClearScreen();
}


/*******************************************************************************
//函数名：void ClearScreen(void)
//功能：清屏
//输入：无
//输出：无
********************************************************************************/
void ClearScreen(void)
{
	uint8 i , j;
	
	for (i = 0 ; i < 8 ; i++)
	{
		SetRamAddr(i,0);
		for (j=0;j<132; j++) Lcdwritedata(0x00);
	}
}


/*******************************************************************************
//函数名：void Printn(uint8 xx ,uint8 yy , uint32 no,uint8 yn,uint8 le)
//功能：显示一个6*8无符号数据
//输入：xx , yy屏幕当中位置,no待显示数据 yn=0正常显示 yn=1反黑显示  le有效位
//输出：无
********************************************************************************/
void Printn(uint8 xx ,uint8 yy , uint32 no,uint8 yn,uint8 le)
{
	uint8 ch2[6];
	uint8 ii;

	for(ii = 1 ; ii <= le ;)
	{
		ch2[le - ii] = no % 10 + 0x30;
		no /= 10;
		ii += 1;
	}
	ch2[le] = '\0';
	Print6(xx ,yy ,ch2 ,yn);
}




/*******************************************************************************
//函数名：void Printn8(uint8 xx ,uint8 yy , uint32 no,uint8 yn,uint8 le)
//功能：显示8*8一个无符号数据
//输入：xx , yy屏幕当中位置,no待显示数据 yn=0正常显示 yn=1反黑显示  le有效位
//输出：无
********************************************************************************/
void Printn8(uint8 xx ,uint8 yy , uint32 no,uint8 yn,uint8 le)
{
	uint8 ch2[6];
	uint8 ii;

	for(ii = 1 ; ii <= le ;){
		ch2[le - ii] = no % 10 + 0x30;
		no /= 10;
		ii += 1;
	}
	ch2[le] = '\0';
	Print(xx ,yy ,ch2 ,yn);
}



/*******************************************************************************
//函数名：void Print6(uint8 xx, uint8 yy, uint8 ch1[], uint8 yn)
//功能：显示6*8字符串
//输入：xx ,yy 坐标,ch1待显示的字符串,yn是否反黑
//输出：无
********************************************************************************/
void Print6(uint8 xx, uint8 yy, uint8 ch1[], uint8 yn)		
{
	uint8 ii = 0;
	uint8 bb = 0;
	unsigned int index = 0 ;	

	SetRamAddr(xx , yy);		
	while(ch1[bb] != '\0')
	{
                index = (unsigned int)(ch1[bb] - 0x20);
		index = (unsigned int)index*6;		
		for(ii=0;ii<6;ii++)
		{
			if(yn)
			{
				Lcdwritedata(FontSystem6x8[index]);
			}
			else
			{
				Lcdwritedata(~FontSystem6x8[index]);
			}		
			index += 1;
		}		
		bb += 1;
	}
}


/*******************************************************************************
//函数名：void Print8(uint16 y,uint16 x, uint8 ch[],uint16 yn)
//功能：显示8*8字符串
//输入：xx ,yy 坐标,ch1待显示的字符串,yn是否反黑
//输出：无
********************************************************************************/
void Print8(uint16 y,uint16 x, uint8 ch[],uint16 yn)
{
	uint8 wm ,ii = 0;
	uint16 adder;

	while(ch[ii] != '\0')
	{
		adder = (ch[ii] - 0x20) * 16;

		SetRamAddr(y , x);
		for(wm = 0;wm < 8;wm++)
		{
			if(yn == 0)
			{
				Lcdwritedata(~Font8X8[adder]);
			}
			else
			{
				Lcdwritedata(Font8X8[adder]);
			}
			adder += 1;
		}
		SetRamAddr(y + 1 , x);
		for(wm = 0;wm < 8;wm++)
		{
			if(yn == 0)
			{
				Lcdwritedata(~Font8X8[adder]);	
			}
			else
			{
				Lcdwritedata(Font8X8[adder]);	
			}
			adder += 1;
		}
		ii += 1;
		x += 8;
	}

}


/*******************************************************************************
//函数名：void Print16(uint16 y,uint16 x,uint8 ch[],uint16 yn)
//功能：在屏幕上显示汉字
//输入：x ,y 坐标,ch[]待显示的汉字,yn是否反黑
//输出：无
********************************************************************************/
void Print16(uint16 y,uint16 x,uint8 ch[],uint16 yn)
{
	uint8 wm ,ii = 0;
	uint16 adder;

	wm = 0;
	adder = 1;
	while(FontNew8X16_Index[wm] > 100)
	{
		if(FontNew8X16_Index[wm] == ch[ii])
		{
			if(FontNew8X16_Index[wm + 1] == ch[ii + 1])
			{
				adder = wm * 14;
				break;
			}
		}
		wm += 2;				//找到汉字在索引中的位置
	}
	SetRamAddr(y , x);

	if(adder != 1)					//找到汉字，显示出来	
	{
		SetRamAddr(y , x);
		for(wm = 0;wm < 14;wm++)
		{
			if(yn == 0)
			{
				Lcdwritedata(~FontNew16X16[adder]);
			}
			else
			{
				Lcdwritedata(FontNew16X16[adder]);
			}
			adder += 1;
		}
                for(wm = 0;wm < 2;wm++)
		{
			if(yn == 0)
			{
				Lcdwritedata(0xff);
			}
			else
			{
				Lcdwritedata(0x00);
			}
		}
		SetRamAddr(y + 1 , x);

		for(wm = 0;wm < 14;wm++)
		{
			if(yn == 0)
			{
				Lcdwritedata(~FontNew16X16[adder]);
			}
			else
			{
				Lcdwritedata(FontNew16X16[adder]);
			}
			adder += 1;
		}
                for(wm = 0;wm < 2;wm++)
		{
			if(yn == 0)
			{
				Lcdwritedata(0xff);
			}
			else
			{
				Lcdwritedata(0x00);
			}
		}


	}
	else						//找不到字显示空格			
	{
		ii += 1;SetRamAddr(y , x);
		for(wm = 0;wm < 16;wm++)
		{
			if(yn == 0)
			{
				Lcdwritedata(0xff);
			}
			else
			{
				Lcdwritedata(0x00);
			}
		}
		SetRamAddr(y + 1 , x);
		for(wm = 0;wm < 16;wm++)
		{
			if(yn == 0)
			{
				Lcdwritedata(0xff);
			}
			else
			{
				Lcdwritedata(0x00);
			}
		}
	}
}
/*******************************************************************************
//函数名：void Print(uint8 y, uint8 x, uint8 ch[], uint16 yn)
//功能：实现汉字及字母混合显示
//输入：x ,y 坐标,ch[]待显示的汉字或字母,yn是否反黑
//输出：无
********************************************************************************/
void Print(uint8 y, uint8 x, uint8 ch[], uint16 yn)
{
	uint8 ch2[3];
	uint8 ii;
        ii = 0;
	while(ch[ii] != '\0')
	{
		if(ch[ii] > 120)
		{
			ch2[0] = ch[ii];
	 		ch2[1] = ch[ii + 1];
			ch2[2] = '\0';			//汉字为两个字节
			Print16(y , x , ch2 , yn);	//显示汉字
			x += 16;
			ii += 2;
		}
		else
		{
			ch2[0] = ch[ii];	
			ch2[1] = '\0';			//字母占一个字节
			Print8(y , x , ch2 , yn);	//显示字母
			x += 8;
			ii += 1;
		}
	}
}


/*******************************************************************************
//函数名：void ClearCol(uint8 Begin , uint8 End)
//功能：清除列
//输入：Begin开始处   End结束处
//输出：无
********************************************************************************
void ClearCol(uint8 Begin , uint8 End)
{
	uint8 i;
	for(i=Begin;i <= End ; i++)
	{
		Print6(Begin , 0 ,"                   ", 1);
	}
}*/

/*******************************************************************************
//函数名：void Rectangle(uint8 x1,uint8 y1,uint8 x2,uint8 y2)
//功能：画直线函数，本函数目前只能画水平和垂直线
//输入：x1,y1(第一个点)   x2,y2第二个点
//输出：无
********************************************************************************/
void Rectangle(uint8 x1,uint8 y1,uint8 x2,uint8 y2)
{

	uint8 ii;

	for(ii=x1; ii<x2; ii++)
	{		
		SetRamAddr(y1,ii);
		Lcdwritedata(0x08);
		SetRamAddr(y2,ii);
		Lcdwritedata(0x08);		//画横线
	}
	SetRamAddr(y1,x1);
	Lcdwritedata(0xF0);
	SetRamAddr(y1,x2);
	Lcdwritedata(0xF0);

	for(ii = y1+1;ii<y2;ii++)
	{		
	 	SetRamAddr(ii,x1);
		Lcdwritedata(0xff);
	 	SetRamAddr(ii,x2);
		Lcdwritedata(0xff);		//画竖线
	}

	SetRamAddr(y2,x1);
	Lcdwritedata(0x0F);
	SetRamAddr(y2,x2);
	Lcdwritedata(0x0F);
}


/*******************************************************************************
//函数名：void Rectangle_x(uint8 x1,uint8 y1,uint8 x2,uint8 y2)
//功能：画直线函数，本函数目前只能画水平和垂直线
//输入：x1,y1(第一个点)   x2,y2第二个点
//输出：无
********************************************************************************/
void Rectangle_x(uint8 x1,uint8 y1,uint8 x2,uint8 y2)
{

	uint8 ii;

	for(ii=x1; ii<x2; ii++)
	{		
		SetRamAddr(y1,ii);
		Lcdwritedata(0x80);
		SetRamAddr(y2,ii);
		Lcdwritedata(0x01);		//画横线
	}
	SetRamAddr(y1,x1);
	Lcdwritedata(0x80);
	SetRamAddr(y1,x2);
	Lcdwritedata(0x80);

	for(ii = y1+1;ii<y2;ii++)
	{		
	 	SetRamAddr(ii,x1);
		Lcdwritedata(0xff);
	 	SetRamAddr(ii,x2);
		Lcdwritedata(0xff);		//画竖线
	}

	SetRamAddr(y2,x1);
	Lcdwritedata(0x01);
	SetRamAddr(y2,x2);
	Lcdwritedata(0x01);
}




//*********************************************************************************
//函数名:void TurnOnDisp(void)
//功能:开机画面显示
//输入：无
//输出：无
//*********************************************************************************
void TurnOnDisp(void)
{
	Print(0, 25, "成都无线龙", 1);
	Print(5, 17, "WXL-WSN-V3.0", 1);
	Rectangle(0, 2, 127, 7);
}


//*********************************************************************************
//功能:画面显示
//输入：mode   -0x1: 搜索
//             -0x11:  发现路由节点
//             -0x12:  发现终端节点
//输出：无
//*********************************************************************************
void TurnShowInterface(uint8 mode)
{
        if(mode==0x1){
                Print(0, 25, "Zigbee测试", 1);
                Print(2, 30, "搜索节点", 1);
                Print(5, 5, ">>>>>>>>>>>>>>>>>", 1);
	        Rectangle(0, 4, 127, 7);
        }
        else if(mode & 0x10){
                  Print(0, 25, "Zigbee测试", 1);
                if(mode == 0x11){
                  Print(3, 15, "发现路由节点", 1);
                }
                else if(mode == 0x12){
                  Print(3, 15, "发现终端节点", 1);
                }
                
        }
}

