#include <string.h>
#include <Font.h>
#include <Lcd128X64.h>


uint8 ContrastValue = 0x20;//�Ҷȵ���

void ClearScreen(void);
/*******************************************************************************
//��������void delaylcd (uint16 x)
//���ܣ�͢ʱ
//���룺ʱ��
//�������
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
//��������void Lcdwritecom(uint8 com)
//���ܣ�lcdдָ��
//���룺comָ��
//�������
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
//��������void Lcdwritedata(uint8 dat)
//���ܣ�lcdд����
//���룺dat����
//�������
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
//��������void Prog_Reset(void)
//���ܣ�lcd��λ
//���룺��
//�������
********************************************************************************/
void Prog_Reset(void)
{
	LCD_RESET =  LOW;
	delaylcd(100);
	LCD_RESET =  HIGH;
}
/*******************************************************************************
//��������void Resetchip(void)
//���ܣ�lcd�����λ
//���룺��
//�������
********************************************************************************/
void Resetchip(void)
{
	Prog_Reset();
}

/*******************************************************************************
//��������void SetRamAddr (uint8 Page, uint8 Col)
//���ܣ�lcdλ��ѡ��
//���룺Page-ҳ��Col-��
//�������
********************************************************************************/
void SetRamAddr (uint8 Page, uint8 Col)
{
	Lcdwritecom(0xB0 + Page);
	Lcdwritecom(Col & 0x0f); //Set lower column address
	Lcdwritecom(0x10 | ((Col & 0xf0) >> 4)); //Set higher column address
}



/*******************************************************************************
//��������void SetContrast(uint8 Gain, uint8 Step)
//���ܣ�lcd�Աȶ��趨
//���룺Page-ҳ��Col-��
//�������
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
//��������void InitLcd(void)
//���ܣ�lcd��ʼ��
//���룺��
//�������
********************************************************************************/
void InitLcd(void)
{
	LcdPortInit();
        LCD_POWER =  LOW;    //OLED POWER ON     //����
        
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
//��������void ClearScreen(void)
//���ܣ�����
//���룺��
//�������
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
//��������void Printn(uint8 xx ,uint8 yy , uint32 no,uint8 yn,uint8 le)
//���ܣ���ʾһ��6*8�޷�������
//���룺xx , yy��Ļ����λ��,no����ʾ���� yn=0������ʾ yn=1������ʾ  le��Чλ
//�������
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
//��������void Printn8(uint8 xx ,uint8 yy , uint32 no,uint8 yn,uint8 le)
//���ܣ���ʾ8*8һ���޷�������
//���룺xx , yy��Ļ����λ��,no����ʾ���� yn=0������ʾ yn=1������ʾ  le��Чλ
//�������
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
//��������void Print6(uint8 xx, uint8 yy, uint8 ch1[], uint8 yn)
//���ܣ���ʾ6*8�ַ���
//���룺xx ,yy ����,ch1����ʾ���ַ���,yn�Ƿ񷴺�
//�������
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
//��������void Print8(uint16 y,uint16 x, uint8 ch[],uint16 yn)
//���ܣ���ʾ8*8�ַ���
//���룺xx ,yy ����,ch1����ʾ���ַ���,yn�Ƿ񷴺�
//�������
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
//��������void Print16(uint16 y,uint16 x,uint8 ch[],uint16 yn)
//���ܣ�����Ļ����ʾ����
//���룺x ,y ����,ch[]����ʾ�ĺ���,yn�Ƿ񷴺�
//�������
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
		wm += 2;				//�ҵ������������е�λ��
	}
	SetRamAddr(y , x);

	if(adder != 1)					//�ҵ����֣���ʾ����	
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
	else						//�Ҳ�������ʾ�ո�			
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
//��������void Print(uint8 y, uint8 x, uint8 ch[], uint16 yn)
//���ܣ�ʵ�ֺ��ּ���ĸ�����ʾ
//���룺x ,y ����,ch[]����ʾ�ĺ��ֻ���ĸ,yn�Ƿ񷴺�
//�������
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
			ch2[2] = '\0';			//����Ϊ�����ֽ�
			Print16(y , x , ch2 , yn);	//��ʾ����
			x += 16;
			ii += 2;
		}
		else
		{
			ch2[0] = ch[ii];	
			ch2[1] = '\0';			//��ĸռһ���ֽ�
			Print8(y , x , ch2 , yn);	//��ʾ��ĸ
			x += 8;
			ii += 1;
		}
	}
}


/*******************************************************************************
//��������void ClearCol(uint8 Begin , uint8 End)
//���ܣ������
//���룺Begin��ʼ��   End������
//�������
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
//��������void Rectangle(uint8 x1,uint8 y1,uint8 x2,uint8 y2)
//���ܣ���ֱ�ߺ�����������Ŀǰֻ�ܻ�ˮƽ�ʹ�ֱ��
//���룺x1,y1(��һ����)   x2,y2�ڶ�����
//�������
********************************************************************************/
void Rectangle(uint8 x1,uint8 y1,uint8 x2,uint8 y2)
{

	uint8 ii;

	for(ii=x1; ii<x2; ii++)
	{		
		SetRamAddr(y1,ii);
		Lcdwritedata(0x08);
		SetRamAddr(y2,ii);
		Lcdwritedata(0x08);		//������
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
		Lcdwritedata(0xff);		//������
	}

	SetRamAddr(y2,x1);
	Lcdwritedata(0x0F);
	SetRamAddr(y2,x2);
	Lcdwritedata(0x0F);
}


/*******************************************************************************
//��������void Rectangle_x(uint8 x1,uint8 y1,uint8 x2,uint8 y2)
//���ܣ���ֱ�ߺ�����������Ŀǰֻ�ܻ�ˮƽ�ʹ�ֱ��
//���룺x1,y1(��һ����)   x2,y2�ڶ�����
//�������
********************************************************************************/
void Rectangle_x(uint8 x1,uint8 y1,uint8 x2,uint8 y2)
{

	uint8 ii;

	for(ii=x1; ii<x2; ii++)
	{		
		SetRamAddr(y1,ii);
		Lcdwritedata(0x80);
		SetRamAddr(y2,ii);
		Lcdwritedata(0x01);		//������
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
		Lcdwritedata(0xff);		//������
	}

	SetRamAddr(y2,x1);
	Lcdwritedata(0x01);
	SetRamAddr(y2,x2);
	Lcdwritedata(0x01);
}




//*********************************************************************************
//������:void TurnOnDisp(void)
//����:����������ʾ
//���룺��
//�������
//*********************************************************************************
void TurnOnDisp(void)
{
	Print(0, 25, "�ɶ�������", 1);
	Print(5, 17, "WXL-WSN-V3.0", 1);
	Rectangle(0, 2, 127, 7);
}


//*********************************************************************************
//����:������ʾ
//���룺mode   -0x1: ����
//             -0x11:  ����·�ɽڵ�
//             -0x12:  �����ն˽ڵ�
//�������
//*********************************************************************************
void TurnShowInterface(uint8 mode)
{
        if(mode==0x1){
                Print(0, 25, "Zigbee����", 1);
                Print(2, 30, "�����ڵ�", 1);
                Print(5, 5, ">>>>>>>>>>>>>>>>>", 1);
	        Rectangle(0, 4, 127, 7);
        }
        else if(mode & 0x10){
                  Print(0, 25, "Zigbee����", 1);
                if(mode == 0x11){
                  Print(3, 15, "����·�ɽڵ�", 1);
                }
                else if(mode == 0x12){
                  Print(3, 15, "�����ն˽ڵ�", 1);
                }
                
        }
}

