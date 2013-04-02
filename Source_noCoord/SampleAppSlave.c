/*********************************************************************
  Filename:       SampleApp.c
  Revised:        $Date: 2007-05-31 15:56:04 -0700 (Thu, 31 May 2007) $
  Revision:       $Revision: 14490 $

  Description:
				  - Sample Application (no Profile).
				
          This application isn't intended to do anything useful,
          it is intended to be a simple example of an application's
          structure.

          This application sends it's messages either as broadcast or
          broadcast filtered group messages.  The other (more normal)
          message addressing is unicast.  Most of the other
          sample applications are written to support the unicast
          message model.

          Key control:
            SW1:  Sends a flash command to all devices in Group 1.
            SW2:  Adds/Removes (toggles) this device in and out
                  of Group 1.  This will enable and disable the
                  reception of the flash command.

  Notes:

  Copyright (c) 2007 by Texas Instruments, Inc.
  All Rights Reserved.  Permission to use, reproduce, copy, prepare
  derivative works, modify, distribute, perform, display or sell this
  software and/or its documentation for any purpose is prohibited
  without the express written consent of Texas Instruments, Inc.
*********************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include "OSAL.h"
#include "ZGlobals.h"
#include "AF.h"
#include "aps_groups.h"
#include "ZDApp.h"

#include "SampleApp.h"
#include "SampleAppHw.h"

#include "OnBoard.h"
#include "Assoclist.h"

/* HAL */
#include "hal_lcd.h"
#include "hal_led.h"
#include "hal_key.h"
#include "sensor.h"
#ifdef   EXSENSOR
#include "exsensor.h"
#endif
#include "string.h"
#include "wsn.h"
#include "hal_adc.h"
#include "SPIMgr.h"
/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * TYPEDEFS
 */
#define			IDLE		0x00
#define			RAS_ADD		0X01
#define			RAD_ADC		0x10
#define			CKG_KG		0x20




/*********************************************************************
 * GLOBAL VARIABLES
 */

// This list should be filled with Application specific Cluster IDs.
const cId_t SampleApp_ClusterList[SAMPLEAPP_MAX_CLUSTERS] =
{
  SAMPLEAPP_PERIODIC_CLUSTERID,
  SAMPLEAPP_FLASH_CLUSTERID
};

const SimpleDescriptionFormat_t SampleApp_SimpleDesc =
{
  SAMPLEAPP_ENDPOINT,              //  int Endpoint;
  SAMPLEAPP_PROFID,                //  uint16 AppProfId[2];
  SAMPLEAPP_DEVICEID,              //  uint16 AppDeviceId[2];
  SAMPLEAPP_DEVICE_VERSION,        //  int   AppDevVer:4;
  SAMPLEAPP_FLAGS,                 //  int   AppFlags:4;
  SAMPLEAPP_MAX_CLUSTERS,          //  uint8  AppNumInClusters;
  (cId_t *)SampleApp_ClusterList,  //  uint8 *pAppInClusterList;
  SAMPLEAPP_MAX_CLUSTERS,          //  uint8  AppNumInClusters;
  (cId_t *)SampleApp_ClusterList   //  uint8 *pAppInClusterList;
};

// This is the Endpoint/Interface description.  It is defined here, but
// filled-in in SampleApp_Init().  Another way to go would be to fill
// in the structure here and make it a "const" (in code space).  The
// way it's defined in this sample app it is define in RAM.
endPointDesc_t SampleApp_epDesc;

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
uint8 SampleApp_TaskID;   // Task ID for internal task/event processing
                          // This variable will be received when
                          // SampleApp_Init() is called.
devStates_t SampleApp_NwkState;

uint8 SampleApp_TransID;  // This is the unique message ID (counter)

afAddrType_t SampleApp_Periodic_DstAddr;
afAddrType_t SampleApp_Flash_DstAddr;

aps_Group_t SampleApp_Group;

uint8 SampleAppPeriodicCounter = 0;
uint8 SampleAppFlashCounter = 0;

extern struct Ver
{
	UINT8 Flag;
	UINT8 GM[8];
  	UINT8 WD[8];
}WsnVer;
/*
union i
{
	UINT8 TxBuf[22];
	struct UARTBUF2
	{
  		UINT8 Head;
		UINT8 Com[3]; //命令头
		UINT8 DataBuf[16];  //数据缓冲区
		UINT8 CRC;    		//校验位
		UINT8 Tail;
	}TXDATA;
}UartTxBuf;//从串口接收到的数据帧
*/
/*
union h
{
	UINT8 RxBuf[22];
	struct UARTBUF
	{
  		UINT8 Head;
		UINT8 Com[3]; //命令头
		UINT8 DataBuf[16];  //数据缓冲区
		UINT8 CRC;    		//校验位
		UINT8 Tail;
	}RXDATA;
}UartRxBuf;//从串口接收到的数据帧
*/



//UINT8 UartState;	//串口状态


union f{
  UINT8 RxBuf[29];
  struct RFRXBUF
  {
        UINT8 HeadCom[3]; //命令头
        //UINT8 Format[2];//格式
        UINT8 Laddr[8];
        UINT16 Saddr;
        //UINT8 TempAddr[10];//备用地址
        UINT8 DataBuf[16];  //数据缓冲区
        //UINT8 TempDataBuf[8];//备用数据
  }RXDATA;
}RfRece;//无线接收缓冲区


union e{
  UINT8 TxBuf[29];
  struct RFTXBUF
  {
        UINT8 HeadCom[3]; //命令头
        //UINT8 Format[2];//格式
        UINT8 Laddr[8];
        UINT16 Saddr;
        //UINT8 TempAddr[10];//备用地址
        UINT8 DataBuf[16];  //数据缓冲区
        //UINT8 TempDataBuf[8];//备用数据
  }TXDATA;
}RfTx;//无线发送缓冲区

UINT8 RfRxNewData;//无线有新的数据收到
UINT8 RfHaveTxDara;//无线有数据要发送


UINT16 LedCycle1;//LED1的闪烁周期
UINT16 LedCycle2;//LED2的闪烁周期
UINT8 Led1Flag;//LED1闪烁标致
UINT8 Led2Flag;//LED2闪烁标致

UINT16 EXSEN;

/*********************************************************************
 * LOCAL FUNCTIONS
 */
void SampleApp_HandleKeys( uint8 shift, uint8 keys );
void SampleApp_MessageMSGCB( afIncomingMSGPacket_t *pckt );
void SampleApp_SendPeriodicMessage( void );
void SampleApp_SendFlashMessage( uint16 flashTime );
UINT16 StrToNum16(UINT8 *buf, UINT8 n);
UINT8 SendData(UINT8 *buf, UINT16 addr, UINT8 Leng);
void halPutch(char c);
void Sensor_Delay(UINT16);
/*********************************************************************
 * NETWORK LAYER CALLBACKS
 */






/*********************************************************************
 * PUBLIC FUNCTIONS
 */






/*


void halPutch(char c)
{
	EA = 0;
	UTX0IF = 0;
	U0DBUF = c;
	while (!UTX0IF);
	UTX0IF = 0;
	EA = 1;
}


*/


void Sensor_Delay(UINT16 t)
{
  UINT8 i;
  while(t)
  {
   i=0xff;
   t--;
   while(i)
   {
     i--;
   }
  }
}


UINT16 StrToNum16(UINT8 *buf, UINT8 n)
{
	UINT16 num = 0;
	
	while(n--)
	{
		num = num*10 + ((*buf)-0X30);
		buf++;
	}
	return num;
}


//-------------------------------------------------------------------------
//发送一组数据
//-------------------------------------------------------------------------
UINT8 SendData(UINT8 *buf, UINT16 addr, UINT8 Leng)
{
	afAddrType_t SendDataAddr;
	
	SendDataAddr.addrMode = (afAddrMode_t)Addr16Bit;
	SendDataAddr.endPoint = SAMPLEAPP_ENDPOINT;
	SendDataAddr.addr.shortAddr = addr;
        if ( AF_DataRequest( &SendDataAddr, &SampleApp_epDesc,
                       SAMPLEAPP_PERIODIC_CLUSTERID,
                       Leng,
                       buf,
                       &SampleApp_TransID,
                       AF_DISCV_ROUTE,
                       AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
	{
		return 1;
	}
	else
	{
		return 0;// Error occurred in request to send.
	}
}

/*********************************************************************
 * @fn      SampleApp_Init
 *
 * @brief   Initialization function for the Generic App Task.
 *          This is called during initialization and should contain
 *          any application specific initialization (ie. hardware
 *          initialization/setup, table initialization, power up
 *          notificaiton ... ).
 *
 * @param   task_id - the ID assigned by OSAL.  This ID should be
 *                    used to send messages and set timers.
 *
 * @return  none
 */
void SampleApp_Init( uint8 task_id )
{
  SampleApp_TaskID = task_id;
  SampleApp_NwkState = DEV_INIT;
  SampleApp_TransID = 0;

  // Device hardware initialization can be added here or in main() (Zmain.c).
  // If the hardware is application specific - add it here.
  // If the hardware is other parts of the device add it in main().

 #if defined ( SOFT_START )
  // The "Demo" target is setup to have SOFT_START and HOLD_AUTO_START
  // SOFT_START is a compile option that allows the device to start
  //  as a coordinator if one isn't found.
#ifdef  WXL_ROUTER
    zgDeviceLogicalType = ZG_DEVICETYPE_ROUTER;
#endif

#ifdef  WXL_RFD
    zgDeviceLogicalType = ZG_DEVICETYPE_ENDDEVICE;
#endif

#endif // SOFT_START

#if defined ( HOLD_AUTO_START )
  // HOLD_AUTO_START is a compile option that will surpress ZDApp
  //  from starting the device and wait for the application to
  //  start the device.
  ZDOInitDevice(0);
#endif

  // Setup for the periodic message's destination address
  // Broadcast to everyone
  SampleApp_Periodic_DstAddr.addrMode = (afAddrMode_t)AddrBroadcast;
  SampleApp_Periodic_DstAddr.endPoint = SAMPLEAPP_ENDPOINT;
  SampleApp_Periodic_DstAddr.addr.shortAddr = 0xFFFF;

  // Setup for the flash command's destination address - Group 1
  SampleApp_Flash_DstAddr.addrMode = (afAddrMode_t)afAddrGroup;
  SampleApp_Flash_DstAddr.endPoint = SAMPLEAPP_ENDPOINT;
  SampleApp_Flash_DstAddr.addr.shortAddr = SAMPLEAPP_FLASH_GROUP;

  // Fill out the endpoint description.
  SampleApp_epDesc.endPoint = SAMPLEAPP_ENDPOINT;
  SampleApp_epDesc.task_id = &SampleApp_TaskID;
  SampleApp_epDesc.simpleDesc
            = (SimpleDescriptionFormat_t *)&SampleApp_SimpleDesc;
  SampleApp_epDesc.latencyReq = noLatencyReqs;

  // Register the endpoint description with the AF
  afRegister( &SampleApp_epDesc );

  // Register for all key events - This app will handle all key events
  RegisterForKeys( SampleApp_TaskID );

  SPIMgr_RegisterTaskID(SampleApp_TaskID);

  // By default, all devices start out in Group 1
  SampleApp_Group.ID = 0x0001;
  osal_memcpy( SampleApp_Group.name, "Group 1", 7  );
  aps_AddGroup( SAMPLEAPP_ENDPOINT, &SampleApp_Group );

}

/*********************************************************************
 * @fn      SampleApp_ProcessEvent
 *
 * @brief   Generic Application Task event processor.  This function
 *          is called to process all events for the task.  Events
 *          include timers, messages and any other user defined events.
 *
 * @param   task_id  - The OSAL assigned task ID.
 * @param   events - events to process.  This is a bit map and can
 *                   contain more than one event.
 *
 * @return  none
 */
//afIncomingMSGPacket_t *sight;

#ifdef  EXSENSOR
extern unsigned char DA300STA;    //判断是否插上DA300的标志
#endif
//afIncomingMSGPacket_t *GBMSG;
uint16 SampleApp_ProcessEvent( uint8 task_id, uint16 events )
{

	uint8 *ieeeAddr;
	afIncomingMSGPacket_t *MSGpkt;
        unsigned int temp1;
	if ( events & SYS_EVENT_MSG )
	{
		MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( SampleApp_TaskID );
                //GBMSG=MSGpkt;	
                while ( MSGpkt )
		{
//                  sight = MSGpkt;
			switch ( MSGpkt->hdr.event )
			{
				// Received when a key is pressed
//				case KEY_CHANGE:
//					SampleApp_HandleKeys( ((keyChange_t *)MSGpkt)->state, ((keyChange_t *)MSGpkt)->keys );
//				break;
				
				case AF_INCOMING_MSG_CMD:
					SampleApp_MessageMSGCB( MSGpkt );
				break;
//		case SPI_INCOMING_ZTOOL_PORT:                                       	
//                                break;

				// Received whenever the device changes state in the network
				case ZDO_STATE_CHANGE:
					SampleApp_NwkState = (devStates_t)(MSGpkt->hdr.status);
                                        LED2 = 1;
                                        LED1 = 1;

					if ( (SampleApp_NwkState == DEV_ZB_COORD)
					|| (SampleApp_NwkState == DEV_ROUTER)
					|| (SampleApp_NwkState == DEV_END_DEVICE) )
					{
#ifdef	WXL_RFD
						memset(RfTx.TxBuf,'x',29);
						RfTx.TXDATA.HeadCom[0] = 'J';
						RfTx.TXDATA.HeadCom[1] = 'O';
						RfTx.TXDATA.HeadCom[2] = 'N';
						ieeeAddr = NLME_GetExtAddr();
                                                memcpy(RfTx.TXDATA.Laddr,ieeeAddr,8);
                                                RfTx.TXDATA.Saddr = NLME_GetShortAddr();
						RfTx.TXDATA.DataBuf[0] = 'R';
						RfTx.TXDATA.DataBuf[1] = 'F';
						RfTx.TXDATA.DataBuf[2] = 'D';

                                              NLME_GetCoordExtAddr(&RfTx.TXDATA.DataBuf[5]);
                                              temp1 = NLME_GetCoordShortAddr();
                                              RfTx.TXDATA.DataBuf[13] = (unsigned char)(temp1>>8);
                                              RfTx.TXDATA.DataBuf[14] = (unsigned char)(temp1);

                                              
                                             //   RfTx.TxBuf[47] = (INT8U)(wxlparentaddr>>8);
                                              //  RfTx.TxBuf[48] = (INT8U)(wxlparentaddr);
                                               //AssocMatchDeviceStatus( DEV_LINK_STATUS ); //wxlstate =
					//	RfTx.TxBuf[47] = (INT8U)(req.dstAddr);//MSGpkt->parentAddr;//(INT8U)(device->parentAddr>>8);
						//RfTx.TxBuf[48] = (INT8U)(device->parentAddr);
						SendData(RfTx.TxBuf, 0x0000, 29);
#endif
						
#ifdef	WXL_ROUTER
						memset(RfTx.TxBuf,'x',29);
						RfTx.TXDATA.HeadCom[0] = 'J';
						RfTx.TXDATA.HeadCom[1] = 'O';
						RfTx.TXDATA.HeadCom[2] = 'N';
                                                ieeeAddr = NLME_GetExtAddr();
                                                memcpy(RfTx.TXDATA.Laddr,ieeeAddr,8);
                                                RfTx.TXDATA.Saddr = NLME_GetShortAddr();
						RfTx.TXDATA.DataBuf[0] = 'R';
						RfTx.TXDATA.DataBuf[1] = 'O';
						RfTx.TXDATA.DataBuf[2] = 'U';
                                            NLME_GetCoordExtAddr(&RfTx.TXDATA.DataBuf[5]);
                                            temp1 = NLME_GetCoordShortAddr();
                                            RfTx.TXDATA.DataBuf[13] = (unsigned char)(temp1>>8);
                                            RfTx.TXDATA.DataBuf[14] = (unsigned char)(temp1);
						SendData(RfTx.TxBuf, 0x0000, 29);
#endif
//#ifdef  EXSENSOR
                                             DA300STA = 0;
                                             if (ReadSensorAdc(XOUT)>10)     //判断DA300插上
                                             {
                                               DA300STA = 1;
                                             }
//#endif
                                                // Start sending the periodic message in a regular interval.
						osal_start_timerEx( SampleApp_TaskID,
						SAMPLEAPP_SEND_PERIODIC_MSG_EVT,
						SAMPLEAPP_SEND_PERIODIC_MSG_TIMEOUT );
					}
					else
					{
						// Device is no longer in the network
					}
				break;
				
				default:
				break;
			}
		
			// Release the memory
			osal_msg_deallocate( (uint8 *)MSGpkt );
			
			// Next - if one is available
			MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( SampleApp_TaskID );
		}
		
		// return unprocessed events
		return (events ^ SYS_EVENT_MSG);
	}
		
	// Send a message out - This event is generated by a timer
	//  (setup in SampleApp_Init()).
	if ( events & SAMPLEAPP_SEND_PERIODIC_MSG_EVT )
	{
                // Send the periodic message
                //SampleApp_SendPeriodicMessage();

		// Setup to send message again in normal period (+ a little jitter)
		//osal_start_timerEx( SampleApp_TaskID, SAMPLEAPP_SEND_PERIODIC_MSG_EVT,
		//(SAMPLEAPP_SEND_PERIODIC_MSG_TIMEOUT + (osal_rand() & 0x00FF)) );
			
		// return unprocessed events
		return (events ^ SAMPLEAPP_SEND_PERIODIC_MSG_EVT);
	}
	
	// Discard unknown events
	return 0;

}

/*********************************************************************
 * Event Generation Functions
 */
/*********************************************************************
 * @fn      SampleApp_HandleKeys
 *
 * @brief   Handles all key events for this device.
 *
 * @param   shift - true if in shift/alt.
 * @param   keys - bit field for key events. Valid entries:
 *                 HAL_KEY_SW_2
 *                 HAL_KEY_SW_1
 *
 * @return  none
 */
#ifdef BORD_KEY
void SampleApp_HandleKeys( uint8 shift, uint8 keys )
{
  if ( keys & HAL_KEY_SW_1 )
  {
    /* This key sends the Flash Command is sent to Group 1.
     * This device will not receive the Flash Command from this
     * device (even if it belongs to group 1).
     */
    SampleApp_SendFlashMessage( SAMPLEAPP_FLASH_DURATION );
  }

  if ( keys & HAL_KEY_SW_2 )
  {
    /* The Flashr Command is sent to Group 1.
     * This key toggles this device in and out of group 1.
     * If this device doesn't belong to group 1, this application
     * will not receive the Flash command sent to group 1.
     */
    aps_Group_t *grp;
    grp = aps_FindGroup( SAMPLEAPP_ENDPOINT, SAMPLEAPP_FLASH_GROUP );
    if ( grp )
    {
      // Remove from the group
      aps_RemoveGroup( SAMPLEAPP_ENDPOINT, SAMPLEAPP_FLASH_GROUP );
    }
    else
    {
      // Add to the flash group
      aps_AddGroup( SAMPLEAPP_ENDPOINT, &SampleApp_Group );
    }
  }
}

#endif
/*********************************************************************
 * LOCAL FUNCTIONS
 */

/*********************************************************************
 * @fn      SampleApp_MessageMSGCB
 *
 * @brief   Data message processor callback.  This function processes
 *          any incoming data - probably from other devices.  So, based
 *          on cluster ID, perform the intended action.
 *
 * @param   none
 *
 * @return  none
 */

const unsigned char __code HEXcode[16]="0123456789ABCDEF";


void SampleApp_MessageMSGCB( afIncomingMSGPacket_t *pkt )
{
	UINT8 temp;
	UINT16 temp1;
	uint8 *ieeeAddr;
       // uint8 *parantIEEEAddr;
	ieeeAddr = NLME_GetExtAddr();
  	memcpy(RfRece.RxBuf,pkt->cmd.Data,29);
	switch(RfRece.RXDATA.HeadCom[0]){
				
		case 'R'://读
			if((RfRece.RXDATA.HeadCom[1] == 'A') && (RfRece.RXDATA.HeadCom[2] == 'S'))//读传感器
			{
				if((RfRece.RXDATA.DataBuf[0] == 'G') && (RfRece.RXDATA.DataBuf[1] == 'M'))//读光敏
				{

					memset(RfTx.TxBuf,'x',29);
					RfTx.TXDATA.HeadCom[0] = 'R';
					RfTx.TXDATA.HeadCom[1] = 'A';
					RfTx.TXDATA.HeadCom[2] = 'S';
					
					
                                        memcpy(RfTx.TXDATA.Laddr,ieeeAddr,8);
					RfTx.TXDATA.Saddr = NLME_GetShortAddr();
					RfTx.TXDATA.DataBuf[0] = 'G';
					RfTx.TXDATA.DataBuf[1] = 'M';
                                        #ifdef POWER_SAVING
                                        //temp = ReadSensorAdc(VBAT);
                                        Sensor_Delay(3000);
                                        #endif
					temp = ReadSensorAdc(VPHOTO);
					RfTx.TXDATA.DataBuf[2] = temp/100 + 0x30;
					temp = temp%100;
					RfTx.TXDATA.DataBuf[3] = temp/10 + 0x30;
					RfTx.TXDATA.DataBuf[4] = temp%10 + 0x30;
					
					RfHaveTxDara = 1;;
				}
				else if((RfRece.RXDATA.DataBuf[0] == 'W') && (RfRece.RXDATA.DataBuf[1] == 'D'))//读温度
				{
					memset(RfTx.TxBuf,'x',29);

                                        WriteTc77(1);
                                        Sensor_Delay(5000);
                                        temp = ReadTc77();
                                        WriteTc77(0);
					
                                        RfTx.TXDATA.HeadCom[0] = 'R';
					RfTx.TXDATA.HeadCom[1] = 'A';
					RfTx.TXDATA.HeadCom[2] = 'S';				
					memcpy(RfTx.TXDATA.Laddr,ieeeAddr,8);
					RfTx.TXDATA.Saddr = NLME_GetShortAddr();
					RfTx.TXDATA.DataBuf[0] = 'W';
					RfTx.TXDATA.DataBuf[1] = 'D';
					RfTx.TXDATA.DataBuf[2] = temp/10 + 0x30;//HEXcode[(temp>>4)];//
					RfTx.TXDATA.DataBuf[3] = temp%10 + 0x30;//HEXcode[(temp&0x0f)];//
					
					RfHaveTxDara = 1;
				}
				else if((RfRece.RXDATA.DataBuf[0] == 'K') && (RfRece.RXDATA.DataBuf[1] == 'T') )//读可调电位器
				{
					memset(RfTx.TxBuf,'x',29);
					RfTx.TXDATA.HeadCom[0] = 'R';
					RfTx.TXDATA.HeadCom[1] = 'A';
					RfTx.TXDATA.HeadCom[2] = 'S';

                                        memcpy(RfTx.TXDATA.Laddr,ieeeAddr,8);
					RfTx.TXDATA.Saddr = NLME_GetShortAddr();
                                        RfTx.TXDATA.DataBuf[0] = 'K';
					RfTx.TXDATA.DataBuf[1] = 'T';
                                        //ReadSensorAdc(VBAT);
                                        //ReadSensorAdc(VBAT);
                                        JVCCON();
                                        #ifdef POWER_SAVING
                                        //temp = ReadSensorAdc(VBAT);
                                        Sensor_Delay(3000);
                                        #endif
                                        temp = ReadSensorAdc(VBAT);
					RfTx.TXDATA.DataBuf[2] = temp/100 + 0x30;
					temp = temp%100;
					RfTx.TXDATA.DataBuf[3] = temp/10 + 0x30;
					RfTx.TXDATA.DataBuf[4] = temp%10 + 0x30;
                                        RfHaveTxDara = 1;
				}
                                
                                else if(RfRece.RXDATA.DataBuf[0] == 'E')//读可调电位器
				{
                                        UINT8 adc_channel = RfRece.RXDATA.DataBuf[1];
                                        adc_channel -= 0x30;
					memset(RfTx.TxBuf,'x',29);
					RfTx.TXDATA.HeadCom[0] = 'R';
					RfTx.TXDATA.HeadCom[1] = 'A';
					RfTx.TXDATA.HeadCom[2] = 'S';

                                        memcpy(RfTx.TXDATA.Laddr,ieeeAddr,8);
					RfTx.TXDATA.Saddr = NLME_GetShortAddr();
                                        RfTx.TXDATA.DataBuf[0] = 'E';
					RfTx.TXDATA.DataBuf[1] = 'S';
                                        JVCCON();
                                        #ifdef POWER_SAVING
                                        //temp = ReadSensorAdc(VBAT);
                                        Sensor_Delay(3000);
                                        #endif
                                        temp = ReadSensorAdc(adc_channel);
					RfTx.TXDATA.DataBuf[2] = temp/100 + 0x30;
					temp = temp%100;
					RfTx.TXDATA.DataBuf[3] = temp/10 + 0x30;
					RfTx.TXDATA.DataBuf[4] = temp%10 + 0x30;
                                        RfHaveTxDara = 1;
				}
                                
			}
			
			else if((RfRece.RXDATA.HeadCom[1] == 'N') && (RfRece.RXDATA.HeadCom[2] == 'S'))//读模块连接状态
			{
				memset(RfTx.TxBuf,'x',29);
				RfTx.TXDATA.HeadCom[0] = 'R';
				RfTx.TXDATA.HeadCom[1] = 'N';
				RfTx.TXDATA.HeadCom[2] = 'S';
					
				memcpy(RfTx.TXDATA.Laddr,ieeeAddr,8);
				RfTx.TXDATA.Saddr = NLME_GetShortAddr();
                                temp = pkt->LinkQuality;
				RfTx.TXDATA.DataBuf[0] = temp/100 + 0x30;
                                temp %= 100;
                                RfTx.TXDATA.DataBuf[1] = temp/10 + 0x30;
                                RfTx.TXDATA.DataBuf[2] = temp%10 + 0x30;
                                NLME_GetCoordExtAddr(&RfTx.TXDATA.DataBuf[3]);
                                temp1 = NLME_GetCoordShortAddr();
                                RfTx.TXDATA.DataBuf[11] = (INT8U)(temp1>>8);
                                RfTx.TXDATA.DataBuf[12] = (INT8U)(temp1);
				RfHaveTxDara = 1;
			}//end 读模块连接状态
                        break;
		case 'T'://测试
			if((RfRece.RXDATA.HeadCom[1] == 'L') && (RfRece.RXDATA.HeadCom[2] == 'D'))//LED测试
			{
				if(RfRece.RXDATA.DataBuf[0] == 'C')
				{
					if((RfRece.RXDATA.DataBuf[1] == 'D') && (RfRece.RXDATA.DataBuf[2] == '1'))
					{
						if(RfRece.RXDATA.DataBuf[3] == '1')
						{
							LED1 = 0;//开
						}
						else if(RfRece.RXDATA.DataBuf[3] == '0')
						{
							LED1 = 1;//关
						}
					}
					else if((RfRece.RXDATA.DataBuf[1] == 'D') && (RfRece.RXDATA.DataBuf[2] == '2'))
					{
						if(RfRece.RXDATA.DataBuf[3] == '1')
						{
							LED2 = 0;//开
						}
						else if(RfRece.RXDATA.DataBuf[3] == '0')
						{
							LED2 = 1;//关
						}
					}
				}//end if(RfRece.RXDATA.DataBuf[0] == 'C')
                                else if(RfRece.RXDATA.DataBuf[0] == 'T')
                                {
                                	if((RfRece.RXDATA.DataBuf[1] == 'D') && (RfRece.RXDATA.DataBuf[2] == '1'))//控制LED1
					{
						//LedCycle1 = StrToNum16(&RfRece.RXDATA.DataBuf[3],4);
                                                LedCycle1 = (RfRece.RXDATA.DataBuf[3]-0x30)*400;
						if(LedCycle1)
						{
							Led1Flag = 1;
							TIMER3_RUN(1);
						}
						else
						{
							Led1Flag = 0;
							if((Led1Flag == 0) && (Led2Flag == 0))
							{
								TIMER3_RUN(0);
							}
						}
					}
					else if((RfRece.RXDATA.DataBuf[1] == 'D') && (RfRece.RXDATA.DataBuf[2] == '2'))//控制LED2
					{
						//LedCycle2 = StrToNum16(&RfRece.RXDATA.DataBuf[3],4);
                                                LedCycle2 = (RfRece.RXDATA.DataBuf[3]-0x30)*200;
						if(LedCycle2)
						{
							Led2Flag = 1;
							TIMER3_RUN(1);
						}
						else
						{
							Led2Flag = 0;
							if((Led1Flag == 0) && (Led2Flag == 0))
							{
								TIMER3_RUN(0);
							}
						}
					}
				}
				memset(RfTx.TxBuf,'x',29);
				RfTx.TXDATA.HeadCom[0] = 'T';
				RfTx.TXDATA.HeadCom[1] = 'L';
				RfTx.TXDATA.HeadCom[2] = 'D';
						
				memcpy(RfTx.TXDATA.Laddr,ieeeAddr,8);
				RfTx.TXDATA.Saddr = NLME_GetShortAddr();
				
				RfTx.TXDATA.DataBuf[0] = 'O';
				RfTx.TXDATA.DataBuf[1] = 'K';
				RfHaveTxDara = 1;

			}//end LED测试
			if((RfRece.RXDATA.HeadCom[1] == 'B') && (RfRece.RXDATA.HeadCom[2] == 'L'))//测试电池电压
			{
				memset(RfTx.TxBuf,'x',29);
				RfTx.TXDATA.HeadCom[0] = 'T';
				RfTx.TXDATA.HeadCom[1] = 'B';
				RfTx.TXDATA.HeadCom[2] = 'L';

                                memcpy(RfTx.TXDATA.Laddr,ieeeAddr,8);
				RfTx.TXDATA.Saddr = NLME_GetShortAddr();

                                temp1 = ReadAdcCheckVdd();
                                temp1 = temp1*375;
                                temp1 = temp1/64;
                                RfTx.TXDATA.DataBuf[0] = temp1/100 + 0x30;
				temp = temp1%100;
				RfTx.TXDATA.DataBuf[1] = '.';
                                RfTx.TXDATA.DataBuf[2] = temp/10 + 0x30;
				//RfTx.TXDATA.DataBuf[3] = temp%10 + 0x30;
			        RfTx.TXDATA.DataBuf[3] = 'V';	
				RfHaveTxDara = 1;
			}
                        if((RfRece.RXDATA.HeadCom[1] == 'B') && (RfRece.RXDATA.HeadCom[2] == 'E'))//测试蜂鸣器
			{
				memset(RfTx.TxBuf,'x',29);
				RfTx.TXDATA.HeadCom[0] = 'T';
				RfTx.TXDATA.HeadCom[1] = 'B';
				RfTx.TXDATA.HeadCom[2] = 'E';

                                if(RfRece.RXDATA.DataBuf[0]=='1')
                                {
                                    BeeOn();
                                }

                                else if(RfRece.RXDATA.DataBuf[0]=='0')
                                {
                                    BeeOff();
                                }						
				memcpy(RfTx.TXDATA.Laddr,ieeeAddr,8);
				RfTx.TXDATA.Saddr = NLME_GetShortAddr();

				RfTx.TXDATA.DataBuf[0] = 'O';
				RfTx.TXDATA.DataBuf[1] = 'K';

				RfHaveTxDara = 1;
			}
			break;
                #ifdef  EXSENSOR
		case 'E'://扩展DA300
                  if(DA300STA)
                  {
                        if((RfRece.RXDATA.HeadCom[1] == 'R') && (RfRece.RXDATA.HeadCom[2] == 'E'))    //控制继电器
			{
				if((RfRece.RXDATA.DataBuf[0] == 'K') && (RfRece.RXDATA.DataBuf[1] == '1'))
				{
                                      if(RfRece.RXDATA.DataBuf[2] == '1')
                                      {
                                        ERE1ON;
                                      }

                                      else if (RfRece.RXDATA.DataBuf[2] == '0')
                                      {
                                        ERE1OFF;
                                      }
				}
				else if((RfRece.RXDATA.DataBuf[0] == 'K') && (RfRece.RXDATA.DataBuf[1] == '2'))
				{
                                      if(RfRece.RXDATA.DataBuf[2] == '1')
                                      {
                                        ERE2ON;
                                      }

                                      else if (RfRece.RXDATA.DataBuf[2] == '0')
                                      {
                                        ERE2OFF;
                                      }
				}
				memset(RfTx.TxBuf,'x',29);
				RfTx.TXDATA.HeadCom[0] = 'E';
				RfTx.TXDATA.HeadCom[1] = 'R';
				RfTx.TXDATA.HeadCom[2] = 'E';
						
				memcpy(RfTx.TXDATA.Laddr,ieeeAddr,8);
				RfTx.TXDATA.Saddr = NLME_GetShortAddr();
				
				RfTx.TXDATA.DataBuf[0] = 'O';
				RfTx.TXDATA.DataBuf[1] = 'K';
				RfHaveTxDara = 1;
			}//end 控制继电器

                        if((RfRece.RXDATA.HeadCom[1] == 'A') && (RfRece.RXDATA.HeadCom[2] == 'D'))    //控制加速度传感器
			{
				JVCCON();
                                P1_4 = 1;
                                Sensor_Delay(5);
                                memset(RfTx.TxBuf,'x',29);
                //                SHT1X_INT();
                                if(RfRece.RXDATA.DataBuf[0] == 'X')
				{
                                        RfTx.TXDATA.DataBuf[0] = 'X';
                                        #ifdef POWER_SAVING
                                        //temp = ReadSensorAdc(VBAT);
                                        Sensor_Delay(3000);
                                        #endif
                                  	temp = ReadSensorAdc(XOUT);
				}
				else if(RfRece.RXDATA.DataBuf[0] == 'Y')
				{
                                        RfTx.TXDATA.DataBuf[0] = 'Y';
                                        #ifdef POWER_SAVING
                                        //temp = ReadSensorAdc(VBAT);
                                        Sensor_Delay(3000);
                                        #endif
                                  	temp = ReadSensorAdc(YOUT);
				}
                                else if(RfRece.RXDATA.DataBuf[0] == 'Z')
				{
                                        RfTx.TXDATA.DataBuf[0] = 'Z';
                                        #ifdef POWER_SAVING
                                        //temp = ReadSensorAdc(VBAT);
                                        Sensor_Delay(3000);
                                        #endif
                                  	temp = ReadSensorAdc(ZOUT);
				}

				RfTx.TXDATA.HeadCom[0] = 'E';
				RfTx.TXDATA.HeadCom[1] = 'A';
				RfTx.TXDATA.HeadCom[2] = 'D';
                                RfTx.TXDATA.DataBuf[1] = temp/100 + 0x30;
				temp = temp%100;
				RfTx.TXDATA.DataBuf[2] = temp/10 + 0x30;
				RfTx.TXDATA.DataBuf[3] = temp%10 + 0x30;
                                memcpy(RfTx.TXDATA.Laddr,ieeeAddr,8);
				RfTx.TXDATA.Saddr = NLME_GetShortAddr();
				

				RfHaveTxDara = 1;
			}//end
                        if((RfRece.RXDATA.HeadCom[1] == 'P') && (RfRece.RXDATA.HeadCom[2] == 'R'))    //压力传感器
                        {
                          	JVCCON();
                                memset(RfTx.TxBuf,'x',29);
                                #ifdef POWER_SAVING
                                //temp = ReadSensorAdc(VBAT);
                                Sensor_Delay(3000);
                                #endif
                                temp = ReadSensorAdc(VOUT);
				RfTx.TXDATA.HeadCom[0] = 'E';
				RfTx.TXDATA.HeadCom[1] = 'P';
				RfTx.TXDATA.HeadCom[2] = 'R';
                                RfTx.TXDATA.DataBuf[0] = 'P';
                                RfTx.TXDATA.DataBuf[1] = 'R';
                                RfTx.TXDATA.DataBuf[2] = 'E';
                                RfTx.TXDATA.DataBuf[3] = '=';
                                RfTx.TXDATA.DataBuf[4] = temp/100 + 0x30;
				temp = temp%100;
				RfTx.TXDATA.DataBuf[5] = temp/10 + 0x30;
				RfTx.TXDATA.DataBuf[6] = temp%10 + 0x30;
                                RfTx.TXDATA.DataBuf[7] = 'P';
                                memcpy(RfTx.TXDATA.Laddr,ieeeAddr,8);
				RfTx.TXDATA.Saddr = NLME_GetShortAddr();
                                RfHaveTxDara = 1;
                        }
                        if((RfRece.RXDATA.HeadCom[1] == 'S') && (RfRece.RXDATA.HeadCom[2] == 'H'))    //控制温湿度传感器
			{
				JVCCON();
                                Sensor_Delay(1000);
                                memset(RfTx.TxBuf,'x',29);
                                SHT1X_INT();
                                if((RfRece.RXDATA.DataBuf[0] == 'W') && (RfRece.RXDATA.DataBuf[1] == 'D'))
				{
                                  RfTx.TXDATA.DataBuf[0] = 'W';
				  RfTx.TXDATA.DataBuf[1] = 'D';
                                  temp1 = Read_SHT1X(3);
                                  //RfHaveTxDara = 1;
				}
				else if((RfRece.RXDATA.DataBuf[0] == 'S') && (RfRece.RXDATA.DataBuf[1] == 'D'))
				{
                                  RfTx.TXDATA.DataBuf[0] = 'S';
                                  RfTx.TXDATA.DataBuf[1] = 'D';
                                  temp1=Read_SHT1X(5);
                                  //RfHaveTxDara = 1;
				}
				RfTx.TXDATA.HeadCom[0] = 'E';
				RfTx.TXDATA.HeadCom[1] = 'S';
				RfTx.TXDATA.HeadCom[2] = 'H';
                                RfTx.TXDATA.DataBuf[2] =(UINT8) (temp1>>8);
                                RfTx.TXDATA.DataBuf[3] =(UINT8) (temp1&0xff);
                                memcpy(RfTx.TXDATA.Laddr,ieeeAddr,8);
				RfTx.TXDATA.Saddr = NLME_GetShortAddr();
				

				RfHaveTxDara = 1;
			}//end
                  }
                  else// (DA300STA==0)
                  {
                          RfTx.TXDATA.HeadCom[0] = 'E';
                          RfTx.TXDATA.HeadCom[1] = 'N';
			  RfTx.TXDATA.HeadCom[2] = 'A';
                          //memcpy(RfTx.TXDATA.DataBuf,"NA",2);
                          memset(RfTx.TXDATA.DataBuf,'*',16);
                          RfTx.TXDATA.DataBuf[0] ='E';
                          RfTx.TXDATA.DataBuf[1] ='2';
                          RfHaveTxDara = 1;
                  }
		      break;
                     #endif
		
		}//end
	if(RfHaveTxDara)//如果有数据要发送
	{
		SendData(RfTx.TxBuf, 0x0000, 29);//发送数据
		RfHaveTxDara = 0;
	}
}

/*********************************************************************
 * @fn      SampleApp_SendPeriodicMessage
 *
 * @brief   Send the periodic message.
 *
 * @param   none
 *
 * @return  none
 */
void SampleApp_SendPeriodicMessage( void )
{
  if ( AF_DataRequest( &SampleApp_Periodic_DstAddr, &SampleApp_epDesc,
                       SAMPLEAPP_PERIODIC_CLUSTERID,
                       1,
                       (uint8*)&SampleAppPeriodicCounter,
                       &SampleApp_TransID,
                       AF_DISCV_ROUTE,
                       AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
  {
  }
  else
  {
    // Error occurred in request to send.
  }
}

/*********************************************************************
 * @fn      SampleApp_SendFlashMessage
 *
 * @brief   Send the flash message to group 1.
 *
 * @param   flashTime - in milliseconds
 *
 * @return  none
 */
void SampleApp_SendFlashMessage( uint16 flashTime )
{
  uint8 buffer[3];
  buffer[0] = (uint8)(SampleAppFlashCounter++);
  buffer[1] = LO_UINT16( flashTime );
  buffer[2] = HI_UINT16( flashTime );

  if ( AF_DataRequest( &SampleApp_Flash_DstAddr, &SampleApp_epDesc,
                       SAMPLEAPP_FLASH_CLUSTERID,
                       3,
                       buffer,
                       &SampleApp_TransID,
                       AF_DISCV_ROUTE,
                       AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
  {
  }
  else
  {
    // Error occurred in request to send.
  }
}

/*********************************************************************
*********************************************************************/
