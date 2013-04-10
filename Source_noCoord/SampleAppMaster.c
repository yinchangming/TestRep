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

/* HAL */
#include "hal_lcd.h"
#include "hal_uart.h"
#include "hal_led.h"
#include "hal_key.h"
#include "SPIMgr.h"

#include "string.h"

#include "Lcd128X64.h"

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * TYPEDEFS
 */

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



union f1
{
  UINT8 RxBuf[32];
  struct UARTCOMBUF
  {
        UINT8 Head;       //ͷ
        UINT8 HeadCom[3]; //����ͷ
  	//UINT8 Format[2];  //֡��ʽ
        UINT8 Laddr[8];   //�����ַ
        UINT8 Saddr[2];   //�����ַ
        //UINT8 TempAddr[10];//���õ�ַ
        UINT8 DataBuf[16];  //���ݻ�����
        //UINT8 TempDataBuf[8];//��������
        UINT8 CRC;    //У��λ
  	UINT8 LastByte;//֡β
  }RXDATA;
}UartRxBuf;//�Ӵ��ڽ��յ�������֡

union e
{
  UINT8 TxBuf[32];
  struct UARTBUF
  {
        UINT8 Head;       //ͷ
        UINT8 HeadCom[3]; //����ͷ
  	//UINT8 Format[2];  //֡��ʽ
        UINT8 Laddr[8];   //�����ַ
        UINT8 Saddr[2];   //�����ַ
        //UINT8 TempAddr[10];//���õ�ַ
        UINT8 DataBuf[16];  //���ݻ�����
        //UINT8 TempDataBuf[8];//��������
        UINT8 CRC;    //У��λ
  	UINT8 LastByte;//֡β
  }TXDATA;
}UartTxBuf;//�Ӵ��ڷ�������֡


union h{
  UINT8 RxBuf[50];
  struct RFRXBUF
  {
        UINT8 HeadCom[3]; //����ͷ
        //UINT8 Format[2];//��ʽ
        UINT8 Laddr[8];
        UINT8 Saddr[2];
        //UINT8 TempAddr[10];//���õ�ַ
        UINT8 DataBuf[37];  //���ݻ�����
        //UINT8 TempDataBuf[8];//��������
  }RXDATA;
}RfRx;//���߽��ջ�����


union j{
  UINT8 TxBuf[29];
  struct RFTXBUF
  {
        UINT8 HeadCom[3]; //����ͷ
        //UINT8 Format[2];//��ʽ
        UINT8 Laddr[8];
        UINT8 Saddr[2];
        //UINT8 TempAddr[10];//���õ�ַ
        UINT8 DataBuf[16];  //���ݻ�����
        //UINT8 TempDataBuf[8];//��������
  }TXDATA;
}RfTx;//���߷��ͻ�����


UINT8 RfHaveTxDara;//������������Ҫ����


UINT8 RfRxNewData;

#define MAXJoinDevice    5       //�洢�豸��Ϣ����

typedef struct 
{
        //UINT8 PaterAddr[10];//�游�ڵ��ַ   ����λΪ�����ַ����8λΪ�����ַ
        UINT8 nodeLAddr[8]; //���ڵ������ַ
        UINT8 nodeSAddr[2]; //���ڵ������ַ
        UINT8 nodeRSSI;  //���ڵ� RSSI
        UINT8 nodeType;  //���ڵ� ����
}typedef_join;

UINT8 Sys_RfdCount=0;		//RFD������
UINT8 Sys_RouterCount=0;	        //·����������
typedef_join JoinNode[MAXJoinDevice];	//�豸�б�(����������ַַͳ��)��

UINT8 HaveFlag;//�����ظ�����


/*********************************************************************
 * LOCAL FUNCTIONS
 */
void SampleApp_HandleKeys( uint8 shift, uint8 keys );
void SampleApp_MessageMSGCB( afIncomingMSGPacket_t *pckt );
void SampleApp_SendPeriodicMessage( void );
void SampleApp_SendFlashMessage( uint16 flashTime );
UINT8 CheckUartData(UINT8 *arr, UINT8 n);
void UartOutNetDis(void);
UINT8 SendData(UINT8 *buf, UINT16 addr, UINT8 Leng);
void halPutch(char c);
uint8 SampleApp_FindNodeLAddr(typedef_join joinNew);


void halPutch(char c)
{char i,j;
	EA = 0;
        for(j=0;j<200;j++){
                ;
        }
	UTX0IF = 0;
	U0DBUF = c;
	while (UTX0IF==0);
	UTX0IF = 0;
        for(j=0;j<200;j++){
                ;
        }
	EA = 1;
}




/*********************************************************************
 * NETWORK LAYER CALLBACKS
 */

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

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

  P1DIR |= 0x03;
  // Device hardware initialization can be added here or in main() (Zmain.c).
  // If the hardware is application specific - add it here.
  // If the hardware is other parts of the device add it in main().

 #if defined ( SOFT_START )
  // The "Demo" target is setup to have SOFT_START and HOLD_AUTO_START
  // SOFT_START is a compile option that allows the device to start
  //  as a coordinator if one isn't found.
  // We are looking at a jumper (defined in SampleAppHw.c) to be jumpered
  // together - if they are - we will start up a coordinator. Otherwise,
  // the device will start as a router.
//  if ( readCoordinatorJumper() )
    zgDeviceLogicalType = ZG_DEVICETYPE_COORDINATOR;
//  else
//    zgDeviceLogicalType = ZG_DEVICETYPE_ROUTER;
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

#if defined ( LCD_SUPPORTED )
  HalLcdWriteString( "SampleApp", HAL_LCD_LINE_1 );
#endif
}




//-------------------------------------------------------------------------
//����У��,���������ǼӺ�У�飬��ȷ����1,���󷵻�0
//-------------------------------------------------------------------------
UINT8 CheckUartData(UINT8 *arr, UINT8 n)
{
	UINT8 sum=0;
	UINT8 i;
	for(i=0; i<n; i++)
	{
		sum += *arr;
		arr++;
	}
	return sum;
}


UINT8 UTXCNT;
//-------------------------------------------------------------------------
//�����������ṹ.
//����豸�б�
//-------------------------------------------------------------------------
void UartOutNetDis(void)
{   UINT8 *data;
    UINT8 i,j;
    
    typedef_join *join;
    join = JoinNode;
    
    UartTxBuf.TXDATA.Head = '&';
    UartTxBuf.TXDATA.HeadCom[0] = 'R';
    UartTxBuf.TXDATA.HeadCom[1] = 'N';
    UartTxBuf.TXDATA.HeadCom[2] = 'D';
    UartTxBuf.TXDATA.LastByte = '*';
	                          
    for(i=0;i<MAXJoinDevice;i++){
            if(((join+i)->nodeSAddr[0] !=0 )||((join+i)->nodeSAddr[1] !=0 )){
                      if((join+i)->nodeType != 0){
	                      memset(&UartTxBuf.TxBuf[4],'0',26);
        
                              if((join+i)->nodeType == 2){  //·��
	                                  UartTxBuf.TXDATA.DataBuf[0] = 'R';
	                                  UartTxBuf.TXDATA.DataBuf[1] = 'O';
	                                  UartTxBuf.TXDATA.DataBuf[2] = 'U';
                                      
                              }
                              else if((join+i)->nodeType == 3){  //�ն�
	                                  UartTxBuf.TXDATA.DataBuf[0] = 'R';
	                                  UartTxBuf.TXDATA.DataBuf[1] = 'F';
	                                  UartTxBuf.TXDATA.DataBuf[2] = 'D';
                              }        

		              memcpy(UartTxBuf.TXDATA.Laddr,(join+i)->nodeLAddr,8);//���������ַ
                              //memcpy(&UartTxBuf.TxBuf[19],JoinNode.RouterPaterAddr[temp],10);//���ڵ��ַ
		              memset(&UartTxBuf.TxBuf[19],'0',10);//���ڵ��ַ
		              memcpy(UartTxBuf.TXDATA.Saddr,(join+i)->nodeSAddr,2);//���������ַ
		              UartTxBuf.TXDATA.DataBuf[3] = i+0x30;
		              UartTxBuf.TXDATA.DataBuf[4] = i+0x30;
		              UartTxBuf.TXDATA.CRC = CheckUartData(&UartTxBuf.TxBuf[1],29);
		              UartTxBuf.TXDATA.LastByte = '*';
		              for(j=0; j<32; j++){
			          halPutch(UartTxBuf.TxBuf[j]);//�������
		              }
                      }
            }
    }
	
    
        //memset(&UartTxBuf.TxBuf[4],'0',26);
      data = &UartTxBuf.TxBuf[4];
      for(i=0;i<26;i++){
           *data = '0';     data++;
      }
      
	UartTxBuf.TXDATA.DataBuf[0] = 'E';
	UartTxBuf.TXDATA.DataBuf[1] = 'N';
	UartTxBuf.TXDATA.DataBuf[2] = 'D';//���ͽ���֡
	UartTxBuf.TXDATA.CRC = CheckUartData(&UartTxBuf.TxBuf[1],29);

	for(i=0; i<32; i++)
	{
		halPutch(UartTxBuf.TxBuf[i]);//�������
	}
        
        /*
	UINT8 temp, i;
	
	temp = JoinNode.RouterCount;
        if(temp > MAXROUTER)
          temp = MAXROUTER;
        UTXCNT = temp;
	UartTxBuf.TXDATA.Head = '&';
	UartTxBuf.TXDATA.HeadCom[0] = 'R';
	UartTxBuf.TXDATA.HeadCom[1] = 'N';
	UartTxBuf.TXDATA.HeadCom[2] = 'D';
	memset(&UartTxBuf.TxBuf[4],'0',26);
	UartTxBuf.TXDATA.DataBuf[0] = 'R';
	UartTxBuf.TXDATA.DataBuf[1] = 'O';
	UartTxBuf.TXDATA.DataBuf[2] = 'U';
	UartTxBuf.TXDATA.LastByte = '*';

	while(temp)//����·�ɽڵ��ַ
	{
		temp--;
		memcpy(UartTxBuf.TXDATA.Laddr,JoinNode.RouterAddr[temp],8);//�����ַ
                memcpy(&UartTxBuf.TxBuf[19],JoinNode.RouterPaterAddr[temp],10);//���ڵ��
		//memcpy(UartTxBuf.TXDATA.Saddr,&JoinNode.RouterAddr[temp][8],2);//�����ַ
		UartTxBuf.TXDATA.DataBuf[3] = temp/10+0x30;
		UartTxBuf.TXDATA.DataBuf[4] = temp%10+0x30;
		UartTxBuf.TXDATA.CRC = CheckUartData(&UartTxBuf.TxBuf[1],29);
		UartTxBuf.TXDATA.LastByte = '*';
		for(i=0; i<32; i++)
		{
			halPutch(UartTxBuf.TxBuf[i]);//�������
		}
	}
	
	temp = JoinNode.RfdCount;
        if(temp > MAXRFD)
          temp = MAXRFD;
        
	UartTxBuf.TXDATA.DataBuf[0] = 'R';
	UartTxBuf.TXDATA.DataBuf[1] = 'F';
	UartTxBuf.TXDATA.DataBuf[2] = 'D';
	while(temp)//����RFD�ڵ��ַ
	{
		temp--;

		memcpy(UartTxBuf.TXDATA.Laddr,JoinNode.RfdAddr[temp],8);//�����ַ
                 memcpy(&UartTxBuf.TxBuf[19],JoinNode.RouterPaterAddr[temp],10);
		//memcpy(UartTxBuf.TXDATA.Saddr,&JoinNode.RfdAddr[temp][8],2);//�����ַ
		UartTxBuf.TXDATA.DataBuf[3] = temp/10+0x30;
		UartTxBuf.TXDATA.DataBuf[4] = temp%10+0x30;
		UartTxBuf.TXDATA.CRC = CheckUartData(&UartTxBuf.TxBuf[1],29);
		UartTxBuf.TXDATA.LastByte = '*';

		for(i=0; i<32; i++)
		{
			halPutch(UartTxBuf.TxBuf[i]);//�������
		}

	}
	
	memset(&UartTxBuf.TxBuf[4],'0',26);
	UartTxBuf.TXDATA.DataBuf[0] = 'E';
	UartTxBuf.TXDATA.DataBuf[1] = 'N';
	UartTxBuf.TXDATA.DataBuf[2] = 'D';//���ͽ���֡
	UartTxBuf.TXDATA.CRC = CheckUartData(&UartTxBuf.TxBuf[1],29);

	for(i=0; i<32; i++)
	{
		halPutch(UartTxBuf.TxBuf[i]);//�������
	}
        */
	
}



//-------------------------------------------------------------------------
//����һ������
//-------------------------------------------------------------------------
UINT8 SendData(UINT8 *buf, UINT16 addr, UINT8 Leng)
{
	afAddrType_t SendDataAddr;
	
	SendDataAddr.addrMode = (afAddrMode_t)Addr16Bit;
	SendDataAddr.endPoint = SAMPLEAPP_ENDPOINT;
	SendDataAddr.addr.shortAddr = addr;
        if ( AF_DataRequest( &SendDataAddr, &SampleApp_epDesc,
                       2,//SAMPLEAPP_PERIODIC_CLUSTERID,
                       Leng,
                       buf,
                       &SampleApp_TransID,
                       AF_DISCV_ROUTE,
                     //  AF_ACK_REQUEST,
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
uint16 SampleApp_ProcessEvent( uint8 task_id, uint16 events )
{
	afIncomingMSGPacket_t *MSGpkt;
	UINT16 SrcSaddr;
	UINT8 flag,i;
        UINT8 *data;
	
	UINT8 ReadFlag;  //���Լ����Ƕ�����
        
	static UINT8 __xdata LastRecLaddr[8];
        //HalUARTWrite ( 0,"OK",2);
	
	//HalLedSet ( HAL_LED_4, HAL_LED_MODE_FLASH );
	//HalLedSet ( HAL_LED_1, HAL_LED_MODE_FLASH );
	if ( events & SYS_EVENT_MSG )
	{
		MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( SampleApp_TaskID );
		while ( MSGpkt )
		{
			switch ( MSGpkt->hdr.event )
			{
					// Received when a key is pressed
				case KEY_CHANGE:
					SampleApp_HandleKeys( ((keyChange_t *)MSGpkt)->state, ((keyChange_t *)MSGpkt)->keys );
				break;
					// Received when a messages is received (OTA) for this endpoint
				case AF_INCOMING_MSG_CMD:   //���յ�����
					SampleApp_MessageMSGCB( MSGpkt );
				break;
				
				// Received whenever the device changes state in the network
				case ZDO_STATE_CHANGE:
					SampleApp_NwkState = (devStates_t)(MSGpkt->hdr.status);
					if ( (SampleApp_NwkState == DEV_ZB_COORD)
					|| (SampleApp_NwkState == DEV_ROUTER)
					|| (SampleApp_NwkState == DEV_END_DEVICE) )
					{
						// Start sending the periodic message in a regular interval.
						//osal_start_timerEx( SampleApp_TaskID,
						//SAMPLEAPP_SEND_PERIODIC_MSG_EVT,
						//SAMPLEAPP_SEND_PERIODIC_MSG_TIMEOUT );
                                                ClearScreen();//����
                                                TurnShowInterface(1);
                                          ;
					}
					else
					{
						// Device is no longer in the network
					}
				break;

                                case SPI_INCOMING_ZTOOL_PORT://
                                	//P1_1=!P1_1;
                                        HalUARTRead( SPI_MGR_DEFAULT_PORT, UartRxBuf.RxBuf, 32 );
                                        
                                        //HalUARTWrite ( SPI_MGR_DEFAULT_PORT, UartRxBuf.RxBuf, 32);//�Ӵ������
                                        
                                        //memcpy(LastRecLaddr,UartRxBuf.RXDATA.Laddr,8);
                                        HalLedBlink(HAL_LED_1,2,50,1000);
                                        ReadFlag = 0;
					if('&' == UartRxBuf.RxBuf[0]){
					    if(1)//(CheckUartData(&UartRxBuf.RxBuf[1],29) == UartRxBuf.RxBuf[30])//���У��ͨ��
                                            {
						switch(UartRxBuf.RXDATA.HeadCom[0])//��������ͷ
						{
							case 'R':
								if((UartRxBuf.RXDATA.HeadCom[1] == 'N') && (UartRxBuf.RXDATA.HeadCom[2] == 'D'))//���緢��
								{
									ReadFlag = 1;//���Լ�
								}
								else
								{
									ReadFlag = 0;//������
								}
									
								break;
							case 'S':
							case 'T':
							case 'C':
								ReadFlag = 0;
								break;
						}
                                                
						if(ReadFlag){
							UartOutNetDis();//�����������ṹ    �������ַ
						}
						else{      
                                                        memcpy(&RfTx.TxBuf[0],&UartRxBuf.RxBuf[1],29);//װ������
							SrcSaddr = 0;
							flag = 0;
                                                        
                                                        typedef_join joinNew;
                                                        memcpy(joinNew.nodeLAddr,RfTx.TXDATA.Laddr,8 );
                                                        i=SampleApp_FindNodeLAddr(joinNew);
                                                        
							//if(flag == 1)
                                                        if(i<MAXJoinDevice){
                                                               typedef_join *join;
                                                                join = JoinNode;
								SrcSaddr = (join+i)->nodeSAddr[0] <<8;
								SrcSaddr += (join+i)->nodeSAddr[1];	//���ҵ������ַ
                                                                
                                                                UartTxBuf.TXDATA.Head = '&';
                                                                memcpy(&UartTxBuf.TxBuf[1],&RfTx.TxBuf[0],29);
                                                                if(SendData(UartTxBuf.TxBuf, SrcSaddr, 30) == 0)//��������ʧ��
								{
									memset(&RfTx.TxBuf[3],'x',26);
									memcpy(RfTx.TXDATA.Laddr,UartRxBuf.RXDATA.Laddr,8);//�����ַ
									RfTx.TXDATA.DataBuf[0] = 'E';
									RfTx.TXDATA.DataBuf[1] = '0';//�������E0
								        memcpy(&UartTxBuf.TxBuf[1],&RfTx.TxBuf[0],29);
                                                                        UartTxBuf.TXDATA.CRC = CheckUartData(&UartTxBuf.TxBuf[1],29);
                                                                        UartTxBuf.TXDATA.Head = '&';
		                                                        UartTxBuf.TXDATA.LastByte = '*';
                                                                        HalUARTWrite ( SPI_MGR_DEFAULT_PORT, UartTxBuf.TxBuf, 32);//�Ӵ������
								}
                                                                else
                                                                {
                                                                        //P1_1=!P1_1;
                                                                        osal_start_timerEx( SampleApp_TaskID,
						                        SAMPLEAPP_SEND_PERIODIC_MSG_EVT,
						                        SAMPLEAPP_SEND_PERIODIC_MSG_TIMEOUT );
                                                                }
							}
							else{
								memset(&RfTx.TxBuf[3],'x',26);
								memcpy(RfTx.TXDATA.Laddr,UartRxBuf.RXDATA.Laddr,8);//�����ַ
								RfTx.TXDATA.DataBuf[0] = 'E';
								RfTx.TXDATA.DataBuf[1] = '1';//�������E0
								memcpy(&UartTxBuf.TxBuf[1],&RfTx.TxBuf[0],29);
                                                                UartTxBuf.TXDATA.CRC = CheckUartData(&UartTxBuf.TxBuf[1],29);
                                                                UartTxBuf.TXDATA.Head = '&';
		                                                UartTxBuf.TXDATA.LastByte = '*';
                                                                HalUARTWrite ( SPI_MGR_DEFAULT_PORT, UartTxBuf.TxBuf, 32);//�Ӵ������
							}
                                                        
                                                    }
						}

					}
                                memset(UartRxBuf.RxBuf,0,32);
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
	if ( events & SAMPLEAPP_SEND_PERIODIC_MSG_EVT )
	{
                //P1_1=!P1_1;

		//memset(&RfTx.TxBuf[3],'x',26);
                   data = &RfTx.TxBuf[3];
                   for(i=0;i<26;i++){
                       *data = 'x';     data++;
                     }
		memcpy(RfTx.TXDATA.Laddr,LastRecLaddr,8);//�����ַ
		RfTx.TXDATA.DataBuf[0] = 'E';
		RfTx.TXDATA.DataBuf[1] = '0';//�������E0
		memcpy(&UartTxBuf.TxBuf[1],&RfTx.TxBuf[0],29);
                UartTxBuf.TXDATA.CRC = CheckUartData(&UartTxBuf.TxBuf[1],29);
                UartTxBuf.TXDATA.Head = '&';
		UartTxBuf.TXDATA.LastByte = '*';
                HalUARTWrite ( SPI_MGR_DEFAULT_PORT, UartTxBuf.TxBuf, 32);//�Ӵ������
		return (events ^ SAMPLEAPP_SEND_PERIODIC_MSG_EVT);
	}
        
        if(events &  SAMPLEAPP_ShowInterface_MSG_EVT){
                ClearScreen();//����
                TurnShowInterface(1);
                return (events ^ SAMPLEAPP_ShowInterface_MSG_EVT);
        }
        
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

/********************************************************************
*���ҵ�ַ��ͬ�ļ�¼
*input:     -join; ���������ַ
*out:       0xff:  û���ҵ���Ӧ�ļ�¼
*           <0xff: ��¼����
********************************************************************/
uint8 SampleApp_FindNodeLAddr(typedef_join joinNew)
{
      typedef_join *join;
      uint8 i,j;
      join = JoinNode;     
      
      for(i=0;i<MAXJoinDevice;i++){   //����ڵ��б����ݱȶ�
                       if(joinNew.nodeLAddr[0] == (join+i)->nodeLAddr[0]){
                                   for(j=1;j<=7;j++){
                                       if(joinNew.nodeLAddr[j] != (join+i)->nodeLAddr[j]){
                                               break;
                                       }        
                                   }  
                                   if(j>7){
                                           break;
                                   }
                       }  
      }
      
      if(i>=MAXJoinDevice){
            return 0xff;  
      }
       return i;
}


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
void SampleApp_MessageMSGCB( afIncomingMSGPacket_t *pkt )
{
	uint8 i , j;
        byte *data,*dataNew;
	osal_stop_timerEx( SampleApp_TaskID, SAMPLEAPP_SEND_PERIODIC_MSG_EVT);
        
        //------�����ǲ��Ǿ���Э�� (�Զ���Э��)
        data = pkt->cmd.Data;
        for(i=0;i<10;i++){   
                if(*data == 'A'){
                        if(*(data+1) == 'K'){
                                break;
                        }        
                }    
                data++;
        }
        
        if(i<10){     //����Э�� (�Զ���Э��)
               typedef_join *join,joinNew;
               join = JoinNode;
/*
typedef struct 
{
        //UINT8 PaterAddr[10];//�游�ڵ��ַ   ����λΪ�����ַ����8λΪ�����ַ
        UINT8 nodeLAddr[8]; //���ڵ������ַ
        UINT8 nodeSAddr[2]; //���ڵ������ַ
        UINT8 nodeType;  //���ڵ� ����
}typedef_join;
typedef_join JoinNode[MAXJoinDevice];	//�豸�б�(����������ַַͳ��)��
*/
               dataNew=data;
               data+=5;
               
               memcpy(joinNew.nodeLAddr,data,8 );
               i=SampleApp_FindNodeLAddr(joinNew);
               j=0;
               if(i>=MAXJoinDevice){
                       for(i=0;i<MAXJoinDevice;i++){   ////�ҿ�λ��
                              if((join+i)->nodeSAddr[0] == 0){
                                      j=1;
                                      memcpy((join+i)->nodeLAddr,data,8); //���������ַ
                                      break;
                              }        
                       }        
               }
               
               if(i < MAXJoinDevice){   //��Ч����
                    //data = &RfRx.RxBuf[3];   
                       memcpy(RfRx.RxBuf,pkt->cmd.Data,pkt->cmd.DataLength -1);  
                    data = dataNew;
                    data+=3;
                    memcpy((join+i)->nodeSAddr,data,2); data+=2; //���������ַ
                    

                    data +=8; //�����ַ
                    data ++;
                    (join+i)->nodeType = *data;    //�����豸����
                    if(*data==3){//�ն�
                            Sys_RfdCount++;
                    }
                    else if(*data ==2){ //·��
                            Sys_RouterCount++;
                    }
                    
                    if(j==1){    //��ʾ��Ϣ
                            ClearScreen();//����
                            if(*data==3){//�ն�
                               TurnShowInterface(0x12);
                            }
                            else if(*data ==2){ //·��
                               TurnShowInterface(0x11);
                            }
                            
                            osal_start_timerEx( SampleApp_TaskID,
                               SAMPLEAPP_ShowInterface_MSG_EVT,
			       SAMPLEAPP_ShowInterface_MSG_TIMEOUT );
                                      
                    }
               }
        }
        else{       //WSNЭ��
             data = pkt->cmd.Data;
             for(i=0;i<10;i++){   
                if(*data == '&'){
                        break;
                }    
                data++;
            }
            if(i<10){  //��WSNЭ��
               memcpy(RfRx.RxBuf,data,pkt->cmd.DataLength - i);        
               
                UartTxBuf.TXDATA.Head = '&';
		memcpy(&UartTxBuf.TxBuf[1],&RfRx.RxBuf[1],28);
		for(i=0; i<8; i++)
		{
			UartTxBuf.TXDATA.Laddr[i] = RfRx.RXDATA.Laddr[i];//����ַ
		}
		for(i=0; i<2; i++)
		{
			UartTxBuf.TXDATA.Saddr[i] = RfRx.RXDATA.Saddr[1-i];//�̵�ַ
		}
		UartTxBuf.TXDATA.CRC = CheckUartData(&UartTxBuf.TxBuf[1],29);
		UartTxBuf.TXDATA.LastByte = '*';
		HalUARTWrite ( SPI_MGR_DEFAULT_PORT, UartTxBuf.TxBuf, 32);//�Ӵ������	
                    
            }
        }
        
        /*
	if((RfRx.RXDATA.HeadCom[0] == 'J') && (RfRx.RXDATA.HeadCom[1] == 'O') && (RfRx.RXDATA.HeadCom[2] == 'N'))//���½ڵ��������
	{
                uint8 TempAddr[10];
                uint8 TempPaterAddr[10];
		if((RfRx.RXDATA.DataBuf[0] == 'R') && (RfRx.RXDATA.DataBuf[1] == 'F') && (RfRx.RXDATA.DataBuf[2] == 'D'))//RFD�ڵ�
		{
                        uint8 RfdTempCount = JoinNode.RfdCount % MAXRFD;
			for(i=0; i<8; i++)
			{
				//JoinNode.RfdAddr[RfdTempCount][i] = RfRx.RXDATA.Laddr[i];
                                TempAddr[i] = RfRx.RXDATA.Laddr[i];
			}
			for(i=0; i<2; i++)
			{
				//JoinNode.RfdAddr[RfdTempCount][8+i] = RfRx.RXDATA.Saddr[1-i];
                                TempAddr[8+i] = RfRx.RXDATA.Saddr[1-i];
			}
                        for(i = 0;i<10;i++)
                        {
                                //JoinNode.RfdPaterAddr[RfdTempCount][i] = RfRx.RxBuf[18+i];
                                TempPaterAddr[i] = RfRx.RxBuf[18+i];
                        }
                                               
			for(j=0; j<MAXRFD; j++)//�Ѷ������ظ�����Ľڵ�
			{
				HaveFlag = 1;
				for(i=0; i<8; i++)
				{
					//if(JoinNode.RfdAddr[RfdTempCount][i] != JoinNode.RfdAddr[j][i] || RfdTempCount == j)
                                        if(TempAddr[i] != JoinNode.RfdAddr[j][i])
					{
						HaveFlag = 0;
						break;//����
					}
				}
				if(HaveFlag == 0)continue;
				JoinNode.RfdCount--;//��
				JoinNode.RfdAddr[j][8] = RfRx.RXDATA.Saddr[1];
				JoinNode.RfdAddr[j][9] = RfRx.RXDATA.Saddr[0];	//�޸����������ַ
				break;
			}
                        if(HaveFlag == 0)
                        {
                            memcpy(JoinNode.RfdAddr[RfdTempCount],TempAddr,10);
                            memcpy(JoinNode.RfdPaterAddr[RfdTempCount],TempPaterAddr,10);
                        }
			JoinNode.RfdCount++;
                        
		}
		else if((RfRx.RXDATA.DataBuf[0] == 'R') && (RfRx.RXDATA.DataBuf[1] == 'O') && (RfRx.RXDATA.DataBuf[2] == 'U'))//·�ɽڵ�
		{
                        uint8 RouTempCount = JoinNode.RouterCount % MAXRFD;
			for(i=0; i<8; i++)
			{
				//JoinNode.RouterAddr[RouTempCount][i] = RfRx.RXDATA.Laddr[i];
                                TempAddr[i] = RfRx.RXDATA.Laddr[i];
			}
			for(i=0; i<2; i++)
			{
				//JoinNode.RouterAddr[RouTempCount][8+i] = RfRx.RXDATA.Saddr[1-i];
                                TempAddr[8+i] = RfRx.RXDATA.Saddr[1-i];
			}
                         for(i = 0;i<10;i++)
                        {
                                //JoinNode.RouterPaterAddr[RouTempCount][i] = RfRx.RxBuf[18+i];
                                TempPaterAddr[i] = RfRx.RxBuf[18+i];
                        }
				
			for(j=0; j<MAXROUTER; j++)//�Ѷ������ظ�����Ľڵ�
			{
				HaveFlag = 1;
				for(i=0; i<8; i++)
				{
					//if(JoinNode.RouterAddr[RouTempCount][i] != JoinNode.RouterAddr[j][i] || RouTempCount == j)
                                        if(TempAddr[i] != JoinNode.RouterAddr[j][i])
					{
						HaveFlag = 0;
						break;//����
					}
				}
				if(HaveFlag == 0)
                                {
                                  continue;
                                }
				JoinNode.RouterCount--;//��
				JoinNode.RouterAddr[j][8] = RfRx.RXDATA.Saddr[1];
				JoinNode.RouterAddr[j][9] = RfRx.RXDATA.Saddr[0];	//�޸����������ַ
				break;
			}
                        if(HaveFlag == 0)
                        {
                          memcpy(JoinNode.RouterAddr[RouTempCount],TempAddr,10);
                          memcpy(JoinNode.RouterPaterAddr[RouTempCount],TempPaterAddr,10);
                        }
			JoinNode.RouterCount++;
		}
	}
	else
	{
		UartTxBuf.TXDATA.Head = '&';
		memcpy(&UartTxBuf.TxBuf[1],&RfRx.RxBuf[0],29);
		for(i=0; i<8; i++)
		{
			UartTxBuf.TXDATA.Laddr[i] = RfRx.RXDATA.Laddr[i];//����ַ
		}
		for(i=0; i<2; i++)
		{
			UartTxBuf.TXDATA.Saddr[i] = RfRx.RXDATA.Saddr[1-i];//�̵�ַ
		}
		UartTxBuf.TXDATA.CRC = CheckUartData(&UartTxBuf.TxBuf[1],29);
		UartTxBuf.TXDATA.LastByte = '*';
		HalUARTWrite ( SPI_MGR_DEFAULT_PORT, UartTxBuf.TxBuf, 32);//�Ӵ������	
	}
        */
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
