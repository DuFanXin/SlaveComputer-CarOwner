/**************************************************************************************************
  Filename:       SampleApp.c
  Revised:        $Date: 2009-03-18 15:56:27 -0700 (Wed, 18 Mar 2009) $
  Revision:       $Revision: 19453 $

  Description:    Sample Application (no Profile).


  Copyright 2007 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights
  granted under the terms of a software license agreement between the user
  who downloaded the software, his/her employer (which must be your employer)
  and Texas Instruments Incorporated (the "License").  You may not use this
  Software unless you agree to abide by the terms of the License. The License
  limits your use, and you acknowledge, that the Software may not be modified,
  copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio
  frequency transceiver, which is integrated into your product.  Other than for
  the foregoing purpose, you may not use, reproduce, copy, prepare derivative
  works of, modify, distribute, perform, display or sell this Software and/or
  its documentation for any purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED 揂S IS?WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
  TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
  INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
  OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
  OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

  Should you have any questions regarding your right to use this Software,
  contact Texas Instruments Incorporated at www.TI.com.
**************************************************************************************************/

/*********************************************************************
  This application isn't intended to do anything useful, it is
  intended to be a simple example of an application's structure.

  This application sends it's messages either as broadcast or
  broadcast filtered group messages.  The other (more normal)
  message addressing is unicast.  Most of the other sample
  applications are written to support the unicast message model.

  Key control:
    SW1:  Sends a flash command to all devices in Group 1.
    SW2:  Adds/Removes (toggles) this device in and out
          of Group 1.  This will enable and disable the
          reception of the flash command.
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
#include "hal_led.h"
#include "hal_key.h"

#include "MT_UART.h"  // 串口
#include "MT.h"

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
unsigned char T[13]; 	//通讯消息	
bool f=0;//火警判断
bool s=0;//呼叫状态
bool e=0;//火警闪烁
bool r=0;//呼叫闪烁
bool z=0;//等待状态
uint8 w[1]={0};//车位状态
char x,d;



/*********************************************************************
 * LOCAL FUNCTIONS
 */
void SampleApp_HandleKeys( uint8 shift, uint8 keys );
void SampleApp_MessageMSGCB( afIncomingMSGPacket_t *pckt );
void SampleApp_SendPeriodicMessage( void );
void SampleApp_SendFlashMessage( uint16 flashTime );

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

    MT_UartInit();  // 串口初始化
    osal_set_event(SampleApp_TaskID, MY_SEND_UART); //设置事件
    osal_set_event(SampleApp_TaskID, MY_SEND2_UART); //设置事件
    osal_set_event(SampleApp_TaskID, MY_SEND3_UART); //设置事件
    MT_UartRegisterTaskID(SampleApp_TaskID);  // 注册，有串口事件通知我

    //初始化光敏模块
    P2SEL &= ~0X01;     //设置P20为普通IO口
    P2DIR &= ~0X01;    // 在P20口，设置为输入模式

    // Device hardware initialization can be added here or in main() (Zmain.c).
    // If the hardware is application specific - add it here.
    // If the hardware is other parts of the device add it in main().

#if defined ( BUILD_ALL_DEVICES )
    // The "Demo" target is setup to have BUILD_ALL_DEVICES and HOLD_AUTO_START
    // We are looking at a jumper (defined in SampleAppHw.c) to be jumpered
    // together - if they are - we will start up a coordinator. Otherwise,
    // the device will start as a router.
    if ( readCoordinatorJumper() )
        zgDeviceLogicalType = ZG_DEVICETYPE_COORDINATOR;
    else
        zgDeviceLogicalType = ZG_DEVICETYPE_ROUTER;
#endif // BUILD_ALL_DEVICES

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

    // By default, all devices start out in Group 1
    SampleApp_Group.ID = 0x0001;
    osal_memcpy( SampleApp_Group.name, "Group 1", 7  );
    aps_AddGroup( SAMPLEAPP_ENDPOINT, &SampleApp_Group );

#if defined ( LCD_SUPPORTED )
    HalLcdWriteString( "SampleApp", HAL_LCD_LINE_1 );
#endif
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
    (void)task_id;  // Intentionally unreferenced parameter

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
            case AF_INCOMING_MSG_CMD:   //接收到空口发过来的消息
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
                    osal_start_timerEx( SampleApp_TaskID,
                                        SAMPLEAPP_SEND_PERIODIC_MSG_EVT,
                                        SAMPLEAPP_SEND_PERIODIC_MSG_TIMEOUT );
                }
                else
                {
                    // Device is no longer in the network
                }
                break;
            case CMD_SERIAL_MSG: // 串口收到消息#include "MT.h"
                uint8 len = ((mtOSALSerialData_t*)MSGpkt)->msg[0];  //串口消息长度
                uint8* msg = &(((mtOSALSerialData_t*)MSGpkt)->msg[1]);//串口消息内容
                afAddrType_t dstAddr;
                dstAddr.addrMode = Addr16Bit;		// 地址模式为16位段地址
                dstAddr.endPoint = SAMPLEAPP_ENDPOINT;
                dstAddr.addr.shortAddr = 0xFFFF;   // 0xFFFF表示广播,0x0000为协调器
                AF_DataRequest( &dstAddr, &SampleApp_epDesc,  // 通过空口发送消息
                                SAMPLEAPP_MY_CLUSTERID,
                                len,  // 消息长度
                                (uint8*)msg, // 消息内容
                                &SampleApp_TransID,  // 消息编号
                                AF_DISCV_ROUTE,
                                AF_DEFAULT_RADIUS );
                unsigned char u[13];//提取消息进行校验
                for(int i=0;i<9;i++)
                {u[i]=msg[i];}
                int a=0,b=0,c,i;
                for(i=0;i<9;i+=2)
                    a+=u[i];
                for(i=1;i<9;i+=2)
                    b+=(3*u[i]);
                c=a+b;
                u[9]=(c/1000)%10+48;
                u[10]=(c/100)%10+48;
                u[11]=(c/10)%10+48;
                u[12]=c%10+48;
                if(u[9]!=msg[9]||u[10]!=msg[10]||u[11]!=msg[11]||u[12]!=msg[12])
                {
                    HalUARTWrite(0,"Message error\n",14);
                }
		uint8* ch=msg;//分析接收到的命令
                x=ch[3];
		if(x=='3')x=ch[6];
		if(x=='1')f=1;
		else f=0;
		x=ch[3];
		if(x=='4')s=0;
		else if (x=='2'){
                    w[0]=1;


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
    }

    // Send a message out - This event is generated by a timer
    //  (setup in SampleApp_Init()).
    if ( events & SAMPLEAPP_SEND_PERIODIC_MSG_EVT )
    {
        // Send the periodic message
        SampleApp_SendPeriodicMessage();

        // Setup to send message again in normal period (+ a little jitter)
        osal_start_timerEx( SampleApp_TaskID, SAMPLEAPP_SEND_PERIODIC_MSG_EVT,
                            (SAMPLEAPP_SEND_PERIODIC_MSG_TIMEOUT + (osal_rand() & 0x00FF)) );

        // return unprocessed events
        return (events ^ SAMPLEAPP_SEND_PERIODIC_MSG_EVT);
    }

    if (events & MY_SEND_UART)  // 串口发送消息的事件
    {
  	//读取光敏电阻状态，暗时点亮LED
        if(P2_0==0)
        {
            s=0;
            T[0]='*';
            T[1]='1';
            T[2]='3';
            T[3]='1';
            T[4]='0';
            T[5]='0';
            T[6]='0';
            T[7]='2';
            T[8]='0';//无车信号
            int a=0,b=0,c,i;
            for(i=0;i<9;i+=2)
                a+=T[i];
            for(i=1;i<9;i+=2)
                b+=(3*T[i]);
            c=a+b;
            T[9]=(c/1000)%10+48;
            T[10]=(c/100)%10+48;
            T[11]=(c/10)%10+48;
            T[12]=c%10+48;
            HalUARTWrite(0,"\n",1);
            afAddrType_t dstAddr;
            dstAddr.addrMode = Addr16Bit;		// 地址模式为16位段地址
            dstAddr.endPoint = SAMPLEAPP_ENDPOINT;
            dstAddr.addr.shortAddr = 0xFFFF;   // 0xFFFF表示广播,0x0000为协调器
            AF_DataRequest( &dstAddr, &SampleApp_epDesc,  // 通过空口发送消息
                            SAMPLEAPP_MY_CLUSTERID,
                            13,  // 消息长度
                            T, // 消息内容
                            &SampleApp_TransID,  // 消息编号
                            AF_DISCV_ROUTE,
                            AF_DEFAULT_RADIUS );
            HalUARTWrite(0,T,13);
            if(w[0]==2)w[0]=0;//将占有转换为空闲

        }
        else
        {

            T[0]='*';
            T[1]='1';
            T[2]='3';
            T[3]='1';
            T[4]='0';
            T[5]='0';
            T[6]='0';
            T[7]='2';
            T[8]='1';//有车信号
            int a=0,b=0,c,i;
            for(i=0;i<9;i+=2)
                a+=T[i];
            for(i=1;i<9;i+=2)
                b+=(3*T[i]);
            c=a+b;
            T[9]=(c/1000)%10+48;
            T[10]=(c/100)%10+48;
            T[11]=(c/10)%10+48;
            T[12]=c%10+48;
            HalUARTWrite(0,"\n",1);
            afAddrType_t dstAddr;
            dstAddr.addrMode = Addr16Bit;		// 地址模式为16位段地址
            dstAddr.endPoint = SAMPLEAPP_ENDPOINT;
            dstAddr.addr.shortAddr = 0xFFFF;   // 0xFFFF表示广播,0x0000为协调器
            AF_DataRequest( &dstAddr, &SampleApp_epDesc,  // 通过空口发送消息
                            SAMPLEAPP_MY_CLUSTERID,
                            13,  // 消息长度
                            T, // 消息内容
                            &SampleApp_TransID,  // 消息编号
                            AF_DISCV_ROUTE,
                            AF_DEFAULT_RADIUS );
            HalUARTWrite(0,T,13);
            w[0]=2;//置为占有状态
        }
        osal_start_timerEx (SampleApp_TaskID, MY_SEND_UART, 1000); //5s定时器

        return (events ^ MY_SEND_UART);
    }
    if (events & MY_SEND2_UART)
    {
        if(f==1)//火警
     	{
            e=!e;
            if(e){
                HalLedSet(HAL_LED_1,HAL_LED_MODE_ON);
            }
            else HalLedSet(HAL_LED_1,HAL_LED_MODE_OFF);
        }
        else HalLedSet(HAL_LED_1,HAL_LED_MODE_OFF);
        if(s==1)//呼叫
     	{
            r=!r;
            if(r){
                HalLedSet(HAL_LED_2,HAL_LED_MODE_ON);
            }
            else HalLedSet(HAL_LED_2,HAL_LED_MODE_OFF);
        }
        else HalLedSet(HAL_LED_2,HAL_LED_MODE_OFF);
        osal_start_timerEx (SampleApp_TaskID, MY_SEND2_UART, 1000);
    }
    if (events & MY_SEND3_UART)
    {

        if(w[0]==1){//等待驶入
            z=!z;
            if(z){
                HalLedSet(HAL_LED_2,HAL_LED_MODE_ON);
            }
            else HalLedSet(HAL_LED_2,HAL_LED_MODE_OFF);
        }

        osal_start_timerEx (SampleApp_TaskID, MY_SEND3_UART, 500);
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
void SampleApp_HandleKeys( uint8 shift, uint8 keys )
{
    (void)shift;  // Intentionally unreferenced parameter

    if ( keys & HAL_KEY_SW_7 )
    {

        if(w[0]==2){//只有占有状态才能呼叫
            if(s==0)
            {s=1;
                T[0]='*';
                T[1]='1';
                T[2]='3';
                T[3]='4';
                T[4]='0';
                T[5]='0';
                T[6]='0';
                T[7]='2';
                T[8]='0';
                int a=0,b=0,c,i;
                for(i=0;i<9;i+=2)
                    a+=T[i];
                for(i=1;i<9;i+=2)
                    b+=(3*T[i]);
                c=a+b;
                T[9]=(c/1000)%10+48;
                T[10]=(c/100)%10+48;
                T[11]=(c/10)%10+48;
                T[12]=c%10+48;
                HalUARTWrite(0,"\n",1);
                afAddrType_t dstAddr;
                dstAddr.addrMode = Addr16Bit;		// 地址模式为16位段地址
                dstAddr.endPoint = SAMPLEAPP_ENDPOINT;
                dstAddr.addr.shortAddr = 0xFFFF;   // 0xFFFF表示广播,0x0000为协调器
                AF_DataRequest( &dstAddr, &SampleApp_epDesc,  // 通过空口发送消息
                                SAMPLEAPP_MY_CLUSTERID,
                                13,  // 消息长度
                                T, // 消息内容
                                &SampleApp_TransID,  // 消息编号
                                AF_DISCV_ROUTE,
                                AF_DEFAULT_RADIUS );
                HalUARTWrite(0,T,13);
            }
            else
            {
                s=0;
		T[0]='*';
		T[1]='1';
		T[2]='3';
		T[3]='4';
		T[4]='0';
		T[5]='0';
		T[6]='0';
		T[7]='2';
		T[8]='3';//取消呼叫
		int a=0,b=0,c,i;
                for(i=0;i<9;i+=2)
                    a+=T[i];
                for(i=1;i<9;i+=2)
                    b+=(3*T[i]);
                c=a+b;
                T[9]=(c/1000)%10+48;
                T[10]=(c/100)%10+48;
                T[11]=(c/10)%10+48;
                T[12]=c%10+48;
		HalUARTWrite(0,"\n",1);
		afAddrType_t dstAddr;
                dstAddr.addrMode = Addr16Bit;		// 地址模式为16位段地址
                dstAddr.endPoint = SAMPLEAPP_ENDPOINT;
                dstAddr.addr.shortAddr = 0xFFFF;   // 0xFFFF表示广播,0x0000为协调器
                AF_DataRequest( &dstAddr, &SampleApp_epDesc,  // 通过空口发送消息
                                SAMPLEAPP_MY_CLUSTERID,
                                13,  // 消息长度
                                T, // 消息内容
                                &SampleApp_TransID,  // 消息编号
                                AF_DISCV_ROUTE,
                                AF_DEFAULT_RADIUS );
		HalUARTWrite(0,T,13); 

            }
  	}
    }

    if ( keys & HAL_KEY_SW_6 )
    {
        /* The Flashr Command is sent to Group 1.
     * This key toggles this device in and out of group 1.
     * If this device doesn't belong to group 1, this application
     * will not receive the Flash command sent to group 1.
     */
        //HalLedSet(HAL_LED_2,HAL_LED_MODE_OFF);
        uint8* msg="0";
	afAddrType_t dstAddr;
	dstAddr.addrMode=Addr16Bit;//地址模式为16位短地址
	dstAddr.endPoint=SAMPLEAPP_ENDPOINT;
	dstAddr.addr.shortAddr=0xFFFF;//0xFFFF表示广播
	AF_DataRequest( &dstAddr, &SampleApp_epDesc,
                        SAMPLEAPP_MY_CLUSTERID,
                        sizeof(msg),//消息长度
                        msg,//消息内容,
                        &SampleApp_TransID,//消息编号
                        AF_DISCV_ROUTE,
                        AF_DEFAULT_RADIUS );
    }
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
    uint16 flashTime;

    switch ( pkt->clusterId )
    {
    case SAMPLEAPP_PERIODIC_CLUSTERID:
        break;

    case SAMPLEAPP_FLASH_CLUSTERID:
        flashTime = BUILD_UINT16(pkt->cmd.Data[1], pkt->cmd.Data[2] );
        HalLedBlink( HAL_LED_4, 4, 50, (flashTime / 4) );
        break;
	
    case SAMPLEAPP_MY_CLUSTERID:   // 我们自定义的消息
        HalUARTWrite(0,pkt->cmd.Data,pkt->cmd.DataLength);	// 空口消息转发
        uint8* q=(uint8*)pkt->cmd.Data;
        unsigned char u[13];
        for(int i=0;i<9;i++)
        {u[i]=q[i];}
        int a=0,b=0,c,i;
        for(i=0;i<9;i+=2)
            a+=u[i];
        for(i=1;i<9;i+=2)
            b+=(3*u[i]);
        c=a+b;
        u[9]=(c/1000)%10+48;
        u[10]=(c/100)%10+48;
        u[11]=(c/10)%10+48;
        u[12]=c%10+48;
        if(u[9]!=q[9]||u[10]!=q[10]||u[11]!=q[11]||u[12]!=q[12])
        {
            HalUARTWrite(0,"Message error\n",14);
        }
        uint8* ch=q;
        x= ch[3];
        if(x=='3')x= ch[6];
        if(x=='1'){
            f=1;
            HalLedSet(HAL_LED_1,HAL_LED_MODE_ON);}
        else f=0;
        x= ch[3];
        if(x=='4')s=0;
        else if (x=='2'){
            w[0]=1;
        }
        break;
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
