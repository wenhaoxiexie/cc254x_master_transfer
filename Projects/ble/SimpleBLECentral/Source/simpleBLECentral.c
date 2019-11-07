/**************************************************************************************************
  Filename:       simpleBLECentral.c
  Revised:        $Date: 2011-06-20 11:57:59 -0700 (Mon, 20 Jun 2011) $
  Revision:       $Revision: 28 $

  Description:    This file contains the Simple BLE Central sample application 
                  for use with the CC2540 Bluetooth Low Energy Protocol Stack.

  Copyright 2010 Texas Instruments In corporated. All rights reserved.

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
/*******************************************************************************
V1.1(2014-09-19)
1、波特率115200


*/




/*********************************************************************
 * INCLUDES
 */

#include "bcomdef.h"
#include "OSAL.h"
#include "OSAL_PwrMgr.h"
#include "OnBoard.h"
#include "hal_led.h"
#include "hal_key.h"
#include "hal_lcd.h"
#include "gatt.h"
#include "ll.h"
#include "hci.h"
#include "gapgattserver.h"
#include "gattservapp.h"
#include "central.h"
#include "gapbondmgr.h"
#include "simpleGATTprofile.h"
#include "simpleBLECentral.h"
#include "NPI.h"
#include "serial.h"
#include "string.h"

#include "order.h"
#include "stdlib.h"
#include "osal_snv.h"
/*********************************************************************
 * MACROS
 */

// Length of bd addr as a string
#define B_ADDR_STR_LEN                        15

/*********************************************************************
 * CONSTANTS
 */

// Maximum number of scan responses
#define DEFAULT_MAX_SCAN_RES                  10

// Scan duration in ms
#define DEFAULT_SCAN_DURATION                 5000   //默认扫描时间 ms

// Discovey mode (limited, general, all)
#define DEFAULT_DISCOVERY_MODE                DEVDISC_MODE_ALL

// TRUE to use active scan
#define DEFAULT_DISCOVERY_ACTIVE_SCAN         TRUE

// TRUE to use white list during discovery
#define DEFAULT_DISCOVERY_WHITE_LIST          FALSE

// TRUE to use high scan duty cycle when creating link
#define DEFAULT_LINK_HIGH_DUTY_CYCLE          FALSE

// TRUE to use white list when creating link
#define DEFAULT_LINK_WHITE_LIST               FALSE

// Default RSSI polling period in ms
#define DEFAULT_RSSI_PERIOD                   1000

// Whether to enable automatic parameter update request when a connection is formed
#define DEFAULT_ENABLE_UPDATE_REQUEST         FALSE

// Minimum connection interval (units of 1.25ms) if automatic parameter update request is enabled
#define DEFAULT_UPDATE_MIN_CONN_INTERVAL      8

// Maximum connection interval (units of 1.25ms) if automatic parameter update request is enabled
#define DEFAULT_UPDATE_MAX_CONN_INTERVAL      8

// Slave latency to use if automatic parameter update request is enabled
#define DEFAULT_UPDATE_SLAVE_LATENCY          0

// Supervision timeout value (units of 10ms) if automatic parameter update request is enabled
#define DEFAULT_UPDATE_CONN_TIMEOUT           200

// Default passcode
#define DEFAULT_PASSCODE                      19655

// Default GAP pairing mode
#define DEFAULT_PAIRING_MODE                  GAPBOND_PAIRING_MODE_WAIT_FOR_REQ

// Default MITM mode (TRUE to require passcode or OOB when pairing)
#define DEFAULT_MITM_MODE                     FALSE

// Default bonding mode, TRUE to bond
#define DEFAULT_BONDING_MODE                  TRUE

// Default GAP bonding I/O capabilities
#define DEFAULT_IO_CAPABILITIES               GAPBOND_IO_CAP_KEYBOARD_DISPLAY

// Default service discovery timer delay in ms
#define DEFAULT_SVC_DISCOVERY_DELAY           1000

// TRUE to filter discovery results on desired service UUID
#define DEFAULT_DEV_DISC_BY_SVC_UUID          TRUE



/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

 extern uint8  passcodebuff[7];

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */

// Task ID for internal task/event processing
uint8 simpleBLETaskId;

// GAP GATT Attributes
static const uint8 simpleBLEDeviceName[GAP_DEVICE_NAME_LEN] = "Simple BLE Central";

// Number of scan results and scan result index
uint8 simpleBLEScanRes;  //扫描结果
uint8 simpleBLEScanIdx;

// Scan result list
static gapDevRec_t simpleBLEDevList[DEFAULT_MAX_SCAN_RES];

// Scanning state
uint8 simpleBLEScanning = FALSE;

// RSSI polling state
uint8 simpleBLERssi = FALSE;

// Connection handle of current connection 
uint16 simpleBLEConnHandle = GAP_CONNHANDLE_INIT;

// Application state
uint8 simpleBLEState = BLE_STATE_IDLE;

// Discovery state
static uint8 simpleBLEDiscState = BLE_DISC_STATE_IDLE;

// Discovered service start and end handle
static uint16 simpleBLESvcStartHdl = 0;
static uint16 simpleBLESvcEndHdl = 0;

// Discovered characteristic handle
uint16 simpleBLECharHdl = 0;
//1.8版本使用的handle
uint16 simpleBLECharHd6[2]={0x0036,0x0023};
//1.7版本使用的handle
//uint16 simpleBLECharHd6 = 0x0023;
bool simpleBLEChar6DoWrite = TRUE;
//bool simpleBLECentralCanSend = FALSE;          //  主机可发送数据
// Value to write
//static uint8 simpleBLECharVal = 0;

// Value read/write toggle
// static bool simpleBLEDoWrite = FALSE;

// GATT read/write procedure state
 bool simpleBLEProcedureInProgress = FALSE;

bStatus_t STATUS = SUCCESS;

//static uint8 i;

uint8 My_BondAddr[7] = {0};
uint8 My_BondAddrType[2] = {0};


uint8 VER_FLAG = 0;

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void simpleBLECentralProcessGATTMsg( gattMsgEvent_t *pMsg );
static void simpleBLECentralRssiCB( uint16 connHandle, int8  rssi );
static void simpleBLECentralEventCB( gapCentralRoleEvent_t *pEvent );
static void simpleBLECentralPasscodeCB( uint8 *deviceAddr, uint16 connectionHandle,
                                        uint8 uiInputs, uint8 uiOutputs );
static void simpleBLECentralPairStateCB( uint16 connHandle, uint8 state, uint8 status );
static void simpleBLECentral_HandleKeys( uint8 shift, uint8 keys );
static void simpleBLECentral_ProcessOSALMsg( osal_event_hdr_t *pMsg );
static void simpleBLEGATTDiscoveryEvent( gattMsgEvent_t *pMsg );
static void simpleBLECentralStartDiscovery( void );
static bool simpleBLEFindSvcUuid( uint16 uuid, uint8 *pData, uint8 dataLen );
static void simpleBLEAddDeviceInfo( uint8 *pAddr, uint8 addrType );
char *bdAddr2Str ( uint8 *pAddr );



/*********************************************************************
 * PROFILE CALLBACKS
 */

// GAP Role Callbacks
static const gapCentralRoleCB_t simpleBLERoleCB =
{
  simpleBLECentralRssiCB,       // RSSI callback
  simpleBLECentralEventCB       // Event callback
};

// Bond Manager Callbacks
static const gapBondCBs_t simpleBLEBondCB =
{
  simpleBLECentralPasscodeCB,
  simpleBLECentralPairStateCB
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      SimpleBLECentral_Init
 *
 * @brief   Initialization function for the Simple BLE Central App Task.
 *          This is called during initialization and should contain
 *          any application specific initialization (ie. hardware
 *          initialization/setup, table initialization, power up
 *          notification).
 *
 * @param   task_id - the ID assigned by OSAL.  This ID should be
 *                    used to send messages and set timers.
 *
 * @return  none
 */
void SimpleBLECentral_Init( uint8 task_id )
{
  simpleBLETaskId = task_id;
  #if defined ADD_CC2590
  HCI_EXT_ExtendRfRangeCmd();
  TXPOWER = 0xF1;
  //设置接收灵敏度
  HCI_EXT_SetRxGainCmd(HCI_EXT_RX_GAIN_HIGH);
  #else
  P1SEL &= ~0x0E;
  P1DIR |= 0x0E;
  P1 |= 0;
  #endif
  
  P1SEL &= ~0xF1; // P1 1111 0001 ,P11\P12\P13 PA CONTROL
  P1DIR |= 0xF1; //P1 1111 0001,to output
  P1_6 = 0;

  //NPI_InitTransport(NpiSerialCallback);
  update_BPS();
  // Setup Central Profile
  {
    uint8 scanRes = DEFAULT_MAX_SCAN_RES;
    GAPCentralRole_SetParameter ( GAPCENTRALROLE_MAX_SCAN_RES, sizeof( uint8 ), &scanRes );
  }
  //设置断开连接反应时间
  GAP_SetParamValue(TGAP_CONN_EST_SUPERV_TIMEOUT, 100);
  // Setup GAP
  //设置扫描时间，这里设置扫面4秒钟
  GAP_SetParamValue( TGAP_GEN_DISC_SCAN, DEFAULT_SCAN_DURATION );
  GAP_SetParamValue( TGAP_LIM_DISC_SCAN, DEFAULT_SCAN_DURATION );
  GGS_SetParameter( GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, (uint8 *) simpleBLEDeviceName );

  // Setup the GAP Bond Manager
  {
    uint32 passkey = DEFAULT_PASSCODE;
    uint8 pairMode = DEFAULT_PAIRING_MODE;
    uint8 mitm = DEFAULT_MITM_MODE;
    uint8 ioCap = DEFAULT_IO_CAPABILITIES;
    uint8 bonding = DEFAULT_BONDING_MODE;
    GAPBondMgr_SetParameter( GAPBOND_DEFAULT_PASSCODE, sizeof( uint32 ), &passkey );
    GAPBondMgr_SetParameter( GAPBOND_PAIRING_MODE, sizeof( uint8 ), &pairMode );
    GAPBondMgr_SetParameter( GAPBOND_MITM_PROTECTION, sizeof( uint8 ), &mitm );
    GAPBondMgr_SetParameter( GAPBOND_IO_CAPABILITIES, sizeof( uint8 ), &ioCap );
    GAPBondMgr_SetParameter( GAPBOND_BONDING_ENABLED, sizeof( uint8 ), &bonding );
  }  

  // Initialize GATT Client
  VOID GATT_InitClient();

  // Register to receive incoming ATT Indications/Notifications
  GATT_RegisterForInd( simpleBLETaskId );

  // Initialize GATT attributes
  GGS_AddService( GATT_ALL_SERVICES );         // GAP
  GATTServApp_AddService( GATT_ALL_SERVICES ); // GATT attributes

  // Register for all key events - This app will handle all key events
  RegisterForKeys( simpleBLETaskId );
  
  // makes sure LEDs are off
  HalLedSet( (HAL_LED_1 | HAL_LED_2), HAL_LED_MODE_OFF );
  
  // Setup a delayed profile startup
  osal_set_event( simpleBLETaskId, START_DEVICE_EVT );
}

/*********************************************************************
 * @fn      SimpleBLECentral_ProcessEvent
 *
 * @brief   Simple BLE Central Application Task event processor.  This function
 *          is called to process all events for the task.  Events
 *          include timers, messages and any other user defined events.
 *
 * @param   task_id  - The OSAL assigned task ID.
 * @param   events - events to process.  This is a bit map and can
 *                   contain more than one event.
 *
 * @return  events not processed
 */
uint16 SimpleBLECentral_ProcessEvent( uint8 task_id, uint16 events )
{
  
  VOID task_id; // OSAL required parameter that isn't used in this function
  
  if ( events & SYS_EVENT_MSG )
  {
    uint8 *pMsg;

    if ( (pMsg = osal_msg_receive( simpleBLETaskId )) != NULL )
    {
      simpleBLECentral_ProcessOSALMsg( (osal_event_hdr_t *)pMsg );

      // Release the OSAL message
      VOID osal_msg_deallocate( pMsg );
    }

    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }

  if ( events & START_DEVICE_EVT )
  {
    uint8 M_Addr[7] = {0};
    uint8 M_Adrtype[2] = {0};
    // Start the Device启动设备时调用一次
    VOID GAPCentralRole_StartDevice( (gapCentralRoleCB_t *) &simpleBLERoleCB );

    // Register with bond manager after starting device
    GAPBondMgr_Register( (gapBondCBs_t *) &simpleBLEBondCB );
    
    osal_snv_read(BLE_BOND_DEVICE_ADR,7,M_Addr);
    osal_snv_read(BLE_BOND_DEVICE_ADRTYPE,1,M_Adrtype);


   // 开始时候连接一次
    if(M_Addr[0] == 1)
    {
      GAPCentralRole_EstablishLink( DEFAULT_LINK_HIGH_DUTY_CYCLE,
                                                        DEFAULT_LINK_WHITE_LIST,
                                                         M_Adrtype[0],&M_Addr[1] );
    
    }
   
/*
    if ( !simpleBLEScanning & simpleBLEScanRes == 0 )
    {
      simpleBLEScanning = TRUE;
      simpleBLEScanRes = 0;
      GAPCentralRole_StartDiscovery( DEFAULT_DISCOVERY_MODE,
                                     DEFAULT_DISCOVERY_ACTIVE_SCAN,
                                     DEFAULT_DISCOVERY_WHITE_LIST );   
     // LCD_WRITE_STRING( "Scanning...", HAL_LCD_LINE_1 );
    }
    else
    {
      LCD_WRITE_STRING( "No Scan", HAL_LCD_LINE_1 );
    }
*/
    return ( events ^ START_DEVICE_EVT );
  }

  if ( events & START_DISCOVERY_EVT )
  {
    simpleBLECentralStartDiscovery( );
   
    return ( events ^ START_DISCOVERY_EVT );
  }
  //Uart Buff pro
  if ( events & SBC_UART_EVT )
  {
    uart_remain_len = Rx_q - SerialRxBuff;
    //减去偏移地址
    uart_remain_len -= offs_RxBuf;
    
    //解析命令
    if(Uart_Command(SerialRxBuff,0) != STATUS_CMD_ERR)
    {
      Rx_q = SerialRxBuff;
      uart_remain_len =0;
      memset(SerialRxBuff,0,200);
      return (events ^ SBC_UART_EVT);
    }
    
    //连续判断4次，一个连接间隔最多发4个包
  
   if(uart_remain_len >= 20)
    {
       SendData(20);
    }
    if(uart_remain_len >= 20)
    {
       SendData(20);
    } 
    if(uart_remain_len >= 20)
    {
       SendData(20);
    }
    if( uart_remain_len >=20)
    {
       SendData(20);
      if (uart_remain_len>0) 
      { 
        osal_start_timerEx( simpleBLETaskId, SBC_UART_EVT, 40 );
      }
    }
    else if (uart_remain_len>0) 
    {
      SendData(uart_remain_len);
     // if(STATUS == SUCCESS)
     // {
        Rx_q = SerialRxBuff;
        offs_RxBuf=0;
        memset(SerialRxBuff,0,200); 
        
    //  }
     // else
     // {
    //    osal_set_event(simpleBLETaskId,SBC_UART_EVT);
        //STATUS = SUCCESS; 
     // }
    }
   
    return ( events ^ SBC_UART_EVT );
  }
  
  
  if ( events & CONNECT_BOUND_EVENT )
  {
    osal_start_timerEx( simpleBLETaskId, CONNECT_BOUND_EVENT,1000);
    uint8 My_Addr[7] = {0};
    uint8 My_Adrtype[2] = {0};
    osal_snv_read(BLE_BOND_DEVICE_ADR,7,My_Addr);
    osal_snv_read(BLE_BOND_DEVICE_ADRTYPE,1,My_Adrtype);
    
    if(My_Addr[0] == 1)
    {
      GAPCentralRole_EstablishLink( DEFAULT_LINK_HIGH_DUTY_CYCLE,
                                        DEFAULT_LINK_WHITE_LIST,
                                        My_Adrtype[0],&My_Addr[1] );
    }  
    return ( events ^ CONNECT_BOUND_EVENT );
  }
  
  
  // Discard unknown events
  return 0;
}

/*********************************************************************
 * @fn      simpleBLECentral_ProcessOSALMsg
 *
 * @brief   Process an incoming task message.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
static void simpleBLECentral_ProcessOSALMsg( osal_event_hdr_t *pMsg )
{
  switch ( pMsg->event )
  {
    case KEY_CHANGE:
      simpleBLECentral_HandleKeys( ((keyChange_t *)pMsg)->state, ((keyChange_t *)pMsg)->keys );
      break;

    case GATT_MSG_EVENT:
      simpleBLECentralProcessGATTMsg( (gattMsgEvent_t *) pMsg );
      break;
  }
}

/*********************************************************************
 * @fn      simpleBLECentral_HandleKeys
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
uint8 gStatus;
static void simpleBLECentral_HandleKeys( uint8 shift, uint8 keys )
{
  (void)shift;  // Intentionally unreferenced parameter
  /*

  if ( keys & HAL_KEY_SW_1 )   
  {
    attWriteReq_t AttReq;       
    uint8 ValueBuf[2];
       
    AttReq.handle = 0x0039;
    AttReq.len = 2;
    AttReq.sig = 0;
    AttReq.cmd = 0;
    ValueBuf[0] = 0x01;
    ValueBuf[1] = 0x00;
    osal_memcpy(AttReq.value,ValueBuf,2);
    GATT_WriteCharValue( 0, &AttReq, simpleBLETaskId );
   // NPI_WriteTransport("Enable Notice\n", 14 ); 
  }
  
  if ( keys & HAL_KEY_SW_2 )   
  {
    NPI_WriteTransport("KEY K2\n",7);
  }
*/
}

/*********************************************************************
 * @fn      simpleBLECentralProcessGATTMsg
 *
 * @brief   Process GATT messages
 *
 * @return  none
 */
static void simpleBLECentralProcessGATTMsg( gattMsgEvent_t *pMsg )
{
  if ( simpleBLEState != BLE_STATE_CONNECTED )
  {
    // In case a GATT message came after a connection has dropped,
    // ignore the message
    return;
  }
  
  if ( ( pMsg->method == ATT_READ_RSP ) ||
       ( ( pMsg->method == ATT_ERROR_RSP ) &&
         ( pMsg->msg.errorRsp.reqOpcode == ATT_READ_REQ ) ) )
  {
    if ( pMsg->method == ATT_ERROR_RSP )
    {
      uint8 status = pMsg->msg.errorRsp.errCode;
      
      LCD_WRITE_STRING_VALUE( "Read Error", status, 10, HAL_LCD_LINE_1 );
    }
    else
    {
      // After a successful read, display the read value
      uint8 valueRead = pMsg->msg.readRsp.value[0];

      LCD_WRITE_STRING_VALUE( "Read rsp:", valueRead, 10, HAL_LCD_LINE_1 );
    }
    
    simpleBLEProcedureInProgress = FALSE;
  }
  else if ( ( pMsg->method == ATT_WRITE_RSP ) ||
       ( ( pMsg->method == ATT_ERROR_RSP ) &&
         ( pMsg->msg.errorRsp.reqOpcode == ATT_WRITE_REQ ) ) )
  {
    
    if ( pMsg->method == ATT_ERROR_RSP == ATT_ERROR_RSP )
    {
    // uint8 status = pMsg->msg.errorRsp.errCode;
    //这个地方会出现Write Error 3   如果通道写错的话  
    // LCD_WRITE_STRING_VALUE( "Write Error", status, 10, HAL_LCD_LINE_1 );
    }
    else
    {
      // After a succesful write, display the value that was written and increment value
      //LCD_WRITE_STRING_VALUE( "Write sent:", simpleBLECharVal++, 10, HAL_LCD_LINE_1 );    
    
      // 这个变量用于表明上一次写数据到从机已经成功， 可用于判断写数据时的判断， 以确保数据的完整性  
      simpleBLEChar6DoWrite = TRUE;
    }
    
    simpleBLEProcedureInProgress = FALSE;    

  }
  else if ( simpleBLEDiscState != BLE_DISC_STATE_IDLE )
  {
    simpleBLEGATTDiscoveryEvent( pMsg );
  }
  else if ( ( pMsg->method == ATT_HANDLE_VALUE_NOTI ) ||
          (pMsg->method == ATT_HANDLE_VALUE_IND) )  
  {
    attHandleValueNoti_t noti;
    noti.handle = pMsg->msg.handleValueNoti.handle;
    noti.len = pMsg->msg.handleValueNoti.len;
    osal_memcpy(&noti.value,&pMsg->msg.handleValueNoti.value,noti.len);
    NPI_WriteTransport(noti.value,noti.len);
    
  }
    
  
}

/*********************************************************************
 * @fn      simpleBLECentralRssiCB
 *
 * @brief   RSSI callback.
 *
 * @param   connHandle - connection handle
 * @param   rssi - RSSI
 *
 * @return  none
 */
static void simpleBLECentralRssiCB( uint16 connHandle, int8 rssi )
{
    LCD_WRITE_STRING_VALUE( "RSSI -dB:", (uint8) (-rssi), 10, HAL_LCD_LINE_1 );   
}

/*********************************************************************
 * @fn      simpleBLECentralEventCB
 *
 * @brief   Central event callback function.
 *
 * @param   pEvent - pointer to event structure
 *
 * @return  none
 */

static void simpleBLECentralEventCB( gapCentralRoleEvent_t *pEvent )
{
  
  static uint8 i=0;
  switch ( pEvent->gap.opcode )
  {
    case GAP_DEVICE_INIT_DONE_EVENT:  
      {
       // LCD_WRITE_STRING( "BLE Central", HAL_LCD_LINE_1 );
        //主机模块儿的地址
       // LCD_WRITE_STRING( bdAddr2Str( pEvent->initDone.devAddr ),  HAL_LCD_LINE_2 );
      }
      break;

    case GAP_DEVICE_INFO_EVENT:
      { 
           LCD_WRITE_STRING( "device discovering", HAL_LCD_LINE_2 );
           char Device_Name[15] = {0};

           if(pEvent->deviceInfo.eventType == GAP_ADRPT_SCAN_RSP )       //scan_rsp  
           {  
                 LCD_WRITE_STRING_VALUE( "NUM:",i++,10,HAL_LCD_LINE_2 );
                 HalUARTWrite(HAL_UART_PORT_0,"  ",2);
                 
                 
                 HalUARTWrite(HAL_UART_PORT_0,"NAME:",5);
                uint8 a = 0; 
                for(a = 0; a <  ((uint8)(pEvent->deviceInfo.pEvtData[0]) - 1); a++)
                {
                    Device_Name[a] = pEvent->deviceInfo.pEvtData[a + 2];
                }
                
                LCD_WRITE_STRING(Device_Name,HAL_LCD_LINE_2); 
                HalUARTWrite(HAL_UART_PORT_0,"  ",2); 
                
                HalUARTWrite(HAL_UART_PORT_0,"MAC:",4);
                LCD_WRITE_STRING( bdAddr2Str( pEvent->deviceInfo.addr ) + 2, HAL_LCD_LINE_2 );
                HalUARTWrite(HAL_UART_PORT_0,"\r\n",2);
                  
 
                 
                LCD_WRITE_STRING_VALUE( "rssi:-",(uint8)(-( pEvent->deviceInfo.rssi )),10,HAL_LCD_LINE_2 );
                 
                // HalUARTWrite(HAL_UART_PORT_0,"dB\r\n",4);
             // LCD_WRITE_STRING(  (char *)(pEvent->deviceInfo.pEvtData) , HAL_LCD_LINE_2 );
            //将查找到的设备保存到设备列表中(保存MAC地址和设备类型)
            //simpleBLEAddDeviceInfo( pEvent->deviceInfo.addr, pEvent->deviceInfo.addrType ); 

           } 
      }
      break;

    case GAP_DEVICE_DISCOVERY_EVENT:
      {
        i=0;
        // discovery complete
        simpleBLEScanning = FALSE;
        // if not filtering device discovery results based on service UUID
        if ( DEFAULT_DEV_DISC_BY_SVC_UUID == FALSE )
        {
          // Copy results
          simpleBLEScanRes = pEvent->discCmpl.numDevs;
          osal_memcpy( simpleBLEDevList, pEvent->discCmpl.pDevList,
                       (sizeof( gapDevRec_t ) * pEvent->discCmpl.numDevs) );
        }
        
        if ( simpleBLEScanRes > 0 )
        {
            //LCD_WRITE_STRING( "<- To Select", HAL_LCD_LINE_2 );
            //HalLedSet(HAL_LED_3, HAL_LED_MODE_ON );   
            if ( simpleBLEState == BLE_STATE_IDLE )
            {                      
             simpleBLEScanIdx = 0;

            }
            
            //HalLedSet(HAL_LED_3, HAL_LED_MODE_OFF ); 
        }
      }
      break;
      

    case GAP_LINK_ESTABLISHED_EVENT:    //连接结束事件
      {
        P1_6 = 1;
        
        LCD_WRITE_STRING( "Connected\r\n", HAL_LCD_LINE_1 );
        if ( pEvent->gap.hdr.status == SUCCESS )
        {          
          simpleBLEState = BLE_STATE_CONNECTED;
          simpleBLEConnHandle = pEvent->linkCmpl.connectionHandle;
          simpleBLEProcedureInProgress = TRUE;    

          // If service discovery not performed initiate service discovery
          if ( simpleBLECharHdl == 0 )
          {
            osal_start_timerEx( simpleBLETaskId, START_DISCOVERY_EVT, DEFAULT_SVC_DISCOVERY_DELAY );
          }
          osal_stop_timerEx(simpleBLETaskId, CONNECT_BOUND_EVENT);
         //连接上显示连接状态和站到的从机的地址。     
         // LCD_WRITE_STRING( "Connected", HAL_LCD_LINE_1 );
         // LCD_WRITE_STRING( bdAddr2Str( pEvent->linkCmpl.devAddr ), HAL_LCD_LINE_2 );  
         // HalLedSet(HAL_LED_3, HAL_LED_MODE_ON );   
        }
        else
        {
          simpleBLEState = BLE_STATE_IDLE;
          simpleBLEConnHandle = GAP_CONNHANDLE_INIT;
          simpleBLERssi = FALSE;
          simpleBLEDiscState = BLE_DISC_STATE_IDLE;
          
          LCD_WRITE_STRING( "Connect Failed", HAL_LCD_LINE_1 );
          //LCD_WRITE_STRING_VALUE( "Reason:", pEvent->gap.hdr.status, 10, HAL_LCD_LINE_2 );
          //HalLedSet(HAL_LED_3, HAL_LED_MODE_OFF );   
        }
      }
      break;
//断开连接完成事件
    case GAP_LINK_TERMINATED_EVENT:
      {
        simpleBLEState = BLE_STATE_IDLE;
        simpleBLEConnHandle = GAP_CONNHANDLE_INIT;
        simpleBLERssi = FALSE;
        simpleBLEDiscState = BLE_DISC_STATE_IDLE;
        simpleBLECharHdl = 0;
        simpleBLEProcedureInProgress = FALSE;
        P1_6 = 0;  
        LCD_WRITE_STRING( "Disconnected\r\n", HAL_LCD_LINE_1 );
        //LCD_WRITE_STRING_VALUE( "Reason:", pEvent->linkTerminate.reason,
         //                       10, HAL_LCD_LINE_2 );
        
     //   HalLedSet(HAL_LED_3, HAL_LED_MODE_OFF );  
        
        simpleBLEScanRes = 0;
        simpleBLEScanning = 0;
        
        osal_start_timerEx( simpleBLETaskId, CONNECT_BOUND_EVENT,100);
   /* 
       //断开连接后再重新扫描   
      GAPCentralRole_StartDiscovery( DEFAULT_DISCOVERY_MODE,
                                     DEFAULT_DISCOVERY_ACTIVE_SCAN,
                                     DEFAULT_DISCOVERY_WHITE_LIST );   
      LCD_WRITE_STRING( "Scanning...", HAL_LCD_LINE_1 );

      }
   */
      break;

    case GAP_LINK_PARAM_UPDATE_EVENT:
      {
        attWriteReq_t AttReq;       
        uint8 ValueBuf[2];
        //打开指定通道的notify开关   
        AttReq.handle = simpleBLECharHd6[VER_FLAG];
        AttReq.len = 2;
        AttReq.sig = 0;
        AttReq.cmd = 0;
        ValueBuf[0] = 0x01;
        ValueBuf[1] = 0x00;
        osal_memcpy(AttReq.value,ValueBuf,2);
       // GATT_WriteCharValue( 0, &AttReq, simpleBLETaskId );
      //  NPI_WriteTransport("Enable Notice\n", 14 ); 
      }
      break;
      
    default:
      break;
  }
 }
}

/*********************************************************************
 * @fn      pairStateCB
 *
 * @brief   Pairing state callback.
 *
 * @return  none
 */
static void simpleBLECentralPairStateCB( uint16 connHandle, uint8 state, uint8 status )
{
  if ( state == GAPBOND_PAIRING_STATE_STARTED )
  {
    LCD_WRITE_STRING( "Pairing started\r\n", HAL_LCD_LINE_1 );
  }
  else if ( state == GAPBOND_PAIRING_STATE_COMPLETE )
  {
    if ( status == SUCCESS )
    {
      LCD_WRITE_STRING( "Pairing success\r\n", HAL_LCD_LINE_1 );
    }
    else
    {
      LCD_WRITE_STRING_VALUE( "Pairing fail\r\n", status, 10, HAL_LCD_LINE_1 );
    }
  }
  else if ( state == GAPBOND_PAIRING_STATE_BONDED )
  {
    if ( status == SUCCESS )
    {
      LCD_WRITE_STRING( "Bonding success\r\n", HAL_LCD_LINE_1 );
    }
  }
}

/*********************************************************************
 * @fn      simpleBLECentralPasscodeCB
 *
 * @brief   Passcode callback.
 *
 * @return  none
 */
static void simpleBLECentralPasscodeCB( uint8 *deviceAddr, uint16 connectionHandle,
                                        uint8 uiInputs, uint8 uiOutputs )
{
//#if (HAL_LCD == TRUE)

  uint32  passcode;
  //uint8   str[7];

  // Create random passcode
  //LL_Rand( ((uint8 *) &passcode), sizeof( uint32 ));
  //passcode %= 1000000;
  
  passcode = ((uint32)passcodebuff[0]-48)*100000+((uint32)passcodebuff[1]-48)*10000+
             ((uint32)passcodebuff[2]-48)*1000+((uint32)passcodebuff[3]-48)*100+((uint32)passcodebuff[4]-48)*10
             +((uint32)passcodebuff[5]-48);
  
  // Display passcode to user
  //if ( uiOutputs != 0 )
  //{
  //  LCD_WRITE_STRING( "Passcode:",  HAL_LCD_LINE_1 );
  //  LCD_WRITE_STRING( (char *) _ltoa(passcode, str, 10),  HAL_LCD_LINE_2 );
 // }
  
  // Send passcode response
  GAPBondMgr_PasscodeRsp( connectionHandle, SUCCESS, passcode );
//#endif
}

/*********************************************************************
 * @fn      simpleBLECentralStartDiscovery
 *
 * @brief   Start service discovery.
 *
 * @return  none
 */
static void simpleBLECentralStartDiscovery( void )
{
  uint8 uuid[ATT_BT_UUID_SIZE] = { LO_UINT16(SIMPLEPROFILE_SERV_UUID),
                                   HI_UINT16(SIMPLEPROFILE_SERV_UUID) };
  
  // Initialize cached handles
  simpleBLESvcStartHdl = simpleBLESvcEndHdl = simpleBLECharHdl = 0;

  simpleBLEDiscState = BLE_DISC_STATE_SVC;
  
  // Discovery simple BLE service
  GATT_DiscPrimaryServiceByUUID( simpleBLEConnHandle,
                                 uuid,
                                 ATT_BT_UUID_SIZE,
                                 simpleBLETaskId );
}

/*********************************************************************
 * @fn      simpleBLEGATTDiscoveryEvent
 *
 * @brief   Process GATT discovery event
 *
 * @return  none
 */
static void simpleBLEGATTDiscoveryEvent( gattMsgEvent_t *pMsg )
{
  attReadByTypeReq_t req;
  
  if ( simpleBLEDiscState == BLE_DISC_STATE_SVC )
  {
    // Service found, store handles
    if ( pMsg->method == ATT_FIND_BY_TYPE_VALUE_RSP &&
         pMsg->msg.findByTypeValueRsp.numInfo > 0 )
    {
      simpleBLESvcStartHdl = pMsg->msg.findByTypeValueRsp.handlesInfo[0].handle;
      simpleBLESvcEndHdl = pMsg->msg.findByTypeValueRsp.handlesInfo[0].grpEndHandle;
    }
    
    // If procedure complete
    if ( ( pMsg->method == ATT_FIND_BY_TYPE_VALUE_RSP  && 
           pMsg->hdr.status == bleProcedureComplete ) ||
         ( pMsg->method == ATT_ERROR_RSP ) )
    {
      if ( simpleBLESvcStartHdl != 0 )
      {
        // Discover characteristic
        simpleBLEDiscState = BLE_DISC_STATE_CHAR;
        
        req.startHandle = simpleBLESvcStartHdl;
        req.endHandle = simpleBLESvcEndHdl;
        req.type.len = ATT_BT_UUID_SIZE;
        req.type.uuid[0] = LO_UINT16(SIMPLEPROFILE_CHAR6_UUID);
        req.type.uuid[1] = HI_UINT16(SIMPLEPROFILE_CHAR6_UUID);

        GATT_ReadUsingCharUUID( simpleBLEConnHandle, &req, simpleBLETaskId );
      }
    }
  }
  else if ( simpleBLEDiscState == BLE_DISC_STATE_CHAR )
  {
    // Characteristic found, store handle
    if ( pMsg->method == ATT_READ_BY_TYPE_RSP && 
         pMsg->msg.readByTypeRsp.numPairs > 0 )
    {
      simpleBLECharHd6[VER_FLAG] = BUILD_UINT16( pMsg->msg.readByTypeRsp.dataList[0],
                                       pMsg->msg.readByTypeRsp.dataList[1] );
      
      LCD_WRITE_STRING( "Simple Svc Found", HAL_LCD_LINE_1 );
      simpleBLEProcedureInProgress = FALSE;
    }
    
    simpleBLEDiscState = BLE_DISC_STATE_IDLE;

    
  }    
}


/*********************************************************************
 * @fn      simpleBLEFindSvcUuid
 *
 * @brief   Find a given UUID in an advertiser's service UUID list.
 *
 * @return  TRUE if service UUID found
 */
static bool simpleBLEFindSvcUuid( uint16 uuid, uint8 *pData, uint8 dataLen )
{
  uint8 adLen;
  uint8 adType;
  
  uint8 *pEnd;
  
  pEnd = pData + dataLen - 1;
  
  // While end of data not reached
  while ( pData < pEnd )
  {
    // Get length of next AD item
    adLen = *pData++;
    if ( adLen > 0 )
    {
      adType = *pData;
      
      // If AD type is for 16-bit service UUID
      if ( adType == GAP_ADTYPE_16BIT_MORE || adType == GAP_ADTYPE_16BIT_COMPLETE )
      {
        pData++;
        adLen--;
        
        // For each UUID in list
        while ( adLen >= 2 && pData < pEnd )
        {
          // Check for match
          if ( pData[0] == LO_UINT16(uuid) && pData[1] == HI_UINT16(uuid) )
          {
            // Match found
            return TRUE;
          }
          
          // Go to next
          pData += 2;
          adLen -= 2;
        }
        
        // Handle possible erroneous extra byte in UUID list
        if ( adLen == 1 )
        {
          pData++;
        }
      }
      else
      {
        // Go to next item
        pData += adLen;
      }
    }
  }
  
  // Match not found
  return FALSE;
}

/*********************************************************************
 * @fn      simpleBLEAddDeviceInfo
 *
 * @brief   Add a device to the device discovery result list
 *
 * @return  none
 */
static void simpleBLEAddDeviceInfo( uint8 *pAddr, uint8 addrType )
{
  uint8 i;
  
  // If result count not at max
  if ( simpleBLEScanRes < DEFAULT_MAX_SCAN_RES )
  {
    // Check if device is already in scan results
    for ( i = 0; i < simpleBLEScanRes; i++ )
    {
      if ( osal_memcmp( pAddr, simpleBLEDevList[i].addr , B_ADDR_LEN ) )
      {
        return;
      }
    }
    
    // Add addr to scan result list
    osal_memcpy( simpleBLEDevList[simpleBLEScanRes].addr, pAddr, B_ADDR_LEN );
    simpleBLEDevList[simpleBLEScanRes].addrType = addrType;
    
    // Increment scan result count
    simpleBLEScanRes++;
  }
}


/*********************************************************************
 * @fn      bdAddr2Str
 *
 * @brief   Convert Bluetooth address to string
 *
 * @return  none
 */
char *bdAddr2Str( uint8 *pAddr )
{
  uint8       i;
  char        hex[] = "0123456789ABCDEF";
  static char str[B_ADDR_STR_LEN];
  char        *pStr = str;
  
  *pStr++ = '0';
  *pStr++ = 'x';
  
  // Start from end of addr
  pAddr += B_ADDR_LEN;
  
  for ( i = B_ADDR_LEN; i > 0; i-- )
  {
    *pStr++ = hex[*--pAddr >> 4];
    *pStr++ = hex[*pAddr & 0x0F];
  }
  
  *pStr = 0;
  
  return str;
}

/*********************************************************************
 *  函数名：SendData
 *  功能：向指定handle(发送数据
 *  参数：数据长度send_len
 */

void SendData(uint8 send_len)
{
  
  attWriteReq_t AttReq;
  uint8 *p = AttReq.value;      
  for (uint8 i=0;i<send_len;i++) 
  {
    *p++=SerialRxBuff[i+offs_RxBuf];
  }  
    
  AttReq.handle = simpleBLECharHd6[VER_FLAG];
  AttReq.len = send_len;
  AttReq.sig = false;
  AttReq.cmd = true;    //如果要使用 GATT_WriteCharValue函数此处设置为false
  //STATUS = GATT_WriteCharValue( 0, &AttReq, simpleBLETaskId );
  STATUS = GATT_WriteNoRsp( 0, &AttReq ); 
  uart_remain_len-=send_len;
  offs_RxBuf+=send_len; 
 /*
  //发送失败，重发
  if(STATUS == SUCCESS)
  {
    uart_remain_len-=send_len;
    offs_RxBuf+=send_len; 
  }
  else
  {
    STATUS = SUCCESS;
  }
 */
}


void simpleBLEStartScan()
{
    simpleBLEScanRes = 0;
    if ( !simpleBLEScanning & simpleBLEScanRes == 0 )
    {
      simpleBLEScanning = TRUE;
      simpleBLEScanRes = 0;
      GAPCentralRole_StartDiscovery( DEFAULT_DISCOVERY_MODE,
                                     DEFAULT_DISCOVERY_ACTIVE_SCAN,
                                     DEFAULT_DISCOVERY_WHITE_LIST );   
    }
    else
    {
      LCD_WRITE_STRING( "No Scan", HAL_LCD_LINE_1 );
    }
}




/*********************************************************************
 * @fn      Change_ASCII_HEX
 *
 * @brief   将ASCII码转换成16进制数
 *
 * @param   p转换后存储的位置，q要转换的字符串,ss要转换的长度
 *
 * @return  none
 */

/*********************************************************************
*********************************************************************/
void Change_ASCII_HEX(uint8 *p,uint8 *q,int ss)
{
      int i = 0;  
      while(i < ss)
    {
          if(q[i]>='0' && q[i]<='9')
          {
             p[i]=q[i]-'0' + 0x00; 
          }
           else if(q[i]>='a' && q[i]<='f')
          {
             p[i]=q[i]-'a'+0x0a;
          }
           else if(q[i]>='A' && q[0]<='F')
          {
             p[i]=q[i]-'A'+0x0a; 
          }
     i++;

    }
}

/*********************************************************************
 * @fn      toadd
 *
 * @brief   将相邻的两个16进制数相加
 *
 * @param   p转换后存储的位置，q要转换的16进制数组,ss要转换的长度
 *
 * @return  none
 */

/*********************************************************************
*********************************************************************/
void toadd(uint8*p,uint8 *q,int ss)
{
      int i = 0;
      int j = ss/2 - 1;  
      while(i < ss)
     {
      p[j] = q[i] * 0x10 + q[i + 1];
      j--;
      i += 2;
     }
}


/*********************************************************************
 *  函数名：User_Connect
 *  功能：用户主动连接想要连接的从机
 *  参数：从机的MAC地址
 */
void User_Connect(uint8* FoundMAC)
{
   int i = 0;
   uint8 addrType;
   uint8 peerAddr_12[13] = {0};
   uint8 peerAddr[6] = {0};
   simpleBLEState = BLE_STATE_CONNECTING;
   
   Change_ASCII_HEX(peerAddr_12,FoundMAC,12);
   toadd(peerAddr,peerAddr_12,12);
   
   
   addrType = 0x00;
   for(i = 0; i < 6; i++)
   {
    My_BondAddr[i + 1] = peerAddr[i];
   }
   
   My_BondAddr[0] = 1;
   My_BondAddrType[0] = addrType;
   osal_snv_write(BLE_BOND_DEVICE_ADR,7,My_BondAddr);
   osal_snv_write(BLE_BOND_DEVICE_ADRTYPE,1,My_BondAddrType);
   
 GAPCentralRole_EstablishLink( DEFAULT_LINK_HIGH_DUTY_CYCLE,
                                      DEFAULT_LINK_WHITE_LIST,
                                           addrType, peerAddr );
  
  HalUARTWrite(HAL_UART_PORT_0,"Connecting\r\n",10);
   
}








/*********************************************************************
*********************************************************************/

