//******************************************************************************        
//name:         Central_Order.c        
//introduce:    主机AT指令任务    
//author:       ZXL      
//changetime:   2016年7月30日15:06:22    
//******************************************************************************  
  
/*********************头文件************************/   
#include "order.h"  
#include "OSAL.h"  
#include "ioCC2541.h"
#include "simpleBLECentral.h"
#include "central.h"
#include "simpleGATTprofile.h"
#include "osal_snv.h"
#include "string.h"
#include "hal_uart.h"
#include "serial.h"
/*********************宏定义************************/   



/***************内部变量声明************************/   
static uint8 order_TaskID;  

/***************内部函数声明************************/ 
static void order_ProcessOSALMsg( osal_event_hdr_t *pMsg ); 


/***************外部变量声明************************/ 
extern uint8 simpleBLEScanning;
extern uint8 simpleBLEScanRes;
extern uint16 simpleBLEConnHandle;
extern uint8 My_BondAddr[7];
extern uint8 My_BondAddrType[2];
uint8  passcodebuff[7];
extern uint8 VER_FLAG;
//******************************************************************************            
//name:             order_Init            
//introduce:        主机AT指令任务初始化           
//parameter:        task_id:任务的ID           
//return:           none          
//author:           ZXL       
//changetime:       2016年7月30日15:12:20       
//******************************************************************************  
void order_Init( uint8 task_id )  
{  
  order_TaskID = task_id;     
} 

//******************************************************************************            
//name:             order_ProcessEvent            
//introduce:        主机任务处理事件          
//parameter:        task_id:任务的ID,         
//parameter：       events:事件  
//return:           返回执行后的事件状态          
//author:           ZXL                 
//******************************************************************************  
uint16 order_ProcessEvent( uint8 task_id, uint16 events )  
{  
  VOID task_id;   
  
  //系统事件，首先要处理系统事件
  if ( events & SYS_EVENT_MSG )  
  { 
    
    uint8 *pMsg;  
  
    if ( (pMsg = osal_msg_receive( order_TaskID )) != NULL )  
    {  
      
     order_ProcessOSALMsg( (osal_event_hdr_t *)pMsg );  
  
      VOID osal_msg_deallocate( pMsg );  
    }
     return (events ^ SYS_EVENT_MSG); 
  }  
    
   if ( events & bps_data_event)
  {
    //更新波特率并启动串口
    update_BPS();
    return (events ^ bps_data_event);
  }
  
  

    
  return 0;  
}


//******************************************************************************            
//name:             order_ProcessOSALMsg            
//introduce:        主机任务的消息处理           
//parameter:        pMsg:消息           
//return:           none          
//author:           ZXL                 
//******************************************************************************  

static void order_ProcessOSALMsg( osal_event_hdr_t *pMsg )  
{  
  switch ( pMsg->event )  
  {  
    default:  
      break;  
  }  
} 


/*********************************************************************
 * 函数名称：Uart_Command
 * 功能：分析串口数据，解析命令
 * 返回：命令状态
 */
uint8 Uart_Command(uint8 *p,uint8 typecommand)
{
  uint8 i=0;
  uint8 * OffData_Buff;
  OffData_Buff=p;
  if(!osal_memcmp(OffData_Buff, "AT+", 3)  )
  {
    return STATUS_CMD_ERR;
  }
  else
  {  
         OffData_Buff+=3;
         while(((*OffData_Buff) != '='))
         {
             if((*OffData_Buff)=='\0')
             {
               *OffData_Buff = '=';
             }
              else
              {
                OffData_Buff++;
                i++;
              }
         }
             OffData_Buff-=i;
              i--;
    /*******************判断命令类型*******************************************/
    //开始扫描
    if(osal_memcmp(OffData_Buff,"SCAN=",5))
    {
      OffData_Buff+=5;
      Uart_CommandService(STATUS_ORD_SCAN,OffData_Buff,typecommand);
      return STATUS_ORD_SCAN;
    } 
    
    //主机断开连接
    if(osal_memcmp(OffData_Buff,"DSCN=",5))
    {
      OffData_Buff+=6;
      Uart_CommandService(STATUS_ORD_DSCN,OffData_Buff,typecommand);
      return STATUS_ORD_DSCN;
    } 
    
    //主机连接显示的地址的编号的从机
    if(osal_memcmp(OffData_Buff,"CONN=",5))
    {
      OffData_Buff+=5;
      Uart_CommandService(STATUS_ORD_CONN,OffData_Buff,typecommand);
      return STATUS_ORD_CONN;
    }
    
     //主机上电连接解除绑定
    if(osal_memcmp(OffData_Buff,"CLEAR=",6))
    {
      OffData_Buff+=6;
      Uart_CommandService(STATUS_ORD_CLEAR,OffData_Buff,typecommand);
      return STATUS_ORD_DSCN;
    } 
     //测试指令
    if(osal_memcmp(OffData_Buff,"TEST",4))
    {
      OffData_Buff+=4;
      Uart_CommandService(STATUS_ORD_TEST,OffData_Buff,typecommand);
      return STATUS_ORD_DSCN;
    } 
    //修改串口波特率
    if(osal_memcmp(OffData_Buff, "BPS",3))
    {
      OffData_Buff+=4;
      Uart_CommandService(STATUS_CMD_BPS,OffData_Buff,typecommand);
      return STATUS_CMD_BPS;
    }
    
    //查询角色名称
    if(osal_memcmp(OffData_Buff,"ROLE",4))
    {
      OffData_Buff+=5;
      Uart_CommandService(STATUS_FD_ROLE,OffData_Buff,typecommand);
      return STATUS_FD_ROLE;
    }    
    
     //切换通信版本
    if(osal_memcmp(OffData_Buff,"CGVER",5))
    {
      OffData_Buff+=6;
      Uart_CommandService(STATUS_CG_VER,OffData_Buff,typecommand);
      return STATUS_CG_VER;
    }   
    
    
    
    
    
    
    
    
    return STATUS_CMD_ERR;
  }
}


/*********************************************************************
 * 函数名称：Uart_CommandService
 * 功能：分析串口命令，调用处理函数
 * 返回：
 */
void Uart_CommandService(uint8 Command , uint8 *DataBuff , uint8 typecommand)
{  
  switch(Command)
  {
    case STATUS_ORD_SCAN:
      Ord_Scan(typecommand);                   //控制主机开始扫描
      break;
      
    case STATUS_ORD_DSCN:
      Ord_Disconnect(typecommand);             //控制主机主动断开连接
      break;
      
    case STATUS_ORD_CONN:
      Ord_Connecting(DataBuff,typecommand);    //控制主机主动断开连接
      break;
      
    case STATUS_ORD_CLEAR:
      Ord_Clear(typecommand);                 //主机清除上电连接命令
      break;
      
    case STATUS_ORD_TEST:
      Ord_Test(typecommand);                 //测试串口指令
      break; 
      
    case STATUS_CMD_BPS:
      Ord_Modify_BPS( DataBuff,typecommand );       //修改波特率
      break; 
      
    case STATUS_FD_ROLE:
      CMD_Find_ROLE( typecommand );             //查询角色
      break;   
      
    case STATUS_CG_VER:
      CMD_CG_VER( typecommand );             //切换通信版本
      break;         
      
    default:
      break;
  }
}

/*******************************************************************
*      函数名称：update_BPS
*      功能：串口波特率更新
*/
void update_BPS()
{
  static uint8 my_baudRate;
  uint8 SC_BR[6]={0};
  osal_snv_read(BLE_NVID_BPS_PARA,6,SC_BR);
    //判断波特率，有变化就更新
  if(SC_BR[0]!='\0')
  { 
    if(osal_memcmp(SC_BR,"4800",4))
      my_baudRate=HAL_UART_BR_4800;
      
    if(osal_memcmp(SC_BR,"9600",4))
      my_baudRate=HAL_UART_BR_9600;
    
    if(osal_memcmp(SC_BR,"19200",5))
      my_baudRate=HAL_UART_BR_19200;
    
    if(osal_memcmp(SC_BR,"38400",5))
      my_baudRate=HAL_UART_BR_38400;
    
    if(osal_memcmp(SC_BR,"57600",5))
      my_baudRate=HAL_UART_BR_57600;
    
    if(osal_memcmp(SC_BR,"115200",6))
      my_baudRate=HAL_UART_BR_115200;
    
    if(osal_memcmp(SC_BR,"230400",6))
      my_baudRate=HAL_UART_BR_230400;
  }
  else
  {
     //默认波特率115200
    my_baudRate=HAL_UART_BR_9600;
  }
  SC_InitTransport(NpiSerialCallback,my_baudRate); 
}





/*********************************************************************
 * 函数名称：Ord_Scan
 * 功能：主机开始扫描
 * 返回：void
 */
void Ord_Scan(uint8 typecommand)
{
    simpleBLEStartScan();
}
  

/*********************************************************************
 * 函数名称：Ord_Disconnect
 * 功能：主机主动断开连接
 * 返回：void
 */
void  Ord_Disconnect(uint8 typecommand)
{
  GAPCentralRole_TerminateLink( simpleBLEConnHandle );
  
   HalUARTWrite(HAL_UART_PORT_0,"disconnect\r\n",strlen("disconnect\r\n"));
}

/*********************************************************************
 * 函数名称：Ord_Connecting
 * 功能：主机连接从机地址对应的数字
 * 返回：void
 */
void  Ord_Connecting(uint8 * FoundMAC,uint8 typecommand)
{    

  User_Connect(FoundMAC);


}

/*********************************************************************
 * 函数名称：Ord_Clear
 * 功能：主机清除上电连接之前绑定的模块
 * 返回：void
 */
void  Ord_Clear(uint8 typecommand)
{    
   memset(My_BondAddr,0,7);
   memset(My_BondAddrType,0,2);
   osal_snv_write(BLE_BOND_DEVICE_ADR,7,My_BondAddr);
   osal_snv_write(BLE_BOND_DEVICE_ADRTYPE,1,My_BondAddrType);
   HalUARTWrite(HAL_UART_PORT_0,"CLEAR OK\r\n",10);
}

/*********************************************************************
 * 函数名称：Ord_Test
 * 功能：测试串口
 * 返回：void
 */
void Ord_Test(uint8 typecommand)
{
   HalUARTWrite(HAL_UART_PORT_0,"OK\r\n",4);
}


/*****************************************************************************
*      函数名称：CMD_Modify_BPS
*      功能：串口接收波特率改写
*      CMD：AT+BPS=NEWBPS
*      掉电保存
*/
void Ord_Modify_BPS(uint8 *newbps,uint8 typecommand)
{
  uint8 SC_BR[6]={0};
  osal_snv_read(BLE_NVID_BPS_PARA,6,SC_BR);
  offs_RxBuf=0; 
  if(
      osal_memcmp("4800",newbps,4)||
      osal_memcmp("9600",newbps,4) || osal_memcmp("19200",newbps,5) ||
      osal_memcmp("38400",newbps,5) || osal_memcmp("57600",newbps,5) || 
      osal_memcmp("115200",newbps,6) || osal_memcmp("230400",newbps,6) )
  {
      //波特率正确且有改动开始写flash
      if(!osal_memcmp(SC_BR,newbps,osal_strlen((char*)newbps)))
      {
        osal_snv_write(BLE_NVID_BPS_PARA,osal_strlen((char*)newbps),newbps);
      }
         
      //应答命令到串口
      osal_memcpy(SerialRxBuff,"OK+BPS:",7);
      osal_memcpy(&SerialRxBuff[7],newbps,osal_strlen((char*)newbps));
      osal_memcpy(&SerialRxBuff[7+osal_strlen((char*)newbps)],"\r\n",2);
      NPI_WriteTransport(SerialRxBuff,7+2+osal_strlen((char*)newbps));                  
      osal_start_timerEx( order_TaskID, bps_data_event,100);

  }
  else
  {  
      //应答命令到串口
      HalUARTWrite(HAL_UART_PORT_0,"AT+BPS=ERR\r\n",12);
  }
  memset(SerialRxBuff,0,200);
}
  
/****************************************************************
*      函数名称：CMD_Find_ROLE
*      功能：查询角色名称
*      返回：void
*/
void CMD_Find_ROLE( uint8 typecommand )
{

   HalUARTWrite(HAL_UART_PORT_0,"ROLE:MASTER\r\n",13);

}


/****************************************************************
*      函数名称：CMD_CG_VER
*      功能：切换版本通信
*      返回：void
*/
 void CMD_CG_VER( uint8 typecommand )
 {
   if(VER_FLAG == 0)
   {
     VER_FLAG = 1;
     HalUARTWrite(HAL_UART_PORT_0,"Start Ver 1.7\r\n",15);
   }
   else if(VER_FLAG == 1)
   {
    VER_FLAG = 0;
    HalUARTWrite(HAL_UART_PORT_0,"Start Ver 1.8\r\n",15);
   }
 }

