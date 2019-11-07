//******************************************************************************        
//name:         Central_Order.c        
//introduce:    ����ATָ������    
//author:       ZXL      
//changetime:   2016��7��30��15:06:22    
//******************************************************************************  
  
/*********************ͷ�ļ�************************/   
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
/*********************�궨��************************/   



/***************�ڲ���������************************/   
static uint8 order_TaskID;  

/***************�ڲ���������************************/ 
static void order_ProcessOSALMsg( osal_event_hdr_t *pMsg ); 


/***************�ⲿ��������************************/ 
extern uint8 simpleBLEScanning;
extern uint8 simpleBLEScanRes;
extern uint16 simpleBLEConnHandle;
extern uint8 My_BondAddr[7];
extern uint8 My_BondAddrType[2];
uint8  passcodebuff[7];
extern uint8 VER_FLAG;
//******************************************************************************            
//name:             order_Init            
//introduce:        ����ATָ�������ʼ��           
//parameter:        task_id:�����ID           
//return:           none          
//author:           ZXL       
//changetime:       2016��7��30��15:12:20       
//******************************************************************************  
void order_Init( uint8 task_id )  
{  
  order_TaskID = task_id;     
} 

//******************************************************************************            
//name:             order_ProcessEvent            
//introduce:        �����������¼�          
//parameter:        task_id:�����ID,         
//parameter��       events:�¼�  
//return:           ����ִ�к���¼�״̬          
//author:           ZXL                 
//******************************************************************************  
uint16 order_ProcessEvent( uint8 task_id, uint16 events )  
{  
  VOID task_id;   
  
  //ϵͳ�¼�������Ҫ����ϵͳ�¼�
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
    //���²����ʲ���������
    update_BPS();
    return (events ^ bps_data_event);
  }
  
  

    
  return 0;  
}


//******************************************************************************            
//name:             order_ProcessOSALMsg            
//introduce:        �����������Ϣ����           
//parameter:        pMsg:��Ϣ           
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
 * �������ƣ�Uart_Command
 * ���ܣ������������ݣ���������
 * ���أ�����״̬
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
    /*******************�ж���������*******************************************/
    //��ʼɨ��
    if(osal_memcmp(OffData_Buff,"SCAN=",5))
    {
      OffData_Buff+=5;
      Uart_CommandService(STATUS_ORD_SCAN,OffData_Buff,typecommand);
      return STATUS_ORD_SCAN;
    } 
    
    //�����Ͽ�����
    if(osal_memcmp(OffData_Buff,"DSCN=",5))
    {
      OffData_Buff+=6;
      Uart_CommandService(STATUS_ORD_DSCN,OffData_Buff,typecommand);
      return STATUS_ORD_DSCN;
    } 
    
    //����������ʾ�ĵ�ַ�ı�ŵĴӻ�
    if(osal_memcmp(OffData_Buff,"CONN=",5))
    {
      OffData_Buff+=5;
      Uart_CommandService(STATUS_ORD_CONN,OffData_Buff,typecommand);
      return STATUS_ORD_CONN;
    }
    
     //�����ϵ����ӽ����
    if(osal_memcmp(OffData_Buff,"CLEAR=",6))
    {
      OffData_Buff+=6;
      Uart_CommandService(STATUS_ORD_CLEAR,OffData_Buff,typecommand);
      return STATUS_ORD_DSCN;
    } 
     //����ָ��
    if(osal_memcmp(OffData_Buff,"TEST",4))
    {
      OffData_Buff+=4;
      Uart_CommandService(STATUS_ORD_TEST,OffData_Buff,typecommand);
      return STATUS_ORD_DSCN;
    } 
    //�޸Ĵ��ڲ�����
    if(osal_memcmp(OffData_Buff, "BPS",3))
    {
      OffData_Buff+=4;
      Uart_CommandService(STATUS_CMD_BPS,OffData_Buff,typecommand);
      return STATUS_CMD_BPS;
    }
    
    //��ѯ��ɫ����
    if(osal_memcmp(OffData_Buff,"ROLE",4))
    {
      OffData_Buff+=5;
      Uart_CommandService(STATUS_FD_ROLE,OffData_Buff,typecommand);
      return STATUS_FD_ROLE;
    }    
    
     //�л�ͨ�Ű汾
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
 * �������ƣ�Uart_CommandService
 * ���ܣ���������������ô�����
 * ���أ�
 */
void Uart_CommandService(uint8 Command , uint8 *DataBuff , uint8 typecommand)
{  
  switch(Command)
  {
    case STATUS_ORD_SCAN:
      Ord_Scan(typecommand);                   //����������ʼɨ��
      break;
      
    case STATUS_ORD_DSCN:
      Ord_Disconnect(typecommand);             //�������������Ͽ�����
      break;
      
    case STATUS_ORD_CONN:
      Ord_Connecting(DataBuff,typecommand);    //�������������Ͽ�����
      break;
      
    case STATUS_ORD_CLEAR:
      Ord_Clear(typecommand);                 //��������ϵ���������
      break;
      
    case STATUS_ORD_TEST:
      Ord_Test(typecommand);                 //���Դ���ָ��
      break; 
      
    case STATUS_CMD_BPS:
      Ord_Modify_BPS( DataBuff,typecommand );       //�޸Ĳ�����
      break; 
      
    case STATUS_FD_ROLE:
      CMD_Find_ROLE( typecommand );             //��ѯ��ɫ
      break;   
      
    case STATUS_CG_VER:
      CMD_CG_VER( typecommand );             //�л�ͨ�Ű汾
      break;         
      
    default:
      break;
  }
}

/*******************************************************************
*      �������ƣ�update_BPS
*      ���ܣ����ڲ����ʸ���
*/
void update_BPS()
{
  static uint8 my_baudRate;
  uint8 SC_BR[6]={0};
  osal_snv_read(BLE_NVID_BPS_PARA,6,SC_BR);
    //�жϲ����ʣ��б仯�͸���
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
     //Ĭ�ϲ�����115200
    my_baudRate=HAL_UART_BR_9600;
  }
  SC_InitTransport(NpiSerialCallback,my_baudRate); 
}





/*********************************************************************
 * �������ƣ�Ord_Scan
 * ���ܣ�������ʼɨ��
 * ���أ�void
 */
void Ord_Scan(uint8 typecommand)
{
    simpleBLEStartScan();
}
  

/*********************************************************************
 * �������ƣ�Ord_Disconnect
 * ���ܣ����������Ͽ�����
 * ���أ�void
 */
void  Ord_Disconnect(uint8 typecommand)
{
  GAPCentralRole_TerminateLink( simpleBLEConnHandle );
  
   HalUARTWrite(HAL_UART_PORT_0,"disconnect\r\n",strlen("disconnect\r\n"));
}

/*********************************************************************
 * �������ƣ�Ord_Connecting
 * ���ܣ��������Ӵӻ���ַ��Ӧ������
 * ���أ�void
 */
void  Ord_Connecting(uint8 * FoundMAC,uint8 typecommand)
{    

  User_Connect(FoundMAC);


}

/*********************************************************************
 * �������ƣ�Ord_Clear
 * ���ܣ���������ϵ�����֮ǰ�󶨵�ģ��
 * ���أ�void
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
 * �������ƣ�Ord_Test
 * ���ܣ����Դ���
 * ���أ�void
 */
void Ord_Test(uint8 typecommand)
{
   HalUARTWrite(HAL_UART_PORT_0,"OK\r\n",4);
}


/*****************************************************************************
*      �������ƣ�CMD_Modify_BPS
*      ���ܣ����ڽ��ղ����ʸ�д
*      CMD��AT+BPS=NEWBPS
*      ���籣��
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
      //��������ȷ���иĶ���ʼдflash
      if(!osal_memcmp(SC_BR,newbps,osal_strlen((char*)newbps)))
      {
        osal_snv_write(BLE_NVID_BPS_PARA,osal_strlen((char*)newbps),newbps);
      }
         
      //Ӧ���������
      osal_memcpy(SerialRxBuff,"OK+BPS:",7);
      osal_memcpy(&SerialRxBuff[7],newbps,osal_strlen((char*)newbps));
      osal_memcpy(&SerialRxBuff[7+osal_strlen((char*)newbps)],"\r\n",2);
      NPI_WriteTransport(SerialRxBuff,7+2+osal_strlen((char*)newbps));                  
      osal_start_timerEx( order_TaskID, bps_data_event,100);

  }
  else
  {  
      //Ӧ���������
      HalUARTWrite(HAL_UART_PORT_0,"AT+BPS=ERR\r\n",12);
  }
  memset(SerialRxBuff,0,200);
}
  
/****************************************************************
*      �������ƣ�CMD_Find_ROLE
*      ���ܣ���ѯ��ɫ����
*      ���أ�void
*/
void CMD_Find_ROLE( uint8 typecommand )
{

   HalUARTWrite(HAL_UART_PORT_0,"ROLE:MASTER\r\n",13);

}


/****************************************************************
*      �������ƣ�CMD_CG_VER
*      ���ܣ��л��汾ͨ��
*      ���أ�void
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

