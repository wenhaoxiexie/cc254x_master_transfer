#ifndef ORDER_H
#define ORDER_H
   
#ifdef __cplusplus
extern "C"
{
#endif
  

/*********************头文件************************/   
#include "bcomdef.h"   
  

  
/*********************宏定义************************/   
//Cmd type
#define SERIAL_PARA_CMD                             0
#define BLE_PARA_CMD                                1

#define bps_data_event                                0x0010 
/*******************函数声明************************/ 
extern void order_Init( uint8 task_id );  
extern uint16 order_ProcessEvent( uint8 task_id, uint16 events );
uint8 Uart_Command(uint8 *p,uint8 typecommand);  
void Uart_CommandService(uint8 Command , uint8 *DataBuff , uint8 typecommand);
void Ord_Scan(uint8 typecommand);
void SendData(uint8 send_len);
void  Ord_Disconnect(uint8 typecommand);
void  Ord_Connecting(uint8 * Connect_Number,uint8 typecommand);
void  Ord_Clear(uint8 typecommand);
void Ord_Test(uint8 typecommand);
extern void update_BPS(void);
void Ord_Modify_BPS(uint8 *newbps,uint8 typecommand);
void CMD_Find_ROLE( uint8 typecommand );
 void CMD_CG_VER( uint8 typecommand );
enum ord_status
{
  STATUS_CMD_ERR,
  STATUS_ORD_SCAN,
  STATUS_ORD_DSCN,
  STATUS_ORD_CONN,
  STATUS_ORD_CLEAR,
  STATUS_ORD_TEST,
  STATUS_CMD_BPS,
  STATUS_FD_ROLE,
  STATUS_CG_VER
};




  
#ifdef __cplusplus
}
#endif

#endif