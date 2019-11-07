/*********************************************************************
 * EXTERNAL VARIABLES DEFINE
 */
#include "NPI.h"
#ifdef __cplusplus
extern "C"
{
#endif
extern uint8 SerialRxBuff[200]; //串口接收数据缓存
extern uint8 *Rx_q;//指向串口缓冲区
extern uint8 uart_remain_len;//串口数据剩余长度
extern uint8 offs_RxBuf;
extern uint8 uarting;
/*********************************************************************
 * FUNCTIONS
 */
void NpiSerialCallback(uint8 port, uint8 event);
extern void SC_InitTransport( npiCBack_t npiCBack,uint8 current_BD);