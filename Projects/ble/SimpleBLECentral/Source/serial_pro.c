#include "serial_pro.h"
#include "npi.h"
#include "gatt.h"
#include "simpleBLECentral.h"


/*********************************************************************
 * CONSTANS
 */
#define SIMPLEPROFILE_CHAR6_LEN  20
/*********************************************************************
 * EXTERNAL VARIABLES
 */
extern uint8 simpleBLEState;
// Discovered characteristic handle
extern uint16 simpleBLECharHdl;
extern uint16 simpleBLECharHd6;
extern bool simpleBLEChar6DoWrite;
extern uint8 simpleBLETaskId;


/*********************************************************************
 * @fn      NpiSerialCallback
 *
 * @brief   串口回调函数
 *
 * @return  none
 */
void NpiSerialCallback( uint8 port, uint8 events )
{
  (void)port;
  static uint8 numBytes = 0;
  static uint8 buf[128];
  static uint8 send_byte_cnt;
  static uint16 i;
  for(i=12000;i>0;i--)
  {
    asm("nop");
  }
  
  if (events & HAL_UART_RX_TIMEOUT)  
  {
    numBytes = NPI_RxBufLen();       
    if(numBytes)
    {     
      if ( ( simpleBLEState == BLE_STATE_CONNECTED  ) && ( simpleBLECharHd6 != 0 ) ) 
      {
        if(simpleBLEChar6DoWrite)               
        {
          attWriteReq_t AttReq;       
          if ( numBytes >= SIMPLEPROFILE_CHAR6_LEN ) send_byte_cnt = SIMPLEPROFILE_CHAR6_LEN;
          else send_byte_cnt = numBytes;
          NPI_ReadTransport(buf,send_byte_cnt);    
             
          AttReq.handle = simpleBLECharHd6;
          AttReq.len = send_byte_cnt;
          AttReq.sig = 0;
          AttReq.cmd = 0;
          osal_memcpy(AttReq.value,buf,send_byte_cnt);
          GATT_WriteCharValue( 0, &AttReq, simpleBLETaskId );
          simpleBLEChar6DoWrite = FALSE;
        }
      }
      else
      {
        NPI_ReadTransport(buf,numBytes);     
      }
    }
  }
}