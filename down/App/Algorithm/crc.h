#ifndef __CRC_H__
#define __CRC_H__
#include "stm32f4xx.h" 
#include <stdbool.h>

/* CRC8 校验函数 */
unsigned char Get_CRC8_Check_Sum(unsigned char *pchMessage,unsigned int dwLength,unsigned char ucCRC8);
unsigned int Verify_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength);
void Append_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength);

/* CRC16 检验函数 */
uint16_t Get_CRC16_Check_Sum(uint8_t *pchMessage,uint32_t dwLength,uint16_t wCRC);
bool Verify_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength);
void Append_CRC16_Check_Sum(uint8_t * pchMessage,uint32_t dwLength);



uint8_t CRC08_Calculate(uint8_t *Data, uint32_t Lenth /* Without check code lenth 1 */);
uint32_t CRC08_Verify(uint8_t *Data, uint32_t Lenth /* With check code lenth 1 */);
uint32_t CRC08_Append(uint8_t *Data, uint32_t Lenth /* With check code lenth 1 */);


uint16_t CRC16_Calculate(uint8_t *Data, uint32_t Lenth /* Without check code lenth 2 */);
uint32_t CRC16_Verify(uint8_t *Data, uint32_t Lenth /* With check code lenth 2 */);
uint32_t CRC16_Append(uint8_t *Data, uint32_t Lenth /* With check code lenth 2 */);



#endif
