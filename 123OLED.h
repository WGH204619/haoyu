/*
 * 123OLED.h
 *
 *  Created on: Jun 4, 2024
 *      Author: 86183
 */

#ifndef INC_123OLED_H_
#define INC_123OLED_H_


void OLED_Init(void);
void OLED_Clear(void);
void OLED_ShowChar(uint8_t Line, uint8_t Column, char Char);
void OLED_ShowString(uint8_t Line, uint8_t Column, char *String);
void OLED_ShowNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);
void OLED_ShowSignedNum(uint8_t Line, uint8_t Column, int32_t Number, uint8_t Length);
void OLED_ShowHexNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);
void OLED_ShowBinNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length);
void Return_arry(char words[8],char subwords[3],int i,int n);



#endif /* INC_123OLED_H_ */
