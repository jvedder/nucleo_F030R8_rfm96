/*
 * lcd.c
 *
 *
 * Pin Definitions:
 *   LCD_RS = Register Select: 0 = Command, 1= Data
 *   LCD_RW = Read/Write Select: 1 = Read, 0 = Write
 *   LCD_E  = Operation Enable signal. Falling edge triggered.
 *   LCD_DB4-7 = Four high order bi-directional three-state data bus lines.
 *
 * DRAM Addresses (4 rows x 20 chars)
 *   Row1: 0x00 to 0x13
 *   Row2: 0x40 to 0x53
 *   Row3: 0x14 to 0x27
 *   Row4: 0x54 to 0x67
 */


/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "lcd.h"
#include <string.h>

/* Private define ------------------------------------------------------------*/
void LCD_Write(uint8_t data);
void LCD_SetDataBits(uint8_t data);
void LCD_ClockData(void);


/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* User code -----------------------------------------------------------------*/
void inline LCD_SetCommandMode(void)
{
    HAL_GPIO_WritePin(LCD_RS_GPIO_Port, LCD_RS_Pin, GPIO_PIN_RESET); // RS=LOW: send instruction
}

void inline LCD_SetDataMode(void)
{
    HAL_GPIO_WritePin(LCD_RS_GPIO_Port, LCD_RS_Pin, GPIO_PIN_SET); // RS=HIGH: send data
}

void inline LCD_SetWriteMode(void)
{
    HAL_GPIO_WritePin(LCD_RW_GPIO_Port, LCD_RW_Pin, GPIO_PIN_RESET); // RW=LOW : Write
}

void inline LCD_SetReadMode(void)
{
    HAL_GPIO_WritePin(LCD_RW_GPIO_Port, LCD_RW_Pin, GPIO_PIN_SET); // RW=HIGH : Read
}

void LCD_Init( void )
{
    // Set everything low
    LCD_SetDataBits(0x00);
    HAL_GPIO_WritePin(LCD_E_GPIO_Port, LCD_E_Pin, GPIO_PIN_RESET);    // E=0
    LCD_SetWriteMode();         // RW=LOW : Write
    LCD_SetCommandMode();       // RS=LOW: send instruction
    Delay_ms(100);              // Wait >40 msec after power is applied
    LCD_SetDataBits(0x30);      // put 0x30 on the output port
    Delay_ms(30);               // must wait 5ms, busy flag not available
    LCD_ClockData();            // command 0x30 = Wake up
    Delay_ms(10);               // must wait 160us, busy flag not available
    LCD_ClockData();            // command 0x30 = Wake up #2
    Delay_ms(10);               // must wait 160us, busy flag not available
    LCD_ClockData();            // command 0x30 = Wake up #3
    Delay_ms(10);               // can check busy flag now instead of delay
    LCD_SetDataBits(0x20);      // Function set: 4-bit
    LCD_ClockData();
    LCD_SetCommandMode();
    LCD_Write(0x28);            // Function set: 4-bit/2-line DL=0; N=1; F=0
    LCD_Write(0x10);            // Set cursor or display shift: S/C=0; R/L=0
    LCD_Write(0x0F);            // Display ON/OFF Control: D=1 (on); C=1 (Cursor on); B=1 (Blink on)
    LCD_Write(0x06);            // Entry Mode set: I/D=1 SH=0
}

void LCD_ClearDisplay(void)
{
    LCD_SetCommandMode();
    LCD_Write(0x01);
}


void LCD_ReturnHome(void)
{
    LCD_SetCommandMode();
    LCD_Write(0x02);
}

void LCD_SetDataAddress(uint8_t addr)
{
    LCD_SetCommandMode();
    LCD_Write(addr | 0x80);
}

void LCD_SetCGramAddress(uint8_t addr)
{
    LCD_SetCommandMode();
    LCD_Write((addr & 0x3F) | 0x40);
}

void LCD_WriteData(uint8_t data)
{
    LCD_SetDataMode();
    LCD_Write(data);
}


void LCD_Write(uint8_t data)
{
    LCD_SetWriteMode();
    LCD_SetDataBits(data);        // put high 4 data bits on output Port
    LCD_ClockData();
    LCD_SetDataBits(data << 4);   // put low 4 data bits on output Port
    LCD_ClockData();
}


void LCD_SetDataBits(uint8_t data)
{
    GPIO_PinState bit;

    bit = (data & 0x80) ? GPIO_PIN_SET : GPIO_PIN_RESET;
    HAL_GPIO_WritePin(LCD_DB7_GPIO_Port, LCD_DB7_Pin, bit);

    bit = (data & 0x40) ? GPIO_PIN_SET : GPIO_PIN_RESET;
    HAL_GPIO_WritePin(LCD_DB6_GPIO_Port, LCD_DB6_Pin, bit);

    bit = (data & 0x20) ? GPIO_PIN_SET : GPIO_PIN_RESET;
    HAL_GPIO_WritePin(LCD_DB5_GPIO_Port, LCD_DB5_Pin, bit);

    bit = (data & 0x10) ? GPIO_PIN_SET : GPIO_PIN_RESET;
    HAL_GPIO_WritePin(LCD_DB4_GPIO_Port, LCD_DB4_Pin, bit);
}


void LCD_ClockData(void)
{
    Delay_ms(1);
    HAL_GPIO_WritePin(LCD_E_GPIO_Port, LCD_E_Pin, GPIO_PIN_SET);        // E=1
    Delay_ms(1);  //enable pulse width >= 300ns
    HAL_GPIO_WritePin(LCD_E_GPIO_Port, LCD_E_Pin, GPIO_PIN_RESET);      // E=0
    Delay_ms(1);  //enable pulse width >= 300ns
}

