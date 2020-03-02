// RFM96.h

#ifndef LCD_h
#define LCD_h


/* Public define -------------------------------------------------------------*/

#define LCD_CHAR_DEGREE     0xDF

/* Public function prototypes ------------------------------------------------*/

void LCD_Init( void );
void LCD_ClearDisplay(void);
void LCD_ReturnHome(void);
void LCD_SetDataAddress(uint8_t addr);
void LCD_SetCGramAddress(uint8_t addr);
void LCD_WriteData(uint8_t data);

#endif
