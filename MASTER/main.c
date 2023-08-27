/*
 * File:   main.c
 * Author: ANGELA
 *
 * Created on 27 de agosto de 2023, 0:12
 */

// CONFIG1
#pragma config FOSC = INTRC_NOCLKOUT// Oscillator Selection bits (RCIO oscillator: I/O function on RA6/OSC2/CLKOUT pin, RC on RA7/OSC1/CLKIN)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config MCLRE = OFF      // RE3/MCLR pin function select bit (RE3/MCLR pin function is digital input, MCLR internally tied to VDD)
#pragma config CP = OFF         // Code Protection bit (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Code Protection bit (Data memory code protection is disabled)
#pragma config BOREN = OFF      // Brown Out Reset Selection bits (BOR disabled)
#pragma config IESO = OFF       // Internal External Switchover bit (Internal/External Switchover mode is disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is disabled)
#pragma config LVP = OFF        // Low Voltage Programming Enable bit (RB3 pin has digital I/O, HV on MCLR must be used for programming)

// CONFIG2
#pragma config BOR4V = BOR40V   // Brown-out Reset Selection bit (Brown-out Reset set to 4.0V)
#pragma config WRT = OFF        // Flash Program Memory Self Write Enable bits (Write protection off)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

//*********
// Definición e importación de librerías
//*********
#include <stdint.h>
#include <pic16f887.h>
#include <string.h>
#include "I2C.h"
#include <xc.h>
#include "LCD.h"
#include "USART.h"
//*********
// Definición de variables
//*********
#define _XTAL_FREQ 8000000
uint8_t TemperatureValue;
uint8_t HumidityValue;
void uint8ToString(uint8_t num, char *str);
void reverse(char str[], int length);
int intToStr(int x, char str[], int d);
void floatToStr(float value, char* buffer, int precision);
int d2b (int to_convert);
int b2d(int to_convert);
char bufferQ[4];
char bufferHum[4];
char bufferTemp[4];
float caudal;
//*********
// Definición de funciones para que se puedan colocar después del main de lo 
// contrario hay que colocarlos todas las funciones antes del main
//*********
void setup(void);

void __interrupt() isr (void){
        
     if (RCIF){
        PORTD = 0;
        if (RCREG == '1'){
            printf(bufferQ); // Incrementar el valor del contador al recibir el carácter '+'
        }
        else if (RCREG == '2'){
            printf(bufferTemp); // Decrementar el valor del contador al recibir el carácter '-'
        }
    }
    return;
}

void uint8ToString(uint8_t num, char *str) {
    int i = 0;
    do {
        str[i++] = '0' + (num % 10);
        num /= 10;
    } while (num > 0);
    str[i] = '\0';  // Agregar el carácter nulo para indicar el final de la cadena
    //reverse(str, i); // Invertir la cadena para que esté en el orden correcto
}

//*********
// Main
//*********
void main(void) {
    setup();
    printf(" ");
            
    while(1){

        //comunicación con sensor de temperatura
        
        I2C_Master_Start();
        I2C_Master_Write(0x80);
        I2C_Master_Write(0);
        I2C_Master_Stop();
        __delay_ms(200);
       
        I2C_Master_Start();
        I2C_Master_Write(0x81);
        HumidityValue = I2C_Master_Read(0);
        I2C_Master_Stop();
        __delay_ms(200);
        
        I2C_Master_Start();
        I2C_Master_Write(0x80);
        I2C_Master_Write(1);
        I2C_Master_Stop();
        __delay_ms(200);
       
        I2C_Master_Start();
        I2C_Master_Write(0x81);
        TemperatureValue = I2C_Master_Read(0);
        I2C_Master_Stop();
        __delay_ms(200);
        
        
        Lcd_Clear();
        __delay_ms(1);
        
        uint8ToString(TemperatureValue, bufferTemp);
        uint8ToString(HumidityValue,bufferHum);
        
        __delay_ms(1);
        Lcd_Write_String(" ");
        __delay_ms(1);
        Lcd_Write_String("H:");
        __delay_ms(1);
        Lcd_Write_String(bufferHum);
        
        __delay_ms(1);
        Lcd_Write_String(" ");
        __delay_ms(1);
        Lcd_Write_String("T:");
        __delay_ms(1);
        Lcd_Write_String(bufferTemp);
         __delay_ms(1);
         
        
    }
    return;
}
//*********
// Función de Inicialización
//*********
void setup(void){
     // CONFIGURACION DEL OSCILADOR
    OSCCONbits.IRCF2 = 1;
    OSCCONbits.IRCF1 = 1;
    OSCCONbits.IRCF0 = 1; // 8MHZ
    OSCCONbits.SCS = 1;  // OSCILADOR INTERNO 
    ANSEL = 0;
    ANSELH = 0;
    TRISA =0;
    PORTA = 0;
    TRISD = 0;
    PORTB = 0;
    PORTD = 0;
    
    INTCONbits.PEIE = 1;        // Int. de perifericos
    INTCONbits.GIE = 1;         // Int. globales
    I2C_Master_Init(100000);    // Inicializar Comuncación I2C
    
    Lcd_Init();
    Lcd_Clear();
    Lcd_Set_Cursor(1,1);
        
    Lcd_Write_String("Bienvenido");
    Lcd_Set_Cursor(2,1);
    Lcd_Write_String("Cargando");
        
        
}