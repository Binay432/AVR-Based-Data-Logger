#define F_CPU 16000000UL //16Mhz Clk frequency 

#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>

// UART DEFINITIONS

#define BAUD 9600
#define UBRR_VALUE ((F_CPU / (16UL * BAUD)) - 1)

// Supporting Functions

//---------- UART INIT ---------- 

void UART_init(void)
{
    UBRR0H = (uint8_t)(UBRR_VALUE >> 8);
    UBRR0L = (uint8_t)UBRR_VALUE; //Setting the baud rate register

    UCSR0B = (1 << TXEN0);                 // Enabling the transmit hardware 
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);// Configuring the  8 bit standard mode 
}

//---------- UART TX ---------- 
void UART_tx(char data)
{
    while (!(UCSR0A & (1 << UDRE0)));  // waiting till the transmit buffer is empty 
    UDR0 = data;
}

//---------- UART PRINT ----------

void UART_print(char *str)
{
    while (*str){
      UART_tx(*str++);
    }
        
}

//---------- ADC INIT ----------
void ADC_init(void)
{
    ADMUX = (1 << REFS0);   // Setting External AREF (3.3V)
    ADCSRA = (1 << ADEN) |(1 << ADPS2) | (1 << ADPS1);  //Enabling ADC and Setting the clock speed
}

//---------- ADC READ ----------
uint16_t ADC_read(uint8_t channel)
{
    // Setting the pin to read data , 0 for LDR and 1 for Temp
    ADMUX = (ADMUX & 0xF0) | channel;  

    // Start the converstaion 
    ADCSRA |= (1 << ADSC);

    //Wait for the hardware to finsh the conversion 
    while (ADCSRA & (1 << ADSC));

    // Return the 10 bits result 
    
    return ADC;
}

// --------- I2C INIT ---------- 
void I2C_init(void)
{
    TWSR = 0x00;
    TWBR = 72;      // 100kHz @ 16MHz
    TWCR = (1 << TWEN);
}

//---------- DS1307 READ ----------
uint8_t DS1307_read(uint8_t addr)
{
    TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)));

    TWDR = 0xD0;
    TWCR = (1 << TWINT) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)));

    TWDR = addr;
    TWCR = (1 << TWINT) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)));

    TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)));

    TWDR = 0xD1;
    TWCR = (1 << TWINT) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)));

    TWCR = (1 << TWINT) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)));

    return TWDR;
}

//---------- SETUP (Arduino Entry) ----------
void setup(void)
{
    UART_init();
    ADC_init();
    I2C_init();

    UART_print("AVR DATA LOGGER STARTED\r\n");
}


void loop()
{
    char buffer[64];

    uint16_t ldr = ADC_read(0);      // ADC0
    uint16_t temp_adc = ADC_read(1); // ADC1

    float voltage = temp_adc * 3.3 / 1024.0;
    float temperature = (voltage - 0.5) * 100.0;

    //Floating part is not allow in avr
    int temp_whole = (int)temperature;
    int temp_frac = (int)((temperature - temp_whole) * 100);
    if (temp_frac < 0) temp_frac *= -1; // Handle negative numbers

    uint8_t sec  = DS1307_read(0x00);
    uint8_t min  = DS1307_read(0x01);
    uint8_t hour = DS1307_read(0x02);
  
    sprintf(buffer,
        "TIME %02X:%02X:%02X | TEMP: %d.%2d C | LDR: %u\r\n",
        hour, min, sec, temperature, ldr);

    UART_print(buffer);
    _delay_ms(1000);
}
