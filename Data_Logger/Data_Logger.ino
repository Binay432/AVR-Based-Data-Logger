#define F_CPU 16000000UL 

#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>

// Definitions
#define BAUD 9600
#define UBRR_VALUE ((F_CPU / (16UL * BAUD)) - 1) 

// Pin Definitions based on Schematic
#define START_PIN PD4
#define STOP_PIN  PD5

// Logging States
typedef enum {
    STATE_IDLE,
    STATE_LOGGING
} SystemState;

SystemState currentState = STATE_IDLE;

//---------- UART & ADC FUNCTIONS ----------
void UART_init(void) {
    UBRR0H = (uint8_t)(UBRR_VALUE >> 8); // Shift the bit to get high part , cz Atmega is a 8 bit register 
    UBRR0L = (uint8_t)UBRR_VALUE;   
    UCSR0B = (1 << TXEN0);     // Turns on the transmitter hardware 
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);     // data format to 8N1 (8 BIT , NO PARITY, 1 STOP BIT )
}

void UART_tx(char data) {
    while (!(UCSR0A & (1 << UDRE0)));
    UDR0 = data;
}

void UART_print(char *str) {
    while (*str) UART_tx(*str++);
}

void ADC_init(void) {
    ADMUX = 0x00; // External AREF
    ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
}

uint16_t ADC_read(uint8_t channel) {
    ADMUX = (ADMUX & 0xF0) | (channel & 0x0F);  // SELECT WHICH PIN LISTEN , 0 FOR LIGHT AND CHANNEL 1 FOR TEMP 
    ADCSRA |= (1 << ADSC);
    while (ADCSRA & (1 << ADSC));
    return ADC;
}

//---------- I2C / RTC FUNCTIONS ----------
void I2C_init(void) {
    TWSR = 0x00;
    TWBR = 72;
    TWCR = (1 << TWEN);
}


// Read Function with proper I2C Handshaking
uint8_t DS1307_read(uint8_t addr) {
    // 1. Send START condition
    TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)));

    // 2. Send Slave Address + Write (0xD0)
    TWDR = 0xD0; 
    TWCR = (1 << TWINT) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)));

    // 3. Send the Register Address we want to read
    TWDR = addr;
    TWCR = (1 << TWINT) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)));

    // 4. Send REPEATED START
    TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)));

    // 5. Send Slave Address + Read (0xD1)
    TWDR = 0xD1;
    TWCR = (1 << TWINT) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)));

    // 6. Read Data and send NACK (to end communication)
    TWCR = (1 << TWINT) | (1 << TWEN); 
    while (!(TWCR & (1 << TWINT)));
    uint8_t data = TWDR;

    //7. SEND STOP, if not done bus never properly released and next start becomes invalid
    TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);
    return TWDR;
}

// to avoid raw BCD printing 
uint8_t bcdToDec(uint8_t val)
{
    return ((val >> 4) * 10) + (val & 0x0F);
}

//---------- GPIO INIT ----------
void GPIO_init(void) {
    // Set PD2 and PD3 as Inputs (0)
    DDRD &= ~((1 << START_PIN) | (1 << STOP_PIN));
    // No internal pull-ups needed because schematic has external pull-downs
}


//---------- MAIN LOGIC ----------
void setup(void) {
    UART_init();
    ADC_init();
    I2C_init();
    GPIO_init();
    UART_print("CUSTOM DATA LOGGER TEST BY BINAY.\r\n");
    UART_print("SYSTEM READY. PRESS START TO LOG.\r\n");
}

int main(void) {
    setup();
    char buffer[80];
    while (1) {
        // Check for START button (Active High)
        if (PIND & (1 << START_PIN)) {
            _delay_ms(50); // Debounce
            UART_print("--- Start Button Pressed ---\r\n");
            if (PIND & (1 << START_PIN)) {
                if (currentState == STATE_IDLE) {
                    currentState = STATE_LOGGING;
                    UART_print("--- LOGGING STARTED ---\r\n");
                }
            }
        }

        // Check for STOP button (Active High)
        if (PIND & (1 << STOP_PIN)) {
            _delay_ms(50); // Debounce
            if (PIND & (1 << STOP_PIN)) {
                if (currentState == STATE_LOGGING) {
                    currentState = STATE_IDLE;
                    UART_print("--- LOGGING STOPPED ---\r\n");
                }
            }
        }

        // Action based on State
        if (currentState == STATE_LOGGING) {
            uint16_t ldr = ADC_read(0);
            uint16_t temp_adc = ADC_read(1);

            long mv = (temp_adc * 3300L) / 1024;
            int temp_c = (mv - 500) / 10;

            // DS130 Chip , data logger shield to keep track of the time 
            uint8_t sec  = bcdToDec(DS1307_read(0x00));
            uint8_t mint = bcdToDec(DS1307_read(0x01));
            uint8_t hour = bcdToDec(DS1307_read(0x02));
            uint8_t date = bcdToDec(DS1307_read(0x04));
            uint8_t month= bcdToDec(DS1307_read(0x05));
            uint8_t year = bcdToDec(DS1307_read(0x06));

            
            sprintf(buffer,
            "Date & Time: 20%02d-%02d-%02d %02d:%02d:%02d\r\nTemp: %d C | Light: %u\r\n",
            year, month, date, hour, mint, sec, temp_c, ldr);

            UART_print(buffer);
            
            _delay_ms(1000); // 1 second sample rate
        } else {
            // Idle loop - tiny delay to save processing power in simulation
            _delay_ms(100); 
        }
    }
}
