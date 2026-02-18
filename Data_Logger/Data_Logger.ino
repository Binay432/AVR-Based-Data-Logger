#define F_CPU 16000000UL 

#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>

// Definitions
#define BAUD 9600
#define UBRR_VALUE ((F_CPU / (16UL * BAUD)) - 1)

// Pin Definitions based on Schematic
#define START_PIN PD2
#define STOP_PIN  PD3

// Logging States
typedef enum {
    STATE_IDLE,
    STATE_LOGGING
} SystemState;

SystemState currentState = STATE_IDLE;

//---------- UART & ADC FUNCTIONS (Keep existing logic) ----------
void UART_init(void) {
    UBRR0H = (uint8_t)(UBRR_VALUE >> 8);
    UBRR0L = (uint8_t)UBRR_VALUE;
    UCSR0B = (1 << TXEN0);
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
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
    ADMUX = (ADMUX & 0xF0) | (channel & 0x0F);
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

uint8_t DS1307_read(uint8_t addr) {
    // Simplified single-byte read for brevity 
    // (Assuming existing logic provided in prompt works)
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
    TWCR = (1 << TWINT) | (1 << TWEN); // NACK
    while (!(TWCR & (1 << TWINT)));
    return TWDR;
}

//---------- GPIO INIT ----------
void GPIO_init(void) {
    // Set PD2 and PD3 as Inputs (0)
    DDRD &= ~((1 << START_PIN) | (1 << STOP_PIN));
    // No internal pull-ups needed because schematic shows external pull-downs
}

//---------- MAIN LOGIC ----------
void setup(void) {
    UART_init();
    ADC_init();
    I2C_init();
    GPIO_init();
    UART_print("SYSTEM READY. PRESS START TO LOG.\r\n");
}

int main(void) {
    setup();
    char buffer[80];

    while (1) {
        // Check for START button (Active High)
        if (PIND & (1 << START_PIN)) {
            _delay_ms(50); // Debounce
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

            uint8_t sec  = DS1307_read(0x00);
            uint8_t min  = DS1307_read(0x01);
            uint8_t hour = DS1307_read(0x02);

            sprintf(buffer, "[%02X:%02X:%02X] Temp: %d C | Light: %u\r\n", 
                    hour, min, sec, temp_c, ldr);
            UART_print(buffer);
            
            _delay_ms(1000); // 1 second sample rate
        } else {
            // Idle loop - tiny delay to save processing power in simulation
            _delay_ms(100); 
        }
    }
}
