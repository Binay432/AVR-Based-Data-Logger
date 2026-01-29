# ATmega328P Bare-Metal Data Logger

A high-performance, register-level data logging system built for the **ATmega328P** (Arduino Uno architecture). This project bypasses standard Arduino libraries (like `Wire.h`, `SPI.h`, or `Serial`) to interact directly with the microcontroller's hardware registers.

## 🛠 Project Overview
The system monitors environmental data using an LDR (Light) and a TMP36 (Temperature) sensor. It timestamps each entry using a DS1307 Real-Time Clock (RTC) and outputs the data to a Serial Terminal.

### Key Features
* **Direct Register Manipulation:** Uses `ADMUX`, `ADCSRA`, `TWCR`, and `UCSR0` for hardware control.
* **Optimized ADC:** Configured to use an **External 3.3V AREF** for precise sensor scaling.
* **Custom UART Driver:** Manual baud rate calculation for 9600 bps communication.
* **I2C/TWI Interface:** Bare-metal implementation of the Two-Wire Interface to communicate with the DS1307 RTC.
* **Memory Efficiency:** Avoids floating-point libraries by using integer scaling to prevent `sprintf` "garbage" outputs.

---

## 📐 Hardware Configuration
The circuit diagram is from proteous learnig:
<img width="1225" height="617" alt="image" src="https://github.com/user-attachments/assets/72b139af-b2e4-49ab-9485-5aaf26f574d5" />

---

