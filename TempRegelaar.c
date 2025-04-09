#define F_CPU 8e6
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
 
#define BIT(x) (1 << (x))
#define DDR_SPI      DDRB                    // SPI Data direction register
#define PORT_SPI     PORTB                   // SPI Output register
#define SPI_SCK      1                       // PB1: SPI Pin System Clock
#define SPI_MOSI     2                       // PB2: SPI Pin MOSI
#define SPI_MISO     3                       // PB3: SPI Pin MISO
#define SPI_SS       0                       // PB0: SPI Pin Slave Select
 
// --- Wachtfunctie ---
void wait(int ms) {
	for (int tms = 0; tms < ms; tms++) {
		_delay_ms(1);
	}
}
 
// --- SPI initialisatie ---
void spi_masterInit(void) {
	DDR_SPI = 0xFF;                          // Alle pinnen output: MOSI, SCK, SS
	DDR_SPI &= ~BIT(SPI_MISO);               // MISO als input
	PORT_SPI |= BIT(SPI_SS);                 // SS_ADC = 1: deselect slave
	SPCR = (1 << SPE) | (1 << MSTR) | (1 << SPR1);  // Enable SPI, Master Mode
}
 
// Schrijf een byte naar slave
void spi_write(unsigned char data) {
	SPDR = data;                             // Schrijf byte naar Data register
	while (!(SPSR & BIT(SPIF)));             // Wachten tot verzending compleet
}
 
// Selecteer slave apparaat
void spi_slaveSelect(unsigned char chipNumber) {
	PORTB &= ~BIT(chipNumber);
}
 
// Deselecteer slave apparaat
void spi_slaveDeSelect(unsigned char chipNumber) {
	PORTB |= BIT(chipNumber);
}
 
// Stuur een adres + databyte naar slave
void spi_writeWord(unsigned char address, unsigned char data) {
	spi_slaveSelect(0);
	spi_write(address);
	spi_write(data);
	spi_slaveDeSelect(0);
}
 
// Initialiseer de displaydriver (MAX7219)
void displayDriverInit() {
	spi_writeWord(0x09, 0xFF);
	spi_writeWord(0x0A, 0x0F);
	spi_writeWord(0x0B, 0x03);
	spi_writeWord(0x0C, 0x01);
}
 
// Schrijf data naar het LED-display
void writeLedDisplay(int value) {
	char digits[4];
	int tempValue = value;
	int isNegative = 0;
 
	if (value < 0) {
		isNegative = 1;
		tempValue = -value;
	}
 
	for (int i = 0; i < 4; i++) {
		digits[i] = tempValue % 10;
		tempValue /= 10;
	}
 
	if (isNegative) {
		digits[0] = 10; // Voor minteken
	}
 
	for (int i = 0; i < 4; i++) {
		spi_writeWord(i + 1, digits[i]);
	}
}
 
// --- ADC Initialisatie ---
void adcInit(void) {
	ADMUX = 0b11100010;  // Interne 2.56V referentie, kanaal 2
	ADCSRA = 0b10000110; // ADC aan, prescaler 64
}
 
// Lees ADC-waarde
uint8_t adcRead(void) {
	ADCSRA |= BIT(6);          // Start conversie
	while (ADCSRA & BIT(6));   // Wachten op conversie
	return ADCH;               // Retourneer waarde
}
 
// --- LCD Functies ---
void lcd_strobe_lcd_e(void) {
	PORTA |= (1 << 6);
	_delay_ms(1);
	PORTA &= ~(1 << 6);
	_delay_ms(1);
}
 
void lcd_command(unsigned char cmd) {
	PORTC = (cmd & 0xF0);
	PORTA &= ~(1 << 4);
	lcd_strobe_lcd_e();
	PORTC = (cmd & 0x0F) << 4;
	PORTA &= ~(1 << 4);
	lcd_strobe_lcd_e();
}
 
void lcd_data(unsigned char data) {
	PORTC = (data & 0xF0);
	PORTA |= (1 << 4);
	lcd_strobe_lcd_e();
	PORTC = (data & 0x0F) << 4;
	PORTA |= (1 << 4);
	lcd_strobe_lcd_e();
}
 
void lcd_init(void) {
	DDRC |= 0xF0;
	DDRA |= 0x50;
	_delay_ms(15);
	PORTC = 0x20; // Function set: 0010
	lcd_strobe_lcd_e();
	lcd_command(0x28); // 4-bit modus, 2 lijnen
	lcd_command(0x0C); // Display aan, cursor uit
	lcd_command(0x06); // Cursor increment
	lcd_command(0x01); // Wis display
}
 
void lcd_clear(void) {
	lcd_command(0x01);
}
 
void lcd_set_line1(void) {
	lcd_command(0x80);
}
 
void lcd_set_line2(void) {
	lcd_command(0xC0);
}
 
void display_text(char *str) {
	for (int i = 0; i < strlen(str); i++) {
		lcd_data(str[i]);
	}
}
 
void lcd_write_integer(uint16_t num) {
	char buffer[6];
	itoa(num, buffer, 10);
	display_text(buffer);
}
 
// --- LEDs Updaten ---
void updateLEDs(uint8_t temperature) {
	PORTA = temperature;
}
 
// --- LCD Updaten ---
void updateLCD(uint8_t temperature) {
	lcd_set_line2();
	lcd_write_integer(temperature);
	display_text(" C");
}
 
// --- Hoofdprogramma ---
int main(void) {
	DDRF &= ~BIT(2); // PF2 als input
	DDRA = 0xFF;     // PORTA als output
	DDRC = 0x00;
	DDRA |= 0x50;    // PA4 en PA6 als output
	DDRB = 0x01;     // PB0 als output
	spi_masterInit();
	displayDriverInit();
	lcd_init();
	lcd_clear();
	lcd_set_line1();
	display_text("Temp:");
 
	adcInit();
	uint8_t measuredTemp = adcRead(); // Lees temperatuur bij opstart
	int desiredTemp = measuredTemp;   // Stel gewenste temperatuur in
	while (1) {
		// --- Lees knoppen ---
		if (PINC & BIT(0)) {       // PC0 ingedrukt: verhogen
			desiredTemp++;
			if (desiredTemp > 99) desiredTemp = 99;
			wait(300); // debounce
		}
		if (PINC & BIT(1)) {       // PC1 ingedrukt: verlagen
			desiredTemp--;
			if (desiredTemp < 0) desiredTemp = 0;
			wait(300); // debounce
		}
		// --- Lees actuele temperatuur ---
		measuredTemp = adcRead();
		// --- Toon gemeten temperatuur op LCD ---
		updateLEDs(measuredTemp);
		updateLCD(measuredTemp);
		// --- Toon gewenste temperatuur op 7-seg ---
		writeLedDisplay(desiredTemp);
		wait(200);
	}
}