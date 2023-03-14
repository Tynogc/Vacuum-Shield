#include <Arduino.h>
#include <stdio.h>
//#include <Wire.h>
#include <SPI.h>

// Ausgänge
#define IndikatorLED PB0
#define Dimmer PB1
#define SSlatch PB2
#define RS485DE PD4
#define Pieper PD7

// ADCs
#define ID1 A0
#define ID2 A1
#define PSU_5V A2
#define PSU_24V A3
#define MESS1 A6
#define MESS2 A7


volatile uint16_t adcWerte[6][8]; // Speicherstelle für alle Daten vom ADC, 8 pro Quelle damit man sauber Gleitmittelwert rechnen kann

void setup() {
  // put your setup code here, to run once:
  pinMode(IndikatorLED, OUTPUT);
  pinMode(Dimmer, OUTPUT);
  pinMode(SSlatch, OUTPUT);
  pinMode(RS485DE, OUTPUT);
  pinMode(Pieper, OUTPUT);

  // Peripherie!
  // ADC:
  ADCSRA = 1<<ADEN | 1<<ADIF | 1<<ADIE | 1<<ADPS2 | 1<<ADPS1 | 1<<ADPS0;
  ADCSRB = 0;

  // Serielle Schnittstelle:
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(IndikatorLED, 1);
  delay(100);
  digitalWrite(IndikatorLED, 0);
  delay(900);
}





ISR (ADC_vect, ISR_BLOCK) {
  static uint8_t varNummer = 0;

  // Auslesen!
  if (ADMUX <= 3) {
    adcWerte[ADMUX][varNummer] = ADC;
  } else {
    adcWerte[ADMUX-2][varNummer] = ADC;
  }

  // Wähle nächsten Kanal zur ADC-Messung
  switch (ADMUX & 0x0f) {
    case 0:
    ADMUX = 1;
    break;

    case 1:
    ADMUX = 2;
    break;

    case 2:
    ADMUX = 3;
    break;

    case 3:
    ADMUX = 6;
    break;

    case 6: 
    ADMUX = 7;
    break;

    case 7: 
    ADMUX = 0;
    if (varNummer == 8) {
      varNummer = 0;
    } else {
      varNummer++;
    }

    default:
    ADMUX = 0;
    varNummer = 0;
  }
  ADCSRA |= 1<<ADSC; // Start.
}

//ISR (USART0_RX_vect, ISR_BLOCK) {

//}

void ssSchieber(bool stat) {
  if (stat) {
    digitalWrite(SSlatch, 1);
  } else {
    digitalWrite(SSlatch, 0);
  }
}

void send74hc (uint8_t *txData, uint8_t *rxData, uint8_t len) {
  // sende und empfange Daten mit der HW-SPI Schnittstelle
  SPI.beginTransaction(SPISettings(10000000, MSBFIRST, SPI_MODE0));
  ssSchieber(0);

  for (uint8_t i = 0; i<len; i++) {
    rxData[i] = SPI.transfer(txData[i]);
  }

  ssSchieber(1);
  SPI.endTransaction();
}