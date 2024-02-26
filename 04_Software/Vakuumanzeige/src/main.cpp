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

uint8_t lutZeichenGross[] = {
  0b11111010, // 0
  0b00110000, // 1
  0b11011001, // 2
  0b01111001, // 3
  0b00110011, // 4
  0b01101011, // 5
  0b11101011, // 6
  0b00111000, // 7
  0b11111011, // 8
  0b01111011, // 9
  0b10111011, // A
  0b11100011, // b
  0b11001010, // C
  0b11110001, // d
  0b11001011, // E
  0b10001011, // F
};

uint8_t lutZeichenKlein[] = {
  0b01111101, // 0
  0b01100000, // 1
  0b01010111, // 2
  0b01110110, // 3
  0b01101010, // 4
  0b00111110, // 5
  0b00111111, // 6
  0b01110000, // 7
  0b01111111, // 8
  0b01111110, // 9
  0b01111011, // A
  0b00101111, // b
  0b00011101, // C
  0b01100111, // d
  0b00011111, // E
  0b00011011, // F
};

typedef struct
{
  bool signGross;
  bool signKlein;
  uint8_t zeichenGross[4];
  uint8_t zeichenKlein[3];
  uint8_t dezPunktGross;
  uint8_t dezPunktKlein;
  uint16_t helligkeit;
  uint8_t plusMinus;
  uint8_t statusLeds;
} segments_t; 

typedef struct
{
  segments_t anzeige;
  uint8_t outputs;
  uint8_t inputs;
} shiftReg_t;

typedef struct 
{
  int16_t mantisse;
  int8_t exponent;
} miniFloat_t;

// Globale Variablen

segments_t siebenSegments;
shiftReg_t schieberegisterdaten;
miniFloat_t vakuumsensor_1;
miniFloat_t vakuumsensor_2;
miniFloat_t raumdruck;



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

  // Initialisiere Variablen
  analogWrite(Dimmer, 50);


}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(IndikatorLED, 1);
  delay(100);
  digitalWrite(IndikatorLED, 0);
  delay(900);
}




/*
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
*/



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

int miniFloatToSegments(miniFloat_t *zahl, segments_t *anzeige) {
  if (abs(zahl->exponent) > 99 ) {
    return -1;
  }
  if (abs(zahl->mantisse) > 999 ) {
    return -1;
  }

  // lege Zeichen von Zahl in Anzeige ab. 


  return 0; // 0 ist ok
}

void updateAnzeige (segments_t *anzeige, uint8_t outputs, uint8_t *adr) {
  analogWrite(Dimmer, anzeige->helligkeit);
  uint8_t temp[7];
  uint8_t devAdr[7];
  
  temp[0] = anzeige->plusMinus | (anzeige->statusLeds)<<3;
  temp[1] = anzeige->zeichenKlein[1];
  temp[2] = anzeige->zeichenKlein[0];
  temp[3] = anzeige->zeichenGross[2];
  temp[4] = anzeige->zeichenGross[1];
  temp[5] = anzeige->zeichenGross[0];
  temp[6] = outputs;

  send74hc(temp, devAdr, 7);
  *adr = devAdr[0]; // es wird nur das erste Datenfeld sinnvoll gefüllt, danach kommen Nullen.
}