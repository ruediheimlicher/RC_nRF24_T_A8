#include <Arduino.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#include "lcd.h"
#include "expo.h"


#define YAW         0
#define PITCH       1
#define ROLL        2
#define THROTTLE    3
#define AUX         4
#define NUM_SERVOS  8
#define RADIOSTARTED 1
#define LOOPLED 8 // PB0
#define BLINKRATE 0x04FF

uint16_t loopcounter = 0;
uint8_t impulscounter = 0;
uint16_t resetcounter = 0;
uint16_t radiocounter = 1;
uint16_t errcounter = 1;
uint8_t radiostatus = 0;

struct Signal 
{
   byte throttle;
   byte pitch;
   byte roll;
   byte yaw;
   byte aux1;
   byte aux2;
};
Signal data;

#define CE_PIN  PC0   // PC0
#define CSN_PIN PC1  // PC1
const uint64_t pipeIn = 0xABCDABCD71LL;
RF24 radio(CE_PIN, CSN_PIN);

void ResetData()
{

data.throttle = 0;                                         // Define the inicial value of each data input. | Veri girişlerinin başlangıç değerleri
data.roll = 127;
data.pitch = 128;
data.yaw = 129;
data.aux1 = 0;                                              
data.aux2 = 0;
resetcounter++;                                               
}
uint8_t intitRadio_T(void)
{
  ResetData();                                             
  radio.begin();
  radio.setAutoAck(false);
  radio.openWritingPipe(pipeIn); 
  radio.setChannel(124);
  radio.setDataRate(RF24_2MBPS);    // The lowest data rate value for more stable communication  | Daha kararlı iletişim için en düşük veri hızı.
  radio.setPALevel(RF24_PA_MAX); 
  
  radio.stopListening(); 
   if (radio.failureDetected) 
  {
    radio.failureDetected = false;
    lcd_gotoxy(12,0);
    lcd_puts("err");
    return 0;
  }
  else
  {
    ResetData();
    lcd_gotoxy(12,0);
    lcd_puts("OK ");
    return 1;

  }
}
void setup() 
{
  LCD_DDR |= (1<<LCD_RSDS_PIN);
  LCD_DDR |= (1<<LCD_ENABLE_PIN);
  LCD_DDR |= (1<<LCD_CLOCK_PIN);
	lcd_initialize(LCD_FUNCTION_8x2, LCD_CMD_ENTRY_INC, LCD_CMD_ON);
  delay(5);
	lcd_puts("Guten Tag\0");
  pinMode(LOOPLED,OUTPUT);
  pinMode(PC0,OUTPUT); // CE
  pinMode(PC1,OUTPUT); // CSN
  if(intitRadio_T())
  {
    radiostatus |= (1<<RADIOSTARTED);
    lcd_gotoxy(16,0);
    lcd_puts("I OK");
  }
  else
  {
    lcd_gotoxy(16,0);
    lcd_puts("I er");
  }
  _delay_ms(100);
}

unsigned long lastRecvTime = 0;

void recvData()
{
  while ( radio.available() ) 
  {
    radiocounter++;
    radio.read(&data, sizeof(Signal));
    lastRecvTime = millis();                                    // Receive the data | Data alınıyor
  }
}

void sendData()
{
  while ( radio.available() ) 
  {
    if(radio.write(&data, sizeof(Signal)),1)
    {
      radiocounter++;
    }
    else
    {
      errcounter++;
    }
    lastRecvTime = millis();                                    // Receive the data | Data alınıyor
  }
}


void loop() 
{
  loopcounter++;
  if(loopcounter >= BLINKRATE)
  {
    loopcounter = 0;
    impulscounter++;
    digitalWrite(LOOPLED, ! digitalRead(LOOPLED));
    lcd_gotoxy(0,1);
    lcd_putint12(data.yaw);
    
    lcd_gotoxy(12,1);
    lcd_putint12(errcounter);
    lcd_gotoxy(0,3);
    lcd_putint12(impulscounter);
    lcd_gotoxy(6,3);
    lcd_putint12(radiocounter);
    lcd_gotoxy(12,3);
    lcd_putint12(resetcounter);
    radiostatus |= (1<<RADIOSTARTED);
  }
  if( radiostatus & (1<<RADIOSTARTED))
  {
    data.yaw = (127 + 4 * impulscounter) & 0xFF;
    sendData();
    unsigned long now = millis();
    if ( now - lastRecvTime > 1000 ) 
    {
      ResetData();  
    }
    radiostatus &= ~(1<<RADIOSTARTED);
  } 
}

