#include <Arduino.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#include "lcd.h"
#include "expo.h"

/*
RC_nRF_Receiver A328 exp

*/

#define OSZIPORT           PORTD
#define OSZIDDR            DDRD
#define OSZIPORTPIN        PINB
#define OSZI_PULS_A        1
#define OSZI_PULS_B        4




#define OSZI_A_LO OSZIPORT &= ~(1<<OSZI_PULS_A)
#define OSZI_A_HI OSZIPORT |= (1<<OSZI_PULS_A)
#define OSZI_A_TOGG OSZIPORT ^= (1<<OSZI_PULS_A)


#define OSZI_B_LO OSZIPORT &= ~(1<<OSZI_PULS_B)
#define OSZI_B_HI OSZIPORT |= (1<<OSZI_PULS_B)
#define OSZI_B_TOGG OSZIPORT ^= (1<<OSZI_PULS_B)



//PPM
// RC_PPM_328_13
// -----------------------------------------------
// Timer1 starten: Impulspaket steuern
// -----------------------------------------------

#define YAW         0
#define PITCH       1
#define ROLL        2
#define THROTTLE    3
#define AUX         4

#define NUM_SERVOS  8

#define KANAL_PORT            PORTD  //    Ausgang Summensignal 
#define KANAL_DDR             DDRD    //    


#define KANAL_PIN          3                             // Ausgang fuer Summensignal
#define KANAL_LO           KANAL_PORT &= ~(1<<KANAL_PIN)
#define KANAL_HI           KANAL_PORT |= (1<<KANAL_PIN)
volatile uint8_t    ppmcounter=0x00;
#define PPM_FAKTOR 2
volatile uint16_t ppm_signal[NUM_SERVOS] = {};  // Letzter Wert = Sync



void timer1_init(void)
{
   // Quelle http://www.mikrocontroller.net/topic/103629
   TCCR1A = 0;
   TCCR1B |= (1<<CS11); // f/8 2 MHz -> 0.5 us
   TCNT1  = 0;														// reset Timer
   
    OCR1B  = 0x80;				// Impulsdauer des Kanalimpulses  32 us
   
   TIMSK1 |= (1 << OCIE1A);  // enable timer compare interrupt:
   TIMSK1 |= (1 << OCIE1B);  // enable timer compare interrupt:
   
   
  
   ppmcounter = 0;
   /*
      if (ppmarray[impulscounter] > SERVOMAX)
      {
         ppmarray[impulscounter] = SERVOMAX;
      }
      
      else if (ppmarray[impulscounter] < SERVOMIN)
      {
         ppmarray[impulscounter] = SERVOMIN;
      }
   */
   OCR1A  =  ppm_signal[ppmcounter]; // POT_Faktor schon nach ADC

   KANAL_PORT |= (1<<KANAL_PIN);
  //PORTB &= ~(1<<PB1); // inv
   _delay_us(2);
} 
// end timer1

// -----------------------------------------------
// Timer1 nach Ende des Impulspakets beenden
// -----------------------------------------------

void timer1_stop(void)
{
   //TCCR1B = 0;

   
}


#pragma mark timer1 COMPA-vect
// -----------------------------------------------
// Kanalimpuls nach Potentiometerstellung neu starten
// -----------------------------------------------
volatile uint8_t count = 0;
ISR(TIMER1_COMPA_vect)	 //Ende der Pulslaenge fuer einen Kanal
{
   
   if (ppmcounter <= NUM_SERVOS)
   {
      
      // Start Impuls
      //KANAL_HI;
      KANAL_PORT |= (1<<KANAL_PIN);
      PORTB &= ~(1<<PB1); // inv
      TCNT1  = 0;
      
      // -----------------------------------------------
      // Laenge des naechsten Kanals setzen
      // -----------------------------------------------
      /*
      uint16_t tempServoWert=Servo_ArrayInt[impulscounter];
      
      if (tempServoWert > SERVOMAX)
      {
         tempServoWert = SERVOMAX;
      }
      
      if (tempServoWert < SERVOMIN)
      {
         tempServoWert = SERVOMIN;
      }
      OCR1A  = tempServoWert; // POT_Faktor schon nach ADC
   */
        ppmcounter++;
         OCR1A  = ppm_signal[ppmcounter]; // Impulslaenge des naechsten Kanals
         
         
   }
   else
   {
      // Ende Impulspaket
      
      // KANAL_LO;
      KANAL_PORT &= ~(1<<KANAL_PIN);
      PORTB |= (1<<PB1); // inv

      // Alle Impulse gesendet, Timer1 stop. Timer1 wird bei Beginn des naechsten Paketes wieder gestartet
      //timer1_stop();
      
      // SPI fuer device ausschalten
     
      
      //_delay_us(2);
      
     
   }
   
   //OSZI_B_LO ;
 }

#pragma mark timer1 COMPB-vect
// -----------------------------------------------
// Kanalimpuls-Spike beenden
// -----------------------------------------------

ISR(TIMER1_COMPB_vect)	 //Ende des Kanalimpuls. ca 0.3 ms
{
   //OSZI_B_HI;
   KANAL_PORT &= ~(1<<KANAL_PIN);
   PORTB |= (1<<PB1);// Inv
   //OSZI_A_HI;
   if (ppmcounter <= NUM_SERVOS+1)
   {
      
   }
   else
   {
      //timer1_stop();
      //ppmcounter = 0;
      //OCR1A = 1000;
   }
}


void timer2_init(void)
{
    // CTC-Modus aktivieren (Clear Timer on Compare Match)
    TCCR2A = (1 << WGM21);
    
    // Prescaler 128
    TCCR2B = (1 << CS22) | (1 << CS21); // 128 = CS22 + CS20

    // Vergleichswert setzen (250 Takte → 2 ms bei 128er Prescaler)
    OCR2A = 124;

    // Compare-Match-Interrupt aktivieren
    TIMSK2 = (1 << OCIE2A);

}


//MARK: timer2 COMPA-vect
// -----------------------------------------------
// Kanalimpuls nach Potentiometerstellung neu starten
// -----------------------------------------------

ISR(TIMER2_COMPA_vect)	 //Ende des Pulspakets
{
   count++;
    if (count >= 20) {  // 10 x 2ms = 20ms
        count = 0;
        OSZI_A_LO;
        KANAL_PORT &= ~(1<<KANAL_PIN);
        PORTB |= (1<<PB1); // inv
        // Impuls oder Aktion alle 20 ms
        timer1_init();  // timer 1 einschalten
        _delay_us(2);
        OSZI_A_HI;
    }
 }


// PPM

#define LOOPLED 8 // PB0

#define BLINKRATE 0x04FF

uint16_t loopcounter = 0;

uint8_t impulscounter = 0;
uint16_t resetcounter = 0;
uint16_t radiocounter = 1;
uint16_t errcounter = 1;

uint8_t radiostatus = 0;


#define FIRSTTIMEDELAY  0x0FF
#define RADIOSTARTED    1
uint16_t firsttimecounter = 0;

int ch_width_0;

int ch_width_1 = 0;
int ch_width_2 = 0;
int ch_width_3 = 0;
int ch_width_4 = 0;
int ch_width_5 = 0;
int ch_width_6 = 0;


//Servo ch1;
/*
Servo ch2;
Servo ch3;
Servo ch4;
Servo ch5;
Servo ch6;
*/

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
/*
#define S0  10    // PB2 // YAW
#define S1  9     // PB1 // PITCH
#define S2  A2    // PC2 // ROLL
#define S3  A3    // PC3 // THROTTLE
#define IO0 3     // PD3
#define IO1 A0    // PD1
*/

#define S0  A2    // PB2 // YAW
#define S1  A3     // PB1 // PITCH
#define S2  10    // PC2 // ROLL
#define S3  9    // PC3 // THROTTLE
#define IO0 3     // PD3
#define IO1 A0    // PD1



#define CE_PIN  PC0   // PC0
#define CSN_PIN PC1  // PC1

  const uint64_t pipeIn = 0xABCDABCD71LL;

// instantiate an object for the nRF24L01 transceiver
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

uint8_t initradio(void)
{
  ResetData();                                             // Configure the NRF24 module  | NRF24 Modül konfigürasyonu
  radio.begin();
  radio.openReadingPipe(1,pipeIn);
  radio.setChannel(124);
  radio.setAutoAck(false);
  radio.setDataRate(RF24_2MBPS); // Set the speed of the transmission to the quickest available
  radio.setPALevel(RF24_PA_MAX);                           // Output power is set for maximum |  Çıkış gücü maksimum için ayarlanıyor.
  radio.setPALevel(RF24_PA_MIN); 
  radio.setPALevel(RF24_PA_MAX); 
  radio.setPayloadSize(sizeof(Signal));
  
  radio.startListening(); 
   if (radio.failureDetected) 
  {
    radio.failureDetected = false;
    return 0;
  }
  else
  {
    ResetData();
    return 1;

  } 
}

uint8_t intitRadio_T(void)
{
  ResetData();                                             
  // Configure the NRF24 module  | NRF24 Modül konfigürasyonu
  radio.begin();
  radio.setAutoAck(false);
  radio.openWritingPipe(pipeIn); 
  radio.setChannel(124);
  radio.setDataRate(RF24_250KBPS);    // The lowest data rate value for more stable communication  | Daha kararlı iletişim için en düşük veri hızı.
  //radio.setDataRate(RF24_2MBPS);  // Set the speed of the transmission to the quickest available
  //radio.setPALevel(RF24_PA_MAX);                           // Output power is set for maximum |  Çıkış gücü maksimum için ayarlanıyor.
  radio.setPALevel(RF24_PA_LOW); 
  //radio.setPALevel(RF24_PA_MAX); 
  
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
  // Start the radio comunication for receiver | Alıcı için sinyal iletişimini başlatır.
}



void setup() 
{
  //KANAL_DDR |=  (1<<KANAL_PIN);
  //KANAL_PORT |=  (1<<KANAL_PIN); // Ausgang

  //DDRB |= (1<<PB0);
  //DDRB |= (1<<PB1);

  //DDRC |= (1<<PC0);
  //DDRC |= (1<<PC1);

  //PORTB &= ~(1<<PB1);// inv LO

  //timer1_init();
  //timer2_init();


  LCD_DDR |= (1<<LCD_RSDS_PIN);
  LCD_DDR |= (1<<LCD_ENABLE_PIN);
  LCD_DDR |= (1<<LCD_CLOCK_PIN);

	lcd_initialize(LCD_FUNCTION_8x2, LCD_CMD_ENTRY_INC, LCD_CMD_ON);
  delay(5);
	lcd_puts("Guten Tag\0");

  //DDRC |= (1<<PC3);

 // DDRB |= (1<<0);
  pinMode(LOOPLED,OUTPUT);
  //pinMode(2,INPUT); // IRQ
  pinMode(PC0,OUTPUT); // CE
  pinMode(PC1,OUTPUT); // CSN

  /*
  // Set the pins for each PWM signal | Her bir PWM sinyal için pinler belirleniyor.
  ch1.attach(S0); 
  ch2.attach(S1);
  ch3.attach(S2);
  ch4.attach(S3);
  ch5.attach(IO0);
  ch6.attach(IO1);
  */

  //ResetData();                                            
  
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
  
  //ResetData();
  
  
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
    if(radio.write(&data, sizeof(Signal)))
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
    //PORTB ^= (1<<0);
    loopcounter = 0;
    impulscounter++;
    digitalWrite(LOOPLED, ! digitalRead(LOOPLED));
    //digitalWrite(A0, ! digitalRead(A0))
    lcd_gotoxy(0,1);
    lcd_putint12(data.yaw);
    // lcd_gotoxy(6,1);
    //lcd_putint12(ppm_signal[0]);

    lcd_gotoxy(12,1);
    lcd_putint12(errcounter);


    //lcd_gotoxy(0,2);
    //lcd_putint12(data.pitch);
    //lcd_gotoxy(6,1);
    //lcd_putint12(ppm_signal[1]);

    //lcd_gotoxy(12,2);
    //lcd_putint12(ch_width_1);

    

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
      OSZI_B_LO;
      ResetData();  
    }
    OSZI_B_HI;
    radiostatus &= ~(1<<RADIOSTARTED);
  } 
  //data.roll = (impulscounter & 0xFF );//& 0xFF00) >> 8;

  // map: 
  // map(value, fromLow, fromHigh, toLow, toHigh)
  
  /*
  ch_width_1 = map(data.yaw, 0, 255, 1200, 1800);
  ch_width_2 = map(data.pitch, 0, 255, 1000, 2000); 

  ch_width_3 = map(data.roll, 0, 255, 1000, 2000); 
  ch_width_4 = map(data.throttle, 0, 255, 1000, 2000); 

  // ON/OFF
  ch_width_5 = map(data.aux1, 0, 1, 1000, 2000); 
  //ch_width_6 = map(data.aux2, 0, 1, 1000, 2000); 
  //ch_width_6 = map((impulscounter & 0xFF ), 0, 255, 1000, 2000);

  ch1.writeMicroseconds(ch_width_1);                          // Write the PWM signal | PWM sinyaller çıkışlara gönderiliyor
  ch2.writeMicroseconds(ch_width_2);
  ch3.writeMicroseconds(ch_width_3);
  ch4.writeMicroseconds(ch_width_4);
  ch5.writeMicroseconds(ch_width_5);
  //ch6.writeMicroseconds(ch_width_6); 
  */


  ch_width_0 = map(data.yaw, 80, 255, 1000, 2000);
  ch_width_1 = map(data.pitch, 80, 255, 1000, 2000);
  //ch1.writeMicroseconds(ch_width_0);                          // Write the PWM signal | PWM sinyaller çıkışlara gönderiliyor
  //ch2.writeMicroseconds(ch_width_);
  ppm_signal[YAW] = data.yaw * PPM_FAKTOR;    
  ppm_signal[PITCH] = data.pitch * PPM_FAKTOR;    
  ppm_signal[ROLL] = data.roll * PPM_FAKTOR;       
  ppm_signal[THROTTLE] = (data.throttle + 127) * PPM_FAKTOR;
/*
  ppm_signal[YAW+4] = data.yaw * PPM_FAKTOR;    
  ppm_signal[PITCH+4] = data.pitch * PPM_FAKTOR;    
  ppm_signal[ROLL+4] = data.roll * PPM_FAKTOR;       
  ppm_signal[THROTTLE+4] = data.throttle * PPM_FAKTOR;
*/
}

