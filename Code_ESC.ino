#define TRUE        1
#define FALSE       0

uint16_t speed;
char buffer[20];
byte bldc_step;


void setup() {
  // put your setup code here, to run once:

  // Setup ADC 
  ADCSRA|=(1<<ADEN)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0); // ADC_clock = MCU_clock/128 --> 62.5kHz
  
  // Setup PWM 1
  TCCR1A = 0;                         // Reset PWM Output pin + configuration
  TCCR1B = 0;                         // Reset configuration + prescaler
  TCNT1 = 0;                          // Reset counter
  OCR1A = 0;                          // Reset comp A --> Pin13
  OCR1B = 0;                          // Reset comp B --> Pin14


  // Setup PWM 2
  TCCR2A = 0;                         // Reset PWM Output pin + configuration
  TCCR2B = 0;                         // Reset configuration + prescaler
  TCNT2 = 0;                          // Reset counter
  OCR2A = 0;                          // Reset comp A --> Pin15

  // Setup Hardware
  DDRB = 0b00001110;                  // PB3 = C (OC2A), PB2 = B (OC1B), PB1 = A (OC1A)
  PORTB = 0b00000000;
  DDRD = 0b00011100;                  // Configure Pins 13, 14 and 15 as outputs
  PORTD = 0b00000000;

  // Setup Interrupt
  PCICR  = (1 << PCIE2);              // Enable pin change interrupt for pins PD0 to PD7
  PCMSK2 = 0b11100000;                // Enable pin change interrupt for pins PD5, PD6 and PD7


  TCCR1B |= (1 << CS10);
  TCCR2B |= (1 << CS10);

  // BLDC motor first move
  bldc_step = (PIND >> 5) & 7;       // Read hall effect sensors status
  bldc_move();                       // Move the BLDC motor (first move)

}

void loop() {
  ReadADC(0);
  speed = ADC;
  SET_PWM_DUTY();
}

ISR (PCINT2_vect){
  bldc_step = (PIND >> 5) & 7;        // Read hall effect sensors status (PIND: read from PORTD which is arduino pins 0..7)
  bldc_move();                        // Move the BLDC motor
}

void bldc_move(){                     // BLDC motor commutation function
  switch(bldc_step){
    case 1:
      AH_CL();
      break;
    case 2:
      BH_AL();
      break;
    case 3:
      BH_CL();
      break;
    case 4:
      CH_BL();
      break;
    case 5:
      AH_BL();
      break;
    case 6:
      CH_AL();
      break;
    default:
      PORTD = 0;
      PORTB = 0;
      break;
  }
}

void AH_BL(){
  PORTD &= ~0x14;
  PORTD |=  0x08;
  PORTB = 0b00000010;
  TCCR2A =  0;                        // Turn pin PB1 (OC1A) PWM ON (PB2 & PB3 OFF)
  TCCR1A =  0x81;
}
void AH_CL(){
  PORTD &= ~0x0C;
  PORTD |=  0x10;
  PORTB = 0b00000010;
  TCCR2A =  0;                        // Turn pin PB1 (OC1A) PWM ON (PB2 & PB3 OFF)
  TCCR1A =  0x81;
}
void BH_CL(){
  PORTD &= ~0x0C;
  PORTD |=  0x10;
  PORTB = 0b00000100;
  TCCR2A =  0;                        // Turn pin PB2 (OC1B) PWM ON (PB1 & PB3 OFF)
  TCCR1A =  0x21;
}
void BH_AL(){
  PORTD &= ~0x18;
  PORTD |=  0x04;
  PORTB = 0b00000100;
  TCCR2A =  0;                        // Turn pin PB2 (OC1B) PWM ON (PB1 & PB3 OFF)
  TCCR1A =  0x21;
}
void CH_AL(){
  PORTD &= ~0x18;
  PORTD |=  0x04;
  PORTB = 0b00001000;
  TCCR1A =  0;                        // Turn pin PB3 (OC1B) PWM ON (PB1 & PB2 OFF)
  TCCR2A =  0x81;
}
void CH_BL(){
  PORTD &= ~0x14;
  PORTD |=  0x08;
  PORTB = 0b00001000;
  TCCR1A =  0;                        // Turn pin PB3 (OC1B) PWM ON (PB1 & PB2 OFF)
  TCCR2A =  0x81;
}

void SET_PWM_DUTY(){
  OCR1A = (speed*0.249);                  // Set PB1  PWM duty cycle
  OCR1B = (speed*0.249);                  // Set PB2 PWM duty cycle
  OCR2A = (speed*0.249);                  // Set PB3 PWM duty cycle
}

void ReadADC(uint8_t ch)
{
  
  //Select ADC Channel ch must be 0-7
  ch=ch&0b01000111;
  ADMUX|=ch;
  
  ADCSRA|=(1<<ADSC);
  
  while(!(ADCSRA & (1<<ADIF)))
  {
  };

  //Clear ADIF by writing one to it
  ADCSRA|=(1<<ADIF)|(0<<ADSC);
  
}

// For debug
//void USART_Init_9600(void)
//{
//  // fréquence horloge = 1000000 hz, Si Baudrate = 9600 alors UBRR = 12
//  //1xspeed  U2X0 = 1
//  UCSR0A |= (1<<U2X0);
//  // 9600 baud
//  //UBRR0 = 12;
//  UBRR0H = 0x00;
//  UBRR0L = 0xCF;
//  // Configuration en émission seulement.
//  //(UCSR0B) RXCIE0 = 0 | TXCIE0 =0 | UDRIE0 = 0 | RXEN0 =0 | TXEN0 = 1 | UCSZ02 = 0 | RXB80 = 0 | TXB80 = 0
//  UCSR0B = 0b00001000;
//  
//  // Async. mode, 8 bits, 1 bit de stop, pas de contrôle de parité
//  //(UCSR0C) UMSEL01 = 0 | UMSEL00 = 0 | UPM01 = 1 | UPM00 = 0 | USBS0 = 0 | UCSZ01 = 1 | UCSZ00 = 1 | UCPOL0 = 0;
//  UCSR0C|=(1<<UPM01);
//  /*
//  UBRR0L=0b00001100; //Baudrate = 9600bps
//  UBRR0H=0b00000000;
//  UCSR0A = 0b00100010; //receiver is ready to receive data, double speed
//  UCSR0B = 0b10011000; //Activation de l'émetteur et activation de l'interruption sur la reception USART
//  UCSR0C = 0b00000110; //Mode asynchrone, pas de bit de parité, 1 bit de stop, 8 bits de données*/
//}
//
//void Usart_Tx(char data)
//{
//  // UDRE Flag , is the transmit buffer UDR) ready to receive new data ?
//  // if UDRE0 =1 the buffer is empty
//  while (!(UCSR0A & (1<<UDRE0)));
//  UDR0 = data;
//}
//
//void Usart_Tx_String(char *String)
//{
//  char Continue = TRUE;
//  while (Continue)
//  {
//    if(*String==0) Continue = FALSE;
//    else Usart_Tx(*String++);
//  }
//}
