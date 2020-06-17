#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#define MAXVAL_TIMER1 32767

#define ROSU 1
#define ALBASTRU 2
#define GALBEN 3
#define ALB 4


void EXTINT1_ULTRASONIC_SETUP(){    //Parte din modulul de ocolire
  EICRA|=(1<<ISC10)|(1<<ISC11);
  EIMSK|=(1<<INT1);
}

// setare timer1 pt PWM
void timer1_setup_pwm() {     //timer 1 lucreaza in modul Fast PWM, numarand pana la valoarea din ICR1;
  TCNT1 = 0;
  TCCR1A = (1 << COM1A1) | (1 << COM1B1) |(1<<WGM11);
  TCCR1B = (1<<WGM13)|(1 << WGM12) | (1 << CS11);
  ICR1=MAXVAL_TIMER1;
  OCR1A = 0;
  OCR1B = 0;
}

void curba_var(int motor_st_fact,int motor_dr_fact){  //cei 2 factori iau valori intre 0 si 100, robotul se va deplasa inainte dar cu vitezele rotilor variabile prin cei 2 factori
  OCR1A = (32767/100)*motor_dr_fact;
  OCR1B = (32767/100)*motor_st_fact;
  PORTB &= ~0x30;
  TCCR1A&=~((1 << COM1A1) | (1 << COM1B1) | (1 << COM1A0) | (1<< COM1B0));
  TCCR1A |= (1 << COM1A1) | (1 << COM1B1);
}

void fata()
{
  OCR1A = 32767;
  OCR1B = 32767;
  PORTB &= ~0x30;
  TCCR1A&=~((1 << COM1A1) | (1 << COM1B1) | (1 << COM1A0) | (1<< COM1B0));
  TCCR1A |= (1 << COM1A1) | (1 << COM1B1);
}

void spate()
{
  OCR1A = 32767;
  OCR1B = 32767;
  PORTB |= 0x30;
  TCCR1A&=~((1 << COM1A1) | (1 << COM1B1) | (1 << COM1A0) | (1<< COM1B0));
  TCCR1A |= (1 << COM1A1) | (1 << COM1B1) | (1 << COM1A0) | (1<< COM1B0);
}
void stanga_peloc()
{
  OCR1A = 20000;
  OCR1B = 20000;
  PORTB |=0x10;
  PORTB&=~0x20;
  TCCR1A&=~((1 << COM1A1) | (1 << COM1B1) | (1 << COM1A0) | (1<< COM1B0));
  TCCR1A |= (1<<COM1A0)|(1 << COM1A1) | (1 << COM1B1);
}

void dreapta_peloc()
{
  OCR1A = 20000;
  OCR1B = 20000;
  PORTB |=0X20;
  PORTB&=~0x10;
  TCCR1A&=~((1 << COM1A1) | (1 << COM1B1) | (1 << COM1A0) | (1<< COM1B0));
  TCCR1A |= (1 << COM1A1) | (1<<COM1B0)|(1 << COM1B1);
}

void stop_rob(){
  OCR1A=0;
  OCR1B=0;
  PORTB&=~0X30;
  TCCR1A&=~((1 << COM1A1) | (1 << COM1B1) | (1 << COM1A0) | (1<< COM1B0));
}
void adc_init()
{
  ADCSRA |= ((1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0));
  ADCSRA |= (1 << ADEN); //enable ADC
  ADMUX |= (1 << REFS0);
  ADCSRA |= (1 << ADSC); //ADC start conversion
}

int read_adc(int channel)
{
  ADMUX &= ~(1 << channel);
  ADMUX |= channel;//select chanel AO to A5
  ADCSRA |= (1 << ADSC); //start conversion
  while (ADCSRA & (1 << ADSC)); //wait while adc conversion are not updated
  return ADCW; //read and return
}

void actiune(int culoare){
  stop_rob();
  switch(culoare){
    case ROSU: stop_rob();  //stai 3 secunde si mergi in fata 1 secunda
     _delay_ms(3000);
     fata();
     _delay_ms(1000);
    break;
    case ALBASTRU: 
      fata(); //continua
      _delay_ms(1000);
      break;
    case ALB: //nu facem nimic
      break;
    case GALBEN: 
      curba_var(100,80);   //fa o curba la dreapta timp de o secunda
     _delay_ms(1000);
      break;
  }
}

void detectie_culoare(){
  long int magnitudine=0;
  for(int i=0;i<10;i++){
    magnitudine=magnitudine+read_adc(5);
  }
  magnitudine=magnitudine/10;
  if(magnitudine>=175){
    Serial.println("ALB");
    actiune(ALB);
  }
  else if(magnitudine>=140&&magnitudine<175) {
    Serial.println("GALBEN");
    actiune(GALBEN);
  }
  else if(magnitudine>=90&&magnitudine<140) {
    Serial.println("ROSU");
    actiune(ROSU);
  }
  else {
    Serial.println("ALBASTRU");
    actiune(ALBASTRU);
  }
}

int main() {
  cli();
  timer1_setup_pwm();
  adc_init();
  DDRB = 0xFF;
  DDRC = 0x70;
  DDRC&=~(1<<5);
  sei();
  stop_rob();
  PORTC|=0X07;
  _delay_ms(3000);
  Serial.begin(9600);
  while (1) {
      detectie_culoare();
      _delay_ms(1000);
  }
  return 0;
}
