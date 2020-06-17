#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#define MAXVAL_TIMER1 32767
#define timer0_10ms_pres1024 156

#define kp 3
#define ki 0
#define kd 5

#define ROSU 1
#define ALBASTRU 2
#define GALBEN 3
#define ALB 4

#define WAIT 0
#define FREE 1

volatile int err_prec=0;
volatile int I=0;

volatile uint32_t timp_us, timp_initial, timp_final_delay,timp_final,timeout;
volatile uint8_t distanta,contor_obj=0;
volatile uint8_t stare_sonar=FREE;

void timer0_setup_pres1024_10msCTC(){
  TCCR0A|=(1<<WGM01);
  TCCR0B=(1<<CS02)|(1<<CS00);
  OCR0A=timer0_10ms_pres1024;
  TIMSK0|=(1<<OCIE0A);
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

int val_senzori_linie(){
  int val=PIND&0XF8;
  return val>>3;
}

short int error(){
  int val=val_senzori_linie(),err=-2;
  if(val==(0xF8>>3)) return err_prec;
  while(val){
    if(!(val&1)) break;
    else err++;
    val>>=1;
  }
  return err;
}

int pid(){
  int err=error(),D=err-err_prec;
  I+=err;
  if(I>50) I=50;
  else if(I<-50) I=-50;
  err_prec=err;
  return kp*err+kd*D+ki*I;
}

void control(){
  int comanda=pid(),stanga=40,dreapta=40;
  if(comanda<0) dreapta=dreapta-comanda;
  else if(comanda>0) stanga=stanga+comanda;
  curba_var(dreapta,stanga);
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
  switch(culoare){
    case ROSU: 
    cli();
    stop_rob();  //stai 3 secunde si mergi in fata
    _delay_ms(3000);
    fata();
    _delay_ms(200);
    break;
    case ALBASTRU: break;
    case ALB: break;
  case GALBEN: 
    cli();
    stop_rob();
    curba_var(40,99);   //fa o curba la dreapta timp de o secunda
    _delay_ms(700);
    break;
  }
  sei();
}

void detectie_culoare(){
  /*long int vals[3],magnitudine;
  for(int i=0;i<3;i++){
    PORTC|=(1<<i);
    _delay_ms(30);
    vals[i]=read_adc(5);
    PORTC&=~(1<<i);
    _delay_ms(1);
  }
  magnitudine=vals[0]*vals[0]+vals[1]*vals[1]+vals[2]*vals[2];
  magnitudine=sqrt(magnitudine);*/
  long int magnitudine=0;
  PORTC|=0x07;
  for(int i=0;i<10;i++){
    magnitudine=magnitudine+read_adc(5);
    _delay_ms(1);
  }
  magnitudine=magnitudine/10;
  if(magnitudine>=175) actiune(ALB);
  else if(magnitudine>=140&&magnitudine<175) actiune(GALBEN);
  else if(magnitudine>=110&&magnitudine<140) actiune(ROSU);
  else actiune(ALBASTRU);
}

int main() {
  cli();
  DDRB = 0xFF;
  DDRC = 0x70;
  DDRC&=~(1<<5);
  timer1_setup_pwm();
  adc_init();
  sei();
  stop_rob();
  _delay_ms(3000);
  timer0_setup_pres1024_10msCTC();
  stanga_peloc();
  while (1) {
    //detectie_culoare();
  }
  return 0;
}

ISR(TIMER0_COMPA_vect){
  sei();
  control();
}
