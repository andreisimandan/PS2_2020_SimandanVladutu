#define ledR PC5
#define ledG PC4
#define ledB PC3
#define photoPin PC0

//folosit pentru a numara zecimi de secunda
float counter;

//definire load si compare
#define t1_load 0;
#define t1_comp 16000000;

void setup() {
  UCSR0B = 0;
  UCSR0C = 0;

  UCSR0A = (1 << UDRE0);
  UCSR0C = (1 << UCSZ00) | (1 << UCSZ01);
  UCSR0B = (1 << RXEN0) | (1 << TXEN0);

  UBRR0 = 103;//pt Baud de 9600

  DDRC|=(1<<ledR)|(1<<ledG)|(1<<ledB);

  ADCSRA |= ((1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0));
  //seteaza factorul de diviziune intre frecventa clock-ului si input pe ADC-128
  ADMUX |= (1 << REFS0); //AVcc cu capacitor extern la Aref 
  ADCSRA |= (1 << ADSC); //porneste conversia ADC-ului 
  ADCSRA |= (1 << ADEN); //porneste ADC

 TCCR1A = 0;

  TCCR1B &=~ (1<<CS12);
  TCCR1B &=~ (1<<CS11);
  TCCR1B |= (1<<CS10);

  TCNT1 = t1_load;
  OCR1A = t1_comp;

  TIMSK1 = (1<<OCIE1A);

  sei();
}

void USART_send(unsigned char data) {
  while ( !( UCSR0A & (1 << UDRE0)) ) ;
  UDR0 = data;
}

unsigned char USART_GetChar(void) {
  while (!(UCSR0A & (1 << RXC0))) ;
  return UDR0;
}


void afisareNumar(uint32_t numar) {
  char uc;
  while (numar != 0) {
    uc = numar % 10 + '0';
    numar = numar / 10;
    USART_send(uc);
    if (numar == 0) {
      USART_send('\n');
    }
  }
}

int citire_adc(uint8_t canal) //read ADC function
{
  ADMUX &= 0xf0; 
  ADMUX |= canal;//select chanel AO to A5
  ADCSRA |= (1 << ADSC); //start conversion
  while (ADCSRA & (1 << ADSC)); //wait while adc conversion are not updated
  return ADCW; //read and return
}

void citire_tastatura() {
  char c;

  do {
    c = USART_GetChar();
    //USART_send(c);
    
    if (c == 'R') {
      PORTC&=~(1<<ledR);
      PORTC|=(1<<ledG);
      PORTC|=(1<<ledB);
    }
    else if (c == 'G') {
      PORTC|=(1<<ledR);
      PORTC&=~(1<<ledG);
      PORTC|=(1<<ledB);
    }    
    else if(c=='B'){
      PORTC|=(1<<ledR);
      PORTC|=(1<<ledG);
      PORTC&=~(1<<ledB);
    }
  } while (c != '\n');
}

void afisare_culoare(int luminozitate){
  if(380 < luminozitate && luminozitate < 450){
    USART_send('B');
  }
  else if(495 < luminozitate && luminozitate < 570){
    USART_send('G');
  }
  else if(590 < luminozitate && luminozitate < 700){
    USART_send('R');
  }
}

int main() {
  int luminozitate;
  
  setup();
  
  PORTC|=(1<<ledR);
  PORTC|=(1<<ledG);
  PORTC|=(1<<ledB);
  
  do{
    citire_tastatura();
    counter = 0;
    do{
      luminozitate = citire_adc(0);
      afisareNumar(luminozitate);
      afisare_culoare(luminozitate);
    }while(counter < 100);
  }while(1);

  return 0;
}

//counter care numara fiecare 0.1 sec
ISR(TIMER1_COMPA_vect){
  TCNT1 = t1_load;
  counter += 0.1;
}
