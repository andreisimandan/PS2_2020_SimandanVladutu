//definire senzori IR, stanga, mijloc, dreapta
#define PS_IR_stanga PB5
#define PS_IR_mijloc PB4
#define PS_IR_dreapta PB0

//definire frecventa procesor
#define F_CPU 16000000  //Frecventa procesorului, 16MHz

//definire motoare
#define IN1 PD6
#define IN2 PD7
#define IN3 PD5
#define IN4 PD4

//definire load si compare
#define t1_load 0;
#define t1_comp 16000000;

//folosit pentru a numara zecimi de secunda
float counter;

void init() {
  UCSR0B = 0;
  UCSR0C = 0;

  UCSR0A = (1 << UDRE0);
  UCSR0C = (1 << UCSZ00) | (1 << UCSZ01);
  UCSR0B = (1 << RXEN0) | (1 << TXEN0);

  UBRR0 = 103;//pt Baud de 9600

  TCCR0A |= (1 << COM0A1) | (1 << COM0B1);
  // seteaza modul non invertor
  TCCR0A |= (1 << WGM01) | (1 << WGM00);
  // seteaza modul PWM rapid
  TCCR0B |= (1 << CS01);
  // set prescaler-ul de 8

  DDRD |= (1 << IN1) | (1 << IN2) | (1 << IN3) | (1 << IN4);

  TCCR1A = 0;

  TCCR1B &=~ (1<<CS12);
  TCCR1B &=~ (1<<CS11);
  TCCR1B |= (1<<CS10);

  TCNT1 = t1_load;
  OCR1A = t1_comp;

  TIMSK1 = (1<<OCIE1A);

  sei();
}

void seteaza_PWM_A(uint8_t duty) {
  OCR0A = duty;
}

void seteaza_PWM_B(uint8_t duty) {
  OCR0B = duty;
}

void USART_send(unsigned char data) {
  while ( !( UCSR0A & (1 << UDRE0)) ) ;
  UDR0 = data;
}

unsigned char USART_GetChar(void) {
  while (!(UCSR0A & (1 << RXC0))) ;
  return UDR0;
}

void Mers_fata(){
  seteaza_PWM_A(255);
  PORTD&=~(1<<IN2);
  seteaza_PWM_B(255);
  PORTD&=~(1<<IN4);
}

void Mers_stanga_fata(){
  seteaza_PWM_A(255);
  PORTD&=~(1<<IN2);
  seteaza_PWM_B(128);
  PORTD&=~(1<<IN4);
}

void Mers_dreapta_fata(){
  seteaza_PWM_A(128);
  PORTD&=~(1<<IN2);
  seteaza_PWM_B(255);
  PORTD&=~(1<<IN4);
}

void Mers_spate(){
  seteaza_PWM_A(0);
  PORTD|=(1<<IN2);
  seteaza_PWM_B(0);
  PORTD|=(1<<IN4);
}

void Rotit_pe_loc_dreapta(){
  seteaza_PWM_A(255);
  PORTD&=~(1<<IN2);
  seteaza_PWM_B(0);
  PORTD|=(1<<IN4);
}

void Rotit_pe_loc_stanga(){
  seteaza_PWM_A(0);
  PORTD|=(1<<IN2);
  seteaza_PWM_B(255);
  PORTD&=~(1<<IN4);
}

void Oprit(){
  seteaza_PWM_A(0);
  PORTD&=~(1<<IN2);
  seteaza_PWM_B(0);
  PORTD&=~(1<<IN4);
}

bool verificareSenzor(volatile uint8_t senzorIR){
  if(PIND & (1 << senzorIR)){
    return true;
  }
  else{
    return false;
  }
}

int main(){
  init();
  while(1){
    if(verificareSenzor(PS_IR_mijloc)){
      Mers_fata();
      //if(!verificareSenzor(PS_IR_stanga)&&!verificareSenzor(PS_IR_dreapta)){
       // Mers_fata();
      }
    }
    else{
      do{
        counter = 0;
        Rotit_pe_loc_stanga();
      }while(!verificareSenzor(PS_IR_mijloc) && counter < 10);
      if(counter > 10){
        do{
          counter = 0;
          Rotit_pe_loc_dreapta();
        }while(!verificareSenzor(PS_IR_mijloc) && counter < 20);
      }
    }
  }
}

//counter care numara fiecare 0.1 sec
ISR(TIMER1_COMPA_vect){
  TCNT1 = t1_load;
  counter += 0.1;
}
