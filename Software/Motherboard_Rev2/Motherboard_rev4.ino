#include <mcp_can.h>
#include <SPI.h>
 
float APPS1_Voltage;
float APPS2_Voltage;
float BrakeSensor_Voltage;
float APPS1_PedalTravel;
float APPS2_PedalTravel;  
float Delta;
float current;

int Faultpin = 0;
int BrakeLight_CTRLpin = 1;
int APPS_CTRLpin = 2;
int RTDS_CTRLpin = 3;
int PRE_RELAY = 4;
int APPS_SCSpin = 5;
int AIR_AUXpin = 6;
int PRE_CTRLpin = 7;
int RCTOMB = 8;
int Start_Buttonpin = 9;
int fault;

int BrakeSensor_Reading;
int APPS1_newReading;
int APPS2_newReading;
int timer = 1;
int RTD = 0;
int Pre_done = 0;
int TSMS;

const uint16_t t1_load = 0;
uint16_t t1_comp1  = 7813; //500ms
uint16_t t1_comp2 = 1563;  //100ms
uint16_t t1_comp3 = 23437; //1.5s

bool error1 = true;
bool error2 = true;

void Read_APPS_Data();
void Impl1();
void Impl2();
void timer_init(uint16_t comp);
void fault_led(int fault);
int RTDS();
int Brakelight_ON();
 
void setup() {
 
  pinMode(RTDS_CTRLpin,OUTPUT);
  pinMode(APPS_CTRLpin,OUTPUT);            
  pinMode(Start_Buttonpin,INPUT);
  pinMode(RCTOMB,INPUT);
  pinMode(APPS_SCSpin,OUTPUT);
  pinMode(PRE_RELAY,OUTPUT);
  pinMode(Faultpin,OUTPUT);
  pinMode(BrakeLight_CTRLpin,OUTPUT);
  pinMode(PRE_CTRLpin,INPUT);
  pinMode(A0, INPUT_PULLUP);
  pinMode(A1, INPUT_PULLUP);
  digitalWrite(APPS_SCSpin,HIGH);
  digitalWrite(PRE_RELAY,HIGH);
  digitalWrite(BrakeLight_CTRLpin,LOW);
  PORTD |= (1 <<  APPS_CTRLpin);
  TCCR1A = 0;
  TCCR0A = 0;
  TCCR1B |= (1<<CS12);
  TCCR1B &= ~(1<<CS11);
  TCCR1B |= (1<<CS10);
 
  noInterrupts();
 
}

ISR(TIMER1_COMPA_vect){
  timer = 0;
}

void Read_APPS_Data(){
  int apps1_arr[40] = {};
  int apps2_arr[40] = {};
  float paddu_apps1 = 0;
  float paddu_apps2 = 0;
 
  for(int i = 0;i<40;i++){
    apps1_arr[i] = analogRead(A0);
    apps2_arr[i] = analogRead(A1);
    paddu_apps1 += (1.0/40.0)*((float)apps1_arr[i]);
    paddu_apps2 += (1.0/40.0)*((float)apps2_arr[i]);
  }
 
  APPS1_newReading = paddu_apps1;
  APPS2_newReading = paddu_apps2;
  APPS1_PedalTravel = 100.0 * ((APPS1_newReading - 940.0)/(940.0 - 685.0)) + 100.0;     //substitute 5 and 1.1 value of APPS1_Voltage voltage range
  APPS2_PedalTravel = 100.0 * ((APPS2_newReading - 880.0)/(880.0 - 492.0)) + 100.0;    //substitute 8.192 and 1.5 value of APPS2_Voltage voltage range
  Delta = abs(APPS1_PedalTravel - APPS2_PedalTravel);
}

int Brake(){
  int brake_arr[50] = {};
  float paddu = 0;
  for(int i = 0;i<50;i++){
    brake_arr[i] = analogRead(A3);
    paddu+= (1.0/50.0)*((float)brake_arr[i]);
  }
 
  BrakeSensor_Reading = paddu;
  return BrakeSensor_Reading;
 
}

int RTDS(){
 
  int AIR_Check = ((PIND & (1 << AIR_AUXpin)) >> AIR_AUXpin);
  BrakeSensor_Reading = Brake();
 
  if( (digitalRead(Start_Buttonpin)==LOW) && (BrakeSensor_Reading>82) ){
    timer_init(t1_comp3);
    while(!RTD){
      digitalWrite(RTDS_CTRLpin,HIGH);
      if(!timer){
        PORTD &= ~(1<<RTDS_CTRLpin);
        RTD = 1;
        break;
      }
    }  
  }
  return 0;
}  

void loop(){
  digitalWrite(Faultpin,LOW);
  digitalWrite(PRE_RELAY,HIGH);
  if(Brake()>80) digitalWrite(BrakeLight_CTRLpin,HIGH);
  else digitalWrite(BrakeLight_CTRLpin,LOW); 
  PORTD &= ~(1 << APPS_CTRLpin);
  RTD = 0;
  Pre_done = 0;
  int count = 0;
  TSMS = analogRead(A4);
  Read_APPS_Data();
 
  int brake = Brake();

  
  while( TSMS > 300){
    int brake = Brake();
    if(!digitalRead(PRE_CTRLpin) && digitalRead(RCTOMB) && !Pre_done){
        timer_init(t1_comp3);
        while(!digitalRead(PRE_CTRLpin) && digitalRead(RCTOMB)){
          if(!timer){
            digitalWrite(PRE_RELAY,LOW);
            digitalWrite(Faultpin,HIGH);
            Pre_done = 1;
            break;
          }
        }
      }
    TSMS = analogRead(A4);
    int AIR_Check = ((PIND & (1 << AIR_AUXpin)) >> AIR_AUXpin);
    if(Brake()>80) digitalWrite(BrakeLight_CTRLpin,HIGH);
    else digitalWrite(BrakeLight_CTRLpin,LOW); 
    if(!RTD) RTDS();
    if(RTD){
      if(Brake()>80) digitalWrite(BrakeLight_CTRLpin,HIGH);
      else digitalWrite(BrakeLight_CTRLpin,LOW); 
      error1 = false;
      error2 = false;
      Impl1();
      Impl2();
      if(error1||error2){
        PORTD |= (1 << APPS_CTRLpin);
        if(Brake()>80) digitalWrite(BrakeLight_CTRLpin,HIGH);
        else digitalWrite(BrakeLight_CTRLpin,LOW); 
        if(Delta>=50){
             
          }
        }
        else  PORTD &= ~(1 << APPS_CTRLpin);
    }
    else{
        PORTD |= (1 << APPS_CTRLpin);
      }  
  }
}
    

void Impl1(){
  Read_APPS_Data();
  if(Delta >= 50){
    timer_init(t1_comp1);
    while(Delta >= 50){
      if(Brake()>80) digitalWrite(BrakeLight_CTRLpin,HIGH);
      else digitalWrite(BrakeLight_CTRLpin,LOW); 
      Read_APPS_Data();
      if(!timer){
        error1 = true;
        break;
      }
    }
  }
  else {
    error1 = false;
  }
  
}

void Impl2(){
  Read_APPS_Data();
  if( Brake() > 90){
    timer_init(t1_comp2);
    while((Brake() > 90) && (APPS1_PedalTravel > 10)){
      if(Brake()>80) digitalWrite(BrakeLight_CTRLpin,HIGH);
      else digitalWrite(BrakeLight_CTRLpin,LOW);
      if(!timer){
        error2 = true;
        break;
      }
    }
    while(APPS1_PedalTravel > 5.0){
      PORTD |= (1 << APPS_CTRLpin);
      Read_APPS_Data();
    }
  }
  else  error2 = false;
} 
void timer_init(uint16_t comp){
  interrupts();
  TCCR1A = 0;
  TCCR0A = 0;
  TIMSK1 |= (1<<OCIE1A);
  TCNT1 = t1_load;
  OCR1A = comp;
  TCCR1B |= (1<<CS12);
  TCCR1B &= ~(1<<CS11);
  TCCR1B |= (1<<CS10);
  timer = 1;
}

void fault_led(int fault){
  for(int i=0;i<fault;i++){
    timer_init(t1_comp1);
    while(timer){
      digitalWrite(0,HIGH);
    }
    digitalWrite(0,LOW);    
  }
  digitalWrite(0,LOW);
}
