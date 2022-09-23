float APPS1_Voltage, APPS2_Voltage, BrakeSensor_Voltage, APPS1_PedalTravel, APPS2_PedalTravel,  Delta, BSPD_OP, z = 0.6,VBua=0,pre_set = 500;
int BMS_CHUTYA = 4,  APPS_SCSpin = 11, Start_Buttonpin = 9, AIR_AUXpin = 6, RTDS_CTRLpin = 3, APPS_CTRLpin = 2, BrakeSensor_Reading, APPS1_newReading, APPS1_currentReading, APPS1_lastReading, APPS2_newReading, APPS2_currentReading, APPS2_lastReading, timer = 1,RTD = 0, Brakelight_CTRLpin = 7; 
const uint16_t t1_load = 0;
uint16_t t1_comp1  = 7813; //500ms
uint16_t t1_comp2 = 1563;  //100ms
uint16_t t1_comp3 = 23437; //1.5s
bool error1 = true;
bool error2 = true;
void timer_init(uint16_t comp);
void Impl1();
void Impl2();
void timer_init(uint16_t comp);

void setup() {
  Serial.begin(115200);             
  pinMode(Start_Buttonpin,INPUT);
  pinMode(APPS_SCSpin,OUTPUT);
  pinMode(BMS_CHUTYA,OUTPUT);
  pinMode(Brakelight_CTRLpin,OUTPUT);
  pinMode(APPS_CTRLpin,OUTPUT);
  pinMode(AIR_AUXpin,INPUT);
  pinMode(RTDS_CTRLpin,OUTPUT);
  DDRD = B10001100;
  PORTD |= (1 <<  APPS_CTRLpin);
  TCCR1A = 0;
  TCCR0A = 0;
  TCCR1B |= (1<<CS12);
  TCCR1B &= ~(1<<CS11);
  TCCR1B |= (1<<CS10);
  noInterrupts();
  RTD = 0;
  digitalWrite(11,HIGH);
}

ISR(TIMER1_COMPA_vect){
  timer = 0;
}

void Read_APPS_Data(){
  int apps1_arr[150] = {};
  int apps2_arr[150] = {};
  float paddu_apps1 = 0;
  float paddu_apps2 = 0;
  for(int i = 0;i<100;i++){
    apps1_arr[i] = analogRead(A0);
    apps2_arr[i] = analogRead(A1);
    paddu_apps1 += (1.0/150.0)*((float)apps1_arr[i]);
    paddu_apps2 += (1.0/150.0)*((float)apps2_arr[i]);
  }
  APPS1_newReading = paddu_apps1;
  APPS2_newReading = paddu_apps2;
  APPS1_PedalTravel = 100.0 * ((APPS1_newReading - 680.0)/(680.0 -0.0)) + 100.0;  //substitute 5 and 1.1 value of APPS1_Voltage voltage range
  APPS2_PedalTravel = 100.0 * ((APPS2_newReading - 557.0)/(557.0 - 0.0)) + 100.0;    //substitute 8.192 and 1.5 value of APPS2_Voltage voltage range
  Delta = abs(APPS1_PedalTravel - APPS2_PedalTravel);
}

int Brakelight_ON(){
  int brake_arr[50] = {};
  float paddu = 0;
  for(int i = 0;i<50;i++){
    brake_arr[i] = analogRead(A2);
    paddu+= (1.0/50.0)*((float)brake_arr[i]);
  }
  BrakeSensor_Reading = paddu;
  return BrakeSensor_Reading;
  
}
int RTDS(){
  int AIR_Check = ((PIND & (1 << AIR_AUXpin)) >> AIR_AUXpin);
  BrakeSensor_Reading = Brakelight_ON();
  if((digitalRead(Start_Buttonpin)==HIGH)&&(BrakeSensor_Reading>148)){
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
void loop() {
  Read_APPS_Data();
  Serial.print(" APPS1 : " );
  Serial.print(APPS1_PedalTravel);
  Serial.print(" % ");
  Serial.print(" APPS2 : " );
  Serial.print(APPS2_PedalTravel);
  Serial.println(" % ");
  Serial.print(" Delta : ");
  Serial.println(Delta);
  if(Delta>10) {
    Serial.println(" Delta > 10 , APPS implausibility");
    digitalWrite(11,LOW);
  }
      int AIR_Check = ((PIND & (1 << AIR_AUXpin)) >> AIR_AUXpin);
      Serial.print(" Brake Reading : " );
      Serial.print(BrakeSensor_Reading);
      if(Brakelight_ON() > 148){
        digitalWrite(Brakelight_CTRLpin,HIGH);
        Serial.println("BRAKE!!");
      }
      else digitalWrite(Brakelight_CTRLpin,LOW);
     
      if(!RTD){
        RTDS();
      }
      if(RTD){
        error1 = false;
        error2 = false;
        Impl1();
        Impl2();
        if(error1){
          PORTD |= (1 << APPS_CTRLpin);
          Serial.println(" Delta >15 i.e. to mc and apps scs should be low" );
        }
        else{
          PORTD &= ~(1 << APPS_CTRLpin);
        }
      }
      else{
        PORTD |= (1 << APPS_CTRLpin);
        RTD = 0;
      }
  }
void Impl1(){
  Read_APPS_Data();
  Serial.print(" APPS1 : " );
  Serial.print(APPS1_newReading);
  Serial.print(" APPS2 : " );
  Serial.println(APPS2_newReading);
  if(Brakelight_ON() > 148){
        digitalWrite(Brakelight_CTRLpin,HIGH);
      }
      else digitalWrite(Brakelight_CTRLpin,LOW);
  if(Delta >= 20){
    timer_init(t1_comp1);
    while(Delta >= 20){
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
  BSPD_OP = analogRead(A3);
  if(Brakelight_ON() > 148){
        digitalWrite(Brakelight_CTRLpin,HIGH);
  }
  else digitalWrite(Brakelight_CTRLpin,LOW);
  if( BSPD_OP > 716 ){
    timer_init(t1_comp2);
    while(BSPD_OP > 716){
      BSPD_OP = analogRead(A2);
      if(!timer){
        error2 = true;
        break; 
      }
    }
    while(APPS1_PedalTravel > 5.0){
      if(Brakelight_ON() > 148){
        digitalWrite(Brakelight_CTRLpin,HIGH);
        Serial.println("BRAKE!!");
      }
      else digitalWrite(Brakelight_CTRLpin,LOW);
      PORTD |= (1 << APPS_CTRLpin);
      digitalWrite(APPS_SCSpin,LOW);
      Read_APPS_Data();
      Serial.println("pedal release kro mee haat jodk....");
      Serial.println(APPS1_PedalTravel);
    }
  }
  else {
    error2 = false;
  }
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
