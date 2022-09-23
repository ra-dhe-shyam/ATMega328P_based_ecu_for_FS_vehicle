
#include <mcp_can.h>
#include <SPI.h>
 

long unsigned int rxId;
unsigned char len = 0;
unsigned char rxBuf[8];
char msgString[128];                      
unsigned char therm[8];


byte data[3] = {0x3D, 0xCE, 0x00}; 


MCP_CAN CAN0(10);

 
float APPS1_Voltage;
float APPS2_Voltage;
float BrakeSensor_Voltage;
float APPS1_PedalTravel; 
float APPS2_PedalTravel;  
float Delta;
float current;
float VBua=0.0;
float pre_set = 500.0;



int APPS_CTRLpin = 2;
int RTDS_CTRLpin = 3;
int PRE_RELAY = 4;
int APPS_SCSpin = 5;
int AIR_AUXpin = 6;
int PRE_CTRLpin = 7;
int BMS_CHUTYA = 8;
int Start_Buttonpin = 9;



int BrakeSensor_Reading;
int APPS1_newReading;
int APPS2_newReading;
int timer = 1;
int RTD = 0;
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
void Bablya_VBua(float* VBua);
void Pre_set(float* pre_set);
void Therm();
int RTDS();
int Brakelight_ON();
 

void setup() {
  Serial.begin(115200);

  if(CAN0.begin(MCP_ANY, CAN_250KBPS, MCP_16MHZ) == CAN_OK)
   Serial.println("MCP2515 Initialized Successfully!");
  else
   Serial.println("Error Initializing MCP2515...");
  CAN0.setMode(MCP_NORMAL);

  pinMode(RTDS_CTRLpin,OUTPUT);
  pinMode(APPS_CTRLpin,OUTPUT);            
  pinMode(Start_Buttonpin,INPUT);
  pinMode(APPS_SCSpin,OUTPUT);
  pinMode(BMS_CHUTYA,OUTPUT);
  pinMode(PRE_CTRLpin,OUTPUT);
  pinMode(PRE_RELAY,OUTPUT);
  digitalWrite(APPS_SCSpin,HIGH);
  digitalWrite(PRE_CTRLpin,HIGH);
  digitalWrite(PRE_RELAY,HIGH);
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
  int apps1_arr[100] = {};
  int apps2_arr[100] = {};
  float paddu_apps1 = 0;
  float paddu_apps2 = 0;
  for(int i = 0;i<100;i++){
    apps1_arr[i] = analogRead(A0);
    apps2_arr[i] = analogRead(A1);
    paddu_apps1 += (1.0/100.0)*((float)apps1_arr[i]);
    paddu_apps2 += (1.0/100.0)*((float)apps2_arr[i]);
  }
  APPS1_newReading = paddu_apps1;
  APPS2_newReading = paddu_apps2;
  APPS1_PedalTravel = 100.0 * ((APPS1_newReading - 675.0)/(675.0 -300.0)) + 100.0;     //substitute 5 and 1.1 value of APPS1_Voltage voltage range
  APPS2_PedalTravel = 100.0 * ((APPS2_newReading - 890.0)/(890.0 - 425.0)) + 100.0;    //substitute 8.192 and 1.5 value of APPS2_Voltage voltage range
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



void Bablya_VBua(float* VBua){
  byte gya_kya =  CAN0.sendMsgBuf(0x201,0,3,data);
  if(gya_kya == CAN_OK) Serial.println("Bablya cha CAN chalu zala !");
  else  Serial.println("Gu aahe kahitari CAN mdhe...");
  CAN0.readMsgBuf(&rxId,&len,rxBuf);
  Serial.println(rxId);
  if((rxId == 0x385) && (rxBuf[1] == 235)){
    float b = (float)((rxBuf[3] << 8) | rxBuf[2]);
    Serial.print("B : ");
    Serial.println(b);
    *VBua = (b / 31.585);  
  }
}


void Pre_set(float* pre_set){
  CAN0.readMsgBuf(&rxId, &len, rxBuf);
  if((rxId & 0x1fffffff) == 0x6B0){   
    float pov = (float)((rxBuf[1] << 8) | rxBuf[2])/10.00;
    *pre_set = pov*0.955;
  }  
}


void Therm(){
  CAN0.readMsgBuf(&rxId, &len, rxBuf);     
  if((rxId & 0x1fffffff) == 0x380){ 
    for(byte i = 0; i<len; i++) therm[i] = rxBuf[i];   
    if((therm[2] == 235 )){
      analogWrite(BMS_CHUTYA,670);
      Serial.println("Thermistor Open Wire hai, BMS ko chu bnao");  
    }
  }
}


void loop(){
  TSMS = analogRead(A4);
  Serial.print(" TSMS status : ");
  Serial.println(TSMS);
  Bablya_VBua(&VBua);
  Pre_set(&pre_set);
  int brake = Brake();
  while(TSMS > 450){
    digitalWrite(PRE_RELAY,LOW);
    digitalWrite(PRE_CTRLpin,LOW);
    TSMS = analogRead(A4);
    Bablya_VBua(&VBua);
    Pre_set(&pre_set);
    if(VBua>pre_set){
      Serial.println("gu khalo");
      digitalWrite(PRE_CTRLpin,LOW);
    }
    else{
      Serial.println("mt khao fir");
      Serial.print("VBua : ");
      Serial.println(VBua);
      Serial.print("Preset : ");
      Serial.println(pre_set);
    }
    Therm();
    int AIR_Check = ((PIND & (1 << AIR_AUXpin)) >> AIR_AUXpin);
    if(!RTD && AIR_Check) RTDS();
    if((AIR_Check)&&RTD){
      error1 = false;
      error2 = false;
      Impl1();
      Impl2();
      if(error1||error2){
        PORTD |= (1 << APPS_CTRLpin);
        digitalWrite(APPS_SCSpin,LOW);
        if(Delta>10){
          while(Delta>10){
            if(APPS1_newReading == 0.00) Serial.println("APPS g*nd ko lga !!!!");
            else if(APPS1_newReading >= 982) Serial.println("APPS VCC ko lga !!!!");
            else  Serial.println("APPS Ganja !!!!!");
          }
        }
        else  PORTD &= ~(1 << APPS_CTRLpin);
      }
      else{
        PORTD |= (1 << APPS_CTRLpin);
        RTD = 0;
      }
    }
  }
  Therm();
  digitalWrite(PRE_CTRLpin,HIGH);
}


void Impl1(){
  Read_APPS_Data();
  if(Delta >= 10){
    while(Delta >= 10){
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
  if( Brake() > 160){
    timer_init(t1_comp2);
    while((Brake() > 160)){
      if(!timer){
        error2 = true;
        break; 
      }
    }
    while(APPS1_PedalTravel > 5.0){
      PORTD |= (1 << APPS_CTRLpin);
      Read_APPS_Data();
      Serial.println("pedal release kro mee haat jodk....");
      Serial.println(APPS1_PedalTravel);
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
