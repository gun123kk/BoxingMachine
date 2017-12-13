//--------------------------------------
//程式名稱:test CLP Motor
//配合硬體:
//MEGA 2560
//I2C LCD
//CLP MOTOR AND DRIVER
//function
//(1)CW (2)CCW (3)RPM
//備註：
//馬達控制功能失效 i2c占用timer 改用TIMER4 與TIMER5
//ENA如果設定為DISABLE 會失去靜態扭矩
//CLPMotor driver(Hybird Servo Drive) Pulse need to set 1600
//if there haven't thread,Please follow the steps listed below:
//1.Sketch->Include Library -> Manage Libraries
//2.type thread to search and install it
//divide 1600,the smallest pulse is 8(5steps),so at least run once needs 5 steps
//go_step*pulseChange/10 
//5 * 16 /10=8step
//bug:if this tester speed sets 9(display 9=timer4[0]),arduino will crash
//--------------------------------------
#include <Wire.h>
#include <CLP_MOTOR.h>
#include <LiquidCrystal_I2C.h>
#include <Thread.h>
#include <ThreadController.h>
ThreadController controll = ThreadController();
Thread LCD_Thread = Thread();
 LiquidCrystal_I2C lcd(0x27,16,2);
//*===========================================*
//定義區
//*===========================================*
//====================================
//CLP STEP MOTOR定義區
//====================================
CLPMTR *CLPM_tester = new CLPMTR;
#define testerPUL  22
#define testerDIR  23
//#define pulseChange  16
boolean CLPM_testerArrive = 0;
int receiveStep = 0;
boolean CLPM_CW_CCW = 0;      //0 CW ,1 CCW
#define oneCirclePulse 1600
#define oneCircleSteps 1000
//==============================

//==============================
//control board
//====================================
#define pin_btnCW 48
#define pin_btnCCW 49
#define pin_testLED 52
#define pin_CLPMspeed A0
boolean btnCW_pushed = 0;
boolean btnCCW_pushed = 0;
//==============================

//====================================
//timer 定義區
//====================================
int   Timer4Counter = 0;
boolean TimerSW = 0;			//pulse high low change
unsigned int Timer4CountSet[10];
unsigned int Timer5CountSet[7];
byte CLPMTRSpeed;
float Timer5Counter=0;
//==============================

//====================================
//RPM
//====================================
float getTime=0;
boolean runOneCircle=0;
//==============================
//====================================
//interrput
//====================================
const byte encoderAPhase = 2 ;    //interrupt pin detect encoder A phase
const byte encoderBPhase = 3 ;    //interrupt pin detect encoder B phase
int stepCounter = 0;
int ABPCounter = 0;
byte  APstatus = 0;
byte  BPstatus = 0;
char LCD_row1[16];        //0000 RPM DIR:CW
char LCD_row2[16];        //0000 steps SPD: 9
//============================== 
//*===========================================*
 
//*===========================================*
//初始化區
//*===========================================*
void setup() {
  Serial.begin(9600);
  CLPMTR_initial();
  interrupt_initial();  
  Timer4_initial();
  Timer5_initial();
  controlBoard_initial();
  CLPMspeed_initial();
  LCD_initial();
  thread_initial(); 
  delay(100);  
  //Serial.println("start");
}
//*===========================================*

//*===========================================*
//主程式區
//*===========================================*
void loop() { 
  controll.run();  
  JogMode();         
 }
//*===========================================*

//*===========================================*
//函式區
//*===========================================*

//====================================
//閉迴路步進馬達 initial
//====================================
void CLPMTR_initial() {
 CLPM_tester->CLP_MOTOR_Initial(testerPUL,testerDIR); 
 CLPM_tester->setCLPMTR_LOW();
 CLPM_CW_CCW=CLPM_tester->setCLPMTR_CW();
}
//==============================

//====================================
//controlBoard_initial
//====================================
void controlBoard_initial() {
  pinMode(pin_btnCW, INPUT);
  pinMode(pin_btnCCW, INPUT);
  pinMode(pin_testLED, OUTPUT);
}
//==============================

//====================================
//CLPMspeed_initial
//====================================
void CLPMspeed_initial() {
  CLPMTRSpeed = readCLPMspeed();
}
//=============================

//====================================
//timer4 initial
//====================================
void Timer4_initial() {
  Timer4Counter = 0;
  Timer4CountSet[0] = 65531; //0.02ms 20us中斷設定
  Timer4CountSet[1] = 65524; //0.048ms 48us中斷設定
  Timer4CountSet[2] = 65511; //0.1ms 100us中斷設定
  Timer4CountSet[3] = 65473; //0.25ms 250us中斷設定
  Timer4CountSet[4] = 65411; //0.5ms 500us中斷設定
  Timer4CountSet[5] = 65286; //1ms 1000us中斷設定
  Timer4CountSet[6] = 65036; //2ms 2000us中斷設定
  Timer4CountSet[7] = 63036; //10ms 中斷設定
  Timer4CountSet[8] = 53036; //50ms 中斷設定
  Timer4CountSet[9] = 40536; //100ms 中斷設定
  TimerSW = true;
  TCCR4A =  0x00;
  TCCR4B =  0X03;          //設定 除頻=3 16Mhz/64=0.25Mhz
  //1/0.25Mhz=4us,每4us計數一次,假設設定為65531,
  //共65536-65531=5,第5次發生計時中斷,共經時間5*4us=20us
  TCCR4C =  0x00;
  TIMSK4 =  0x00;      //timer4 stop
}
//==============================

//====================================
//timer5 initial
//====================================
void Timer5_initial() {
  Timer5CountSet[0] = 65531; //20us中斷設定
  Timer5CountSet[1] = 65521;    //0.06ms 60us中斷設定
  Timer5CountSet[2] = 65511; //0.1ms 100us中斷設定
  Timer5CountSet[3] = 65473;  //0.25ms 250us中斷設定
  Timer5CountSet[4] = 65411;  //0.5ms 500us中斷設定
  Timer5CountSet[5] = 65286;   //1ms 1000us中斷設定
  Timer5CountSet[6] = 65036;   //2ms 2000us中斷設定
  TCCR5A =  0x00;
  TCCR5B =  0X03;          //除頻=5 16Mhz/1024 除頻=3 16Mhz/64
  TCCR5C =  0x00;
  TIMSK5 =  0x00;      //timer5 stop
}
//==============================

//====================================
// interrupt_initial
//====================================
void interrupt_initial() {
  pinMode(encoderAPhase, INPUT_PULLUP);
  pinMode(encoderBPhase, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoderAPhase), detectABPhase, CHANGE );
  attachInterrupt(digitalPinToInterrupt(encoderBPhase), detectABPhase, CHANGE );   
}
//==============================

//====================================
// interrupt_initial
//====================================
void LCD_initial(){
  lcd.init();
  lcd.clear();
  lcd.backlight();   
  memset(LCD_row1, '\0', sizeof(LCD_row1));         //show RPM and DIR on LCD row1
  memset(LCD_row2, '\0', sizeof(LCD_row2));         //show steps on LCD row2
  LCD_row1[0]='0';
  LCD_row1[1]='0';
  LCD_row1[2]='0';
  LCD_row1[3]='0';
  LCD_row1[4]=' ';
  LCD_row1[5]='R';
  LCD_row1[6]='P';
  LCD_row1[7]='M';
  LCD_row1[8]=' ';
  LCD_row1[9]='D';
  LCD_row1[10]='I';
  LCD_row1[11]='R';
  LCD_row1[12]=':'; 
   
  LCD_row2[4]=' ';
  LCD_row2[5]='s';
  LCD_row2[6]='t';
  LCD_row2[7]='e';
  LCD_row2[8]='p';
  LCD_row2[9]='s'; 
  LCD_row2[10]=' ';  
  LCD_row2[11]='S';  
  LCD_row2[12]='P';  
  LCD_row2[13]='D';  
  LCD_row2[14]=':';
}
//==============================

//====================================
//thread_initial
//====================================
void   thread_initial(){  
  LCD_Thread.onRun(LCD_Callback);
  LCD_Thread.setInterval(100);
  // Adds myThread to the controll
  controll.add(&LCD_Thread);
 }
//====================================
//JOG模式
//====================================
void JogMode() {
  //CLPMTRSpeed = readCLPMspeed(); //read Speed
  // Serial.print(F("CLPMTRSpeed:"));
  // Serial.println(CLPMTRSpeed);
  boolean btnCW = digitalRead(pin_btnCW);   //BUTTON DOWN =LOW NORMAL=HIGH
  boolean btnCCW = digitalRead(pin_btnCCW);   //BUTTON DOWN =LOW NORMAL=HIGH 
  if (btnCW && btnCCW) {
          //MOTOR STOP AND HAVE TORQUE TO HOLD
          //Serial.println(F("MOTOR STOP AND HAVE TORQUE TO HOLD"));
          CLPM_tester->setCLPMTR_LOW();
          btnCW_pushed = 0;
          btnCCW_pushed = 0;   
          //TimerStop();
  } else if (btnCW == LOW && btnCW_pushed == LOW) {
          btnCW_pushed=1;   //express btnCW still push down,so can not enter here!    
          //Serial.println(F("btnCW"));   
          CLPMTR_JogStepSet(0, 0);       
  } else if (btnCCW == LOW && btnCCW_pushed == LOW) {
          btnCCW_pushed=1;  //express btnCCW still push down,so can not enter here!    
          //Serial.println(F("btnCCW"));
          CLPMTR_JogStepSet(1, 0);
  }
}
//==============================

//====================================
//timer start
//====================================
void TimerStart() {
  TIMSK4 = 0x01;       //timer4 start
  TIMSK5 = 0x01;       //timer5 start
}
//=============================

//====================================
//timer stop
//====================================
void TimerStop() {
  TIMSK4 = 0x00;     //timer4 stop
  TIMSK5 = 0x00;     //timer5 stop
}
//=============================

//====================================
//timer set and CLPM StepSet
//====================================
void CLPMTR_JogStepSet(uint8_t CW_CCW, uint8_t CLPM_arrive) {
 // Serial.println(F( "CLPMTR_JogStepSet"));
  if (CW_CCW==0) {    
     CLPM_CW_CCW=CLPM_tester->setCLPMTR_CW();        
  } else if (CW_CCW==1) {    
    CLPM_CW_CCW=CLPM_tester->setCLPMTR_CCW();
  }
  Timer4Counter = 0;
  receiveStep=0;
  TCNT4 = Timer4CountSet[CLPMTRSpeed];
  TCNT5 = Timer5CountSet[2];
  CLPM_testerArrive = CLPM_arrive;
  TimerStart();
  //Serial.println(F( "CLPMTR_JogStepSet over"));
}
//=============================

//====================================
//timer4 interrput ISR
//create Pulse for CLPMOTOR driver
//====================================
ISR (TIMER4_OVF_vect) {
  TIMSK4 = 0x00;     //timer4 stop
  TimerSW = !TimerSW;
   TCNT4 = Timer4CountSet[CLPMTRSpeed];     //CLPMTRSpeed update in LCD_Callback function
  if (!CLPM_testerArrive) {
    if (TimerSW) {
      CLPM_tester->setCLPMTR_HIGH();      
    }  else {
      CLPM_tester->setCLPMTR_LOW();
      Timer4Counter++;
    }
    TIMSK4 = 0x01;       //timer4 start
  } else {
    CLPM_tester->setCLPMTR_LOW(); 
    TIMSK4 = 0x00;     //timer4 stop
  }
}
//=============================

//====================================
//timer5 interrput ISR
//====================================
ISR (TIMER5_OVF_vect) {
  TIMSK5 = 0x00;     //timer5 stop
  TCNT5 = Timer5CountSet[2];  //2 =100uS
 Timer5Counter++;
  //jog mode
  //pin_btnCW=no push BTN so let motor stop
  if (digitalRead(pin_btnCW)  && digitalRead(pin_btnCCW)) {
        //BUTTON DOWN =LOW NORMAL=HIGH
        CLPM_testerArrive = true;
  }
      
   if(Timer4Counter>=oneCirclePulse){     
        digitalWrite(pin_testLED,LOW);
        Timer4Counter=0;
        //CLPM_testerArrive=true;         //if this line is not disable,motor just can run one circle
     }else{
        digitalWrite(pin_testLED,HIGH);
    }
    
   if(receiveStep >=oneCircleSteps){
          receiveStep=0;  
          getTime=Timer5Counter;
          runOneCircle=1;
         Timer5Counter=0;           
   }

  if (!CLPM_testerArrive) {
    TIMSK5 = 0x01;       //timer5 start
  } else {
    TIMSK5 = 0x00;     //timer5 stop
  }
}
//=============================

//====================================
//readCLPMspeed
//0->1023
//fast->slow
//speed level is 10
//0->103->206->....1023,express speed 0-9
//set timer 9 slow...0 fast
//set display 9 fast ...0 slow
//====================================
byte readCLPMspeed() {
  boolean sw = true;
  int SPDIN = analogRead(pin_CLPMspeed);
  //Serial.print(F(" SPDIN:"));
  //Serial.println(SPDIN);
  byte spd = SPDIN / 103;  
  return spd;
}
//=============================

//====================================
//interrupt detectABPhase
//====================================
void detectABPhase() {
  noInterrupts();
    if(!CLPM_testerArrive){
          //CLPM_testerArrive=false express motor doesn't arrive 
            APstatus = digitalRead(encoderAPhase);
            BPstatus = digitalRead(encoderBPhase);    
             if (APstatus == CLPM_CW_CCW && BPstatus == 0 && ABPCounter == 0) {
              ABPCounter = 1;
            } else if (APstatus == CLPM_CW_CCW && BPstatus == 1 && ABPCounter == 1) {
              ABPCounter = 2;
            } else if (APstatus == !CLPM_CW_CCW && BPstatus == 1 && ABPCounter == 2) {
              ABPCounter = 3;
            } else if (APstatus == !CLPM_CW_CCW && BPstatus == 0 && ABPCounter == 3) {
              ABPCounter = 4;
              receiveStep++;
              ABPCounter = 0;
            }
      }
  interrupts();
}
//=============================

//====================================
//show on LCD
//====================================
void showOnLcd(int row,char *information) {     
      char buf[16];
      int col=0;      
      for (col=0;col<16;col++){
          buf[col]=' ';
          lcd.setCursor(col,row-1);
          if(information[col] != '\0'){            
            lcd.print(information[col]);
          }else{
            lcd.print(buf[col]);           
          }               
      }         
}
//==============================

//====================================
//stepChange int to char for show on LCD
//====================================
void stepChange(int receiveStep){
  int buf=receiveStep;
  int i=0;
  int cul[4]={1000,100,10,1};
  int buf1=0;
  boolean judge=0;
  for(i=0;i<4;i++){    
    if(buf >=cul[i]){
      buf1=buf / cul[i];
      LCD_row2[i]=buf1+'0';
      judge=1;
    }else{
      if( judge==0){
        LCD_row2[i]='\0';
      }else{
        LCD_row2[i]='0';
      }
    }    
    buf=buf % cul[i];   
  }  
  if(CLPM_CW_CCW){  
    LCD_row1[13]='C';
    LCD_row1[14]='C';
    LCD_row1[15]='W';
  }else{
    LCD_row1[13]=' ';
    LCD_row1[14]='C';
    LCD_row1[15]='W';
  }
}
//==============================

//====================================
//callback for LCD_Thread
//show CLPMotor information on LCD
//====================================
void LCD_Callback(){           
        int stp=receiveStep;       
        
       static float  cc=0;      
        if( runOneCircle){
          cc=getTime*100/1000000; //*100uS       
              getTime=0;
              runOneCircle=false;
        }                       
         rpmChange(cc);  
        stepChange(stp);       
        showOnLcd(1,LCD_row1); 
        CLPMTRSpeed=readCLPMspeed();
        int displaySPD= abs(CLPMTRSpeed-9);  
        LCD_row2[15]=displaySPD+'0';   
        showOnLcd(2,LCD_row2);    
}
//==============================

//====================================
//Calculate RPM and  int to char for show on LCD
//====================================
void rpmChange(float cc){
  int buf=1/cc*60;
  //Serial.print("rpm: ");
   //Serial.println(buf);
  int i=0;
  int cul[4]={1000,100,10,1};
  int buf1=0;
  boolean judge=0;
  for(i=0;i<4;i++){    
    if(buf >=cul[i]){
      buf1=buf / cul[i];
      LCD_row1[i]=buf1+'0';
      judge=1;
    }else{
      if( judge==0){
        LCD_row1[i]='\0';
      }else{
        LCD_row1[i]='0';
      }
    }    
    buf=buf % cul[i];   
  }  
}
//==============================
//*===========================================*




