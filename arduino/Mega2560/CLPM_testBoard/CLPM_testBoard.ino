//--------------------------------------
//程式名稱:test CLP Motor
//版本：V1.0
//配合硬體:
//MEGA 2560
//驗證Y/N:n
//function
//(1)CW (2)CCW (3)RPM
//備註：
//DATE：2017/10/22
//馬達控制功能失效 i2c占用timer 改用TIMER4 與TIMER5
//ENA如果設定為DISABLE 會失去靜態扭矩
//--------------------------------------
#include <Wire.h>
#include <CLP_MOTOR.h>
//*===========================================*
//定義區
//*===========================================*
//====================================
//CLP STEP MOTOR定義區
//====================================
CLPMTR *CLPM_tester = new CLPMTR;
#define testerPUL  22
#define testerDIR  23
boolean CLPM_testerArrive = 0;
int receiveStep = 0;
boolean CLPM_CW_CCW = 0;      //0 CW ,1 CCW
//==============================

//==============================
//control board
//====================================
#define pin_btnCW 50
#define pin_btnCCW 51
#define pin_testLED 52
#define pin_CLPMspeed A0
boolean btnCW_pushed = 0;
boolean btnCCW_pushed = 0;
int CWcounter = 0;
int CCWcounter = 0;
//==============================

//====================================
//timer 定義區
//====================================
int   Timer4Counter = 0;
boolean TimerSW = 0;			//pulse high low change
unsigned int Timer4CountSet[10];
unsigned int Timer5CountSet[7];
byte CLPMTRSpeed;
boolean testbtnCW=0;
//==============================


//====================================
//interrput
//====================================
const byte encoderAPhase = 2 ;    //interrupt pin detect encoder A phase
const byte encoderBPhase = 3 ;    //interrupt pin detect encoder A phase
int stepCounter = 0;
int ABPCounter = 0;
byte  APstatus = 0;
byte  BPstatus = 0;
int ABPhaseCounter=0;
//*===========================================*


//*===========================================*
//初始化區
//*===========================================*
void setup() {
  Serial.begin(9600);
  CLPMTR_initial();
  interrupt_initial();
  //Timer4_initial();
  //Timer5_initial();
  controlBoard_initial();
  CLPMspeed_initial();
  delay(100);
  Serial.println("start");
}
//*===========================================*

//*===========================================*
//主程式區
//*===========================================*
void loop() {
  
  boolean btnCW = digitalRead(pin_btnCW);   //BUTTON DOWN =LOW NORMAL=HIGH
  static int counter = 0;
  CLPM_tester->setCLPMTR_LOW();
  //if(!btnCW){
  //delay(20);
  if (counter < 1000) {
    counter++;
    CLPM_tester->setCLPMTR_HIGH();
    delayMicroseconds(500);
    CLPM_tester->setCLPMTR_LOW();
    delayMicroseconds(500);
    //Serial.print(F("counter:"));
    //Serial.println(counter);
    //while(digitalRead(pin_btnCW)==0){
    //delay(20);
  }

  if (counter == 1000) {
    delay(20);   
      Serial.print(receiveStep);
      Serial.println(F(" STEP"));
    counter = 1800;
  }
  //
  /*
   static int once=0;
      if(once==0){
          once=1;
          JogMode();          
      }
       if (Timer4Counter > 1600) {
              delay(20);
              Serial.print(receiveStep);
              Serial.println(F(" STEP"));
              Serial.print(F("Timer4Counter: "));  
               Serial.println(Timer4Counter);
              Timer4Counter =0;
      }      
     */
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
  CLPM_tester->setCLPMTR_Reverse();
 //CLPM_tester->setCLPMTR_Forward();
 CLPM_CW_CCW = 1;
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
//JOG模式
//====================================
void JogMode() {
  CLPMTRSpeed = readCLPMspeed(); //read Speed
  // Serial.print(F("CLPMTRSpeed:"));
  // Serial.println(CLPMTRSpeed);

  boolean btnCW = digitalRead(pin_btnCW);   //BUTTON DOWN =LOW NORMAL=HIGH
  boolean btnCCW = digitalRead(pin_btnCCW);   //BUTTON DOWN =LOW NORMAL=HIGH
    testbtnCW=0;
    btnCW_pushed=0;
  if (testbtnCW && btnCCW) {
    //MOTOR STOP AND HAVE TORQUE TO HOLD
    //Serial.println(F("MOTOR STOP AND HAVE TORQUE TO HOLD"));
    CLPM_tester->setCLPMTR_LOW();
    btnCW_pushed = 0;
    btnCCW_pushed = 0;   
    //TimerStop();
  } else if (testbtnCW == 0 && btnCW_pushed == 0) {
    //btnCW_pushed==1 express btnCW still push down,so can not enter here!
    //CWcounter++;
    btnCW_pushed = 1;
   // Serial.println(F("btnCW"));
    //Serial.print(F("CWcounter"));
    //Serial.println(CWcounter);
    CLPMTR_JogStepSet(0, 0);
  } else if (btnCCW == 0 && btnCCW_pushed == 0) {
    //btnCCW_pushed==1 express btnCCW still push down,so can not enter here!
    btnCCW_pushed = 1;
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
  //Serial.println(F( "CLPMTR_JogStepSet"));
  CLPM_tester->setCLPMTR_Enable();
  if (CW_CCW == 0) {
    //CW
    CLPM_tester->setCLPMTR_Forward();
    CLPM_CW_CCW=1;
  } else if (CW_CCW == 1) {
    //CCW
    CLPM_tester->setCLPMTR_Reverse();
  }
  Timer4Counter = 0;
  //TCNT4 = Timer4CountSet[CLPMTRSpeed];
  TCNT4 = Timer4CountSet[1];
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
  TCNT4  = Timer4CountSet[1];
   //TCNT4 = Timer4CountSet[CLPMTRSpeed];
  boolean btnCW = digitalRead(pin_btnCW);   //BUTTON DOWN =LOW NORMAL=HIGH
  boolean btnCCW = digitalRead(pin_btnCCW);   //BUTTON DOWN =LOW NORMAL=HIGH
 
  //if (testbtnCW==0 || btnCCW==0) {
  if (!CLPM_testerArrive) {
    if (TimerSW) {
      CLPM_tester->setCLPMTR_HIGH();
      digitalWrite(pin_testLED, LOW);
    }  else {
      CLPM_tester->setCLPMTR_LOW();
      digitalWrite(pin_testLED, HIGH);
      Timer4Counter++;
    }
    TIMSK4 = 0x01;       //timer4 start
  } else {
    CLPM_tester->setCLPMTR_LOW();    
    //Timer4Counter = 0;
    TIMSK4 = 0x00;     //timer4 stop
  }

}
//=============================



//====================================
//timer5 interrput ISR
//====================================
ISR (TIMER5_OVF_vect) {
  TIMSK5 = 0x00;     //timer5 stop
  //TCNT5 = Timer5CountSet[5];
  TCNT5 = Timer5CountSet[2];
  //jog mode
  //pin_btnCW=no push BTN so let motor stop
  boolean btnCW = digitalRead(pin_btnCW);   //BUTTON DOWN =LOW NORMAL=HIGH
  boolean btnCCW = digitalRead(pin_btnCCW);   //BUTTON DOWN =LOW NORMAL=HIGH
  if (testbtnCW && btnCCW) {
    CLPM_testerArrive = true;
    btnCW_pushed = 0;
    btnCCW_pushed = 0;    
  }
  
   if(Timer4Counter>=1600){
        digitalWrite(pin_testLED,LOW);
        //Timer4Counter=0;
        CLPM_testerArrive=true;
        testbtnCW=1;
     }else{
        digitalWrite(pin_testLED,HIGH);
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
 // noInterrupts();
 /*
  APstatusAll[ABPhaseCounter] = digitalRead(encoderAPhase);
  BPstatusAll[ABPhaseCounter] = digitalRead(encoderBPhase);  
  ABPhaseCounter++;
  */
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
 // interrupts();
}
//=============================
//*===========================================*


