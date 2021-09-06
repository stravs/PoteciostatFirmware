#include <Arduino.h>
#include <Wire.h>


#define SwitchDACEl 2                            
#define SwitchDACIV 3                              
#define ADCIn A3

// #define DAC_RefCH 0
// #define DAC_WCH 1

#define DAC_RefCH 1
#define DAC_WCH 0

//---------------------------------------------------------------
// --- ALL VARIABLES AND SETTINGS
//---------------------------------------------------------------
String commandString;
//--- System setting ---
volatile bool isInterrupt = false;
volatile unsigned int refPot=6581;             //Reference electrode potential DAC value naš
volatile unsigned int wPot=21081;               //Working electrode potential DAC value
char inpMode=0;                             //Input mode 0-potentiometric, 1-amperometric
char nrEl=3;                                //Number of electrodes. 2 or 3
volatile uint_fast16_t timerBaseStep = 25;  //This is timer base step in ms (for interupts). Tested frequencies value --> Measured: 15 --> 67.5Hz, 10 --> 100Hz, 25 -->40.42Hz
volatile uint_fast16_t timerBaseStep1 = 25; //This is timer base step 1 (odd): this value is used for all measurements but DPV, in which case both timerBaseStep1 and 2 are used.
volatile uint_fast16_t timerBaseStep2 = 0;  //This is timer base step 2 (even): which is used only for DPV. In all other cases it should be 0 and only value "1" is used. Default
int refIncOdd=29;    //Reference channel increment at odd steps
int refIncEven=0;   //Reference channel increase at even steps
int wIncOdd=0;      //Working channel increase at odd steps
int wIncEven=0;     //Working channel increase at even steps
int numberOfSteps=2000;   //Number of steps to make

//--- System variables ---
volatile bool sRunning=false;   //Currently in running state
volatile long t1000steps=0;     //time in ms what it took to make 1000steps
volatile int  N=0;              //Current step number
volatile unsigned int last_refPot=0;
volatile unsigned int last_wPot=0;
volatile unsigned int last_Inp=0;
volatile unsigned int last_N=0;
//These are actually the output values
volatile unsigned int outRefPot_odd=0;
volatile unsigned int outRefPot_even=0;
volatile unsigned int outInp_odd=0;
volatile unsigned int outInp_even=0;
volatile unsigned int outN=0;
volatile unsigned long outT=0;
volatile bool outSend=false;

volatile bool last_updated=false;     //Every time reading is updated this is pulled to true, when BLE sending is done it's pulled back to false
volatile bool cycleX=false;           //Every time interrupt is called this will switch polarity, these are two different parts of the process
volatile bool cycleDone=false;        //Every time scan is completed this is pulled true
volatile bool readADCNow=false;       //Read ADC now
volatile long errorcount=0;
volatile int t1000counter=0;
volatile long t0;
volatile long ADCsum=0; //Sum 10x ADC


char datax[21];    //Data buffer
char outIndex='0';

void sendToDAC(byte b1, byte b2, byte b3)
{
  byte error;
  Wire.beginTransmission(0x0C); //DAC address pin is connected to ground
  Wire.write(b1); Wire.write(b2); Wire.write(b3);
  error = Wire.endTransmission(); 
}

void setDAC(byte channel, unsigned int value)
{
  byte lowB=value&0xFF;
  byte highB=(value>>8)&0xFF;
  byte chB=0x18 + channel;
  sendToDAC(chB,highB,lowB); 
}


void setupDAC(bool internalref)
{
  sendToDAC(0x20,0x00,0x03); //First setup command sets both DAC outputs into normal operation mode
  if(internalref) { sendToDAC(0x38,0x00,0x01); } //Use internal reference
  else { sendToDAC(0x38,0x00,0x00); } //Use external reference
  sendToDAC(0x30,0x00,0x03); //Dont use LDAC, update output by software
}


void initHardware()
{
    //Configure I2C bus
    Wire.begin();
    //Initialize I2C DAC
    setupDAC(true);
    //Define switch pins
    pinMode(SwitchDACIV, OUTPUT);
    pinMode(SwitchDACEl, OUTPUT);
    digitalWrite(SwitchDACIV, HIGH); //Potential
    digitalWrite(SwitchDACEl, LOW);  //3-electrode
    //Set DACs
    setDAC(0, 0);
    setDAC(1, 0);
}


void timerInit()                       
{
  Serial.println("start timerInit");
  // NRF_TIMER4->TASKS_STOP = 1;  // Stop timer
  // NRF_TIMER4->MODE = TIMER_MODE_MODE_Timer;  // taken from Nordic dev zone
  NRF_TIMER4->BITMODE = TIMER_BITMODE_BITMODE_16Bit << TIMER_BITMODE_BITMODE_Pos;
  NRF_TIMER4->PRESCALER = 9 << TIMER_PRESCALER_PRESCALER_Pos;  // 32us resolution
  NRF_TIMER4->TASKS_CLEAR = 1; // Clear timer
  // With 32 us ticks, we need to multiply by 31.25 to get milliseconds
  NRF_TIMER4->CC[0] = timerBaseStep * 31;
  NRF_TIMER4->CC[0] += timerBaseStep / 4;
  // NRF_TIMER4->CC[0] = 100;
  NRF_TIMER4->INTENSET = TIMER_INTENSET_COMPARE0_Enabled << TIMER_INTENSET_COMPARE0_Pos;  // taken from Nordic dev zone
  NRF_TIMER4->SHORTS = (TIMER_SHORTS_COMPARE0_CLEAR_Enabled << TIMER_SHORTS_COMPARE0_CLEAR_Pos);
  
  NVIC_EnableIRQ(TIMER4_IRQn);
  NRF_TIMER4->TASKS_START = 1;  // Start TIMER                                                                                
}

void setTimer()
{
  NRF_TIMER4->CC[0] = timerBaseStep * 31;
  NRF_TIMER4->CC[0] += timerBaseStep / 4;
}

extern "C"
{
   void TIMER4_IRQHandler_v()
  {
    if (NRF_TIMER4->EVENTS_COMPARE[0] != 0)
    {
      isInterrupt = !isInterrupt;
      NRF_TIMER4->EVENTS_COMPARE[0] = 0;
    }
  }
}


void sendData()
{
    Serial.print(outIndex);
    outIndex++; 
    if(outIndex>'9'){ outIndex='0'; }
    Serial.print('M');
    Serial.print(' ');
    // Serial.print(outRefPot_odd);
    // Serial.print(' ');
    Serial.print(outInp_odd);
    Serial.print(' ');
    // Serial.print(outRefPot_even);
    // Serial.print(' ');
    Serial.print(outInp_even);
    Serial.print(' ');
    // Serial.print(outT); 
    // Serial.print(' ');
    // Serial.print(last_wPot);
    // Serial.print(' ');
    // Serial.print(outN); 
    Serial.println("");
}

bool action(char ID, long param)
{
  switch(ID)
  {
    case 'A': //Set reference electrode potential value
              refPot=(unsigned int)param;
              setDAC(DAC_RefCH, refPot);
              Serial.println("a:DONE");
              return true;
    case 'B': //Set working electrode potential value
              wPot=(unsigned int)param;
              setDAC(DAC_WCH, wPot);
              Serial.println("b:DONE");
              return true; 
    case 'C': //Set input mode (0-potentsiometric (default), 1-amperometric)
              inpMode=(char)param;
              digitalWrite(SwitchDACIV,inpMode==0);
              Serial.println("c:DONE");
              return true;                   
    case 'D': //Set number of electrodes used 2 or 3
              nrEl=(char)param;
              digitalWrite(SwitchDACEl,nrEl==2);
              Serial.println("d:DONE");
              return true;               
    case 'E': //Set base timer 15 ... 1000 ms (timerBaseStep & timerBaseStep1)
              timerBaseStep=(uint_fast16_t)param;
              if(timerBaseStep<15){ timerBaseStep=15; }
              if(timerBaseStep>1000){ timerBaseStep=1000; }
              timerBaseStep1=timerBaseStep;
              timerInit();
              Serial.println("e:DONE");
              return true;     
    case 'F': //Get time for 100 cycles
              Serial.println("f:"+String(t1000steps,DEC));
              return true;     
    case 'G': //Set scan rate for electrode reference channel to increase at odd steps
              if(param>32767){ refIncOdd=32767; }else
              if(param<-32768){ refIncOdd=-32768; }else
              { refIncOdd=(int)param; }
              Serial.println("g:DONE");
              return true;      
    case 'I': //Set scan rate for reference electrode channel to increase at even steps
              if(param>32767){ refIncEven=32767; }else
              if(param<-32768){ refIncEven=-32768; }else
              { refIncEven=(int)param; }
              Serial.println("i:DONE");
              return true;    
    
    case 'J': //Set scan rate for working electrode channel to increase at odd steps
              if(param>32767){ wIncOdd=32767; }else
              if(param<-32768){ wIncOdd=-32768; }else
              { wIncOdd=(int)param; }
              Serial.println("j:DONE");
              return true;      
    case 'K': //Set scan rate for reference channel to increase at even steps
              if(param>32767){ wIncEven=32767; }else
              if(param<-32768){ wIncEven=-32768; }else
              { wIncEven=(int)param; }
              Serial.println("k:DONE");
              return true;    
    case 'L': //Set number of step to make during the scan
              if(param>1000){ numberOfSteps=1000; }else
              if(param<1){ numberOfSteps=1; }else
              { numberOfSteps=(int)param; }
              Serial.println("l:DONE");
              return true;    
    case 'M': //Run the sequence
              sRunning=true;
              N=0;
              return true;             
    case 'N': //Halt the sequence
              sRunning=false;
              N=0;
              //If DPV mode return back to default timestep 1 
              if(timerBaseStep2>0)
              {
                timerBaseStep=timerBaseStep1;
                timerInit();
              }                
              Serial.println("n:DONE");
              return true;  
    case 'O': //Set base timer 2 for DPV. 0 (equal step mode) or 15 ... 1000 ms (DPV mode) (timerBaseStep2)
              timerBaseStep2=(uint_fast16_t)param;
              if(timerBaseStep2>0)
              {
                if(timerBaseStep2<15){ timerBaseStep2=15; }
                if(timerBaseStep2>1000){ timerBaseStep2=1000; }
              }
              Serial.println("o:DONE");
              return true;                       
    default:
      return false;     
  }
}


bool cmdLine(String data)
{
    char ID;    //Command ID
    long param=0; //Parameter of the function. Default is zero
    char datac[25];
    int begi=-1, endi=-1; //begin and end index: BEGIN "(" and END ")"
    for(char i=0; i<data.length(); i++)
    { 
        if(data[i]=='(') begi=i;
        if(data[i]==')') endi=i;     
    }
    if((begi>0)&&(endi>begi)) //Looks like valid command
    {
      ID=data[begi-1]; //ID character is just one before parenthesis
      if((endi-begi)>1) //There is also a parameter. Some commands dont have parameter
      {
        strncpy(&datac[0],&data[begi+1],endi-begi-1);
        datac[endi-begi]=0x00; //Null terminated
        param=atol(datac); //Convert parameter to number
      }
    }else
    {
      return false; //Something went wrong, not correct command syntax
    }
    //Now interprete the command and take action
    return action(ID, param);

}


void setup() 
{  
  Serial.begin(115200);

  //Other hardware
  initHardware();
  delay(2000);
  timerInit();  
  analogReadResolution(10);  
  pinMode(ADCIn,INPUT); //Input signaL
  Serial.println("setup:DONE");

}

void loop()
{
    if(Serial.available()){
        commandString = Serial.readStringUntil('\n');
        cmdLine(commandString);
    }

    if(isInterrupt){
      long tmp;

      cycleX=!cycleX; //multiplexes every time

      //Measures timing
      if(t1000counter==0) t0=millis();
      if(t1000counter==1000) { t1000steps=millis()-t0; t1000counter=0; }else{ t1000counter++; }
      
      if(cycleX) //Main step
      { 
          //Sum more ADC
          for(char i=0; i<128; i++) ADCsum=ADCsum+analogRead(ADCIn); 
          last_refPot=refPot;
          last_wPot=wPot;
          last_Inp=ADCsum/4; //ADCvalue (256 x 10-bit input)/4 --> 16-bit end value;
          last_N=N;
          //Make steps if measurement is running
          if(sRunning)
          {
              N=N+1; //Increase step number
              if(N>numberOfSteps)
              { 
                sRunning=false; cycleDone=true;                               
              } //Check if desigerd number of steps are done
              if(N == numberOfSteps/2 + 1)
              {
                refIncEven = refIncEven * (-1);
                refIncOdd = refIncOdd * (-1);
              }
          }
          //If running the update
          if(sRunning)
          {
              if(N%2==0) // If even step
              { 
                  //Set reference potential
                  tmp=refPot; tmp=tmp+refIncEven; //Make step
                  if(tmp>65535) { tmp=65535; }else if(tmp<0){ tmp=0; }else; // Check that end value is in the range
                  refPot=(unsigned int)tmp; setDAC(DAC_RefCH, refPot); //Set value
                  //Set working potential (same as reference)
                  tmp=wPot; tmp=tmp+wIncEven; 
                  if(tmp>65535) { tmp=65535; }else if(tmp<0){ tmp=0; }else;
                  wPot=(unsigned int)tmp; setDAC(DAC_WCH, wPot);                
                  //In small cycle even steps come last - push into even step register  
                  outRefPot_even=last_refPot;
                  outInp_even=last_Inp;
                  outN=last_N;
                  outT=millis();
                  outSend=true; //Small cycle done. Now this triggers sending
                  //If DPV mode adjust also timestep, otherwise do nothing
                  if(timerBaseStep2>0)
                  {
                    timerBaseStep=timerBaseStep1; //1 is used for odd steps
                    setTimer();
                  }
              }else // If odd step
              {
                  //Set reference potential
                  tmp=refPot; tmp=tmp+refIncOdd; 
                  if(tmp>65535) { tmp=65535; }else if(tmp<0){ tmp=0; }else;
                  refPot=(unsigned int)tmp; setDAC(DAC_RefCH, refPot);
                  //Set working potential
                  tmp=wPot; tmp=tmp+wIncOdd; 
                  if(tmp>65535) { tmp=65535; }else if(tmp<0){ tmp=0; }else;
                  wPot=(unsigned int)tmp; setDAC(DAC_WCH, wPot);
                  //In small cycle odd steps come first - push into odd step register  
                  outRefPot_odd=last_refPot;
                  outInp_odd=last_Inp;   
                  //If DPV mode adjust also timestep, otherwise do nothing     
                  if(timerBaseStep2>0)
                  {
                    timerBaseStep=timerBaseStep2; //2 is used for even steps
                    setTimer();
                  }                      
              }
          }
        ADCsum=0;
      }else{
        for(char i=0; i<128; i++) ADCsum=ADCsum+analogRead(ADCIn); 
    }
    }


    if(cycleDone){ //Send message if one scan cycle is completed
        //If DPV mode return back to default timestep 1 
        if(timerBaseStep2>0)
        {
          timerBaseStep=timerBaseStep1;
          setTimer();
        }   
      Serial.println("m:DONE"); 
      cycleDone=false; 
   } 
   if(sRunning&&outSend) //New data gathered send it away
   {
      // sendData();       
      outSend=false;  
   }
} 





