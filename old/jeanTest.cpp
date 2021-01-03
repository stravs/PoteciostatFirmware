#include <Arduino.h>
#include <Wire.h>
#define SwitchEl 2                              //Switch 2: reference electrode (0-Referenc ON, Referecne-OFF).
#define SwitchIV 3                              //Switch 1: voltage vs current measurement (0-Current, 1-Voltage).
#define ampToArd A3

int adressDAC = 0x0F;                           //DAC ADRESS
unsigned int    valueA;                         //DAC A OUTPUT
unsigned int    valueB;                         //DAC B OUTPUT
unsigned int    valADC;                         //ADC reading
unsigned long   sumADC;                         //SUM of 16 ADC readings
unsigned int    avgADC_A;                       //average of 16 ADC readings    
unsigned int    avgADC_B;                       //average of 16 ADC readings    

void sendToDAC(byte b1, byte b2, byte b3)       //Send command to DAC function
{
  Wire.beginTransmission(adressDAC);            
  Wire.write(b1); Wire.write(b2); Wire.write(b3);
  Wire.endTransmission(); 
}

void setup() 
{    
    Wire.begin();
    delay(50);    
    Serial.begin(115200);
    delay(50);    
    analogReadResolution(10);   // Arduino ADC from 10bit to 12bit    
    pinMode(SwitchEl,OUTPUT);
    pinMode(SwitchIV,OUTPUT);
    pinMode(ampToArd,INPUT);    
    digitalWrite(SwitchEl,LOW);
    digitalWrite(SwitchIV,LOW);    //DAC SETUP
    sendToDAC(0x20,0x00,0x03);  //First setup command sets both DAC outputs into normal operation mode  // DATASHEET POWER-DOWN MODES
    sendToDAC(0x38,0x00,0x01);  //Use internal reference                                                // DATASHEET INTERNAL REFERENCE SETUP
    sendToDAC(0x30,0x00,0x03);  //Dont use LDAC, update output by software                              // DATASHEET LDAC FUNCTION/Asynchronous LDAC/last paragraph    
    valueB = 22000;              //1.5V output iz DAC B
    byte lowB=valueB&0xFF;
    byte highB=(valueB>>8)&0xFF;
    sendToDAC(0x19,highB,lowB); 
    
}
void loop() {
//SLOW RAMP FUNCTION
/*
for(int i = 0; i <= 4096; i++ )
    {    
    value = i;
    byte lowA=value&0xFF;
    byte highA=(value>>4)&0xFF;
    sendToDAC(0x18,highA,lowA); 
    }

    *///CONSTANT to chanel A       
    valueA = 22000;                                 //3.0V to 0V output na DAC A ---- Rise TIME 480uS ---- Fall TIME 550uS
    byte lowA=valueA&0xFF;
    byte highA=(valueA>>8)&0xFF;
    sendToDAC(0x18,highA,lowA);     
    delay(5);                                      //delay for Rise and Fall Time on analog circuit    
    sumADC=0;                                       //reset sumADC value
    for(int i = 0; i < 128; i++)                    //avering function for noise reduction (no noise error = 40) (averiging for nois reduction error = 4)
        {
            valADC=analogRead(ampToArd);            //read and record transimpedance amplifier output
            sumADC=sumADC+valADC;                   //calculate sum of 128 readings
        }
    avgADC_A=sumADC/128;                            //calculate average    
    valueA = 0;                    
    lowA=valueA&0xFF;
    highA=(valueA>>4)&0xFF;
    sendToDAC(0x18,highA,lowA);     
    delay(5);    
    sumADC=0;
    for(int i = 0; i < 128; i++)
        {
            valADC=analogRead(ampToArd);
            sumADC=sumADC+valADC;
        }
    avgADC_B=sumADC/128;
    Serial.print(avgADC_A); Serial.print(" --- ");  Serial.println(avgADC_B);       //print to serial monitor
    delay(1000);                                                                     //delay for easier reading of serial monitor
}