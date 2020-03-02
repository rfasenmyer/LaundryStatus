#include "math.h"

const int currentPin0 = A0;
const int currentPin1 = A1;

const unsigned long sampleTime = 100000UL;                      // sample over 100ms, it is an exact number of cycles for both 50Hz and 60Hz mains
const unsigned long numSamples = 250UL;                         // choose the number of samples to divide sampleTime exactly, but low enough for the ADC to keep up
const unsigned long sampleInterval = sampleTime/numSamples;     // the sampling interval, must be longer than then ADC conversion time
//const int adc_zero = 522;                                     // relative digital zero of the arudino input from ACS712 (could make this a variable and auto-adjust it)

const int washerVoltageThreshold = 5; //number of volts that must be detected before it is considered active
const int dryerVoltageThreshold = 5;  //number of volts that must be detected before it is considered active

const int washerAlertThreshold = 300; //number of seconds to wait for no current detection before sending alert
const int dryerAlertThreshold = 5;    //number of seconds to wait for no current detection before sending alert

const int washerStartWaitTime = 5;  //number of seconds to wait before considering the device to be on.  Sometimes temporary voltage is detected when the device is not on.
const int dryerStartWaitTime = 5;   //number of seconds to wait before considering the device to be on.  Sometimes temporary voltage is detected when the device is not on.

int washerAlertThresholdCounter = washerAlertThreshold;  //set Counter to threshold
int dryerAlertThresholdCounter = dryerAlertThreshold;  //set counter to threshold
int washerStartWaitTimeCounter = washerStartWaitTime;
int dryerStartWaitTimeCounter = dryerStartWaitTime;

int adc_zero0;                                                   //autoadjusted relative digital zero
int adc_zero1;


bool washerAlertPrimed = false;
bool dryerAlertPrimed = false;


float currentWasherReading;
float currentDryerReading;
void setup()
{
  Serial.begin(9600);
   adc_zero0 = determineVQ(currentPin0); //Quiscent output voltage - the average voltage ACS712 shows with no load on plug 1
   adc_zero1 = determineVQ(currentPin1); //Quiscent output voltage - the average voltage ACS712 shows with no load on plug 2
  delay(1000);
  
   Spark.publish("DryerEvent", "Power On - Washer and dryer notification system is online!", 60, PRIVATE);
}


void loop(){
  //Plug1 Washer
  currentWasherReading = readCurrent(currentPin0,adc_zero0);
  
  if (currentWasherReading > washerVoltageThreshold){
    //Current is Detected  check to see if it needs to be primed

    if (washerAlertPrimed == false){
        washerStartWaitTimeCounter = washerStartWaitTimeCounter - 1;
        if (washerStartWaitTimeCounter == 0){
            //Record that washer is running
            washerAlertPrimed = true;
            //Set timeout threshold
            washerAlertThresholdCounter = washerAlertThreshold;  
        }
    }

    
  } else {
     //Call washerAlertCheck 
    washerAlertCheck();  
    //reset wsherStartWaitTimeCounter
    washerStartWaitTimeCounter = washerStartWaitTime;
  }
  
  
  
  
  //Plug2 Dryer
  currentDryerReading  = readCurrent(currentPin1,adc_zero1);
  
  
  if (currentDryerReading > dryerVoltageThreshold){
    //Record that Washer is running
    dryerAlertPrimed = true;
    //Set timeout threshold
    dryerAlertThresholdCounter = 5;
    
    
  } else {
     //Call dryerAlertCheck 
      dryerAlertCheck();  
  }  

//troublshoot


  Serial.print("Plug1: ");     Serial.print(readCurrent(currentPin0,adc_zero0),0);  Serial.print(" Threshold1 "); Serial.print(washerAlertThresholdCounter);    Serial.print(" ")
  //Serial.print(" Plug2: ");    Serial.print(readCurrent(currentPin1,adc_zero1),0);  Serial.print(" Threshold2 "); Serial.println(dryerAlertThresholdCounter); 

  
  
  //wait one second before checking current again
  delay(1000);
} //end of main loop



void washerAlertCheck(){
    if (washerAlertPrimed == true){
        washerAlertThresholdCounter = washerAlertThresholdCounter - 1;
    }
    
    if (washerAlertThreshold == 0){
        //Send pushmon notification
        Serial.println("Sending Washer Notification");
        //Spark.publish("WasherEvent", "Washer Is Done!", 60, PRIVATE);
        
        washerAlertPrimed = false;
        washerAlertThresholdCounter = washerAlertThreshold;
        
        
    }
    
}


void dryerAlertCheck(){
    if (dryerAlertPrimed == true){
        dryerAlertThresholdCounter = dryerAlertThresholdCounter - 1;
    }
    
    if (dryerAlertThreshold == 0){
        //Send pushmon notification
        Serial.println("Sending Dryer Notification");
        //Spark.publish("DryerEvent", "Dryer Is Done!", 60, PRIVATE);
        
        dryerAlertPrimed = false;
        dryerAlertThresholdCounter = dryerAlertThreshold;
    }
    
}


#This function was obtained from elik745i on the Arduino forum: https://forum.arduino.cc/index.php?topic=179541.msg1423700#msg1423700
int determineVQ(int PIN) {
  Serial.print("estimating avg. quiscent voltage:");
  long VQ = 0;
  //read 5000 samples to stabilise value
  for (int i=0; i<5000; i++) {
    VQ += analogRead(PIN);
    delay(1);//depends on sampling (on filter capacitor), can be 1/80000 (80kHz) max.
  }
  VQ /= 5000;
  Serial.print(map(VQ, 0, 1023, 0, 5000));Serial.println(" mV");
  return int(VQ);
}

float readCurrent(int PIN, int adc_zeroed)
{
  unsigned long currentAcc = 0;
  unsigned int count = 0;
  unsigned long prevMicros = micros() - sampleInterval ;
  while (count < numSamples)
  {
    if (micros() - prevMicros >= sampleInterval)
    {
      int adc_raw = analogRead(PIN) - adc_zeroed;
      currentAcc += (unsigned long)(adc_raw * adc_raw);
      ++count;
      prevMicros += sampleInterval;
    }
  }
  float rms = sqrt((float)currentAcc/(float)numSamples) * (75.7576 / 1024.0);
  return rms;
  //Serial.println(rms);
}
