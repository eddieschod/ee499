/* EE499 Final Project - Current Control
 * Summer 9/20/2016 - Prof. Taylor
 * Example software for running closed-loop current control
 *   of a DC motor using MegaMoto H-bridge shield.
 * 
 * --- Jumper Settings ---
 * Enable pin is on D8
 * PWMA pin on D9
 * PWMB pin on D10
 * Current Sense A on pin A3
 * Current Sense B on pin A0
 */

#define PWM_Count 4000
#define PWM_Half  2000
#define PWM_Period 5E-4

//########################  Global Constants  #######################
const int p08_ENABLE  = 8;      // Enable pin for half-bridges
const int p09_PWM1A   = 9;      // motor armature H-brg A
const int p10_PWM1B   = 10;     // motor armature H-brg B
const int p11_DEBUG   = 11;     // used to debug code in various places
const int p12_ISRUTIL = 12;     // used to monitor CPU utilization
const int p13_LED     = 13;     // LED pin, used for debugging

//########################  Global Variables  #######################
int   SysRunState   = 0;        // Allows user to enter debug states
int   AdcRawData[6];            // Used to hold raw analog data
float AdcRWVals[6];             // Real-world converted values
float ArmCur        = 0;        // Computed value of real winding current
float ArmCurRef     = 0;        // Reference value for winding current
long  SerialCounter = 0;        // Increments every interrupt
char  SerialRxCmd   = 0;        // The command byte of a SCI RX
int   SerialRxData  = 0;        // The data value of RX command
int   FltCode       = 0;        // Holds various fault codes

//####################  PID Controller Structure  ###################
struct PID
{ const float Ts;       //Samplie time used for PID controller
        float Kp;       //Proportional Gain DCDC Converter
        float Ki;       //Integral Gain DCDC Converter
  const float Max;      //Upper saturation limit
  const float Min;      //Lower saturation limit
        float Int;      //Value of Integrator (initial condition)
        float Out;      //Variable to hold output (to check saturation)
};
PID PID_Arm = {PWM_Period,50,500,+PWM_Half,-PWM_Half,0,0};  //Armature current

//###################  Rate Limiter Structure  ######################
struct RateLim
{ const float Ts;        //Sample time used for limiter
  const float Rate;      //Slew Limit
        float Prev;      //Previous Output Value
};
RateLim RtLm_Arm   = {PWM_Period,5,0};    //Armature current ref

//#########   Discrete 1st-Order Digital Filter Structure  #########
struct DigFilt
{ const float Num;       //Numerator Coefficents
  const float Den;       //Denominator Coefficents
        float Mem;       //Values that pass through filter steps
        float Out;       //Buffer for output value of filter
};
DigFilt LPF_iArm = {0.2452373, -0.5095255, 0, 0 };  //Fs/10

//###################  Function Prototypes  ######################
float CalcPID(float err, PID *params);
float CalcRtLm(float target, RateLim *vals);
float CalcLPF(float val, DigFilt *coefs);

//#######################  "SETUP" Function  ######################
void setup()
{ //initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
  Serial.setTimeout(500);

  //set the direction of the digital pins:
  pinMode(p08_ENABLE,  OUTPUT);  //Enable for Half-bridges
  pinMode(p09_PWM1A,   OUTPUT);  //OC1A
  pinMode(p10_PWM1B,   OUTPUT);  //OC1B
  pinMode(p11_DEBUG,   OUTPUT);
  pinMode(p12_ISRUTIL, OUTPUT);
  pinMode(p13_LED,     OUTPUT);
  
  digitalWrite(p08_ENABLE, LOW); //Set initial position
  ADCSRA |=  0x05;          //Increase ADC clock speed...
  ADCSRA &= ~0x02;          //... by changing clock prescaler
  
  noInterrupts();           // disable all interrupts
  //initialize timer1
  TCCR1A = 0; TCCR1B = 0;   // Stop the timer, clear out config bits
  TCNT1  = 0;               // clear out current timer value
  OCR1A = PWM_Half;         // set DutyA initial value
  OCR1B = PWM_Half;         // set DutyB initial value
  ICR1  = PWM_Count;        // set period (freq) value to 2kHz
  TIMSK1 |= (1 << ICIE1);   // enable timer period match interrupt
  TCCR1A = 0b11110000;      // set PWM polarities inverted (needed for ADC sampling)
  
  //turn on timers by setting clock bits
  TCCR1B = 0b00010001;      // Select WGM bits, set clock select bits
  interrupts();             // enable all interrupts
}

//#######################  MAIN Program Entry  ######################
void loop()
{ //Check for any host commands
  if(Serial.available() >= 2)  //if there's at least 2 bytes here
  { SerialRxCmd = Serial.read();      //Grab first byte
    SerialRxData = Serial.parseInt(); //Grab data integer (several bytes)
    Serial.print("I got '"); Serial.print(SerialRxCmd); //Echo back data
    Serial.print(SerialRxData, DEC); Serial.println("'");
    RunDbgCmd(); //Process the recieved command
    while(Serial.available()) Serial.read();  //flush the buffer
  }
  
  //Serial port sending is scheduled by Timer ISR & "SerialCounter" variable
  if(SerialCounter >= 500)
  { SerialCounter = 0;    //Clear out the counter
    digitalWrite(p13_LED, digitalRead(p13_LED) ^ 1);   // toggle LED pin

    //Periodically send out all variable contents for debugging
    Serial.print("iA=");   Serial.print(AdcRWVals[3]);
    Serial.print(" iB=");  Serial.print(AdcRWVals[0]); 
    Serial.print(" iAr="); Serial.print(AdcRawData[3]);
    Serial.print(" iBr="); Serial.print(AdcRawData[0]);
    Serial.print(" iX=");  Serial.print(LPF_iArm.Out);
    Serial.print(" Run="); Serial.print(SysRunState);
    Serial.print(" Flt="); Serial.print(FltCode);
    Serial.print(" DA=");  Serial.print(OCR1A);
    Serial.print(" DB=");  Serial.print(OCR1B);
    Serial.print(" PI=");  Serial.print(PID_Arm.Out);
    Serial.print("\n"); //print new-line character
  }//end IF
}//end LOOP

//###########################  TIMER1 ISR ##########################
ISR(TIMER1_CAPT_vect)      // timer capture interrupt service routine
{ digitalWrite(p12_ISRUTIL, HIGH);    //Set the ISR Util pin
  SerialCounter++;                    //Used to schedule SCI Tx msgs
  
  //Read ADC pins and store data into array
  AdcRawData[3] = analogRead(A3);  //Grab iA value
  AdcRawData[0] = analogRead(A0);  //Grab iB value
  
  //Convert integer data into real-world values
  AdcRWVals[3] = (AdcRawData[3] - 30) * (0.075); //convert iA value
  AdcRWVals[0] = (AdcRawData[0] - 20) * (0.075); //convert iB value

  //Select ADC channel, based on current polarity (ignore negatives)
  if (AdcRWVals[3] >= 0) ArmCur = +AdcRWVals[3]; //Use potitive iA value
  else                   ArmCur = -AdcRWVals[0]; //Use negative iB value
  
  //Filter the armature current value
  CalcLPF(ArmCur, &LPF_iArm);
  
  if (!FltCode)//if no fault code
  { if (LPF_iArm.Out > +10) FltCode = 1;  //iA Pos Over Current
    if (LPF_iArm.Out < -10) FltCode = 2;  //iA Neg Over Current
    
    if (SysRunState & 0x01) //Enable Open-Loop PWM!
    { digitalWrite(p08_ENABLE, HIGH);
      TCCR1A = 0b11110000;  //Turn on PWM!!
    
      if(SysRunState & 0x02) //Enable Closed-Loop Current Control!!
      { CalcPID(CalcRtLm(ArmCurRef, &RtLm_Arm) - LPF_iArm.Out, &PID_Arm);
    
        OCR1A = PWM_Half - PID_Arm.Out;
        OCR1B = PWM_Half + PID_Arm.Out;
      } else
      { ArmCurRef = 0;
        RtLm_Arm.Prev = 0;
        PID_Arm.Int = 0;
        PID_Arm.Out = 0;
      }
    } else
    { digitalWrite(p08_ENABLE, LOW);
      SysRunState = 0;
      TCCR1A = 0;
      OCR1A = PWM_Half;
      OCR1B = PWM_Half;
    }
  } else //fault code!
  { digitalWrite(p08_ENABLE, LOW);
    SysRunState = 0;
    TCCR1A = 0;
    OCR1A = PWM_Half;
    OCR1B = PWM_Half;
  }
  
  digitalWrite(p12_ISRUTIL,LOW);      //Clear the ISR Util pin
}

//####################  "RunDbgCmd" Subroutine  ###################
void RunDbgCmd()
{ switch(SerialRxCmd)
  { case 'A':  //Armature Current Reference
      if((SerialRxData >= -5) && (SerialRxData <= 5))
      { ArmCurRef = SerialRxData;}
      break;

    case 'D':  //Duty of both bridges
      if((SerialRxData >= -PWM_Half) && (SerialRxData <= +PWM_Half))
      { OCR1A = PWM_Half - SerialRxData;
        OCR1B = PWM_Half + SerialRxData;
      }
      break;

    case 'S':  //Run State
      if((SerialRxData >= 0) && (SerialRxData <= 15))
      { SysRunState = SerialRxData;}
      break;
    
    case 'P':  //Armature Current Proportional gain
      if((SerialRxData >= 0) && (SerialRxData <= 100))
      { PID_Arm.Kp = SerialRxData;}
      break;
    
    case 'I':  //Armature Current Integral gain
      if((SerialRxData >= 0) && (SerialRxData <= 1000))
      { PID_Arm.Ki = SerialRxData;}
      break;
    
    case 'T':  //Terminate system
      digitalWrite(p08_ENABLE, LOW);
      SysRunState = 0;
      TCCR1A = 0;
      FltCode = 0;
      ArmCurRef = 0;
      RtLm_Arm.Prev = 0;
      PID_Arm.Int = 0;
      PID_Arm.Out = 0;
      OCR1A = PWM_Half;
      OCR1B = PWM_Half;
      break;
  
    default: Serial.println("Command not processed..."); break;  
  }//switch
  SerialRxCmd = 0;
}//DbgCmd

//#######################  CalcLPF Function  ######################
// This will perform a first order filtering of the input variable
float CalcLPF(float val, DigFilt *coefs)
{ //Perform IIR operation (this is IIR Direct Form 2 Transposed)
  //Function ASSUMES normalized First-Order Butterworth filter;
  //i.e. the coefs b0=b1, and a0=1
 
  //Calculate filter output
  coefs->Out = (coefs->Num * val) + coefs->Mem;
  //Update the filter memory blocks
  coefs->Mem = (coefs->Num * val) - (coefs->Out * coefs->Den);
  return coefs->Out;
}

//#######################  CalcPID Function  ######################
// Function takes the input error signal and computes the output signal
float CalcPID(float err, PID *params)
{ //update integrator value
  params->Int  += (params->Ki * err) * params->Ts;

  //Integrator Saturation
  if(params->Int > params->Max) params->Int = params->Max;
  if(params->Int < params->Min) params->Int = params->Min;

  //update output value
  params->Out = (params->Kp * err) + params->Int;

  //Output Saturation
  if(params->Out > params->Max) params->Out = params->Max;
  if(params->Out < params->Min) params->Out = params->Min;
  
  return params->Out;
}

//#######################  CalcRtLm Function  ######################
// Function limits the rate of change of an input signal
float CalcRtLm(float target, RateLim *vals)
{ float TempVal;
  TempVal = vals->Prev + (vals->Rate * vals->Ts);
  if(TempVal < target) 
  { vals->Prev = TempVal;  //Store previous value
    return TempVal;        //Write output value
  }
  
  TempVal = vals->Prev - (vals->Rate * vals->Ts);
  if(TempVal > target) 
  { vals->Prev = TempVal;  //Store previous value
    return TempVal;        //Write output value
  }
  
  //Else, difference between in & out is small, set out = in & store
  vals->Prev = target;    //Store previous value
  return target;          //Write output value
}

//###########################  END OF FILE ##########################

