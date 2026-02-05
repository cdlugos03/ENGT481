#include <Wire.h>

const uint8_t MT_ADDR = 0x06;
const uint8_t ANGLE_MSB = 0x03;
const uint8_t ANGLE_LSB = 0x04;

static uint32_t lastPrint = 0;
// static uint32_t TotalTime = 0;

uint16_t lastRaw = 0;
int counter = 0;

//for stepper driving
int32_t pulses = 0;
int32_t pulseFinal = 0;
int16_t direction = 0;
int16_t position = 0;

//velocity/frequency calculation
float u = 0;
float v = 0;
float f = 0;

float fLast = 0;
float filter = 1250; //max Hz/second
float filter2 = 0;

static uint32_t tlast = 0; //setup variable for time storage

int16_t raw = 0;
int16_t rawlast = 5850;
//5724 up
uint16_t downVal = 12152; //value when straight down
uint16_t correction = downVal - 8192;//value for straight up
float angleInt = 0;
float angleDiff = 0;
float angleLast = 0;
float targetAngle = 0;
float targetAngleCap = 0;
float targetAngleDiff = 0;
float targetAngleLast = 0;
float error = 0;

int32_t message = 65535;
int32_t messageLast = 65535;
int32_t Test = 123456;
//----------------------------------------------------

void setup() {
  motorInit();
  interruptInit();
  motorControl(65535);

  pinMode(22,INPUT_PULLUP); //limit switch inputs
  pinMode(27,INPUT_PULLUP);

  digitalWrite(9,HIGH); //disable motor  
  
  Serial.begin(115200); 
  while(!Serial){}

  Wire.begin();
  Wire.setClock(400000);
  Wire.setTimeout(5);

 while (Serial.available() <= 3){ //wait for a serial communication
        //delay(0.00001);
      }
      //message = Serial.read();
      //Serial.readBytes((char*)&message, 4);

  digitalWrite(9,LOW); //enable motor
}

void loop() {

//  Serial.write("running");
  
  uint32_t TotalTime = millis();
  

    if (TotalTime - lastPrint >= 4){
      lastPrint = TotalTime;

      if(message == 0xFFFFFFFF){
        TCCR3B = 0x00;
        digitalWrite(9,HIGH); //disable motor
      }
      else{
        //filter recieved value
//        if(abs(abs(message) - abs(messageLast)) > 10000){
//          message = messageLast;
//        }
       motorControl(message);
      }

      raw = readAngleRaw();
      int16_t positionTrue = position;

      Serial.write(positionTrue>>8);
      Serial.write(positionTrue);
      Serial.write(raw>>8); // change to serial.write for communicating in binary
      Serial.write(raw);
      //Serial.write('\n');

//      Serial.write(message>>24);
//      Serial.write(message>>16);
//      Serial.write(message>>8); // change to serial.write for communicating in binary
//      Serial.write(message);
//      Serial.write('\n');

      if(digitalRead(22) == 0 || digitalRead(27) == 0){ //if limit switch is triggered
        digitalWrite(9,HIGH); //disable motor
        TCCR3B = 0x00; //disable timer
        while(1){} //catch program
      }

      messageLast = message;

      while (Serial.available() <= 3){   //confirms data is availible
        //delay(0.00001);
      }
      //message = Serial.read();
      Serial.readBytes((char*)&message, 4);
      
    }

/*
//  this is how to receive data from ipi
  if(Serial.available()>0){   //confirms data is availible
    String message = Serial.readStringUntil('\n');  //reads data from pi
    Serial.println(message);
  }
*/
}

//IC2 read
uint16_t readAngleRaw(){
  uint16_t value = lastRaw;

  Wire.beginTransmission(MT_ADDR);
  Wire.write(ANGLE_MSB);
  uint8_t err = Wire.endTransmission(false);

  if (err != 0){
    return value;
  }

  uint8_t n = Wire.requestFrom((int)MT_ADDR, (uint8_t)2, (uint8_t)true);
  if(n == 2){
    uint8_t msb = Wire.read();
    uint8_t lsb = Wire.read();

    value = ((uint16_t)msb << 6) | (lsb & 0x3F);
  }

  lastRaw = value;
  return value;
}


void motorInit(){
  TCCR3A = 0x00;
	TCCR3B = 0x00;
	TCNT3H = 0x00;
	TCNT3L = 0x00;
	OCR3A = 0x0000;
	OCR3B = 0x0000;
	TIMSK3 = 0x00;
	TIFR3 = 0x00;

  TCCR3A = (1<<COM3A0); //set toggle on compare match
	TCCR3B = (1<<WGM33) | (1<<WGM32) | (1<<CS31) | (1<<CS30); //set the prescaler to 64, set CTC and start timer

  pinMode(9,OUTPUT);
  pinMode(7,OUTPUT);
  pinMode(5,OUTPUT);

  digitalWrite(9,LOW); //set enable high (active low)
  digitalWrite(7,LOW); //set direction high
}

void motorControl(int32_t pulses){
   if(digitalRead(9) == 1) pulses = 65535;
  
  //configure motor ouputs
  if(pulses>0){
    //digitalWrite(9,HIGH); //set enable high
    digitalWrite(7,HIGH); //set direction high
    direction = -1;
    pulseFinal = pulses;
  } else if(pulses<0){
    //digitalWrite(9,HIGH); //set enable high
    digitalWrite(7,LOW); //set direction low
    direction = 1;
    pulseFinal = pulses * -1; //make value positive
  }
 
  //set timer max value
  ICR3 = pulseFinal; //set max timer value
  if(TCNT3>ICR3){ //if the timer value is too big
    TCNT3 = ICR3 - 1;//set TCNT just under ICR3 so it will toggle the output and reset
  }
}

void interruptInit(){
  EICRA = (1<<ISC31);// | (1<<ISC30); //set interrupt 3 to trigger on falling edge
	EIMSK = (1<<INT3); //enable external interrupt 3

  sei(); //enable interrupt
  // interrupts();
}

ISR(INT3_vect) //interrupt from OC3A
{
  //increment position by direction
  position = position + direction;

  //check direction, and increment the position
	// if(direction == 1){
  //   position = position + 1;
  // } else{
  //   position = position - 1;
  // }
}
