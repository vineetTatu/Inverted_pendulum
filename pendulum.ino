int encoderPin1 = 2;
int encoderPin2 = 3;

int PWM = 9 ; 
int IN1 = 8 ;

volatile int lastEncoded = 0;
volatile long encoderValue = 0;
long lastencoderValue = 0;
int lastMSB = 0;
int lastLSB = 0;
long prevT = 0;
float eprev = 0;
float eintegral = 0;
int e = 0;


void setup() {
  
Serial.begin(9600);
pinMode(9,OUTPUT);
pinMode(8,OUTPUT);
pinMode(encoderPin1, INPUT);
pinMode(encoderPin2, INPUT);
digitalWrite(encoderPin1, HIGH); //turn pullup resistor on
digitalWrite(encoderPin2, HIGH); //turn pullup resistor on
 //call updateEncoder() when any high/low changed seen
 //on interrupt 0 (pin 2), or interrupt 1 (pin 3)
attachInterrupt(0, updateEncoder, CHANGE);
attachInterrupt(1, updateEncoder, CHANGE);

}
void loop() {

 //Serial.println(encoderValue);
// set target position
int target = 0;
int angle = 0;
// PID constants
float kp = 2;
float kd = 0;
float ki = 0;

//real time  angle 
 angle = int (0.225 * encoderValue) ;
 //Serial.println(angle);

// error
int e = angle - target;
Serial.println(e);

// derivative
float dedt = (e-eprev);

// integral
eintegral +=e ;

// control signal
float u = kp*e + kd*dedt + ki*eintegral;

// motor power
float pwr = fabs(u);
if( pwr > 255 ){
pwr = 255;
}
//Serial.println(pwr);

// signal the motor
setMotor(pwr,PWM,IN1, e);
// store previous error
eprev = e;
//Serial.println(target);
}
void setMotor(int pwr, int PWM, int IN1, int e){
analogWrite(PWM,pwr);
if(e > 0){
digitalWrite(IN1,HIGH);
}
else if(e < 0){
digitalWrite(IN1,LOW);
}
//else{
//digitalWrite(IN1,LOW);
//}
}

void updateEncoder(){
int MSB = digitalRead(encoderPin1); //MSB = most significant bit
int LSB = digitalRead(encoderPin2); //LSB = least significant bit
int encoded = (MSB << 1) |LSB; //converting the 2 pin value to single number

int sum = (lastEncoded << 2) | encoded; //adding it to the previous encoded value 
 if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011)
encoderValue ++;
 if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000)
encoderValue --;
 lastEncoded = encoded; //store this value for next time
}
