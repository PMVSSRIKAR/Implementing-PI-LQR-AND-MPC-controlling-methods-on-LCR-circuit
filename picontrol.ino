int sensorPin = A0;    // select the input pin for the potentiometer
double sensorValue = 0;  // variable to store the value coming from the sensor
//int myarray[100];
//int i=1;
int pushbutton=A1;

//PID constants
double kp = 2.1304;
double ki = 27.6952;
//double kp = 1;
//double ki = 0.2;


unsigned long currentTime, previousTime;
double elapsedTime;
double error;
double lastError;
double input, output, setPoint;
double cumError;



void setup() {
  // put your setup code here, to run once:
  

  // make the pushbutton's pin an input:
  pinMode(pushbutton, INPUT);

  // begin the serial monitor @ 9600 baud
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:

  
  input = analogRead(A0);                //read from rotary encoder connected to A0
  double inputgiven=0.004887*input;
  output = computePI(inputgiven);
  double outputgiven=51*output;
  double k=outputgiven;
  if (outputgiven < 0){
          outputgiven = 0;
        }
  if (outputgiven > 255){
          outputgiven =k ;
  }
  delay(5);
  analogWrite(5, outputgiven);   
  // read the value from the sensor:
  sensorValue = analogRead(sensorPin);
  float voltage = sensorValue * 0.004887;

  
  

  Serial.println(voltage);

  Serial.print(" ");
 
  //delay(2);
}

double computePI(double inp){     
  currentTime = millis();                //get current time
  elapsedTime = (double)(currentTime - previousTime)*0.001;        //compute time elapsed from previous computation
        
  error=(analogRead(A1)*0.004887)-inp;                                // determine error
  cumError += error * elapsedTime;                // compute integral
 
  double out = kp*error + ki*cumError;                //PID output               
 
  lastError = error;                                //remember current error
  previousTime = currentTime;                        //remember current time
 
  return out;                                        //have function return the PID output
}