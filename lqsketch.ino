int sensorPin = A0;    // select the input pin for the potentiometer
double sensorValue = 0;  // variable to store the value coming from the sensor
//int myarray[100];
//int i=1;
int pushbutton=A1;

//LQR constants
double k1 = 3.2386;     
double k2 = 162.1769;
double k3 = -3.1623;



unsigned long currentTime, previousTime;
double elapsedTime;
double error;
double lastError;
double input, output;
double setPoint=A1;
double cumError;
double laststate2=0;


void setup() {
  // put your setup code here, to run once:

  // begin the serial monitor @ 9600 baud
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:

  input = analogRead(A0);                
  double inputgiven=0.004887*input;
  output = computeLQ(inputgiven);
  double outputgiven=(51)*output;
  delay(50);
  analogWrite(5, outputgiven);   
  // read the value from the sensor:
  sensorValue = analogRead(sensorPin);
  float voltage = sensorValue *0.004887;

  if (output < 0){
          output = 0;
        }
  if (output > 255){
          output = 255;
  }
  

  Serial.println(voltage);
  //Serial.println(referencetaken);
  Serial.print(" ");
 
  
}

double computeLQ(double inp){     
  currentTime = millis();                //get current time
  elapsedTime = (double)(currentTime - previousTime)*0.001;        //compute time elapsed from previous computation
        
  double state2=inp*0.000470;//voltage across capacitor multiplied by capacitance
  double state1=(state2-laststate2)/elapsedTime;

  double outputreq=state2*2127.7;


  error=(analogRead(A1)*0.004887)-outputreq;                                // determine error
  cumError += error * elapsedTime;                // compute integral
 
  double out = 0 -k1*state1 -k2*state2 - k3*cumError;                //PID output               
 
  lastError = error;                                //remember current error
  laststate2=state2;
  previousTime = currentTime;                        //remember current time
 
  return out;                                        //have function return the PID output
}