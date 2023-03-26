int sensorPin = A0;    // select the input pin for the potentiometer
double sensorValue = 0;  // variable to store the value coming from the sensor
//int myarray[100];
//int i=1;
int pushbutton=A1;

//LQR constants
//double k1 = 66.7;
//double k2 = 3660.3;
//double k3 = 0.8;

double k1 = 27.1;
double k2 = 130.4;
double k3 = 0.3;


unsigned long currentTime, previousTime;
double elapsedTime;
double error;
double lastError;
double input, output, setPoint;
double cumError;
double x1=0;
double x2=0;
double lastx1=0;
double lastx2=0;
double lastu=0;
double Ap11=0.5312;
double Ap12=-16.1583;
double Ap21=0.0076;
double Ap22=0.9109;
double Bp11=0.0076;
double Cp=2127.7;
double deltax1=0;
double deltax2=0;
double deltau=0;
double u=0;


void setup() {
  // put your setup code here, to run once:


  // begin the serial monitor @ 9600 baud
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  
  input = analogRead(A0);                //read from rotary encoder connected to A0
  double inputgiven=0.004887*input;
  output = computeMPC(inputgiven);
  double outputgiven=51*output;
  delay(50);
  analogWrite(5, outputgiven);   
  // read the value from the sensor:
  sensorValue = analogRead(A0);
  //double voltage = sensorValue*0.004887;

  if (output < 0){
          output = 0;
        }
  if (output > 255){
          output = 255;
  }
  

  Serial.println(output);
  //Serial.print(" ");
}

double computeMPC(double inp){     
  currentTime = millis();                //get current time
  elapsedTime = (double)(currentTime - previousTime)*0.001;        //compute time elapsed from previous computation

      
  //x2=inp*0.00047;
  //x1=(x2-lastx2)/elapsedTime;

  x1=(Ap11*x1)+(Ap12*x2)+(Bp11*lastu);
  x2=(Ap21*x1)+(Ap22*x2);
  double y=inp;

  deltax1=x1-lastx1;
  deltax2=x2-lastx2;

  error=0-(analogRead(A1)*0.004887)+y;                                // determine error
  //cumError += error * elapsedTime;                // compute integral
 

  deltau=0-(k1*deltax1)-(k2*deltax2)-(k3*error);

  u=u+deltau;          
 
  lastError = error;                                //remember current error
  double lastdeltau=deltau;

  previousTime = currentTime;                        //remember current time
  lastx1=x1;
  lastx2=x2;
  lastu=u;
 
  return u;                                        //have function return the PID output
}