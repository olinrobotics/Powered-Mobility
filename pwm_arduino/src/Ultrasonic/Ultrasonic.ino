 //Feel free to use this code.
//Please be respectful by acknowledging the author in the code if you use or modify it.
//Author: Bruce Allen
//Date: 23/07/09

//Digital pin 7 for reading in the pulse width from the MaxSonar device.
//This variable is a constant because the pin will not change throughout execution of this code.
const int pwPinBack = 7;
const int pwPinFront = 4;

//variables needed to store values
long pulseBack, pulseFront, inchesBack, inchesFront, cmB, cmF;

void setup()
{
  //This opens up a serial connection to shoot the results back to the PC console
  Serial.begin(9600);
}

void loop()
{
  pinMode(pwPinBack, INPUT);
  pinMode(pwPinFront, INPUT);

  //Used to read in the pulse that is being sent by the MaxSonar device.
  //Pulse Width representation with a scale factor of 147 uS per Inch.

  pulseBack = pulseIn(pwPinBack, HIGH);
  pulseFront = pulseIn(pwPinFront, HIGH);
  //147uS per inch
  inchesBack = pulseBack / 147;
  inchesFront = pulseFront / 147;
  //change inches to centimetres
  cmB = inchesBack * 2.54;
  cmF = inchesFront * 2.54;

  Serial.print("Back: ");
  Serial.print(inchesBack);
  Serial.print("in, ");
  Serial.print(cmB);
  Serial.print("cm; ");
  Serial.print("Front: ");
  Serial.print(inchesFront);
  Serial.print("in, ");
  Serial.print(cmF);
  Serial.print("cm");
  Serial.println();

  delay(500);
}
