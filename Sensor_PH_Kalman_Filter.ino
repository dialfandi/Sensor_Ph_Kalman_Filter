#define analogPh 13

// kalman variables
float varVolt = 0.051364211;  // variance determined using excel and reading samples of raw sensor data
float varProcess = 1e-6;
float Pc = 0.0;
float G = 0.0;
float P = 1.0;
float Xp = 0.0;
float Zp = 0.0;
float Xe = 0.0;

float varAnalog = 482.4634921;  // variance determined using excel and reading samples of raw sensor data
float varProcAnalog = 1e-2;
float PcAn = 0.0;
float GAn = 0.0;
float PAn = 1.0;
float XpAn = 0.0;
float ZpAn = 0.0;
float XeAn = 0.0;

long previousMillis = 0;
unsigned long interval = 100;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(analogPh, INPUT);
}

void loop() {
  int sensorVal = analogRead(analogPh);
  float volt = sensorVal * 5.0 / 4096;
  // kalman process
  Pc = P + varProcess;
  G = Pc / (Pc + varVolt);  // kalman gain
  P = (1 - G) * Pc;
  Xp = Xe;
  Zp = Xp;
  Xe = G * (volt - Zp) + Xp; // the kalman estimate of the sensor voltage

  PcAn = PAn + varProcAnalog;
  GAn = PcAn / (PcAn + varAnalog);  // kalman gain
  PAn = (1 - GAn) * PcAn;
  XpAn = XeAn;
  ZpAn = XpAn;
  XeAn = GAn * (sensorVal - ZpAn) + XpAn; // the kalman estimate of the sensor voltage
  //  serialFloatPrint(volt);
  //  serialFloatPrint(Xe);
  //Serial.print(sensorVal); Serial.print(", "); Serial.print(XeAn); Serial.print(", "); Serial.print(volt);Serial.print(", ");Serial.println(Xe);
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis > interval) {
    // save the last time you blinked the LED
    previousMillis = currentMillis;
    float phValue = -4.6786*Xe + 8.3291;
    Serial.println(phValue, 1);
  }
}


// function to print out floats in HEX
