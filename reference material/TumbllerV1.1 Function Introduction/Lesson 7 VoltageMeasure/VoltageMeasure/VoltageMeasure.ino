#define VOL_MEASURE_PIN A2
unsigned long vol_measure_time = 0;

void setup() {
  Serial.begin(9600);
  voltageInit();
}


void loop() 
{
    Voltage_Measure();
}

void voltageInit()
{
  analogReference(INTERNAL);
}

void Voltage_Measure()
{
    if (millis() - vol_measure_time > 1000) //Measured every 1000 milliseconds
  {
    vol_measure_time = millis();
    double voltage = (analogRead(VOL_MEASURE_PIN) * 1.1 / 1024) * ((10 + 1.5) / 1.5); //Read voltage value
    Serial.print("Current voltage value : ");
    Serial.println(voltage);
    if(voltage>7.8)
      Serial.println("The battery is fully charged");
    else
      Serial.println("Low battery");
  }
}
