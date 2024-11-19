void setup() {
  Serial.begin(9600);

}

void loop() {
  Bluetooth_Data_Processing();
}

void Bluetooth_Data_Processing()
{
  if (Serial.available())
  {
    char data = Serial.read();
    Serial.print("data:");Serial.println(data);
    if (data != '\0' && data != '\n')
    {
      switch (data)
      {

        case '1': Serial.println("1 is pressed by you"); break;

        case '2': Serial.println("2 is pressed by you"); break;

        case '3': Serial.println("3 is pressed by you"); break;

        case '4': Serial.println("4 is pressed by you"); break;

        case '5': Serial.println("5 is pressed by you"); break;

        case '6': Serial.println("6 is pressed by you"); break;
      }
    }
  }
}
