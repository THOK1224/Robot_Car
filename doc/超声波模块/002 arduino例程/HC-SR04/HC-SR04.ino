float checkdistance_A5_A4() {
  digitalWrite(A5, LOW);
  delayMicroseconds(2);
  digitalWrite(A5, HIGH);
  delayMicroseconds(10);
  digitalWrite(A5, LOW);
  float distance = pulseIn(A4, HIGH) / 58.00;
  delay(10);
  return distance;
}

void setup(){
  Serial.begin(115200);
  pinMode(A5, OUTPUT);
  pinMode(A4, INPUT_PULLUP);
}

void loop(){
  // 硬件接口：arduino的S6口-A5-TRIG A4-ECHO
  // 代码逻辑：获取超声波距离并打印并串口打印

  Serial.print(checkdistance_A5_A4());
  Serial.print("cm");
  delay(100);

}
