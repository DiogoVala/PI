#define CTRLpin_PWM 8

#define UART_BAUDRATE 9600
#define PWM_FREQUENCY 2000
#define PWM_RESOLUTION 12
#define MAX_DUTY_CYCLE 4095 // Change according to PWM_RESOLUTION

void setup() {
  // put your setup code here, to run once:
  Serial.begin(UART_BAUDRATE);
  pinMode(CTRLpin_PWM, OUTPUT);

  /*PWM initialization*/
  analogWriteResolution(PWM_RESOLUTION);
  analogWriteFrequency(CTRLpin_PWM, PWM_FREQUENCY);
  analogWrite(CTRLpin_PWM, 0); /* Power controller control signal*/
}

void loop() {
  // put your main code here, to run repeatedly:
  analogWrite(CTRLpin_PWM, 0);
  Serial.println("Startup");
  delay(2000);

  analogWrite(CTRLpin_PWM, 0.3*MAX_DUTY_CYCLE);
  Serial.println("30% Power");
  delay(2000); // 2Sec - Preheat

  analogWrite(CTRLpin_PWM, 0.6*MAX_DUTY_CYCLE);
  Serial.println("60% Power");
  delay(1000); // 1Sec - Sealing

  analogWrite(CTRLpin_PWM, 0);
  Serial.println("Power off");
  delay(20000); // 20Sec - Off
}
