const int PWMA_LEFT_MOTOR = 0;
const int INA1_LEFT_MOTOR = 1;
const int INA2_LEFT_MOTOR = 2;

void setup() {
  // put your setup code here, to run once:
  pinMode(PWMA_LEFT_MOTOR, OUTPUT);
  pinMode(INA1_LEFT_MOTOR, OUTPUT);
  pinMode(INA2_LEFT_MOTOR, OUTPUT);

}

void loop() {
  // put your main code here, to run repeatedly:
  analogWrite(PWMA_LEFT_MOTOR, 124);
  digitalWrite(INA1_LEFT_MOTOR, 1);
  digitalWrite(INA2_LEFT_MOTOR, 0);


}
