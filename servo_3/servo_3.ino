#include <Servo.h>
Servo s1,s2;

int mean_x = 0;
int mean_y = 0;
int angle = 90;


int calculateMean(int pin) {
  Serial.println("Calculating joystick center positions...");
  
  long sumX = 0;
  int count = 0;
  unsigned long startTime = millis();

  while (millis() - startTime < 500) {  // Run for exactly 5 seconds
    sumX += analogRead(pin);
    count++;
    delay(10);  // Small delay to get reasonable samples
  }
  return sumX/count;
}

int mapJoystickValues(int val, int mean) {
   if (val > mean) {
    return map(val, mean, 1023, 0, 10);  // Map positive direction
  } else {
    return map(val, 0, mean, -10, 0);  // Map negative direction
  }
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  s1.attach(9);
 // for(int i = 0;i<=180; i++) s1.write(i);
 s1.write(90);
  mean_x = calculateMean(A0);
  
}

void loop() {
  // put your main code here, to run repeatedly:
  servo_control(s1,mean_x,A0);
}

void servo_control(Servo s1, int mean, int pin_control){
  int value = mapJoystickValues(analogRead(pin_control),mean);
  Serial.print(value);
  if (angle <= 180 && angle>=0){
    angle+=value;
  }
  else angle = angle;
  angle = constrain(angle,0,180);
  Serial.print(" Angle: ");
  Serial.println(angle);
  s1.write(angle);
  delay(50);
}
