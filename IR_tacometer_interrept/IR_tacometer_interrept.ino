
int pin = 2;
unsigned long int pulse = 0;
//int pin_value,prev_pin_value=LOW;
long unsigned int start_time=0;
int no_of_wing=1;// Change with the prpopeller that you use
void setup() {
  // put your setup code here, to run once:
  pinMode(pin, INPUT_PULLUP);
  Serial.begin(115200);
 
  attachInterrupt(digitalPinToInterrupt(pin),count,FALLING);
  start_time=millis();
}

void loop() {
  // put your main code here, to run repeatedly
  long unsigned int last_time=millis();
  
  if(last_time-start_time>=1000){
    detachInterrupt(digitalPinToInterrupt(pin));
    Serial.print(pulse *60/3);
    Serial.print(" -> ");
    Serial.println(last_time);
    pulse=0;
    start_time=last_time;
    attachInterrupt(digitalPinToInterrupt(pin),count,FALLING);
  }
  
}

void count(){
  pulse++;
}
