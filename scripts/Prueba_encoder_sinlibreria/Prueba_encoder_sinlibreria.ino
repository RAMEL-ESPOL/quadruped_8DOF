int CHA = 2;
int CHB = 3;
volatile int contador = 0;
float posicion_actual;
float sp;
float PV;


void setup() {
  pinMode(CHA, INPUT);
  Serial.begin(115200);
  attachInterrupt(1, interrupcion, RISING);
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.print("pulsos:");
  Serial.print(contador);
  posicion_actual = mapFloat(contador, -5000.0, 5000.0, -360.0, 360.0);
  Serial.print("  ");
  Serial.print("Posicion actual:");
  Serial.print(posicion_actual);
  Serial.println(" grados");
  delay(100);

  
}

float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void interrupcion() {
  int temp = digitalRead(CHB);

  if (temp> 0){
    contador ++;
  }
  else {
    contador -- ;
  }

  if ((contador == 5000) || (contador == -5000)){
    contador =0;
  }
}
