#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <RunningMedian.h>

#define ende 0x27
#define buttonPin 15

//const int numReadings = 30;//quantidade de leituras antes de fazer a media
const int mVperAmp = 185.0; // Sensibilidade do sensor ACS712
const int analogPins[] = {36, 39,34}; // Pinos analógicos para Sensor 1 e Sensor 2
const int analogPinstensao[] = {35, 32, 33}; //pinos analogicos para sensor de tensao
// const int numSensors = 3; // N de sensores
int currentState = 0; // Variável para rastrear o estado atual
bool buttonPressed = false;

const int interruptPin = 2;
volatile unsigned long interruptCount = 0; // conta o número de interrupções 
volatile unsigned long lastInterruptTime = 0; // armazena o tempo da última interrupção
float rpm =0;

//criacao de objetos 
LiquidCrystal_I2C lcd(ende, 16, 2);
RunningMedian readings(5); // Inicialize um objeto RunningMedian com tamanho 5

void setup() {
  Serial.begin(115200);
  lcd.init();
  lcd.clear();
  lcd.backlight();
  pinMode(buttonPin, INPUT_PULLUP);
  handleInterrupt();
  //configuração de conexão wifi e do websocket
}

void loop() {

  // Lê o estado do botão
  int buttonState = digitalRead(buttonPin);
  // Verifica se o botão foi pressionado e não estava pressionado anteriormente
  if (buttonState == LOW && !buttonPressed) {
    buttonPressed = true; // Indica que o botão foi pressionado
    currentState = (currentState + 1) % 8; // Alterna entre os estados (0 a 4)
    lcd.clear();
  }

  // Verifica se o botão foi liberado para permitir a próxima mudança de estado
  if (buttonState == HIGH) {
    buttonPressed = false;
  }

  // Executa o caso correspondente ao estado atual
  switch (currentState) {
    case 0:
      lcd.setCursor(0, 0);
      lcd.print(" <<Pressione >>");
      lcd.setCursor(0, 1);
      lcd.print("<< o botao >>");
      break;
    case 1:
      lcd.setCursor(0, 0);
      displaySensorValue(analogPins[0], "Sensor 1");
      break;
    case 2:
      lcd.setCursor(0, 0);
      displaySensorValue(analogPins[1], "Sensor 2");
      break;
    case 3:
      lcd.setCursor(0, 0);
      displaySensorValue(analogPins[3], "Sensor 3");
      break;
    case 4:
      lcd.setCursor(0, 0);
      lcd.print("rpm:");
      lcd.print(rpm);
      break;
    case 5:
  lcd.setCursor(0, 0);
  displaySensorValuetesao(analogPins[0], "Sensor 1");
  break;
case 6:
  lcd.setCursor(0, 0);
  displaySensorValuetesao(analogPins[1], "Sensor 2");
  break;
case 7:
  lcd.setCursor(0, 0);
  displaySensorValuetesao(analogPins[2], "Sensor 3");
  break;
    //pode fazer um caso para mostrar se o esp esta conectaddo ao wifi.
  }
  //delay(50);
}

/*---------------------------- medicao de corrente ------------------------------------*/
float getSensorValue(int analogPin) {
  float maxValue = 0;
  float minValue = 4095;
  uint32_t startTime = millis();
  while ((millis() - startTime) < 1000) {
    int sensorValue = analogRead(analogPin);
    //
    //printar os valores que foram pegos no analogPin e a média pois pode aconttece de pega um unico valor do ADC e ja adicionar na media
    readings.add(sensorValue);
    float average = readings.getMedian();
   // if (average > maxValue) {
    //  maxValue = average;
    //} tesrar essa parte do código
    if (average < minValue) {
      minValue = average;
    }
  }
  float tensaoPinCurrent = ((maxValue - minValue) * 3.3) / 4096; //tensao  
  float VRMS = (tensaoPinCurrent / 2.0) * 0.707;
  float AmpsRMS = ((VRMS * 1000) / mVperAmp);
  return (AmpsRMS);
}

void displaySensorValue(int analogPin, const char* label) {
  float sensorValue = getSensorValue(analogPin);
  lcd.print(label);
  lcd.setCursor(0, 1);
  lcd.print("Valor: ");
  lcd.print(sensorValue, 2); 
}

/* ------------------------------- medicao de rpm ----------------------------------------------- */
// função para interrupção que será executada sempre que ocorre uma interrupção no pino interruptPin.
void handleInterrupt() {
  unsigned long currentMillis = millis();
  if (currentMillis - lastInterruptTime > 100) { // se o tempo desde a última interrupção é maior que 100 milissegundos
    interruptCount++;
    lastInterruptTime = currentMillis;
  }
  Serial.print("opa");
}

void setupAndLoop() {
  pinMode(interruptPin, INPUT_PULLUP); 
  attachInterrupt(digitalPinToInterrupt(interruptPin), handleInterrupt, RISING); // capta a borda de subida
  Serial.begin(9600);
  while (true) {
    //delay(1000);
    detachInterrupt(digitalPinToInterrupt(interruptPin)); // desativa interrupção'
    rpm = (interruptCount / 2.0) * 60.0;  // 2 interrupções por evolução
    Serial.print("RPM: ");
    Serial.println(rpm);
    interruptCount = 0;
    attachInterrupt(digitalPinToInterrupt(interruptPin), handleInterrupt, RISING);
  }
}

/* ------------------------------   medicao tensao  -------------------------------------------------*/

float medTensao(int pinADCTen) {
  int sensorValue = analogRead(pinADCTen);
  readings.add(sensorValue);
  // Calcular a média
  int average = readings.getMedian();
  //talvez tenha que limpar a media anterior...
  float tensaoPin = (average * 3.3) / 4096;
  float ten = 0; 
  if (tensaoPin >= 1.50 && tensaoPin <= 1.99) {
    ten = (94.8 * tensaoPin) + 7.11 - 15;
  } else if (tensaoPin >= 2.0 && tensaoPin <= 2.99) {
    ten = 103 * tensaoPin + (-49.7); // Fix the comma typo
  } else {
    //implementar
    ten =0;
    }
  //delay(100);
  return ten; // Return the calculated value
}

void displaySensorValuetesao(int analogPinstensao, const char* label) {
  float sensorValuet =  medTensao(analogPinstensao);
  lcd.print(label);
  lcd.setCursor(0, 1);
  lcd.print("Valor: ");
  lcd.print(sensorValuet, 2); // Mostrar o valor com 2 casas decimais
}
