#include <SPI.h>
#include <SD.h>
#include <TinyGPS.h>
#include <NeoSWSerial.h>

// variáveis
int tring = 7; // pino TRIG do sensor ultrassônico
int echo = 6; // pino ECHO do sensor ultrassônico
float tempo; // para armazenar o tempo de ida e volta do sinal em microsegundos
float distancia_cm; // para armazenar a distância em centímetros

const int portaLed = 5;
const int pinBuzzer = 3;
const int chipSelect = 4;

//gps
static const int RXPin = 9, TXPin = 8;
static const uint32_t GPSBaud = 9600;

// The TinyGPS++ object
TinyGPS gps;

// The serial connection to the GPS device
NeoSWSerial ss(RXPin, TXPin);
//endGPS


void setup() {
  Serial.begin(115200);
  ss.begin(GPSBaud);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("Cartao nao foi encontrado!");
    // don't do anything more:
    return;
  }
  Serial.println("Cartao iniciado");

  pinMode(tring, OUTPUT);
  pinMode(pinBuzzer, OUTPUT);
  pinMode(portaLed, OUTPUT);
  // deixa pino em LOW
  digitalWrite(tring, LOW);
  delayMicroseconds(10);
  pinMode(echo, INPUT);

}

void loop() {
  sensorUltrassonico();
  sensorLed();
}

void sensorUltrassonico() {
  // disparar pulso ultrassônico
  digitalWrite(tring, HIGH);
  delayMicroseconds(10);
  digitalWrite(tring, LOW);

  // medir tempo de ida e volta do pulso ultrassônico
  tempo = pulseIn(echo, HIGH);

  // calcular as distâncias em centímetros e polegadas
  distancia_cm = (tempo / 29.4) / 2;

  // aguardar um pouquinho antes de começar tudo de novo
  delay(100);

  // apresentar resultados no display LCD
  Serial.print("Distancia: ");
  Serial.print(distancia_cm);
  Serial.println(" cm");

  if (distancia_cm <= 80 && distancia_cm > 0) {
    buzzer();
    coordenadasGPS();
  }
}

void buzzer() {
  tone(pinBuzzer, 1100);
  delay(500);
  noTone(pinBuzzer);
}

void coordenadasGPS() {  
  bool newData = false;
  unsigned long chars;
  unsigned short sentences, failed;

  // For one second we parse GPS data and report some key values
  for (unsigned long start = millis(); millis() - start < 1000;){
    while (ss.available()){
      char c = ss.read();
      // Serial.write(c); // uncomment this line if you want to see the GPS data flowing
      if (gps.encode(c)) // Did a new valid sentence come in?
        newData = true;
    }
  }

  if (newData){
    float flat, flon;
    unsigned long age;
    gps.f_get_position(&flat, &flon, &age);
   
    File dataFile = SD.open("log1.txt", FILE_WRITE);
    
    if (dataFile) {
      dataFile.print(flat, 6);
      dataFile.print("       ");
      dataFile.println(flon, 6);
      dataFile.close();
    }
    else {
      Serial.println("error ao abrir log1.txt");
    }
  }
}

void sensorLed() {
  int estado = analogRead(0);  //Lê o valor fornecido pelo LDR

  // Caso o valor lido na porta analógica seja maior do que
  // 800, acende o LED
  // Ajuste o valor abaixo de acordo com o seu circuito
  Serial.print("Sensor Led: ");
  Serial.print(estado);
  if (estado < 300) {
    digitalWrite(portaLed, HIGH);
    delay(100); //INTERVALO DE 500 MILISSEGUNDOS
    digitalWrite(portaLed, LOW);
    delay(100); //INTERVALO DE 500 MILISSEGUNDOS
  }
  else {
    digitalWrite(portaLed, LOW);
  }
}

