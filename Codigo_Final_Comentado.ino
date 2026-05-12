// Llibreries necessaries
#include <Wire.h>                // Llibreria per a la comunicació I2C (usada pel DAC i el sensor IR)
#include <Adafruit_MCP4725.h>     // Llibreria específica per controlar el DAC (convertidor digital-analògic)
#include <OneWire.h>              // Protocol de comunicació per a la sonda de temperatura digital
#include <DallasTemperature.h>    // Llibreria per gestionar la lectura de la sonda DS18B20
#include <Adafruit_MLX90614.h>    // Llibreria per al sensor de temperatura infraroig sense contacte

// Pins utilitzats
const int pinA = 2;               // Pin de la fase A de l'encoder (connectat a interrupció)
const int pinB_Enc = 3;           // Pin de la fase B de l'encoder per determinar la direcció
const int pinACS758 = A0;         // Entrada analògica per al sensor d'intensitat de corrent
const int pinDS18B20 = 4;         // Pin digital per a les dades de la sonda de temperatura ambient

// Constants
const float TEMP_MAX_SEGURETAT = 60.0; // Temperatura límit per aturar el sistema per seguretat
const int DAC_MIN = 819;               // Valor mínim de sortida del DAC aproximadament 1V
const int DAC_MAX = 3276;              // Valor màxim de sortida del DAC aproximadament 4V
const int DAC_70  = 2457;              // Valor del DAC per una velocitat constant del 70%
const int intervaloMs = 1000;          // Temps d'espera entre lectures de dades (1 segon)
const float TENSIO_BATERIA = 24.0;     // Tensió de les bateries pels càlculs de potència
const unsigned long DURACIO_TOTAL = 7200000; // Durada total de la prova fixada en 120 minuts en ms

// Objectes
Adafruit_MCP4725 dac;                  // Instància per controlar el xip DAC
Adafruit_MLX90614 irMotor = Adafruit_MLX90614(); // Instància per al sensor de temperatura IR
OneWire oneWire(pinDS18B20);           // Configuració del bus OneWire per a la sonda Dallas
DallasTemperature sonda(&oneWire);     // Instància per gestionar la sonda de temperatura ambient

// Variables
volatile long contadorPulsos = 0;      // Comptador de polsos de l'encoder
float offsetVoltaje = 0.0;             // Voltatge de referència del sensor de corrent a 0 Ampers
unsigned long tempsIniciAssaig = 0;    // Desa el moment exacte de l'inici de la prova
unsigned long tiempoProximaLectura = 0; // Controla quan toca fer la següent lectura de dades
int tipusAssaig = 0;                   // Selecció del tipus d'assaig
bool assaigActiu = false;              // Estat que indica si un assaig està en marxa o no

// Declaracions
void mostrarMenu();                    // Funció per imprimir per pantalla les opcions de l'assaig
void iniciarAssaig();                  // Funció per configurar l'iniciar l'assaig
void finalitzarAssaig();               // Funció per aturar l'assaig
void calibrarSensor();                 // Funció per establir el zero del sensor d'intensitat
bool actualitzarMotor(unsigned long t); // Funció per canviar el voltatge del Porter10 i canviar la velocitat del motor
void executarLectures(unsigned long tRelatiu, float tMotor); // Funció per enviar dades CSV
float estimarEficiencia(float rpm);    // Funció per calcular l'eficiència segons les RPM's 

// Funció que s'executa a cada pols detectat per l'encoder
void conteoEncoder() {
  if (digitalRead(pinB_Enc) == HIGH) contadorPulsos++; // Si el pin B és alt, el gir és positiu
  else contadorPulsos--;                               // Si el pin B és baix, el gir és negatiu
}

// Interpolació de l'eficiència segons la corba del motor elèctric. Dades extretes del "PErformance Graph" del motor
float estimarEficiencia(float rpm) {
  if (rpm <= 0 || rpm >= 2050) return 0.0;             
  if (rpm >= 1850) return map(rpm, 1850, 2050, 74, 0) / 100.0;  
  if (rpm >= 1500) return map(rpm, 1500, 1850, 60, 74) / 100.0;  
  if (rpm >= 1000) return map(rpm, 1000, 1500, 40, 60) / 100.0;  
  if (rpm >= 500)  return map(rpm, 500, 1000, 20, 40) / 100.0;   
  return map(rpm, 0, 500, 0, 25) / 100.0;              
}

void setup() {
  Serial.begin(115200);                // Inicia la comunicació a 115200 baudios
  delay(1000);                         // Espera d'un segon per estabilitzar el sistema

  dac.begin(0x60);                     // Connecta amb el DAC a l'adreça I2C per defecte
  dac.setVoltage(DAC_MIN, false);      // Posa el motor al valor mínim per seguretat (aproximadament 1V)

  irMotor.begin();                     // Inicialitza el sensor infraroig de temperatura
  sonda.begin();                       // Inicialitza la sonda de temperatura ambiental

  pinMode(pinA, INPUT_PULLUP);         // Configura l'entrada de l'encoder amb resistència pull-up
  pinMode(pinB_Enc, INPUT_PULLUP);     // Configura la fase B amb resistència pull-up
  attachInterrupt(digitalPinToInterrupt(pinA), conteoEncoder, RISING); // Activa la lectura de polsos

  calibrarSensor();                    // Estableix el punt zero del sensor de corrent
  mostrarMenu();                       // Mostra el menú d'opcions a l'usuari per pantalla
}

void loop() {
  if (Serial.available() > 0) {        // Si hi ha entrada per la consola sèrie
    String entrada = Serial.readStringUntil('\n'); // Llegeix l'entrada fins al salt de línia
    entrada.trim();                    // Elimina espais o caràcters buits

    if (entrada == "ATURA") {          // Si l'usuari escriu "ATURA"
      finalitzarAssaig();              // Atura immediatament la prova
      return;                          // Surt del loop actual
    }

    if (!assaigActiu) {                // Si no hi ha cap prova en curs
      if (entrada == "1" || entrada == "2" || entrada == "3") { // Si tria un assaig vàlid
        tipusAssaig = entrada.toInt(); // Desa el tipus d'assaig triat
        Serial.print(F("\n>>> Assaig ")); // Imprimeix confirmació
        Serial.print(tipusAssaig);
        Serial.println(F(" triat. Calibrant i preparant dades..."));
        iniciarAssaig();               // Prepara i arrenca el cronòmetre de la prova
      }
    }
  }

  if (assaigActiu) {                   // Si estem en mode de prova actiu
    unsigned long tiempoActualReal = millis() - tempsIniciAssaig; // Calcula el temps transcorregut

    if (tiempoActualReal >= tiempoProximaLectura) { // Cada vegada que passa 1 segon
      if (actualitzarMotor(tiempoActualReal)) {     // Actualitza el estat del motor si retorna true, la prova ha acabat
        finalitzarAssaig();            // Finalitza l'assaig automàticament
      } else {
        float tMotor = irMotor.readObjectTempC(); // Llegeix la temperatura del motor

        if (tMotor >= TEMP_MAX_SEGURETAT) {         // Si se supera el límit tèrmic
          Serial.println(F("!!! EMERGÈNCIA: SOBREESCALFAMENT !!!")); // Alerta de seguretat
          finalitzarAssaig();          // Atura el motor per evitar danys
        } else {
          executarLectures(tiempoProximaLectura, tMotor); // Registra dades en format CSV
          tiempoProximaLectura += intervaloMs;          // Programa la propera lectura dintre d'1 segon
        }
      }
    }
  }
}

// Funció per fer-li el zero al sensor d'intensitat ACS758
void calibrarSensor() {
  long sumaCal = 0;
  for (int i = 0; i < 100; i++) {      // Fa una mitjana de 100 lectures analògiques
    sumaCal += analogRead(pinACS758);
    delay(2);                          // Petita espera entre lectures
  }
  offsetVoltaje = ((float)sumaCal / 100.0 * 5.0) / 1023.0; // Converteix la mitjana a voltatge
  Serial.print(F("Zero del sensor fixat a: ")); // Mostra el resultat de la calibració
  Serial.print(offsetVoltaje, 3);
  Serial.println(F(" V"));
}

// Controla el voltatge enviat per la DAC al Porter10 segons el mode d'assaig triat
bool actualitzarMotor(unsigned long t) {
  if (t >= DURACIO_TOTAL) return true; // Si hem arribat als 20 minuts, avisa que cal finalitzar

  switch (tipusAssaig) {
    case 1: // Mode constant: accelera al 70% i manté velocitat durant tota la provva
      dac.setVoltage(DAC_70, false);
      break;

    case 2: { // Mode rampa: puja i baixa la velocitat cada 2 minuts (120 segons)
      unsigned long tCicle = t % 120000;
      if (tCicle <= 60000) { // Primera meitat del cicle: rampa ascendent
        dac.setVoltage(map(tCicle, 0, 60000, DAC_MIN, DAC_MAX), false);
      } else { // Segona meitat del cicle: rampa descendent
        dac.setVoltage(map(tCicle, 60000, 120000, DAC_MAX, DAC_MIN), false);
      }
      break;
    }

    case 3: { // Mode intermitent: alterna la velocitat entre la velocitat màxima i mínima cada 30 segons
      unsigned long tCicle = t % 60000;
      if (tCicle < 30000) { // Primers 30 segons: velocitat al màxim
        dac.setVoltage(DAC_MAX, false);
      } else { // Següents 30 segons: velocitat al mínim 
        dac.setVoltage(DAC_MIN, false);
      }
      break;
    }
  }
  return false;                        // L'assaig encara continua
}

// Llegeix els sensors i envia dades en format CSV
void executarLectures(unsigned long tRelatiu, float tMotor) {
  // Càlcul de Revolucions Per Minut (RPM)
  noInterrupts();                      // Atura interrupcions per llegir el comptador de forma segura
  long p = contadorPulsos;
  contadorPulsos = 0;                  // Reinicia el comptador per al següent interval
  interrupts();                        // Torna a activar interrupcions
  float rpm = abs(p * 60.0 / 360.0);   // Converteix polsos/segon a voltes/minut (encoder de 360 polsos segons especificacions tècniques)

  // Temperatura ambient
  sonda.requestTemperatures();         // Sol·licita la conversió de temperatura a la sonda
  float tAmb = sonda.getTempCByIndex(0); // Llegeix el valor en graus Celsius

  // Temperatura carcassa: mitjana de 5 lectures per aprofitar la resolució del MLX90614 (2 decimals)
  float sumaTMotor = 0;
  for (int i = 0; i < 5; i++) {
    sumaTMotor += irMotor.readObjectTempC();
  }
  float tMotorFi = sumaTMotor / 5.0;

  // Intensitat de corrent: mitjana de 10 lectures analògiques del sensor Hall (per evitar les variacions, es fa la mitja amb 10 lectures)
  long sumaACS = 0;
  for (int i = 0; i < 10; i++) {
    sumaACS += analogRead(pinACS758);
    delay(2);
  }
  float vActual = (sumaACS / 10.0 * 5.0) / 1023.0; // Converteix la lectura analògica a voltatge
  float amps = abs((vActual - offsetVoltaje)) / 0.020; // Aplica la sensibilitat del sensor (20mV/A) 
  if (amps < 0.18) amps = 0.0;         // Filtre de soroll per evitar falsos consums en repòs

  // Càlcul de la Potència elèctrica real consumida
  float potencia = TENSIO_BATERIA * amps;

  // Càlcul del parell estimat fent servir l'eficiència segons la corba del Performance Graph
  float parell = 0.0;
  if (rpm > 0) {
    float eficiencia = estimarEficiencia(rpm); // Obté el rendiment teòric per a aquestes RPM
    float velocitat_angular = rpm * 2.0 * PI / 60.0; // Converteix RPM a radians per segon
    parell = (potencia * eficiencia) / velocitat_angular; // Parell = P_mecànica / velocitat_angular
  }

  // Impressió de dades en format CSV per a la seva posterior anàlisi 
  Serial.print(tRelatiu);    Serial.print(","); // Temps des de l'inici
  Serial.print(rpm, 0);      Serial.print(","); // RPM 
  Serial.print(tMotorFi, 2); Serial.print(","); // Temperatura del motor (2 decimals)
  Serial.print(tAmb, 1);     Serial.print(","); // Temperatura ambient
  Serial.print(amps, 2);     Serial.print(","); // Intensitat (A)
  Serial.print(potencia, 1); Serial.print(","); // Potència (W)
  Serial.println(parell, 3);                   // Parell (Nm) amb 3 decimals
}

// Imprimeix informació dels assajos configurats
void mostrarMenu() {
  Serial.println(F("\n--- BANC DE PROVES ---"));
  Serial.println(F("1. Constant 70% - 20 min"));
  Serial.println(F("2. Cicles rampa 1V->4V->1V (2 min) - 20 min"));
  Serial.println(F("3. Cicles 30s ON / 30s OFF - 20 min"));
  Serial.println(F("Escriu ATURA per aturar en qualsevol moment"));
  Serial.println(F("Tria opcio (1, 2 o 3):"));
}

// Prepara les variables per començar a registrar un nou assaig
void iniciarAssaig() {
  calibrarSensor();                    // Recalibra el zero del sensor de corrent
  assaigActiu = true;                  // Manera de saber si hi ha un assaig en curs
  tempsIniciAssaig = millis();         // Guarda l'hora d'inici
  tiempoProximaLectura = 0;            // Reseteja el rellotge de lectures
  // Imprimeix la capçalera de les columnes del fitxer CSV
  Serial.println(F("Temps(ms),RPM,T_Motor(C),T_Amb(C),Intensitat(A),Potencia_elec(W),Parell_est(Nm)"));
}

// Atura el motor i deixa d'enregistrar daddes
void finalitzarAssaig() {
  dac.setVoltage(DAC_MIN, false);      // Frena el motor posant la velocitat al mínim primer
  assaigActiu = false;                 // Atura la variable que indica si hi ha un assaig actiu
  Serial.println(F("--- ASSAIG FINALITZAT ---")); // Missatge de final d'assaig
  mostrarMenu();                       // Torna a mostrar el menú per si es vol fer qualsevol altre assaig
 } 