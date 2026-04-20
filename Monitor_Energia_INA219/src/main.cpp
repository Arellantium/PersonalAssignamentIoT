#include <Wire.h>
#include <Adafruit_INA219.h>

Adafruit_INA219 ina219;

void setup(void) {
  Serial.begin(115200);
  while (!Serial) { delay(1); } // Attendi connessione seriale
    
  Serial.println("Inizializzazione Sensore INA219...");

  // Inizializza il sensore
  if (!ina219.begin()) {
    Serial.println("Errore: Impossibile trovare il chip INA219");
    while (1) { delay(10); }
  }
  
  // Opzionale: Calibrazione per misurare con più precisione correnti basse (es. 3.3V / 400mA)
  // ina219.setCalibration_32V_1A();
  
  Serial.println("INA219 Connesso! Inizio misurazioni...");
  Serial.println("----------------------------------------");
  Serial.println("Tempo(ms) | Tensione(V) | Corrente(mA) | Potenza(mW)");
}

void loop(void) {
  float busvoltage = 0;
  float current_mA = 0;
  float power_mW = 0;

  // Lettura dei valori
  busvoltage = ina219.getBusVoltage_V();
  current_mA = ina219.getCurrent_mA();
  power_mW = ina219.getPower_mW();

  // Stampa formattata per essere incollata facilmente in Excel per il report
  Serial.print(millis()); Serial.print(" | ");
  Serial.print(busvoltage); Serial.print(" | ");
  Serial.print(current_mA); Serial.print(" | ");
  Serial.println(power_mW);

  // Campiona ogni 100 millisecondi (10 Hz) per avere una curva ad alta risoluzione
  delay(100);
}