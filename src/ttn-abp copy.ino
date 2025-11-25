#include <Arduino.h>
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include "TemperaturaUmidadeAr.h"
#include "UmidadeSolo.h"
#include "Anemometro.h"
#include "DirecaoVento.h"
#include "PostWifi.h"

#include <CayenneLPP.h>
#define RXD2 13
#define TXD2 14

#define cayenne 

#define ttn3_heltec_forte

//Define canais utilizados. Somente um por vez pode ser utilizado
#define canal_unico //Canal único 917 MHz usado no Heltec Single Gateway de sferrigo 

#ifdef ttn3_heltec_forte
  static const PROGMEM u1_t NWKSKEY[16] = { 0xB5, 0x38, 0x6E, 0x7F, 0x9D, 0xBC, 0x02, 0xEC, 0x59, 0x08, 0xF8, 0xFD, 0x07, 0xA1, 0xE5, 0x08 };
  static const u1_t PROGMEM APPSKEY[16] = { 0x37, 0x8A, 0x5B, 0x41, 0xC8, 0x01, 0xC0, 0x72, 0x77, 0x45, 0xCB, 0x5F, 0xBE, 0x77, 0x45, 0x0A };
  static const u4_t DEVADDR = 0x260C3398;
#endif

//Variáveis globais
//contador de quadros
int contador = 0;
//Usado para diferenciar par e ímpar
int resto = 0;
// Indicador de recebimento de dados da TTN
bool recebido = false;
//Variável que armazena os dados recebidos da TTN
String dados_recebidos;

void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

//Limite 51 bytes
int tamanho_vetor = 51;
#ifdef cayenne 
  CayenneLPP lpp(51);
  bool novoPayloadPronto = false;
  uint8_t payloadRecebido[51];
  uint8_t tamanhoPayload = 0;
#else
  static uint8_t  mydata[51];
#endif

static osjob_t sendjob;

const unsigned TX_INTERVAL = 10; //Padrão 60

// Pin mapping TTGO T-Beam v1.2 (AXP2101 + SX1276)
const lmic_pinmap lmic_pins = {
    .nss = 18,                   // CS / NSS
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 23,                   // Reset do SX1276
    .dio = {26, 33, 32}          // DIO0, DIO1, DIO2
};


void processJson(const String &jsonStr) {
    StaticJsonDocument<512> doc;
    DeserializationError error = deserializeJson(doc, jsonStr);

    if (error) {
      Serial.print("Erro no parse JSON: ");
      Serial.println(error.c_str());
      return;
    }
      lpp.reset();

    // Adiciona os campos (ajuste canais conforme quiser)
    int temperatura_ar;
    temperatura_ar      = doc["temperatura_ar"];
    String  direcao_vento = doc["direcao_vento"];
    int umidade_ar = doc["umidade_ar"];
    
    lpp.addTemperature(1, doc["temperatura_ar"]);
    lpp.addRelativeHumidity(2, doc["umidade_ar"]);
    lpp.addAnalogInput(3, doc["umidade_solo"]);
    lpp.addAnalogInput(4, doc["precipitacao"]);
    lpp.addDigitalInput(5, doc["chovendo"]);
    lpp.addAnalogInput(6, doc["velocidade_do_vento"]);
    lpp.addBarometricPressure(7, doc["pressao_do_ar"]);
    //lpp.addBarometricPressure(8, doc["pressao_altitude"]);
    lpp.addAnalogInput(9, doc["co"]);
    lpp.addAnalogInput(10, doc["nh3"]);
    lpp.addAnalogInput(11, doc["no2"]);
    lpp.addBarometricPressure(12, doc["co2"]);
   // lpp.addAnalogInput(13, doc["tensao_alimentacao"]);
    uint16_t tempoSegundos = doc["tempo_ligado"] ; // valor padrão
    lpp.addLuminosity(14, tempoSegundos / 1000.0);
    //lpp.
    // Mostra payload LPP
    Serial.print("");
    //Serial.print("Direção do Vento: ");
    //Serial.print(direcao_vento);
    Serial.print("Payload LPP (");
    Serial.print(lpp.getSize());
    Serial.println(" bytes):");
}

void do_send(osjob_t* j) {

  String myString;

  static String inputString = "";
  while (Serial1.available())
  {
    char c = Serial1.read();
    if (c != '\n') inputString += c;
      else
      {
        Serial.print("String formada: ");
        Serial.println(inputString);
        Serial.println("Mandando string...");
        processJson(inputString);
        inputString = "";
        //break;
    }
    if (!Serial1.available()) delay(100);
  }
  
  // Check if there is not a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND) {
    Serial.println(F("OP_TXRXPEND, not sending"));
  } else {
    resto = contador % 2;
    if (recebido){
      myString = dados_recebidos;
    }
    }
    //#ifdef cayenne
    LMIC_setTxData2(1, lpp.getBuffer(), lpp.getSize(), 0);
    Serial.println(); 
    Serial.println("Packet queued");
    Serial.print("TX nº: ");
    Serial.println(contador);
    #ifndef cayenne
      Serial.println((char*)mydata);
    #endif
    Serial.println(LMIC.freq);
    
       contador++;
    }
   
  
  
void onEvent (ev_t ev) {
  //sleep(5);
  Serial.print(os_getTime());
  Serial.print(": ");
  Serial.print("EV: ");
  Serial.println(ev);
  switch (ev) {
    case EV_SCAN_TIMEOUT:
      Serial.println("EV_SCAN_TIMEOUT");
      break;
    case EV_BEACON_FOUND:
      Serial.println("EV_BEACON_FOUND");
      //LMIC_sendAlive();
      break;
    case EV_BEACON_MISSED:
      Serial.println("EV_BEACON_MISSED");
      break;
    case EV_BEACON_TRACKED:
      Serial.println("EV_BEACON_TRACKED");
      break;
    case EV_JOINING:
      Serial.println("EV_JOINING");
      break;
    case EV_JOINED:
      Serial.println("EV_JOINED");
      //LMIC_setPingable(1);
      //Serial.println("SCANNING...");
      break;
    case EV_RFU1:
      Serial.println("EV_RFU1");
      break;
    case EV_JOIN_FAILED:
      Serial.println("EV_JOIN_FAILED");
      break;
    case EV_REJOIN_FAILED:
      Serial.println("EV_REJOIN_FAILED");
      break;
    case EV_TXCOMPLETE:
      Serial.println("EV_TXCOMPLETE (includes waiting for RX windows)");
      if (LMIC.dataLen) {
        // data received in rx slot after tx
        Serial.println("==========================================");
        Serial.print("Data Received: ");
        Serial.write(LMIC.frame + LMIC.dataBeg, LMIC.dataLen);
        //Armazena dados recebidos na variável dados_recebidos e seta
        // variável booleana de recebimento de dados da TTN para true 
        dados_recebidos = (char*) LMIC.frame + LMIC.dataBeg, LMIC.dataLen;
        Serial.println(dados_recebidos);
        recebido = true;
        Serial.println("==========================================");
        Serial.println();

      }else{
        Serial.println("==========================================");
        Serial.println("Nada recebido!");
        Serial.println("==========================================");
        Serial.println();
      }
      // Schedule next transmission
      os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send);
      Serial.println("Agendando nova transmissão...");
      break;
    case EV_LOST_TSYNC:
      Serial.println("EV_LOST_TSYNC");
      break;
    case EV_RESET:
      Serial.println("EV_RESET");
      break;
    case EV_RXCOMPLETE:
      // data received in ping slot
      Serial.println("EV_RXCOMPLETE");
      break;
    case EV_LINK_DEAD:
      Serial.println("EV_LINK_DEAD");
      break;
    case EV_LINK_ALIVE:
      Serial.println("EV_LINK_ALIVE");
      break;
    default:
      Serial.println("Unknown event");
      break;
  }
}

void setup() {

  //Inicialza pinagem luminosidade
  Serial.begin(115200); // Monitor
  Serial1.begin(115200, SERIAL_8N1, 13, 2); // UART2 (para receber JSON)

  os_init();

  LMIC_reset();
   // On AVR, these values are stored in flash and only copied to RAM
  #ifdef PROGMEM
  uint8_t appskey[sizeof(APPSKEY)];
  uint8_t nwkskey[sizeof(NWKSKEY)];
  memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
  memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
  LMIC_setSession (0x1, DEVADDR, nwkskey, appskey);
  #else
    // If not running an AVR with PROGMEM, just use the arrays directly
    LMIC_setSession (0x1, DEVADDR, NWKSKEY, APPSKEY);
  #endif

  //Disable link check validation
  LMIC_setLinkCheckMode(0);
  // TTN uses SF9 for its RX2 window.
  LMIC.dn2Dr = DR_SF9;
  // Set data rate and transmit power (note: txpow seems to be ignored by the library)
  LMIC_setDrTxpow(DR_SF9, 15); // Ver se GW está no 10; 14 é 14dBM
  #ifdef canal_unico
   //Deixa canal único
   for (int i = 0; i < 9; i++)
   {
    LMIC_disableChannel(i);  // only the first channel 902.3Mhz works now.
    Serial.println("Desabilitando canal ");
   }
   for (int i = 10; i < 64; i++)
   {
    LMIC_disableChannel(i);  // only the first channel 902.3Mhz works now.
    Serial.println("Desabilitando canal ");
   }
   #endif

//   // Start job
   do_send(&sendjob);
  
}

void loop()

{

   os_runloop_once();

}



