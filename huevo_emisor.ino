//Arduino UNO
//Pines: MPU-sda:A4,     MPU-scl:A5       MPU-vcc:5v
//Pines: RFtx-vcc:3,3v  RFtx-Data:4
//Pines: DHT11-v:5v     DHT11-data:7

#include <LowPower.h>
#include <avr/wdt.h>
//#include <Adafruit_SleepyDog.h>

#include <VirtualWire.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif
#include <SPI.h>//include the SPI bus library
#include <MFRC522.h>//include the RFID reader library



#define IRQ_PIN 3
#define SS_PIN 10  //slave select pin
#define RST_PIN 9  //reset pin
MFRC522 mfrc522(SS_PIN, RST_PIN);        // instatiate a MFRC522 reader object.
MFRC522::MIFARE_Key key;//create a MIFARE_Key struct named 'key', which will hold the card information

unsigned char regVal = 0x7F;
void activateRec(MFRC522 mfrc522);
void clearInt(MFRC522 mfrc522);


MPU6050 mpu;

//====================================================================
//==              Opciones deseadas de conversión                   ==
//====================================================================

// uncomment "OUTPUT_READABLE_YAWPITCHROLL" if you want to see the yaw/
// pitch/roll angles (in degrees) calculated from the quaternions coming
// from the FIFO. Note this also requires gravity vector calculations.
// Also note that yaw/pitch/roll angles suffer from gimbal lock (for
// more info, see: http://en.wikipedia.org/wiki/Gimbal_lock)

#define OUTPUT_READABLE_YAWPITCHROLL

// uncomment "OUTPUT_TEAPOT" if you want output that matches the
// format used for the InvenSense teapot demo
//#define OUTPUT_TEAPOT

//===========================================================================




#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)

#define SERIALPRINT

bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
//uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };


// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high

int pinAnalogRandom = 1; // este pin no debe estar conectado, lo usamos como semilla
const int ledDigitalPin = 13;

// RF
const int TX_DIO_Pin = 4;

// DHT22
//#include <dht.h>
//dht DHT;
//#define DHT22_PIN 8

//DHT11
#include "DHT.h"
#define DHTPIN 7 //Sensor t/h
#define DHTTYPE DHT11 
DHT dht(DHTPIN, DHTTYPE);

struct {
    uint32_t total;
    uint32_t ok;
    uint32_t crc_error;
    uint32_t time_out;
    uint32_t connect;
    uint32_t ack_l;
    uint32_t ack_h;
    uint32_t unknown;
} stat = { 0,0,0,0,0,0,0,0};

// CONFIG
char IDArduinoNodo[10] = "EGG";
int espera = 1;

void setup()
{ 
  wdt_enable(WDTO_4S);
  #ifdef SERIALPRINT
    Serial.begin(115200);
    Serial.print("\nIniciando ...\n");
  #endif
  
  //randomSeed(analogRead(pinAnalogRandom));
  dht.begin();
  vw_set_tx_pin(TX_DIO_Pin);
  vw_set_ptt_inverted(true);
  vw_setup(2000); // Bits per sec

  // LED visual indica encendido
  pinMode(ledDigitalPin, OUTPUT);
  blinkLed(1, 10);
  delay(1000);
  blinkLed(1, 10);
  
  wdt_reset();
  setupMPU(); //Inicializar giroscopio
  wdt_reset();
  RFIDsetup();  

}

void loop()
{
  wdt_reset();
  
  String cadena=""; // se dejan todos los parámetros formados "t:10,h:20.2,l:45,q:546"
  String temp = "";
  //int aleatorioProximaLlamada = random(0,5);  // 0_4

  //DHT11
    temp = floatToString(dht.readTemperature(),1);
    if (!temp.equals("")) 
      cadena = addCadena(cadena, "t:" + temp, ",");

    temp = floatToString(dht.readHumidity(),1);
    if (!temp.equals("")) 
      cadena = addCadena(cadena, "h:" + temp, ",");

  // DHT22
  //int chk = DHT.read22(DHT22_PIN);

  //if (chk == DHTLIB_OK) {
    //temp = floatToString(DHT.temperature,1);
    //if (!temp.equals("")) 
      //cadena = addCadena(cadena, "t:" + temp, ",");

    //temp = floatToString(DHT.humidity,1);
    //if (!temp.equals("")) 
      //cadena = addCadena(cadena, "h:" + temp, ",");
  //}
  
  // VCC
  long vcc = readVcc();
  char vccA[10]="";
  if (vcc>0) {
    sprintf(vccA,"%u",vcc);
    temp = String(vccA);
    cadena = addCadena(cadena, "v:" + temp, ",");
    
  }

  //RFID
    String mfr="";
    activateRec(mfrc522);
    if (mfrc522.PICC_ReadCardSerial()){      
      mfr = String(mfrc522.uid.uidByte[0], HEX);
      Serial.println(mfr);
      if (mfr=="75"){   //RFID Aguila 1
        mfr="1";
      }else{                
        if (mfr=="7d"){  //RFID Aguila 2
          mfr="2";      
        }else{
          mfr="-1";       //Error radiofrecuencia -1
        }
      }
     mfrc522.PICC_ReadCardSerial();  //Workaround ???
     }else{
      Serial.println("");
      mfr="0";
    }   
    cadena= addCadena( cadena,"ID:" + mfr, ",");

  //Giroscope
  String giroscope = loopMPU();
  cadena= addCadena( cadena, giroscope, ",");
  wdt_reset();

  char cadenaA[100]="";
  cadena.toCharArray(cadenaA, 100); // convertimos a char[] pues sprintf no admite String, admite %u (long) y %s (char[])
  char mensajeAEnviar[100]="";

  sprintf(mensajeAEnviar, "%s {%s}", IDArduinoNodo, cadenaA);
  #ifdef SERIALPRINT
    Serial.println(String(mensajeAEnviar));
  #endif
  
  // Enviamos por RF
  enviarString(mensajeAEnviar);

  #ifdef SERIALPRINT
    delay(100);
  #endif
  //activateRec(mfrc522);
  delay(100);
  // Próxima llamada
  pausar(espera);
}

void pausar(int seconds) {
  for (int i = 0; i < seconds; i++) {
    
    //LowPower.idle(SLEEP_4S, ADC_OFF, TIMER2_OFF, TIMER1_OFF, TIMER0_OFF, 
    //          SPI_OFF, USART0_OFF, TWI_OFF);
    LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
    wdt_reset();
    //Watchdog.sleep(8000);
    wdt_enable(WDTO_8S);
  }
}

// SLEEP_15MS, SLEEP_30MS, SLEEP_60MS, SLEEP_120MS, SLEEP_250MS, SLEEP_500MS, SLEEP_1S, SLEEP_2S, SLEEP_4S, SLEEP_8S, SLEEP_FOREVER

void blinkLed (int veces, unsigned long duracionMilisegundos) {
  int i;
  for (i = 1; i <= veces; i++) {
    delay (duracionMilisegundos);
    digitalWrite(ledDigitalPin, HIGH);
    delay (duracionMilisegundos);
    digitalWrite(ledDigitalPin, LOW);
  }
}
