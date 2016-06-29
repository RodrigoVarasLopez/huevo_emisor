//Arduino UNO
//Pines: MPU-sda:A4,     MPU-scl:A5       MPU-vcc:5v
//Pines: RFtx-vcc:3,3v  RFtx-Data:4
//Pines: DHT11-v:5v     DHT11-data:7
//MPU
#include <VirtualWire.h>
#include <avr/wdt.h>
#include "LowPower.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 mpu;

//====================================================================
//==              Opciones deseadas de conversión                   ==
//====================================================================

// uncomment "OUTPUT_READABLE_QUATERNION" if you want to see the actual
// quaternion components in a [w, x, y, z] format (not best for parsing
// on a remote host such as Processing or something though)
//#define OUTPUT_READABLE_QUATERNION

// uncomment "OUTPUT_READABLE_EULER" if you want to see Euler angles
// (in degrees) calculated from the quaternions coming from the FIFO.
// Note that Euler angles suffer from gimbal lock (for more info, see
// http://en.wikipedia.org/wiki/Gimbal_lock)
//#define OUTPUT_READABLE_EULER

// uncomment "OUTPUT_READABLE_YAWPITCHROLL" if you want to see the yaw/
// pitch/roll angles (in degrees) calculated from the quaternions coming
// from the FIFO. Note this also requires gravity vector calculations.
// Also note that yaw/pitch/roll angles suffer from gimbal lock (for
// more info, see: http://en.wikipedia.org/wiki/Gimbal_lock)

#define OUTPUT_READABLE_YAWPITCHROLL

// uncomment "OUTPUT_READABLE_REALACCEL" if you want to see acceleration
// components with gravity removed. This acceleration reference frame is
// not compensated for orientation, so +X is always +X according to the
// sensor, just without the effects of gravity. If you want acceleration
// compensated for orientation, us OUTPUT_READABLE_WORLDACCEL instead.
//#define OUTPUT_READABLE_REALACCEL

// uncomment "OUTPUT_READABLE_WORLDACCEL" if you want to see acceleration
// components with gravity removed and adjusted for the world frame of
// reference (yaw is relative to initial orientation, since no magnetometer
// is present in this case). Could be quite handy in some cases.
//#define OUTPUT_READABLE_WORLDACCEL

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
char IDArduinoNodo[10] = "ROD1";
int segundos_espera = 1;

void setup()
{ 
  #ifdef SERIALPRINT
    Serial.begin(115200);
    Serial.print("\nIniciando ...\n");
  #endif

  wdt_enable(WDTO_8S);
  randomSeed(analogRead(pinAnalogRandom));
  dht.begin();
  vw_set_tx_pin(TX_DIO_Pin);
  vw_set_ptt_inverted(true);
  vw_setup(2000); // Bits per sec

  // LED visual indica encendido
  pinMode(ledDigitalPin, OUTPUT);
  blinkLed(1, 10);

  pausar(1); // Pausado 2s 
  blinkLed(1, 10);

  setupMPU(); //Inicializar giroscopio
}

void loop()
{
  wdt_reset();
  String giroscope = loopMPU();
  String cadena=""; // se dejan todos los parámetros formados "t:10,h:20.2,l:45,q:546"
  String temp = "";
  int aleatorioProximaLlamada = random(0,5);  // 0_4

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


  //Giroscope
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

  // Próxima llamada
  int espera = segundos_espera + aleatorioProximaLlamada;
  pausar(espera);
}

void pausar(int seconds) {
  for (int i = 0; i < seconds; i++) {
    //Serial.println("Dormido");
    LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
    wdt_reset();
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