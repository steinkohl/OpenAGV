#include <Arduino.h>
#include <mcp_can.h>
#include <SPI.h>



#define TIMEOUT 50000
#define CH1_START 1200
#define CH1_STOP 1800 
#define CH2_START 1100
#define CH2_STOP 2000
#define CH3_START 1100
#define CH3_STOP 1900
#define CH4_START 1200
#define CH4_STOP 1850
#define CH5_START 900
#define CH5_STOP 2050
#define CH6_START 900
#define CH6_STOP 2100

int ch1 = A0;
int ch2 = A1;
int ch3 = A2;
int ch4 = A3;
int ch5 = A4;
int ch6 = A5;



// set up can
// speed msg id including wheels
#define CAN_ID_SETSPEED 100
// config msg id for each wheel
#define CAN_ID_CONFIG_FR 110
#define CAN_ID_CONFIG_FL 111
#define CAN_ID_CONFIG_BR 112
#define CAN_ID_CONFIG_BL 113

// the cs pin of the version after v1.1 is default to D9
// v0.9b and v1.0 is default D10
#define SPI_CS_PIN 10

// Set CS pin
MCP_CAN CAN(SPI_CS_PIN);                                    


struct config_msg {
  int16_t Kp;
  int16_t Ki;
  int16_t Kd;
  bool pid_enable;
  bool reverse_direction;
};

typedef union {
  struct speeds {
    // int16_t        -->   -32768..32767
    // -20000..20000  -->   -2000..2000mm/s or -100..100%
    // other          -->   disable motor and driver
    int16_t fr;
    int16_t fl;
    int16_t br;
    int16_t bl;
  } speed_vals;
  int16_t wheel[4];
} speed_msg;

typedef union {
  config_msg config;
  speed_msg speed;
  uint8_t bytes[8];
} can_msg;

can_msg msg;

double doubleFromInt16(int16_t val){
  return ((double) val) / 1000.0;
}

int16_t int16FromDouble(double val){
  return (int16_t) (val * 1000);
}

void printSpeeds() {
  Serial.print("Speeds: (FR: ");
  Serial.print(msg.speed.speed_vals.fr/10.0);
  Serial.print(", FL: ");
  Serial.print(msg.speed.speed_vals.fl/10.0);
  Serial.print(", BR: ");
  Serial.print(msg.speed.speed_vals.br/10.0);
  Serial.print(", BL: ");
  Serial.print(msg.speed.speed_vals.bl/10.0);
  Serial.println(")mm/s");
}

void printMsgConfStats(){
  Serial.print("\nSet KP to ");
  Serial.println(doubleFromInt16(msg.config.Kp));
  Serial.print("Set KI to ");
  Serial.println(doubleFromInt16(msg.config.Ki));
  Serial.print("Set KD to ");
  Serial.println(doubleFromInt16(msg.config.Kd));
  Serial.print("Set pid_enable: ");
  Serial.println(msg.config.pid_enable);
  Serial.print("Set reverse_direction: ");
  Serial.println(msg.config.reverse_direction);
}

void setup() 
{
  Serial.begin(115200);

  while (CAN_OK != CAN.begin(CAN_500KBPS)) 
  {            
    // init can bus : baudrate = 500k
    Serial.print("CAN BUS Shield init fail: ");
    Serial.println(CAN.begin(CAN_500KBPS));
    delay(100);
  }
  Serial.println("CAN BUS Shield init ok!\n");

  Serial.println("Configure parameter via CAN");

  msg.config.Kp = int16FromDouble(0.05);
  msg.config.Ki = int16FromDouble(1.0);
  msg.config.Kd = int16FromDouble(0.0);
  msg.config.pid_enable = true;
  msg.config.reverse_direction = false;
  printMsgConfStats();
  Serial.print("Send data to ");
  Serial.println(CAN_ID_CONFIG_FR);
  CAN.sendMsgBuf(CAN_ID_CONFIG_FR, 0, 8, msg.bytes);

  msg.config.reverse_direction = true;
  printMsgConfStats();
  Serial.print("Send data to ");
  Serial.println(CAN_ID_CONFIG_FL);
  CAN.sendMsgBuf(CAN_ID_CONFIG_FL, 0, 8, msg.bytes);

 
  msg.config.reverse_direction = false;
  printMsgConfStats();
  Serial.print("Send data to ");
  Serial.println(CAN_ID_CONFIG_BR);
  CAN.sendMsgBuf(CAN_ID_CONFIG_BR, 0, 8, msg.bytes);

  msg.config.reverse_direction = true;
  printMsgConfStats();
  Serial.print("Send data to ");
  Serial.println(CAN_ID_CONFIG_BL);
  CAN.sendMsgBuf(CAN_ID_CONFIG_BL, 0, 8, msg.bytes);

  
  Serial.println("\nDone! - Start loop\n\n");
  //delay(5000);

  pinMode(ch1, INPUT);
  pinMode(ch2, INPUT);
  pinMode(ch3, INPUT);
  pinMode(ch4, INPUT);
  pinMode(ch5, INPUT);
  pinMode(ch6, INPUT);
}
int sign(int value, int maxvalue){
  if(value > maxvalue){
    return maxvalue;
  }
  if(value < -maxvalue){
    return -maxvalue;
  }
  return value;
}

int zeroMin(int value, int minValue){
  if(value < minValue && value > -minValue) return 0;
  return value;
}

void loop() {
  //int val =0;
  //for (int i=0; i<40; i++) val += (analogRead(A5)-512);
  //if (val < 500 && val > -500) val = 0;
  int16_t speed_x = zeroMin(map(pulseIn(ch4, HIGH, TIMEOUT),CH4_START, CH4_STOP, -10000, 10000), 500);
  int16_t speed_y = zeroMin(map(pulseIn(ch2, HIGH, TIMEOUT),CH2_START, CH2_STOP, 10000, -10000), 500);
  int16_t speed_a = zeroMin(map(pulseIn(ch3, HIGH, TIMEOUT),CH3_START, CH3_STOP, -10000, 10000), 500);
  msg.speed.speed_vals.fr = sign(speed_x + speed_y - speed_a, 20000);
  msg.speed.speed_vals.fl = sign(speed_x - speed_y + speed_a, 20000);
  msg.speed.speed_vals.br = sign(speed_x - speed_y - speed_a, 20000);
  msg.speed.speed_vals.bl = sign(speed_x + speed_y + speed_a, 20000);
  printSpeeds();
  CAN.sendMsgBuf(CAN_ID_SETSPEED, 0, 8, msg.bytes);
  //delay(10);                       // send data per 100ms
}

// END FILE