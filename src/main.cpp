#include <WiFi.h>
#include <Wire.h>
#include <SPI.h>

#define POSITION 0
#define VELOCITY 1
#define DISPLACEMENT 2
#define DIRECT_PWM 3
#define maxPWM 500

static const int spiClk = 1000000; // 1 MHz

const int ss_n = 9;

float Kp = 0.1, Kd = 0.01, err = 0, err_prev = 0, myo_setpoint = 0, result = 0;
int control_mode = POSITION;

int32_t position = 0;
int16_t velocity = 0, current = 0, pwmRef = 0;

const char* ssid = "roboy";
const char* password = "wiihackroboy";

IPAddress server(192, 168, 0, 120);
IPAddress ip_address;
int status = WL_IDLE_STATUS;

void myobrickLoop() {

  SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE1));
  uint16_t data[2];
  for(int i=0;i<12;i++){
    digitalWrite(ss_n,LOW);
    delayMicroseconds(1);
    if(i==0)
      SPI.transfer16(0x8000); // header
    else if(i==1)
      SPI.transfer16((pwmRef& 0x7fff));
    else{
      switch(i){
        case 4:
          data[0] = SPI.transfer16(0);
          break;
        case 5:
          data[1] = SPI.transfer16(0);
          position =  ((data[0]>>8)<<24|(data[0]&0xff)<<16|(data[1]>>8)<<8|(data[1]&0xff));
          break;
        case 6:
          velocity = SPI.transfer16(0);
          break;
        case 7:
          current = SPI.transfer16(0);
          break;
        default:
          SPI.transfer16(0);
         break;
       }
    }
    digitalWrite(ss_n,HIGH);
  }
  SPI.endTransaction();

  // controller
  switch(control_mode){
    case POSITION:
      err = myo_setpoint-position;
      break;
    case VELOCITY:
      err = myo_setpoint-velocity;
      break;
    case DISPLACEMENT: // not implemented
      err = 0;
      break;
    case DIRECT_PWM:
      result = myo_setpoint;
      break;
  }
  if(control_mode!=DIRECT_PWM){
    result = Kp*err + Kd*(err_prev-err);
    err_prev = err;
  }
  if(result > maxPWM){
    result = maxPWM;
  }
  if(result < -maxPWM){
    result = -maxPWM;
  }
  pwmRef = result;
}

void setupWiFi()
{
  WiFi.begin(ssid, password);

  Serial.print("\nConnecting to "); Serial.println(ssid);
  uint8_t i = 0;
  while (WiFi.status() != WL_CONNECTED && i++ < 20) delay(500);
  if(i == 21){
    Serial.print("Could not connect to"); Serial.println(ssid);
    while(1) delay(500);
  }
  Serial.print("Ready! Use ");
  Serial.print(WiFi.localIP());
  Serial.println(" to access client");
}

void setup() {
  Wire.begin();
  Wire.setClock(400000);
  Serial.begin(115200);
  setupWiFi();
  delay(2000);

  // myobrick
  SPI.begin();

  pinMode (ss_n, OUTPUT);
  digitalWrite(ss_n,HIGH);
}

int i = 0;

void loop() {
  myobrickLoop();
  if((i++%1000)==0){
    Serial.print("\tpwm:\t");
    Serial.print(pwmRef);
    Serial.print("\tpos:\t");
    Serial.print(position);
    Serial.print("\tvel:\t");
    Serial.print(velocity);
    Serial.print("\tcur:\t");
    Serial.println(current);
  }
}
