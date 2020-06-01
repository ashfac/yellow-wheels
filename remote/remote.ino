
#include <ESP8266WiFi.h>
#include <WiFiUDP.h>

const unsigned char packet_sof = ':';
const unsigned char packet_eof = ';';

#if 0
const char *wifi_ssid = "whyphy";
const char *wifi_password = "peshawar";
const char* server_ip = "192.168.2.58";
const int udp_port = 65002;
#else
const char *wifi_ssid = "yellow_wheels";
const char *wifi_password = "@$^peshawar#%&";
const char* server_ip = "192.168.4.173";
const int udp_port = 4836;
#endif

const int led_pin    = 5;
const int button_fwd = 12;
const int button_rev = 14;
const int pot_adc    = A0;

const int steering_left = 5;
const int steering_right = 80;

const unsigned long send_interval = 50;
unsigned long last_send_time = 0;

WiFiUDP Client;

enum MotorDirection { Stopped, Forward, Reverse };

void setup()
{
  // put your setup code here, to run once:
  Serial.begin(115200);

  pinMode(led_pin, OUTPUT);
  pinMode(button_fwd, INPUT_PULLUP);
  pinMode(button_rev, INPUT_PULLUP);

  WiFi.mode(WIFI_STA);
  WiFi.begin(wifi_ssid, wifi_password);  

  Serial.println();
  Serial.print("Wait for WiFi... ");
   
  while(WiFi.status() != WL_CONNECTED)
  {
    Serial.print(".");
    delay(500);
  }

  Serial.println();
  Serial.println("WiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());   
}

void loop()
{
  bool btn_fwd_pressed = (digitalRead(button_fwd) == LOW);
  bool btn_rev_pressed = (digitalRead(button_rev) == LOW);

  int steering = analogRead(pot_adc);
  steering = map(steering, 0, 1023, steering_right, steering_left);
  
  unsigned char steer_char[4];
  memcpy(steer_char, &steering, sizeof(steer_char));
  
  Client.beginPacket(server_ip, udp_port);
  Client.write(packet_sof);

  if( btn_fwd_pressed && !btn_rev_pressed )
  {
    Client.write(static_cast<unsigned char>(MotorDirection::Forward));
  }
  else if( !btn_fwd_pressed && btn_rev_pressed )
  {
    Client.write(static_cast<unsigned char>(MotorDirection::Reverse));
  }
  else
  {
    Client.write(static_cast<unsigned char>(MotorDirection::Stopped));
  }

  Client.write(steer_char, sizeof(steer_char));
  Client.write(packet_eof);
  Client.endPacket();
  delay(50);
}

