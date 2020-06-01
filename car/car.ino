
#include <ESP8266WiFi.h>
#include <WiFiUDP.h>
#include <Servo.h>

const unsigned char packet_sof = ':';
const unsigned char packet_eof = ';';

IPAddress local_ip(192,168,4,173);
IPAddress gateway(192,168,4,9);
IPAddress subnet(255,255,255,0);

const char *wifi_ssid = "yellow_wheels";
const char *wifi_password = "@$^peshawar#%&";
unsigned int local_udp_port = 4836;

const int motor_speed_min = 750;
const int motor_speed_forward = 600;
const int motor_speed_reverse = 600;
const int motor_start_pulse_ms = 20;

const int speed_pot = A0;

const int led_pin = D4;

const int motor_c1 = D1;
const int motor_c2 = D2;

const int motor_d1 = D5;
const int motor_d2 = D6;

const int servo_pin = D8;

const int steering_left = 5;
const int steering_center = 34;
const int steering_right = 80;

unsigned char incoming_packet[255];  // buffer for incoming packets
char  replyPacket[] = "OK";

const unsigned long comms_timeout = 150;
unsigned long last_time = 0;
bool motor_running = false;

WiFiUDP Server;
Servo steering_servo;

enum MotorDirection { Stopped, Forward, Reverse };

static MotorDirection motor_direction;
static MotorDirection current_direction = MotorDirection::Stopped;
static int steering = steering_center;

void stop_motor();
void drive_forward(int motor_speed = motor_speed_forward);
void drive_reverse(int motor_speed = motor_speed_reverse);
void turn_steering(int steering_value);

bool parse_packet( unsigned char* packet,
                   int packet_size,
                   MotorDirection& direction,
                   int& steering);

void setup()
{
  Serial.begin(115200);
  Serial.println();

  pinMode(led_pin, OUTPUT);
  digitalWrite(led_pin, HIGH);
  
  pinMode(motor_c1, OUTPUT);
  pinMode(motor_d1, OUTPUT);
  pinMode(motor_c2, OUTPUT);
  pinMode(motor_d2, OUTPUT);
  
  digitalWrite(motor_c1, LOW);
  digitalWrite(motor_d1, LOW);
  digitalWrite(motor_c2, LOW);
  digitalWrite(motor_d2, LOW);

  steering_servo.attach(servo_pin);
  
  steering_servo.write(steering_center);

  WiFi.mode(WIFI_AP);
  
  Serial.print("Setting soft-AP configuration ... ");
  Serial.println(WiFi.softAPConfig(local_ip, gateway, subnet) ? "Ready" : "Failed!");

  Serial.print("Setting soft-AP ... ");
  Serial.println(WiFi.softAP(wifi_ssid, wifi_password) ? "Ready" : "Failed!");

  Server.begin(local_udp_port);
  Serial.println("Server started...");

  Serial.printf("Now listening at IP %s, UDP port %d\r\n", WiFi.softAPIP().toString().c_str(), local_udp_port);
}

void loop()
{
  int packetSize = Server.parsePacket();
  
  if (packetSize)
  {
    // receive incoming UDP packets
    int packet_length = Server.read(incoming_packet, 255);

    if(parse_packet(incoming_packet, packet_length, motor_direction, steering))
    {
      process_motor_direction(motor_direction);
      turn_steering(steering);
    }

    last_time = millis();
  }
  else if (motor_running)
  {
    if( (millis() - last_time) > comms_timeout )
    {
      Serial.println("Comms timeout");
      stop_motor();
    }
  }
}

void stop_motor()
{
  digitalWrite(motor_c1, LOW);
  digitalWrite(motor_d1, LOW);
  digitalWrite(motor_c2, LOW);
  digitalWrite(motor_d2, LOW);

  motor_running = false;
  Serial.println("Motor stopped!");
}

void drive_forward(int motor_speed)
{
  digitalWrite(motor_c1, LOW);
  digitalWrite(motor_d1, LOW);
  digitalWrite(motor_c2, HIGH);
  digitalWrite(motor_d2, HIGH);
  delay(motor_start_pulse_ms);

  analogWrite(motor_c1, motor_speed);
  analogWrite(motor_d1, motor_speed);

  Serial.println("Driving forward");

  motor_running = true;
}

void drive_reverse(int motor_speed)
{
  digitalWrite(motor_c1, HIGH);
  digitalWrite(motor_d1, HIGH);
  digitalWrite(motor_c2, LOW);
  digitalWrite(motor_d2, LOW);
  delay(motor_start_pulse_ms);

  analogWrite(motor_c2, motor_speed);
  analogWrite(motor_d2, motor_speed);

  Serial.println("Driving reverse");
  
  motor_running = true;
}

void turn_steering(int steering_value)
{
  if (steering_value >= steering_left && steering_value <= steering_right)
  {
    steering_servo.write(steering_value);
  }
}

bool parse_packet( unsigned char* packet,
                   int packet_length,
                   MotorDirection& direction,
                   int& steering )
{
#if 0
  Serial.print("packet: ");
  for(int j=0; j<packet_length; j++)
  {
    Serial.printf("%02X ", packet[j]);
  }
  Serial.println();
#endif

  int i;

  for( i = 0; i < packet_length; i++)
  {
    if(packet[i] == packet_sof)
    {
      i++;
      break;
    }
  }

  direction = static_cast<MotorDirection>(packet[i]);
  i++;

  memcpy(&steering, &packet[i], 4);

  return true;
}

void process_motor_direction(MotorDirection motor_direction)
{
  switch(motor_direction)
  {
    case MotorDirection::Forward:
    {
      if(!motor_running || current_direction != MotorDirection::Forward)
      {
        current_direction = MotorDirection::Forward;
        drive_forward(map(analogRead(speed_pot), 0, 1023, 0, motor_speed_min));
      }
      break;
    }

    case MotorDirection::Reverse:
    {
      if(!motor_running || current_direction != MotorDirection::Reverse)
      {
        current_direction = MotorDirection::Reverse;
        drive_reverse(map(analogRead(speed_pot), 0, 1023, 0, motor_speed_min));
      }
      break;
    }

    default:
    {
      if(motor_running)
      {
        stop_motor();
      }

      current_direction = MotorDirection::Stopped;
    }
  }
}

