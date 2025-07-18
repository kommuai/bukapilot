#include <WebServer.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <ezButton.h>

#define POT_RY 33
#define POT_RX 32
#define BUTTON 4

//set up to connect to an existing network (e.g. mobile hotspot from laptop that will run the python code)
const char* ssid = "weedle";
const char* password = "joystick";
WiFiUDP Udp;
unsigned int localUdpPort = 1234;  //  port to listen on
char incomingPacket[255];  // buffer for incoming packets
int curVal = 0;

struct JoystickData {
  uint8_t pot_rx;
  uint8_t pot_ry;
  bool button_pressed;
} data;

ezButton button(BUTTON);

void setup()
{
  button.setDebounceTime(50); // set debounce time to 50 milliseconds

  int status = WL_IDLE_STATUS;
  Serial.begin(115200);
  WiFi.begin(ssid, password);
  Serial.println("");

  // Wait for connection
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("Connected to wifi");
  Udp.begin(localUdpPort);
}

void loop()
{
  button.loop(); // MUST call the loop() function first

  // once we know where we got the inital packet from, send data back to that IP address and port
  Udp.beginPacket("192.168.43.1", 1234);
  data.pot_ry = floor(analogRead(POT_RY) / 16);
  data.pot_rx = floor(analogRead(POT_RX) / 16);
  data.button_pressed = !button.getState();

  if (button.isPressed()) {
    Serial.println("Button pressed");
  }
  Serial.println("RY: " + String(data.pot_rx) + "  RX: " + String(data.pot_rx) + "  BUTTON: " + String(data.button_pressed));
  Udp.write(reinterpret_cast<uint8_t *>(&data), sizeof(data));
  Udp.endPacket();
  delay(1);
}
