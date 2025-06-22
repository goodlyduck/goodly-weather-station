# PubSubClient library for MQTT
https://github.com/knolleary/pubsubclient

This library provides a client for doing simple publish/subscribe messaging with a server that supports MQTT.

## Installation

1. Open Arduino IDE
2. Go to Sketch > Include Library > Manage Libraries...
3. Search for "PubSubClient"
4. Install the library by Nick O'Leary

## Usage

- Include <PubSubClient.h> in your sketch
- Create a WiFiClient and PubSubClient object
- Set the MQTT server IP and port
- Use client.connect(), client.publish(), client.subscribe(), etc.

## Example

```cpp
#include <WiFi.h>
#include <PubSubClient.h>

WiFiClient espClient;
PubSubClient client(espClient);

void setup() {
  client.setServer("192.168.1.184", 1883);
}

void loop() {
  if (!client.connected()) {
    // reconnect logic
  }
  client.loop();
}
```
