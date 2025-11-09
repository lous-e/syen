/*
  firmware.ino
  --------------
  Small Particle firmware that exposes a cloud function to publish
  a message as an event. Kept minimal; no behavioural changes made.
*/

#include "Particle.h"
SYSTEM_MODE(AUTOMATIC);
SYSTEM_THREAD(ENABLED);

// Call this function remotely with just a message like "Hello!"
int smsHandler(String msg) {
    // Directly publish the message as the particle event value
    return Particle.publish("sms_send", msg, PRIVATE) ? 1 : 0;
}

void setup() {
    Particle.function("sms", smsHandler);
}

void loop() {}
