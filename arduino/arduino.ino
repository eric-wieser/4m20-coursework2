#include "Robot.h"

#include "PacketSerial/src/PacketSerial.h"

#include "messages.h"

PacketSerial packet_serial;
Robot *robot;

void setupRobot() {
  // Define where the pins are connected
  static Robot r = {
    Joint(3, 0),
    Joint(5, 1),
    Joint(6, 2),
  };
  robot = &r;
}

void sendReadings() {
  messages::Framed<messages::Sensor> frame;

  // fill the message
  for(int i = 0; i < robot->N; i++) {
    frame.msg.adcs[i] = robot->joints[i].read();
  }

  messages::send(frame, packet_serial);
}


void onPacket(const uint8_t* buffer, size_t size) {
  if(auto m = message_cast<const messages::Control*>(buffer, size)) {
    for(int i = 0; i < robot->N; i++) {
      robot->joints[i].write(m->micros[i]);
    }
  }
  else {
    // bad message type
  }
}

void setup() {
  setupRobot();

  for(int i = 0; i < robot->N; i++) {
    robot->joints[i].write(1500);
  }

  // set up the serial connection
  packet_serial.begin(115200);
  packet_serial.setPacketHandler(onPacket);

  while(1) {
    sendReadings();
    packet_serial.update();

    // the cost of not using loop()
    if (serialEventRun) serialEventRun();
  }
}

// Deliberately left blank, because arduino is dumb;
void loop() {}
