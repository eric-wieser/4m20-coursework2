#include "PacketSerial/src/PacketSerial.h"

#include "Robot.h"
#include "messages.h"
#include "imu_helpers.h"

PacketSerial packet_serial;
Robot *robot;

void setupRobot() {
  // Define where the pins are connected
  static Robot r = {
    Joint(3, 0),
    Joint(5, 1),
    Joint(6, 2),
    {}
  };
  robot = &r;
}

void sendJointReadings() {
  messages::Framed<messages::Sensor> frame;

  // fill the message
  for(int i = 0; i < robot->N; i++) {
    frame.msg.adcs[i] = robot->joints[i].read();
  }

  messages::send(frame, packet_serial);
}

void sendIMUReadings() {
  messages::Framed<messages::IMUScaled> frame;

  // fill the message
  frame.msg.acc[0] = robot->imu.ax;
  frame.msg.acc[1] = robot->imu.ay;
  frame.msg.acc[2] = robot->imu.az;
  frame.msg.gyro[0] = robot->imu.gx;
  frame.msg.gyro[1] = robot->imu.gy;
  frame.msg.gyro[2] = robot->imu.gz;
  frame.msg.mag[0] = robot->imu.mx;
  frame.msg.mag[1] = robot->imu.my;
  frame.msg.mag[2] = robot->imu.mz;

  messages::send(frame, packet_serial);
}

void sendServoPulseWidths() {
  messages::Framed<messages::ServoPulse> frame;

  for(int i = 0; i < robot->N; i++) {
    frame.msg.micros[i] = robot->joints[i].getPeriod();
  }

  messages::send(frame, packet_serial);
}

volatile int pingPending = 0;

void onPacket(const uint8_t* buffer, size_t size) {
  if(auto m = message_cast<const messages::ServoPulse*>(buffer, size)) {
    // update the servo pulse widths
    for(int i = 0; i < robot->N; i++) {
      robot->joints[i].write(m->micros[i]);
    }
  }
  else if(/* auto m =*/ message_cast<const messages::Ping*>(buffer, size)) {
    // send back a response ping
    pingPending += 1;
  }
  else if(auto m = message_cast<const messages::JointConfig*>(buffer, size)) {
    // send back a response ping
    for(int i = 0; i < robot->N; i++) {
      robot->joints[i].setLimits(m->minMicros, m->maxMicros);
      robot->joints[i].setAdcParams(m->adcZero[i], m->servoPerAdc[i]);
    }
  }
  else if(auto m = message_cast<const messages::ServoForce*>(buffer, size)) {
    // enable force control on the joints
    for(int i = 0; i < robot->N; i++) {
      robot->joints[i].writeForce(m->adcs[i]);
    }
  }
  else {
    // bad message type
  }
}

void setup() {
  setupRobot();

  // set up the serial connection
  packet_serial.begin(115200);
  packet_serial.setPacketHandler(onPacket);

  // setup the I2C bus for the MPU9250
  Wire.begin();
  setupIMU(robot->imu);

  while(1) {
    if(pingPending) {
      messages::send(messages::Framed<messages::Ping>(), packet_serial);
      pingPending--;
    }
    updateIMU(robot->imu);
    sendJointReadings();
    sendServoPulseWidths();
    sendIMUReadings();
    packet_serial.update();

    // do any periodic updates needed
    robot->update(millis());

    // the cost of not using loop()
    if (serialEventRun) serialEventRun();
  }
}

// Deliberately left blank, because arduino is dumb;
void loop() {}
