#include "Robot.h"

void sendReadings(Robot robot) {
  // print out the angle readings
  for(int i = 0; i < robot.N; i++) {
    if(i != 0) Serial.print('\t');
    Serial.print(robot.joints[i].read());
  }
  Serial.print('\n');
}

void updateAngles(Robot robot) {
  // control the servos
  for(int i = 0; i < robot.N; i++)
    robot.joints[i].write(90);
}

void setup() {
  // Define where the pins are connected
  Robot robot = {
    Joint(3, 0),
    Joint(5, 1),
    Joint(6, 2),
  };

  // set up the serial connection
  Serial.begin(115200);

  while(1) {
    sendReadings(robot);
    updateAngles(robot);

    // the cost of not using loop()
    if (serialEventRun) serialEventRun();
  }
}

// Deliberately left blank, because arduino is dumb;
void loop() {}
