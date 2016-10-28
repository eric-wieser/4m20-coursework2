#include "Robot.h"

template<int N> void sendReadings(Robot<N> robot) {
  // print out the angle readings
  for(int i = 0; i < N; i++) {
    if(i != 0) Serial.print('\t');
    Serial.print(robot.joints[i].read());
  }
  Serial.print('\n');
}

template<int N> void updateAngles(Robot<N> robot) {
  // control the servos
  for(int i = 0; i < 3; i++)
    robot.joints[i].write(90);
}

void setup() {
  // Define where the pins are connected
  Robot<3> robot = {
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
