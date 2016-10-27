#include <Servo.h>

// class for a single joint, that can read the angle and control the servo
class Joint {
private:
  uint8_t encoderPin_;
  Servo servo_;
public:
  Joint(uint8_t servoPin, uint8_t adcPin) : encoderPin_(adcPin) {
    servo_.attach(servoPin, 700, 2300);
  }

  void write(int angle) {
    servo_.write(angle);
  }

  int read() const {
    return analogRead(encoderPin_);
  }
};

// A Robot<N> is just a set of N joints
template<int N>
struct Robot {
  Joint joints[N];
};

template<int N>
void sendReadings(Robot<N> robot) {
  // print out the angle readings
  for(int i = 0; i < N; i++) {
    if(i != 0) Serial.print('\t');
    Serial.print(robot.joints[i].read());
  }
  Serial.print('\n');
}

template<int N>
void updateAngles(Robot<N> robot) {
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