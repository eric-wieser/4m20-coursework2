#pragma once

#include "PacketSerial/src/PacketSerial.h"
/**
 * For RAII use. Creating an instance of this class will send a debug header,
 * and when the scope is left, it will send a null byte.
 */
struct Debug {
  Debug() {
    Serial.write("!DEBUG");
  }
  virtual ~Debug() {
    Serial.write('\0');
  }
  template<typename T>
  void print(T t) {
    Serial.print(t);
  }
  template<typename T1, typename T2>
  void print(T1 t1, T2 t2) {
    Serial.print(t1, t2);
  }
};
