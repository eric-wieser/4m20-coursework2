/*
Define the list of packet types.

Each packet should be a packed struct, and be followed by a traits definition..
These traits contain metadata about the packet
*/
#pragma once

#include "messaging.h"

namespace messages {
	// Define some message types
	struct PACKED ServoPulse {
		uint16_t micros[3];  // servo durations in uS
	};
	template<> struct traits<ServoPulse> : base_trait<ServoPulse, 'C'>{};

	struct PACKED ServoForce {
		uint16_t adcs[3];  // desired ADC reading
	};
	template<> struct traits<ServoForce> : base_trait<ServoForce, 'F'>{};

	struct PACKED JointConfig {
		uint16_t minMicros;
		uint16_t maxMicros;
		uint16_t adcZero[3];
		float servoPerAdc[3];
	};
	template<> struct traits<JointConfig> : base_trait<JointConfig, 'J'>{};

	struct PACKED Sensor {
		uint16_t adcs[3];
		// TODO: accelerometer data?
	};
	template<> struct traits<Sensor>  : base_trait<Sensor, 'S'>{};

	struct PACKED IMUScaled {
		float acc[3];
		float gyro[3];
		float mag[3];
	};
	template<> struct traits<IMUScaled>  : base_trait<IMUScaled, 'I'>{};

	struct PACKED Ping {};
	template<> struct traits<Ping>  : base_trait<Ping, 'P'>{ static const size_t size = 0;};
}
