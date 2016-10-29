/*
Define the list of packet types.

Each packet should be a packed struct, and be followed by a traits definition..
These traits contain metadata about the packet
*/
#pragma once

#include "messaging.h"

namespace messages {
	// Define some message types
	struct PACKED Control {
		uint16_t micros[3];  // servo durations in uS
	};
	template<> struct traits<Control> : base_trait<Control, 'C'>{};

	struct PACKED Sensor {
		uint16_t adcs[3];
		// TODO: accelerometer data?
	};
	template<> struct traits<Sensor>  : base_trait<Sensor, 'S'>{};
}
