/*
There's a lot of template magic in this file.

All that really matters here is:

  auto m = message_cast<MessageType*>(uint8_t)

which returns nullptr if the message is not of the desired type

And send(const Framed<Message>&, PacketSerial&)
*/

#pragma once

#include "type_traits.h"

// this attribute tells the compiler to put all of the fields of a struct
// next to each other in memory
#define PACKED __attribute__((__packed__))

namespace messages {
  // Define some metadata about the messages
  template<typename T, uint8_t C>
  struct base_trait {
    static const uint8_t code = C;
    static const size_t size = sizeof(T);
  };

  // define the metadata
  template<typename T> struct traits;

  // to send a packet, it must be prefaced by its code.
  template<typename T>
  struct PACKED Framed {
    const uint8_t code = traits<T>::code;
    T msg;
  };

  // send the given framed message on a channel
  template<typename T, typename C>
  inline void send(const Framed<T> &frame, C &channel) {
    channel.send(reinterpret_cast<const uint8_t*>(&frame), sizeof(frame.code) + messages::traits<T>::size);
  }
}

template<typename T, typename U,
  // this verifies that the data is a uint8_t
  typename=typename std::enable_if<
    std::is_same<uint8_t, typename std::remove_cv<U>::type>::value
  >::type
>
static T message_cast(U* data, size_t size) {
  // work out the underlying message type
  typedef typename std::remove_cv<typename std::remove_pointer<T>::type>::type message_type;

  // get the traits for that type
  typedef messages::traits<message_type> message_traits;

  // packet is empty!
  if(size < 1)
    return nullptr;

  // not this type of packet
  if(data[0] != message_traits::code)
    return nullptr;

  // packet is too small
  if(size < message_traits::size + 1)
    return nullptr;

  // return a view on the following data
  return reinterpret_cast<T>(data + 1);
}
