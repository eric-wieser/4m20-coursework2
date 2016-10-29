from struct import Struct

class DecodeError(ValueError): pass

class Message(tuple):
	""" base class for all messages """

	@staticmethod
	def deserialize(data: bytes) -> 'Message':
		if len(data) < 1:
			raise DecodeError("Packet is empty!")

		header, rest = data[:1], data[1:]

		for m_type in Message.__subclasses__():
			if m_type.code == header:
				break
		else:
			raise DecodeError('Unrecognized message code {:c}'.format(data[0]))

		if len(rest) != m_type.fmt.size:
			raise DecodeError('Packet is of length {}, but {} is of length {}'.format(
				len(rest) - 1, m_type.__name__, m_type.fmt.size
			))

		fields = m_type.fmt.unpack(data[1:])
		return m_type(fields)

	def serialize(self) -> bytes:
		return self.code + self.fmt.pack(*self)

class Control(Message):
	code = b'C'
	fmt = Struct('HHH')

class Sensor(Message):
	code = b'S'
	fmt = Struct('HHH')

if __name__ == '__main__':
	m = Sensor((1, 2, 3))
	s = m.serialize()

	m2 = Message.deserialize(s)
	assert m == m2
