#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# This module provides a class that controls the serial servo motor manufactured by Futaba Corp.
# ver1.40716
# This module has been tested on python ver.2.6.6
# It need pySerial(http://pyserial.sourceforge.net/)
# (C) 2012 Matsuda Hiroaki

import serial
import binascii

debug_flug = False

class Rs(object):

	def __init__(self):
		self.myserial = serial.Serial()
		print('Generated the serial object')
		self.mode = 'normal'

	def open_port(self, port='/dev/ttyUSB0', baudrate=115200, timeout=1):
		self.myserial.port = port
		self.myserial.baudrate = baudrate
		self.myserial.timeout = timeout
		self.myserial.parity = serial.PARITY_NONE
		try:
			self.myserial.open()
		except IOError:
			raise IOError('Failed to open port, check the device and port number')
		else:
			print('Succeede to open port: ' + port)

	def close_port(self):
		self.myserial.close()

	def set_port(self, baudrate=115200, timeout=0x01):
		self.myserial.baudrate = baudrate
		self.myserial.timeout = timeout
		self.myserial._reconfigurePort()
		print('Succeede to set baudrate:%d, timeout:%d' % (baudrate, timeout))

	def torque_on(self, id, mode, return_packet=0x02):
		'''
		turn on the servo
		id: servo id
		mode: on = 1, off = 0
		return_packet : retun or not
		'''
		self._check_range(id, 1, 255, 'id')
		self._check_range(mode, 0, 1, 'mode')
		self._check_range(return_packet, 0, 15, 'return_packet')

		if mode == 1: mode16 = 0x80
		elif mode == 0: mode16 = 0xFF 

		send = [0x58,			#hdr: header
				0x82,			#ad_l: low address
				0x00,			#ad_r: high address
				0x01,			#lg: data length
				0x01,			#cnt: servo count
				id,				#id0: servo id
				mode16]			#d00: byte numbers with data length
		send.append(self._calc_checksum(send))
		print("send message={}".format(send))
		print("send message={}".format(bytearray(send)))

		self._write_command(send)

		if return_packet == 0x01:
			return id, 0x01

		elif return_packet == 0x02:
			return self._check_ack(id)

	def unlock_setting(self, id, return_packet=0x02):
		'''

		id: servo id
		return_packet : retun or not
		'''
		self._check_range(id, 1, 255, 'id')
		self._check_range(return_packet, 0, 15, 'return_packet')

		send = [0x58,			#hdr: header
				0x1b,			#ad_l: low address
				0x00,			#ad_r: high address
				0x01,			#lg: data length
				0x01,			#cnt: servo count
				id,				#id0: servo id
				0x75]			#d00: byte numbers with data length
		send.append(self._calc_checksum(send))
		print("send message={}".format(send))
		print("send message={}".format(bytearray(send)))

		self._write_command(send)

		if return_packet == 0x01:
			return id, 0x01

		elif return_packet == 0x02:
			return self._check_ack(id)

	def unlock_flash_setting(self, id, return_packet=0x02):
		'''

		id: servo id
		return_packet : retun or not
		'''
		self._check_range(id, 1, 255, 'id')
		self._check_range(return_packet, 0, 15, 'return_packet')

		send = [0x58,			#hdr: header
				0x14,			#ad_l: low address
				0x00,			#ad_r: high address
				0x01,			#lg: data length
				0x01,			#cnt: servo count
				id,				#id0: servo id
				0x55]			#d00: byte numbers with data length
		send.append(self._calc_checksum(send))
		print("send message={}".format(send))
		print("send message={}".format(bytearray(send)))

		self._write_command(send)

		if return_packet == 0x01:
			return id, 0x01

		elif return_packet == 0x02:
			return self._check_ack(id)

	def read_flash_setting(self, id, ad, return_packet=0x02):
		'''

		id: servo id
		return_packet : retun or not
		'''
		self._check_range(id, 1, 255, 'id')
		self._check_range(return_packet, 0, 15, 'return_packet')

		send = [0x59, 	#hdr: header
				ad, 	#ad_l: low address
				0x78, 	#ad_h: high address
				0x02, 	#lg: data length
				0x01, 	#cnt: servo count
				id]		#id0: servo id

		send.append(self._calc_checksum(send))
		print("send message={}".format(send))
		print("send message={}".format(bytearray(send)))

		self._write_command(send)

		return self._check_read(id)

	def target_position(self, id, position, return_packet=0x01):
		'''
		move target angle
		id: servo id
		position: target angle
		return_packet : retun or not
		'''
		self._check_range(id, 1, 255, 'id')
		self._check_range(position, -180, 180, 'position')
		#self._check_range(time, 0, 16383, 'time')
		self._check_range(return_packet, 0, 15, 'return_packet')

		calc_pos = int(position /180 * 32768)

		send = [0x58,					#hdr: header
				0x80,					#ad_l: low address
				0x00,					#ad_h: high address
				0x02,					#lg: data length
				0x01,					#cnt: servo count
				id,						#id0: servo id
				calc_pos & 0x00FF,		#d00: under bit
				calc_pos >> 8]			#d01: upper bit

		send.append(self._calc_checksum(send))
		print("send message={}".format(send))
	

		send_byte = b''
		
		for i in range(len(send)):
			if send[i] < 0:
				#print(send[i].to_bytes(1, byteorder='big', signed=True))
				send_byte = send_byte + send[i].to_bytes(1, byteorder='big', signed=True)
			else:
				#print(send[i].to_bytes(1, byteorder='big', signed=False))
				send_byte = send_byte + send[i].to_bytes(1, byteorder='big', signed=False)

		print("send message={}".format(send_byte))

		self._write_command(send_byte)

		if return_packet == 0x00:
			return id, 0x00

		elif return_packet == 0x01:
			return self._check_ack(id)

	def multi_torque_on(self, servo_data):
		'''
		multi turn on the servo
		id: servo id
		mode: on = 1, off = 0
		return_packet : retun or not
		'''
		for servo in servo_data:
			self._check_range(servo[0], 1, 255, 'id')
			self._check_range(servo[1], 0, 1, 'mode')

		send = [0x58, 0x82,	0x00, 0x01,	len(servo_data)]
		for servo in servo_data:
			if servo[1] == 1: send.extend([servo[0],0x80])
			elif servo[1] == 0: send.extend([servo[0],0xFF])

		if(debug_flug):print("send message={}".format(send))
		
		send.append(self._calc_checksum(send))
		
		if(debug_flug):print("send message={}".format(send))
		if(debug_flug):print("send message={}".format(bytearray(send)))

		self._write_command(send)
		
		print("receive message={}".format(self.myserial.read(30)))

		return 'multi_torque_on:' + str(servo_data)

	def multi_target_position(self, servo_data):
		for servo in servo_data:
			self._check_range(servo[0], 1, 255, 'id')
			self._check_range(servo[1], -180, 180, 'position')

		send = [0x58, 0x80, 0x00, 0x02, len(servo_data)]
		for servo in servo_data:
			calc_pos = int(servo[1] /180 * 32768)
			send.extend([servo[0], calc_pos & 0x00FF, calc_pos >> 8])
		send.append(self._calc_checksum(send))
		#print("send message={}".format(send))

		send_byte = b''
			
		for i in range(len(send)):
			if send[i] < 0:
				#print(send[i].to_bytes(1, byteorder='big', signed=True))
				send_byte = send_byte + send[i].to_bytes(1, byteorder='big', signed=True)
			else:
				#print(send[i].to_bytes(1, byteorder='big', signed=False))
				send_byte = send_byte + send[i].to_bytes(1, byteorder='big', signed=False)
		
		#print("send message={}".format(send_byte))

		self._write_command(send_byte)
		#print(self.myserial.read(10))

		return 'multi_target_position:' + str(servo_data)

	def get_data(self, id, mode='angle'):
		'''
		get servo parameter
		now only angle data

		'''
		self._check_range(id, 1, 255, 'id')

		modes = ('angle', 'load', 'tempreture', 'voltage', 'lower_limited', 'upper_limited' 'list')
		if mode not in modes:
			raise ValueError('mode is not defined, select from the list below\n'
							 + str(modes))

		elif mode == 'list':
			return modes

		send = [0x59, 	#hdr: header
				0x40, 	#ad_l: low address
				0x00, 	#ad_h: high address
				0x02, 	#lg: data length
				0x01, 	#cnt: servo count
				id]		#id0: servo id

		if mode == 'angle': send[1] = 0x40
		elif mode == 'tempreture':send[1] = 0x42
		elif mode == 'current':send[1] = 0x44
		elif mode == 'voltage':send[1] = 0x46
		elif mode == 'lower_limited':send[1] = 0x94
		elif mode == 'upper_limited':send[1] = 0x96
		send.append(self._calc_checksum(send))

		print("send message={}".format(send))
		print("send message={}".format(bytearray(send)))

		self._write_command(send)

		receive = self.myserial.read(15)
		print('receive raw data = {}'.format(receive))
		receive = [chr(r) for r in receive]
		print('receive data = {}'.format(receive))

		try:
			ord(receive[9])>> 7
		except IndexError:
			print('Could not get the data.Check the cables, connectors, and a power supply.')

		if ord(receive[0]) != 0x59:
			return id, 999

		if mode == 'angle':
			polarity = ord(receive[9])>> 7
			if polarity == 1:
				value = ((ord(receive[9])^0xFF) << 8) | ord(receive[8])^0xFF
				print("16bit angle = {}".format(value + 1))
				value = int((value + 1) / 32768 * 180) * -1
			else:
				value = (ord(receive[9]) << 8) | ord(receive[8])
				value = int(value / 32768 * 180)
			print("servo angle = {}, {} + {}".format(value, receive[9], receive[8]))
			return id, value
		else:
			return id, 999

	def get_multi_data(self, ids, mode='angle'):
		'''
		get servo multi parameter
		now only angle data
		'''
		for id in ids:
			self._check_range(id, 0, 255, 'id')

		modes = ('angle', 'load', 'tempreture', 'voltage', 'lower_limited', 'upper_limited' 'list')
		if mode not in modes:
			raise ValueError('mode is not defined, select from the list below\n'
							 + str(modes))

		elif mode == 'list':
			return modes

		send = [0x59, 0x40, 0x00, 0x02, len(ids)]
				
		for id in ids:
			send.append(id)

		if mode == 'angle': send[1] = 0x40
		elif mode == 'tempreture':send[1] = 0x42
		elif mode == 'current':send[1] = 0x44
		elif mode == 'voltage':send[1] = 0x46
	
		send.append(self._calc_checksum(send))

		if(debug_flug):print("send message={}".format(send))
		if(debug_flug):print("send message={}".format(bytearray(send)))

		self._write_command(send)


		read_target = 5 + len(ids) + 1 

		receive = self.myserial.read(read_target + 4 * len(ids))
		print('receive raw data = {}'.format(receive))
		receive = [chr(r) for r in receive]
		print('receive data = {}'.format(receive))

		try:
			ord(receive[9])>> 7
		except IndexError:
			print('Could not get the data.Check the cables, connectors, and a power supply.')

		if ord(receive[0]) != 0x59:
			return ids, 999

		if mode == 'angle':
			angle_value = []
			for id in ids:
				upper_id = read_target + 3 + 4 * (id-1) - 1
				print("test read {} {}".format(upper_id, hex(ord(receive[upper_id]))))
				polarity = ord(receive[upper_id])>> 7
				if polarity == 1:
					value = ((ord(receive[upper_id])^0xFF) << 8) | ord(receive[upper_id - 1])^0xFF
					print("16bit angle = {}".format(value + 1))
					value = int((value + 1) / 32768 * 180) * -1
					angle_value.append(value)
				else:
					value = (ord(receive[upper_id]) << 8) | ord(receive[upper_id - 1])
					value = int(value / 32768 * 180)
					angle_value.append(value)
				print("servo angle = {}".format(angle_value))
			return ids, angle_value
		else:
			return ids, 999

	'''	
	def servo_reset(self, id):
		self._check_range(id, 1, 127, 'id')

		send = [0xFA, 0xAF, id, 0x20, 0xFF, 0x00, 0x00]
		send.append(self._calc_checksum(send))

		self._write_serial(send, 0)

	def set_torque_limit(self, id, limit=100):
		self._check_range(id, 1, 127, 'id')
		self._check_range(limit, 0, 100, 'limit')

		send = [0xFA, 0xAF, id, 0x01, 0x23, 0x01, 0x01, limit & 0x00FF]
		send.append(self._calc_checksum(send))

		self._write_serial(send, 1)

		return self._check_ack(id)

	def set_damper(self, id, damper=16):
		self._check_range(id, 1, 127, 'id')
		self._check_range(damper, 0, 255, 'damper')

		send = [0xFA, 0xAF, id, 0x01, 0x20, 0x01, 0x01, damper & 0x00FF]
		send.append(self._calc_checksum(send))

		self._write_serial(send, 1)

		return self._check_ack(id)

	def set_compliance(self, id, cwcm=1, ccwcm=1, cwcs=4, ccwcs=4, punch=1300):
		self._check_range(id, 1, 127, 'id')
		self._check_range(cwcm, 0, 255, 'cwcm')
		self._check_range(ccwcm, 0, 255, 'ccwcm')
		self._check_range(cwcs, 0, 255, 'cwcs')
		self._check_range(ccwcs, 0, 255, 'ccwcs')
		self._check_range(punch, 0, 10000, 'punch')

		send = [0xFA, 0xAF, id, 0x01, 0x18, 0x06, 0x01, cwcm & 0x00FF, ccwcm & 0x00FF,
				cwcs & 0x00FF, ccwcs & 0x00FF, punch & 0x00FF, (punch & 0xFF00) >> 8]
		send.append(self._calc_checksum(send))

		self._write_serial(send, 1)

		return self._check_ack(id)
		'''

	# The following functions are provided for use in PRS class
	def _calc_checksum(self, send):
		'''
		calculate check sum
		'''
		sum_calc = sum(send)
		checksum = (~sum_calc & 0xff) + 1

		return checksum

	def _check_range(self, value, lower_range, upper_range, name='value'):
		if value < lower_range or value > upper_range:
			raise ValueError(name + ' must be set in the range from '
							 + str(lower_range) + ' to ' + str(upper_range))

	def _check_ack(self, id):
		receive = self.myserial.read()
		print("check_ack {}".format(receive))
		length = len(receive)

		if length == 1:
			ack = receive
			if ack == 0x30:
				return id, 'ACK'
			elif ack == 0x31:
				return id, 'NACK'
			else:
				return id, 'unKnown'
		elif length != 1:
			return id, 'unReadable'

	def _check_read(self, id):
		receive = self.myserial.readline()
		print("check_ack {}".format(receive))
		length = len(receive)

		return id, receive

	'''
	def _write_rpu(self, send, length):
		if length == 0:
			send_rpu = [0x53, len(send)]
			send_rpu += send
		else:
			send_rpu = [0x54, len(send) + 1]
			send_rpu += send
			send_rpu.append(length)

		self.myserial.flushOutput()
		self.myserial.flushInput()
		self.myserial.write("".join(map(chr, send_rpu)).encode())
	'''

	def _write_command(self, send):
		self.myserial.flushOutput()
		self.myserial.flushInput()
		# print(bytearray(map(chr, send)))
		# print("".join(map(chr, send)))
		# print("".join(map(chr, send)).encode())
		# self.myserial.write(bytearray(map(chr, send)))
		# self.myserial.write(bytearray("".join(map(chr, send)).encode()))
		# self.myserial.write("".join(map(chr, send)).encode())
		self.myserial.write(bytearray(send))
		#self.myserial.write(send)

	#def _write_serial(self, send, length):
	#self._write_command(send)
