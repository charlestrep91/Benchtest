import serial
import sys
import msvcrt
import struct
import array
import io
import threading

helpCmdByte			= 0x00
searchCmdByte 		= 0x01
readCmdByte 		= 0x02
startCmdByte 		= 0x03
stopCmdByte 		= 0x04
resetCmdByte 		= 0x05
modeCmdByte 		= 0x06
listCmdByte 		= 0x07
setCurrentCmdByte 	= 0x08
calCurrentCmdByte 	= 0x09
readScaleCmdByte 	= 0x0a
calScaleCmdByte		= 0x0b
resetScaleCmdByte	= 0x0c

class protocolParser(threading.Thread):
	startByte = 0xF0
	ackByte = 0xF1
	baudrate = 115200
	minCmdLength = 4
	printAscii = 0
	printDebug = 0
	port = 0

	def __init__(self, portName, baudrate):
		self.portName = portName
		self.baudrate = baudrate
		self.timeout = 1
		super(protocolParser, self).__init__()
		self._stopEvent = threading.Event()
		self.port = serial.Serial(self.portName, self.baudrate, timeout = self.timeout)
		print("%s port opened at %d baud" %(self.port.name, self.baudrate))
		print("-->Press 'q' to quit program")

	def printAscii(self, flag):
		if(flag == 0):
			self.printAscii = 0
		else:
			self.printAscii = 1

	def printDebug(self, flag):
		if(flag == 0):
			self.printDebug = 0
		else:
			self.printDebug = 1


	def setCmdCallback(self, cmd, callback):
		if(cmd == 0):
			self.callback0 = callback
		elif(cmd == 1):
			self.callback1 = callback
		elif(cmd == 2):
			self.callback2 = callback
		elif(cmd == 3):
			self.callback3 = callback	
		elif(cmd == 4):
			self.callback4 = callback
		elif(cmd == 5):
			self.callback5 = callback
		elif(cmd == 6):
			self.callback6 = callback
		elif(cmd == 7):
			self.callback7 = callback
		elif(cmd == 8):
			self.callback8 = callback
		elif(cmd == 9):
			self.callback9 = callback
		elif(cmd == 10):
			self.callback10 = callback
		elif(cmd == 11):
			self.callback11 = callback
		elif(cmd == 12):
			self.callback12 = callback
		else:
			print("Unknown command number for callback!")

	def stop(self):
		print("Stopping protocol parser...")
		self._stopEvent.set()
		self.port.close()

	def run(self):
		while(not self._stopEvent.is_set()):
			start = self.port.read(1)
			if(len(start)):
				if(start[0] == self.startByte):
					lengthByte = self.port.read()
					length = ord(lengthByte)
					line = self.port.read(length + 1)
					frame = start + lengthByte + line
					if(length > self.minCmdLength):
						if(self.printDebug == 1):
							print("Received: " + ' '.join(format(x, '02x') for x in frame))
						if(verifyFrame(frame) != 1):
							if(self.printDebug == 1):
								print("invalid checksum")
						else:
							# print("valid checksum")
							self.processFrame(frame)
				elif(start[0] == self.ackByte):
					# print("ack")
					pass
				else:
					if(self.printAscii == 1):
						line = self.port.readline()
						sys.stdout.buffer.write(bytes(start))
						sys.stdout.buffer.write(line)
						sys.stdout.flush()

	def processFrame(self, frame):
		length = frame[1]
		cmd = frame[2]
		data = frame[3:(length + 2)]

		if(cmd == 0x01):
			try: self.callback1(data)
			except AttributeError: print("Error: callback1 not assigned!")

		elif(cmd == 0x02):
			# print("cmd 0x02")
			sensorID = data[0:8]
			tempBytes = [data[11], data[10], data[9], data[8]]	#order of bytes reversed because of endianess
			tmp = struct.pack('4B', *tempBytes)
			tempFloat = struct.unpack('>f', tmp)
			try: self.callback2(sensorID, tempFloat)
			except AttributeError: print("Error: callback2 not assigned!")
			# tempValid = data[12]
			# print("temp valid: %d" % tempValid)
		elif(cmd == 0x03):
			try: self.callback3()
			except AttributeError: print("Error: callback3 not assigned!")
		elif(cmd == 0x04):
			try: self.callback4()
			except AttributeError: print("Error: callback4 not assigned!")
		elif(cmd == 0x05):
			try: self.callback5()
			except AttributeError: print("Error: callback5 not assigned!")
		elif(cmd == 0x06):
			try: self.callback6()
			except AttributeError: print("Error: callback6 not assigned!")
		elif(cmd == 0x07):
			try: self.callback7()
			except AttributeError: print("Error: callback7 not assigned!")
		elif(cmd == 0x08):
			try: self.callback8()
			except AttributeError: print("Error: callback8 not assigned!")
		elif(cmd == 0x09):
			try: self.callback9()
			except AttributeError: print("Error: callback9 not assigned!")
		elif(cmd == 0x0a):
			tempBytes = [data[3], data[2], data[1], data[0]]	#order of bytes reversed because of endianess
			tmp = struct.pack('4B', *tempBytes)
			weightFloat = struct.unpack('>f', tmp)
			try: self.callback10(weightFloat)
			except AttributeError: print("Error: callback10 not assigned!")
		elif(cmd == 0x0b):
			try: self.callback11()
			except AttributeError: print("Error: callback11 not assigned!")
		elif(cmd == 0x0c):
			try: self.callback12()
			except AttributeError: print("Error: callback12 not assigned!")
		else:
			if(self.printDebug == 1):
				print("Unrecognized command: %x" % cmd)

	def sendCmd(self, data):
		length = len(data)
		cmd = data[0]
		checksum = CRC(data)
		frame = bytearray([self.startByte, length])
		frame.extend(data)
		frame.append(checksum)
		self.port.write(frame)
		if(self.printDebug == 1):
			print("sent frame: " + ' '.join(format(x, '02x') for x in frame))

	def helpCmd(self):
		data = bytearray([helpCmdByte])
		self.sendCmd(data)

	def searchCmd(self):
		data = bytearray([searchCmdByte])
		self.sendCmd(data)

	def readCmd(self):
		data = bytearray([readCmdByte])
		self.sendCmd(data)

	def startCmd(self):
		data = bytearray([startCmdByte])
		self.sendCmd(data)

	def stopCmd(self):
		data = bytearray([stopCmdByte])
		self.sendCmd(data)

	def resetCmd(self):
		data = bytearray([resetCmdByte])
		self.sendCmd(data)

	def modeCmd(self, mode):
		data = bytearray([modeCmdByte, int(mode)])
		self.sendCmd(data)

	def listCmd(self):
		data = bytearray([listCmdByte])
		self.sendCmd(data)

	def setCurrentCmd(self, channel, current):
		data = bytearray([setCurrentCmdByte, int(channel), int(current*10)])
		self.sendCmd(data)

	def calCurrentCmd(self):
		data = bytearray([calCurrentCmdByte])
		self.sendCmd(data)

	def readScaleCmd(self):
		data = bytearray([readScaleCmdByte])
		self.sendCmd(data)

	def calScaleCmd(self):
		data = bytearray([calScaleCmdByte])
		self.sendCmd(data)

	def resetScaleCmd(self):
		data = bytearray([resetScaleCmdByte])
		self.sendCmd(data)
		

def CRC(data):
	dataLength = len(data)
	crc = 0x00
	i = 0
	while (dataLength):
		dataLength -= 1
		extract = data[i]
		i += 1
		for j in range(0, 8):
			sum = (crc ^ extract) & 0x01
			crc >>= 1
			if (sum):
				crc = crc ^ 0x8C
			extract = extract>>1
	return crc

def verifyFrame(frame):
	length = frame[1]
	receivedCRC = frame[(length + 2)]
	cmd = frame[2]
	cmdAndData = frame[2:(length + 2)]
	calculatedCRC = CRC(cmdAndData)
	if(receivedCRC == calculatedCRC):
		if(length == (len(frame) - 3)):
			return 1
		else:
			if(self.printDebug == 1):
				print("Invalid frame length for command %x: calculated %d and received %d" % (cmd, (len(frame) - 3), length))
			return 0
	else:
		if(self.printDebug == 1):
			print("Invalid checksum for command %x: calculated %x and received %x" % (cmd, calculatedCRC, receivedCRC))
		return 0