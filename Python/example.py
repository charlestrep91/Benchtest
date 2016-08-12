from python_parser_class import protocolParser
import time

def searchCallback(sensorID):
	print("Found sensor ID: " + ' '.join(format(x, '02x') for x in sensorID))

def readCallback(sensorID, temp):
	print("temp: %0.2f" % temp)
	print("Sensor ID: " + ' '.join(format(x, '02x') for x in sensorID))

def readScaleCallback(weight):
	print("Weight: %0.2f" % weight)

parser = protocolParser('COM7', 115200)
parser.setCmdCallback(1, searchCallback)
parser.setCmdCallback(2, readCallback)
parser.setCmdCallback(10, readScaleCallback)
parser.printDebug(0)
parser.printAscii(1)
parser.start()

while(True):
	line = input()
	if(line == "help"):
		parser.helpCmd()
	elif(line == "search"):
		parser.searchCmd()
	elif(line == "read"):
		parser.readCmd()
	elif(line == "start"):
		parser.startCmd()
	elif(line == "stop"):
		parser.stopCmd()
	elif(line == "reset"):
		parser.resetCmd()
	elif(line == "mode"):
		parser.modeCmd()
	elif(line == "list"):
		parser.listCmd()
	elif(line.startswith("setcurrent")):
		arguments = line.split()
		channel = float(arguments[1])
		current = float(arguments[2])
		parser.setCurrentCmd(channel, current)
	elif(line == "calcurrent"):
		parser.calCurrentCmd()
	elif(line == "readscale"):
		parser.readScaleCmd()
	elif(line == "calscale"):
		parser.calScaleCmd()
	elif(line == "resetscale"):
		parser.resetScaleCmd()
	elif(line == "q"):
		break
	else:
		print("Unknown command!")
parser.stop()
parser.join()
print("Exiting program")

