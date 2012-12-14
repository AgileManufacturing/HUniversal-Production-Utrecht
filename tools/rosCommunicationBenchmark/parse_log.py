import sys

clientLog = open("client.log", "r")
serverLog = open("server.log", "r")
returnLog = open("return.log", "w")
receiveLog = open("receive.log", "w")
verschilLog = open("verschil.log", "w")

clientLines = clientLog.readlines()
serverLines = serverLog.readlines()

if len(clientLines) != (2 * len(serverLines)):
	print "Faal!"
	sys.exit(0)

totalLines = len(clientLines);

for line in xrange(0, totalLines, 2):
	sendTime = long(clientLines[line].replace("SND;", ""))
	receiveTime = long(serverLines[line / 2].replace("RCV;", ""))
	returnTime = long(clientLines[line + 1].replace("RTN;", ""))

	returnLog.write(str(returnTime - sendTime) + "\n")
	receiveLog.write(str(receiveTime - sendTime) + "\n")
	verschilLog.write(str(returnTime - receiveTime) + "\n")


"""f = open("koen3.log", "r")
numOfLines = 0
minTime = 1
maxTime = 0
totalTime = 0
deviation = 0

fOut = open("parsed3.log", "w")
fOut2 = open("parsed3average.log", "w")


for line in f:
    if line.startswith("Diff"):
		time = float(line.replace("Diff ", "").replace("\n", ""))
		
		numOfLines+=1
		if time < minTime:
			minTime = time
			
		if time > maxTime:
			maxTime = time
		
		totalTime += time
		fOut2.write(str(totalTime / numOfLines) + "\n")
		

averageTime = totalTime / numOfLines
f.seek(0)
for line in f:
    if line.startswith("Diff"):
		time = float(line.replace("Diff ", "").replace("\n", ""))
		deviation += abs(averageTime-time)
		fOut.write(str(time) + "\n")

		
print "min time  ", minTime
print "max time  ", maxTime
print "avg time  ", totalTime / numOfLines
print "deviation ", deviation / numOfLines
print "total time", totalTime
print "number of measurements", numOfLines"""
