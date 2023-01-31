import csv
import serial
import json


# Main
def main():
    port = '/dev/tty.usbmodem11301'
    baudrate = 115200
    serialCxn = openSerial(port, baudrate)
    writer = openCsv('test2.csv')

    while True:
        line = readLineSerial(serialCxn)
        print(line)
        dictLine = convertToDict(line)
        writeLine(writer, dictLine)

# Serial functions.
def openSerial(port: str, baudrate: int):
    print('Port: ' + port)
    print('Baudrate: ' + str(baudrate))
    ser = serial.Serial(port, baudrate)
    return ser

def readLineSerial(cxn: serial.Serial):
    return cxn.readline().decode('utf-8')[:-1]

# CSV functions.
def openCsv(url: str):
    csvFile = open(url, 'w', newline='\n')
    csvWriter = csv.writer(csvFile, delimiter=',')
    return csvWriter

def writeLine(writer: csv.writer, line: dict):
    print(line)
    writer.writerow(line)

def convertToDict(line: str) -> dict:
    return json.loads('[' + line + ']')


if __name__ == '__main__':
    main()
