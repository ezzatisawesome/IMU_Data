import csv
import serial

# Main
def main():
    port = '/dev/cu.usbmodem1401'
    baudrate = 115200
    serialCxn = openSerial(port, baudrate)
    writer = openCsv('test.csv')

    while True:
        line = readLineSerial(serialCxn)
        print(line)
        writeLine(writer, line)

# Serial functions.
def openSerial(port: str, baudrate: int):
    print('Port: ' + port)
    print('Baudrate: ' + str(baudrate))
    ser = serial.Serial(port, baudrate)
    return ser

def readLineSerial(cxn: serial.Serial):
    return cxn.readline().decode('utf-8') 

# CSV functions.
def openCsv(url: str):
    csvFile = open(url, 'w', newline='\n')
    csvWriter = csv.writer(csvFile, delimiter=',')
    return csvWriter

def writeLine(writer: csv.writer, line: str, quotechar='\"', quoting=csv.QUOTE_NONE):
    writer.writerow([line])

if __name__ == '__main__':
    main()