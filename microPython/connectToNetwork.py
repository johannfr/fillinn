# main.py -- put your code here!
from pyb import UART
import network
import socket

if __name__ == "__main__":
    nic = network.CC3K(pyb.SPI(2), pyb.Pin.board.Y5, pyb.Pin.board.Y4, pyb.Pin.board.Y3)
    nic.connect('Joi', 'deadbeef')
    while not nic.isconnected():
        pyb.delay(50)
    print(nic.ifconfig())

    uart = UART(1, 9600)
    addr = ('192.168.1.242', 9999)
    while True:
        s = socket.socket()
        s.connect(addr)
        s.send(b"Hello\r\n")
        while True:
            try:
                incoming = "" 
                incoming = s.recv(1024)
                uart.write("%s\r"%incoming.decode().strip())
                while uart.any():
                    serialdata = uart.readline()
                    s.send("%s\n"%serialdata.decode().strip())
            except:
                print("error")
                break
        s.close()

