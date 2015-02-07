#!/usr/bin/env python

import serial
from Tkinter import *
import time

comPort = "/dev/ttyACM0"  #default com port
comPortBaud = 115200

class App:
    grid_size = 15
    num_pixels = 30
    image_started = FALSE
    image_current_row = 0
    pixel_dictionary = {}

    def __init__(self, master):
        self.is_serial_open = False

        # set main window's title
        master.title("ADNS3080ImageGrabber")

        self.frame = Frame(master)
        self.frame.grid(row=0,column=0)

        self.comPortStr = StringVar()
        self.comPort = Entry(self.frame,textvariable=self.comPortStr)
        self.comPort.grid(row=0,column=0)
        self.comPort.delete(0, END)
        self.comPort.insert(0,comPort)

        self.button = Button(self.frame, text="Start", fg="red", command=self.start_loop)
        self.button.grid(row=0,column=1)

        self.entryStr = StringVar()
        self.entry = Entry(self.frame,textvariable=self.entryStr)
        self.entry.grid(row=0,column=2)
        self.entry.delete(0, END)
        self.entry.insert(0,"0x13")

        self.send_button = Button(self.frame, text="Get Data", command=self.get_data)
        self.send_button.grid(row=0,column=3)

        self.canvas = Canvas(master, width=self.grid_size*self.num_pixels, height=self.grid_size*self.num_pixels)
        self.canvas.grid(row=1)

    # TODO: Make more error safe
    def open_serial(self):
        self.ser = serial.Serial(self.comPortStr.get(), comPortBaud)
        self.is_serial_open = True

    def start_loop(self):
        time.sleep(1)
        while True:
            time.sleep(1)
            self.get_data()
            self.frame.update()

    def get_data(self):
        if not self.is_serial_open:
            self.open_serial()

        self.ser.write("optFlow.get_pixel_data()\r\n")
        self.ser.flush()

        stuff = ""
        waiting = self.ser.inWaiting()
        while waiting > 0:
            stuff += self.ser.read()
            waiting = self.ser.inWaiting()

        # TODO: Clean this up
        data = ""
        collect = False
        for idx, item in enumerate(stuff):
            if idx != 0:
                current = item
                if collect:
                    data += item
                if current == "[" and last == "[":
                    collect = True
                if current == "]" and last == "]":
                    collect = False
                    data = data[:-2]
            last = item
        print("data", data)

        for y, line in enumerate(data.split("\r\n")):
            for x, item in enumerate(line.split(",")):
                self.display_pixel(x, y, item)
            
    def display_default_image(self):
        # display the grid
        for x in range(0, self.num_pixels-1):
            for y in range(0, self.num_pixels-1):
                colour = x * y / 3.53
                self.display_pixel(x,y,colour)

    def display_pixel(self, x, y, colour):
        if( x >= 0 and x < self.num_pixels and y >= 0 and y < self.num_pixels ) :
            colour = int(colour)
            #find the old pixel if it exists and delete it
            if self.pixel_dictionary.has_key(x+y*self.num_pixels) :
                self.old_pixel = self.pixel_dictionary[x+y*self.num_pixels]
                self.canvas.delete(self.old_pixel)
                del(self.old_pixel)
                
            fillColour = "#%02x%02x%02x" % (colour, colour, colour)
            #draw a new pixel and add to pixel_array
            self.new_pixel = self.canvas.create_rectangle(x*self.grid_size, y*self.grid_size, (x+1)*self.grid_size, (y+1)*self.grid_size, fill=fillColour)
            self.pixel_dictionary[x+y*self.num_pixels] = self.new_pixel


if __name__ == "__main__":
    root = Tk()
    app = App(root)
    app.display_default_image()

    root.mainloop()
