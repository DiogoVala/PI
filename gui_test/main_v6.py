from PyQt5 import QtWidgets, QtCore
from PyQt5.QtWidgets import QApplication, QMainWindow, QSpinBox, QLabel, QVBoxLayout
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
import pandas as pd
import matplotlib.pyplot as plt

import sys
import random

# This program shows the parameters window and a functional graph with scrolling data

# if the graphs window is closed first, it opens another window
# closing the parameters window first prevents this (I don't know why this happens)
# it also looked like that ocured with smaller intervals of time

class MyWindow(QMainWindow):
    def __init__(self):
        super(MyWindow, self).__init__()
        self.setGeometry(200, 200, 1500, 500)
        self.setWindowTitle("Termorregulador")
        self.Temperature()
        self.pre_Fusion()
        self.Fusion()
        self.duration()
        self.Time()
        self.voltage()
        self.Current()

        self.time_update()
        self.plot()

    def time_update(self): # function that receives the data and triggers the update
        n_data = 50 # number of samples
        self.xdata = list(range(n_data))
        self.ydata = [random.randint(0, 10) for i in range(n_data)] # random data

        plt.ion() # enables interaction
        plt.tight_layout() # layout

        self.plot() # function plot
        plt.show() # display all opening figures

        self.timer = QtCore.QTimer()
        self.timer.setInterval(200) # in msec
        self.timer.timeout.connect(self.plot) # calls the function plot in every 200ms
        self.timer.start()

    def plot(self): # plot function
        self.ydata = self.ydata[1:] + [random.randint(0, 10)] # delete the first sample and adds a random one in the end.
        # this creates the effect of the data scrolling to the left

        plt.cla() # cleans the axes
        plt.plot(self.xdata, self.ydata) # plot
        plt.grid(True)

        plt.draw() # update the plot
        

    def Temperature(self):  # choose the temperature
        self.label = QtWidgets.QLabel(self)
        self.label.setText("Set Temperature")
        self.label.setFont(QFont('Arial', 15))
        self.label.move(200, 50)
        self.update()

        self.spinBox = QSpinBox(self)
        self.spinBox.move(600, 55)
        self.spinBox.setMinimum(0)
        self.spinBox.setMaximum(200)

        self.label1 = QtWidgets.QLabel(self)
        self.label1.setText("ºC")
        self.label1.setFont(QFont('Arial', 10))
        self.label1.move(720, 55)
        self.update()

    def pre_Fusion(self):  # choose the pre-heat temperature
        self.label = QtWidgets.QLabel(self)
        self.label.setText("Pre-Fusion Temperature")
        self.label.setFont(QFont('Arial', 15))
        self.label.move(200, 100)
        self.update()

        self.spinBox = QSpinBox(self)
        self.spinBox.move(600, 105)
        self.spinBox.setMinimum(0)
        self.spinBox.setMaximum(200)

        self.label1 = QtWidgets.QLabel(self)
        self.label1.setText("ºC")
        self.label1.setFont(QFont('Arial', 10))
        self.label1.move(720, 105)
        self.update()

    def Fusion(self):  # choose the fusion temperature
        self.label = QtWidgets.QLabel(self)
        self.label.setText("Fusion Temperature")
        self.label.setFont(QFont('Arial', 15))
        self.label.move(200, 150)
        self.update()

        self.spinBox = QSpinBox(self)
        self.spinBox.move(600, 155)
        self.spinBox.setMinimum(0)
        self.spinBox.setMaximum(200)

        self.label1 = QtWidgets.QLabel(self)
        self.label1.setText("ºC")
        self.label1.setFont(QFont('Arial', 10))
        self.label1.move(720, 155)
        self.update()

    def duration(self):  # choose the fusion duration
        self.label = QtWidgets.QLabel(self)
        self.label.setText("Fusion Duration")
        self.label.setFont(QFont('Arial', 15))
        self.label.move(200, 200)
        self.update()

        self.spinBox = QSpinBox(self)
        self.spinBox.move(600, 205)
        self.spinBox.setMinimum(0)
        self.spinBox.setMaximum(200)

        self.label1 = QtWidgets.QLabel(self)
        self.label1.setText("ms")
        self.label1.setFont(QFont('Arial', 10))
        self.label1.move(720, 205)
        self.update()

    def Time(self):  # choose the cycle time
        self.label = QtWidgets.QLabel(self)
        self.label.setText("Cycle Time")
        self.label.setFont(QFont('Arial', 15))
        self.label.move(200, 250)
        self.update()

        self.spinBox = QSpinBox(self)
        self.spinBox.move(600, 255)
        self.spinBox.setMinimum(0)
        self.spinBox.setMaximum(200)

        self.label1 = QtWidgets.QLabel(self)
        self.label1.setText("ms")
        self.label1.setFont(QFont('Arial', 10))
        self.label1.move(720, 255)
        self.update()

    def voltage(self):  # voltage calibration
        self.label = QtWidgets.QLabel(self)
        self.label.setText("Voltage Calibration")
        self.label.setFont(QFont('Arial', 15))
        self.label.move(200, 300)
        self.update()

        self.spinBox = QSpinBox(self)
        self.spinBox.move(600, 305)
        self.spinBox.setMinimum(0)
        self.spinBox.setMaximum(200)

        self.label1 = QtWidgets.QLabel(self)
        self.label1.setText("mV")
        self.label1.setFont(QFont('Arial', 10))
        self.label1.move(720, 305)
        self.update()

    def Current(self):  # current calibration
        self.label = QtWidgets.QLabel(self)
        self.label.setText("Current Calibration")
        self.label.setFont(QFont('Arial', 15))
        self.label.move(200, 350)
        self.update()

        self.spinBox = QSpinBox(self)
        self.spinBox.move(600, 355)
        self.spinBox.setMinimum(0)
        self.spinBox.setMaximum(200)

        self.label1 = QtWidgets.QLabel(self)
        self.label1.setText("mA")
        self.label1.setFont(QFont('Arial', 10))
        self.label1.move(720, 355)
        self.update()

    def update(self):
        self.label.adjustSize()

def window():
    app = QApplication(sys.argv)
    win = MyWindow()
    win.show()
    sys.exit(app.exec_())
window()
