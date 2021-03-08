from PyQt5 import QtWidgets
from PyQt5.QtWidgets import QApplication, QMainWindow, QSpinBox, QLabel, QVBoxLayout
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *

import sys
import time

# Enable/Disable '=' button - to do
# Print Alarm (delay when there is the transition to the state Alarm) - to do

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
        
        self.ChangeFSM()
        self.SignalZero()
        self.SignalOne()
        self.SignalEqual()
        self.ChoosePreHeat()
        self.ChooseAlarm()

    def Temperature(self): # choose the temperature
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

    def pre_Fusion(self): # choose the pre-heat temperature
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

    def Fusion(self): # choose the fusion temperature
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

    def duration(self): # choose the fusion duration
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

    def Time(self): # choose the cycle time
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

    def voltage(self): # voltage calibration
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

    def Current(self): # current calibration
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

    def ChangeFSM(self): # change FSM button 
        self.label = QtWidgets.QLabel(self)
        self.label.setText("2. Change the State Machine:")
        self.label.setFont(QFont('Arial', 11))
        self.label.move(900, 90)
        self.update()

    def SignalZero(self): # change the FSM machine with signal '0'
        self.button = QtWidgets.QPushButton(self)
        self.button.setText("Signal '0'")
        self.button.setFont(QFont('Arial', 11))
        self.button.move(900, 130)
        self.button.clicked.connect(ClickZero)
        self.update()

    def SignalOne(self): # change the FSM machine with signal '1'
        self.button = QtWidgets.QPushButton(self)
        self.button.setText("Signal '1'")
        self.button.setFont(QFont('Arial', 11))
        self.button.move(900, 170)
        self.button.clicked.connect(ClickOne)
        self.update()

    def SignalEqual(self):  # change the FSM machine with signal '='. Is used when the temperature equals the setpoint
        self.button = QtWidgets.QPushButton(self)
        self.button.setText("=")
        self.button.setFont(QFont('Arial', 11))
        self.button.move(900, 210)
        self.button.clicked.connect(Equal)
        #self.button.setEnabled(enable_button_equal)
        self.update()

    def ChoosePreHeat(self): # the user chooses if it is wanted to pre-heat before the final temperature raise
        self.check = QtWidgets.QCheckBox(self)
        self.check.move(1045, 50)
        self.update()

        self.label = QtWidgets.QLabel(self)
        self.label.setText("1. With Pre-Heat:")
        self.label.setFont(QFont('Arial', 11))
        self.label.move(900, 55)
        self.update()

        self.button = QtWidgets.QPushButton(self)
        self.button.setText("Check")
        self.button.setFont(QFont('Arial', 11))
        self.button.move(1080, 50)
        self.button.clicked.connect(self.Disable)
        self.update() 

    def ChooseAlarm(self): # signal of alarm that alerts something is wrong
        self.button = QtWidgets.QPushButton(self)
        self.button.setText("Reset")
        self.button.setFont(QFont('Arial', 11))
        self.button.move(900, 250)
        self.button.clicked.connect(Alarm)
        self.update() 
    
    def update(self):
        self.label.adjustSize()

    def Disable(self): # Disable the checkbox and enable the state machine buttons
        self.check.setEnabled(False)

        global checkBox_checked 
        checkBox_checked = self.check.isChecked()
        #print(checkBox_checked)

# States of the FSM
State = type("State", (object,), {})

class off(State):
    def Execute(self):
        print ("Sistema desligado")

class on(State):
    def Execute(self):
        print ("Sistema ligado")

class initcycle(State):
    def Execute(self):
        print ("Inicio de ciclo")

class preheat(State):
    def Execute(self):
        print ("Pre-heat")

class raisetemp(State):
    def Execute(self):
        print ("Aumentar temperatura")

class sealing(State):
    def Execute(self):
        print ("Selagem")

class alarm(State):
    def Execute(self):
        print ("Alarme")

class Transition(object): # transitions of the states
    def __init__(self,toState):
        self.toState = toState

    def Execute(self):
        print ("Transition...")

class StateMachine(object): # create the FSM
    def __init__(self,char):
        self.char = char
        self.states = {}
        self.transitions = {}
        self.curState = None
        self.trans = None

    def SetState(self,stateName):
        self.curState = self.states[stateName]

    def Transition(self, transName):
        self.trans = self.transitions[transName]

    def Execute(self):
        if(self.trans):
            self.trans.Execute()
            self.SetState(self.trans.toState)
            self.trans = None
        self.curState.Execute()

class Char(object):
    def __init__(self):
        self.FSM = StateMachine(self)
        self.off = True
       
if __name__ == "__main__": # connect the states with the transitions
    termo = Char()  

    termo.FSM.states["SystemOff"] = off()
    termo.FSM.states["SystemOn"] = on()
    termo.FSM.states["InitiateCycle"] = initcycle()  
    termo.FSM.states["Preheat"] = preheat()
    termo.FSM.states["RaiseTemperature"] = raisetemp()
    termo.FSM.states["Sealing"] = sealing()    
    termo.FSM.states["Alarm"] = alarm()  

    termo.FSM.transitions["toSystemOff"] = Transition("SystemOff")
    termo.FSM.transitions["toSystemOn"] = Transition("SystemOn")
    termo.FSM.transitions["toInitiateCycle"] = Transition("InitiateCycle")
    termo.FSM.transitions["toPreheat"] = Transition("Preheat")
    termo.FSM.transitions["toRaiseTemperature"] = Transition("RaiseTemperature")
    termo.FSM.transitions["toSealing"] = Transition("Sealing")
    termo.FSM.transitions["toAlarm"] = Transition("Alarm")

    termo.FSM.SetState("SystemOff")

    def ClickZero(): # when the input is '0', define the transitions and the next state
        if(termo.off): 
            termo.FSM.Transition("toSystemOff")
            termo.off = True
            termo.on = False
            termo.initcycle = False
            termo.preheat = False
            termo.raisetemp = False
            termo.sealing = False
            termo.alarm = False

        elif(termo.on):
            termo.FSM.Transition("toSystemOff")
            termo.off = True
            termo.on = False
            termo.initcycle = False
            termo.preheat = False
            termo.raisetemp = False
            termo.sealing = False
            termo.alarm = False

        elif(termo.initcycle):
            termo.FSM.Transition("toSystemOn")
            termo.off = False
            termo.on = True
            termo.initcycle = False
            termo.preheat = False
            termo.raisetemp = False
            termo.sealing = False
            termo.alarm = False

        elif(termo.preheat):
            termo.FSM.Transition("toInitiateCycle")
            termo.off = False
            termo.on = False
            termo.initcycle = True
            termo.preheat = False
            termo.raisetemp = False
            termo.sealing = False
            termo.alarm = False

        elif(termo.sealing):
            termo.FSM.Transition("toInitiateCycle")
            termo.off = False
            termo.on = False
            termo.initcycle = True
            termo.preheat = False
            termo.raisetemp = False
            termo.sealing = False
            termo.alarm = False

        termo.FSM.Execute()

    def ClickOne(): # when the input is '0', define the transitions and the next state
        if(termo.off):
            termo.FSM.Transition("toSystemOn")
            termo.off = False
            termo.on = True
            termo.initcycle = False
            termo.preheat = False
            termo.raisetemp = False
            termo.sealing = False
            termo.alarm = False

        elif(termo.on):
            termo.FSM.Transition("toInitiateCycle")
            termo.off = False
            termo.on = False
            termo.initcycle = True
            termo.preheat = False
            termo.raisetemp = False
            termo.sealing = False
            termo.alarm = False

        elif(termo.initcycle):
            termo.off = False
            termo.on = False
            termo.initcycle = False

            if(checkBox_checked == True):
                termo.FSM.Transition("toPreheat")
                termo.preheat = True
                termo.raisetemp = False
            elif(checkBox_checked == False):
                termo.FSM.Transition("toRaiseTemperature")
                termo.raisetemp = True
                termo.preheat = False

            termo.sealing = False
            termo.alarm = False

        elif(termo.preheat):
            termo.FSM.Transition("toRaiseTemperature")
            termo.off = False
            termo.on = False
            termo.initcycle = False
            termo.preheat = False
            termo.raisetemp = True
            termo.sealing = False
            termo.alarm = False

        termo.FSM.Execute()

    def Equal(): # when the temperature equals the setpoint, define the transition and the next state
        if(termo.raisetemp):
            termo.FSM.Transition("toSealing")
            termo.off = False
            termo.on = False
            termo.initcycle = False
            termo.preheat = False
            termo.raisetemp = False
            termo.sealing = True
            termo.alarm = False

        termo.FSM.Execute()

    def Alarm(): # when there is an alarm signal, go to alarm state
        termo.FSM.Transition("toAlarm")
        termo.initcycle = False
        termo.preheat = False
        termo.raisetemp = False
        termo.sealing = False
        termo.alarm = True

        time.sleep(2) # 2 second delay

        # Reseting
        if(termo.on == True):
            termo.FSM.Transition("toSystemOn") 
        else:
            termo.FSM.Transition("toSystemOff") 

        termo.FSM.Execute()

def window():
    app = QApplication(sys.argv)
    win = MyWindow()
    win.show()
    sys.exit(app.exec_())


window()
