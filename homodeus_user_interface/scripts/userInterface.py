#!/usr/bin/python
# -*- coding: utf-8 -*-

import sys
from PyQt4 import QtGui
import rospy
from PyQt4.QtCore import QObject
from PyQt4.QtWidgets import QMainWindow

class UserInterface(QObject):

    def __init__(self, parent=None):
        super(self.__class__, self).__init__(parent)
        # app = QtGui.QApplication(sys.argv)
        # w = QtGui.QWidget()
        # b = QtGui.QLabel(w)
        # b.setText("Hello World!")
        # w.setGeometry(100,100,200,50)
        # b.move(50,20)
        # w.setWindowTitle("PyQt")
        # w.show()
        # sys.exit(app.exec_())

        self.gui = MyMainWindow()
        self.gui.show()       


class MyMainWindow(QMainWindow):
    
    def __init__(self):
        super().__init__()
        self.title = 'Simulateur'
        self.left = 100
        self.top = 100
        self.width = 640
        self.height = 400
        self.initUI()
        
    def initUI(self):
        #Adding the widgets and initializing the main window
        self.mainWidgets = MainWidgets()
        self.setCentralWidget(self.mainWidgets)
        self.setWindowTitle(self.title)
        self.setGeometry(self.left, self.top, self.width, self.height)

class MainWidgets(QWidget):
    
    def __init__(self):
        super().__init__()     
        self.initUI()
        

    def initUI(self):

        #creating layouts
        
        mainlayout = QVBoxLayout()
        headerLayout = QGridLayout()
        tabSelectionLayout = QHBoxLayout()
        
        simulationTabLayout = QVBoxLayout()
        configurationTabLayout = QGridLayout()      
        
        transmissionLayout = QGridLayout()
        loadLayout = QGridLayout()
        communicationLayout = QVBoxLayout()           
        outputLayout = QGridLayout()

        self.setLayout(mainlayout)
        
        
        #Adding tabs
        tabs = QTabWidget()
        self.simulationTab = QWidget()
        self.configurationTab = QWidget()
        tabs.resize(300,200)
        
        tabs.addTab(self.simulationTab,"Simulation")
        tabs.addTab(self.configurationTab,"Configuration")
        
        self.simulationTab.setLayout(simulationTabLayout)
        self.configurationTab.setLayout(configurationTabLayout)
        
        #Creating header widget
        self.comPortText = QLabel('COM port')
        self.comPortTextField = QLineEdit(self)
        self.connectButton = QPushButton("Connect")
        self.disconnectButton = QPushButton("Disconnect")  
        self.spaceText = QLabel('')
        
        #Creating simulationTab widgets
        self.transmissionText = QLabel('Transmission')
        self.loadText = QLabel('Load')
        self.communicationText = QLabel('Communication')
        self.commandText = QLabel('Command')
        self.commandTextField = QLineEdit(self)
        self.sendStringButton = QPushButton("Send as String")
        self.sendHexButton = QPushButton("Send as Hex")
        self.commandSelectionButton = QComboBox()
        self.repeatText = QLabel('Repeat')
        self.repeatTextField = QLineEdit(self)
        self.repeatStringButton = QPushButton("Repeat as String")
        self.repeatHexButton = QPushButton("Repeat as Hex")   
        self.repeatSelectionButton = QComboBox()
        self.intervalText = QLabel('Interval')
        self.intervalTextField = QLineEdit(self)
        self.repeatStopButton = QPushButton("Repeat Stop")
        self.browseButton = QPushButton("Browse")
        self.browseTextField = QLineEdit(self)
        self.loadFilesButton = QPushButton("Load File")
        self.clearButton = QPushButton("Clear")
        self.outputText = QLabel("Output")
        self.outputTextField = QPlainTextEdit()
        
        
        #Creating configurationTab widgets
        self.baudRateText = QLabel('Baud Rate')
        self.flowControlText = QLabel('Flow Control')
        self.bitsStopsText = QLabel('Bits Stops')
        self.parityText = QLabel('Parity')
        self.baudRateTextField = QLineEdit(self)
        self.flowControlSelectionButton = QComboBox()
        self.bitsStopsSelectionButton = QComboBox()
        self.paritySelectionButton = QComboBox()
        
        #Drop down menus configuration
        self.flowControlSelectionButton.addItem("None")
        self.flowControlSelectionButton.addItem("XonXoff")
        self.flowControlSelectionButton.addItem("RequestToSend")
        self.flowControlSelectionButton.addItem("RequestToSendXonXoff")
        
        self.bitsStopsSelectionButton.addItem("None")
        self.bitsStopsSelectionButton.addItem("One")
        self.bitsStopsSelectionButton.addItem("Two")
        self.bitsStopsSelectionButton.addItem("OnePointfive") 
        
        self.paritySelectionButton.addItem("None")
        self.paritySelectionButton.addItem("Odd")
        self.paritySelectionButton.addItem("Even")
        self.paritySelectionButton.addItem("Mark")    
        self.paritySelectionButton.addItem("Space")        
        
        #Texts field configuration
        self.comPortTextField.setFixedWidth(60)
        self.intervalTextField.setFixedWidth(60)
        self.outputTextField.setFixedWidth(600)
        self.outputTextField.setFixedHeight(100)
        
        #Setting default values
        self.comPortTextField.setText("COM6")
        self.baudRateTextField.setText("9600")
        self.bitsStopsSelectionButton.setCurrentIndex(1)
        self.intervalTextField.setText("1")
        
        
        #adding widgets to each layout
        headerLayout.addWidget(self.comPortText, 0, 0)
        headerLayout.addWidget(self.comPortTextField, 0, 1)
        headerLayout.addWidget(self.spaceText, 0, 2, 1, 5)
        headerLayout.addWidget(self.connectButton, 0, 5, 1, 1)
        headerLayout.addWidget(self.disconnectButton, 0, 6, 1, 1)
        
        transmissionLayout.addWidget(self.transmissionText, 0, 0)
        transmissionLayout.addWidget(self.commandText, 1, 0)
        transmissionLayout.addWidget(self.commandTextField, 1, 1, 1, 8)
        transmissionLayout.addWidget(self.sendStringButton, 2, 1, 1, 4)
        transmissionLayout.addWidget(self.sendHexButton, 2, 5, 1, 4)
        transmissionLayout.addWidget(self.commandSelectionButton, 3, 1, 1, 8)
        transmissionLayout.addWidget(self.repeatText, 4, 0)
        transmissionLayout.addWidget(self.repeatTextField, 4, 1, 1, 6)
        transmissionLayout.addWidget(self.intervalText, 4, 7)
        transmissionLayout.addWidget(self.intervalTextField, 4, 8, 1, 1)
        transmissionLayout.addWidget(self.repeatStringButton, 5, 1, 1, 3)
        transmissionLayout.addWidget(self.repeatHexButton, 5, 4, 1, 3)
        transmissionLayout.addWidget(self.repeatStopButton, 5, 7, 1, 2)
        transmissionLayout.addWidget(self.repeatSelectionButton, 6, 1, 1, 6)
    
        loadLayout.addWidget(self.loadText, 0, 0)     
        loadLayout.addWidget(self.browseButton, 1, 0)
        loadLayout.addWidget(self.browseTextField, 1, 1)
        loadLayout.addWidget(self.loadFilesButton, 1, 2)
        
        communicationLayout.addWidget(self.communicationText)
        
        configurationTabLayout.addWidget(self.baudRateText, 0, 0)
        configurationTabLayout.addWidget(self.flowControlText, 1, 0)
        configurationTabLayout.addWidget(self.bitsStopsText, 2, 0)
        configurationTabLayout.addWidget(self.parityText, 3, 0)      
        configurationTabLayout.addWidget(self.baudRateTextField, 0, 1)
        configurationTabLayout.addWidget(self.flowControlSelectionButton, 1, 1)
        configurationTabLayout.addWidget(self.bitsStopsSelectionButton, 2, 1)
        configurationTabLayout.addWidget(self.paritySelectionButton, 3, 1)
        
        outputLayout.addWidget(self.outputText, 0, 0)
        outputLayout.addWidget(self.spaceText, 0, 1, 1, 7)
        outputLayout.addWidget(self.spaceText, 0, 7, 1, 7)
        outputLayout.addWidget(self.clearButton, 0, 15, 1, 1)
        outputLayout.addWidget(self.outputTextField, 1, 0, 1, 16)
        
        tabSelectionLayout.addWidget(tabs)
        
        #Disabling some widgets at init
        self.disconnectButton.setDisabled(True)
        self.commandTextField.setDisabled(True)
        self.sendStringButton.setDisabled(True)
        self.sendHexButton.setDisabled(True)
        self.commandSelectionButton.setDisabled(True)
        self.repeatTextField.setDisabled(True)
        self.repeatStringButton.setDisabled(True)
        self.repeatHexButton.setDisabled(True)
        self.repeatSelectionButton.setDisabled(True)
        self.intervalTextField.setDisabled(True)
        self.repeatStopButton.setDisabled(True)
        self.browseButton.setDisabled(True)
        self.browseTextField.setDisabled(True)
        self.loadFilesButton.setDisabled(True)
        self.outputTextField.setReadOnly(True)
        
        #Layout alignment
        mainlayout.setAlignment(Qt.AlignTop)
        headerLayout.setAlignment(Qt.AlignTop)
        tabSelectionLayout.setAlignment(Qt.AlignTop)
        
        simulationTabLayout.setAlignment(Qt.AlignTop)
        configurationTabLayout.setAlignment(Qt.AlignTop)
        
        transmissionLayout.setAlignment(Qt.AlignTop)
        loadLayout.setAlignment(Qt.AlignTop)
        communicationLayout.setAlignment(Qt.AlignTop)  
        outputLayout.setAlignment(Qt.AlignTop)
        
        #Adding layout to each other
        simulationTabLayout.addLayout(transmissionLayout)
        simulationTabLayout.addLayout(loadLayout)
        simulationTabLayout.addLayout(communicationLayout)
            
        communicationLayout.addLayout(outputLayout)
        
        mainlayout.addLayout(headerLayout)
        mainlayout.addLayout(tabSelectionLayout)
        
        simulationTabLayout.addStretch()

if __name__ == '__main__':

    #rospy.init_node('UserIterface', anonymous=False)
    gui = UserInterface()
    #rospy.spin()
    #sudo apt-get install python-qt4
    #sudo apt install libcanberra-gtk-module