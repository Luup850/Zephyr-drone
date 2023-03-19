#!/usr/bin/python
# -*- coding: utf-8 -*-

#/***************************************************************************
 #*   Copyright (C) 2020 by DTU                             *
 #*   jca@elektro.dtu.dk                                                    *
 #*
 #* Primary class, that holds all the rest
 #*
 #*   This program is free software; you can redistribute it and/or modify  *
 #*   it under the terms of the GNU General Public License as published by  *
 #*   the Free Software Foundation; either version 2 of the License, or     *
 #*   (at your option) any later version.                                   *
 #*                                                                         *
 #*   This program is distributed in the hope that it will be useful,       *
 #*   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 #*   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 #*   GNU General Public License for more details.                          *
 #*                                                                         *
 #*   You should have received a copy of the GNU General Public License     *
 #*   along with this program; if not, write to the                         *
 #*   Free Software Foundation, Inc.,                                       *
 #*   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 #***************************************************************************/

import sys 
import os
import threading
import numpy as np
#import pyqtgraph as pg
import serial
import time
import timeit
import socket
try:
  import configparser
except:
  import ConfigParser  
from upaint import *
from PyQt5 import QtWidgets, QtCore, QtGui

class UWiFi(object):
  threadRunning = False
  thread = []
  stopThread = False
  timeLastCnt = 0;
  sendOKTime = time.time()
  parent = []
  ui = []
  main = []
  debug = []

  wificlient = socket.socket()
  sendLock = threading.Lock()
  wifiPort = 24001
  hopo = ["none"]
  txCnt = 0
  txMiss = 0;
  rxCnt = 0
  connected = False
  wifiTimeLastCnt = 0;

  # color
  dtugreen = QtGui.QColor(0, 136, 53, 127)
  dtured   = QtGui.QColor(153, 0, 0, 127)
  dtugrey  = QtGui.QColor(239, 240, 241, 255)
  dtuyellow = QtGui.QColor(0xf6, 0xd0, 0x4d, 64)
  
  # methods / functions
  def __init__(self, parent):
    self.parent = parent
    self.ui = parent.ui

  def init(self):
    # connect frame
    self.main = self.parent.main
    self.debug = self.parent.debug
    self.ui.checkBox_connect_wifi.clicked.connect(self.connectClicked_wifi)
    self.close();
    self.thread = threading.Thread(target=self.run, name="drone_gui_WIFI_reader")
    self.thread.start()
    
  def isOpen(self):
    return self.connected
  

  def open(self):
    if not self.connected:
      self.hopo = self.ui.lineEdit_connect_wifi.text().split(':')
      try:
        self.wifiPort = int(str(self.hopo[1]), 0)
      except:
        self.wifiPort = 24001
      print("# Network opening to: hopo=" + str(self.hopo[0]) + " port " + str(self.wifiPort))
      try:
        for res in socket.getaddrinfo(str(self.hopo[0]), self.wifiPort, socket.AF_UNSPEC, socket.SOCK_STREAM):
          #print("# socket res " + str(res))
          af = res[0]
          socktype = res[1] 
          proto = res[2]
          canonname = res[3]
          sa = res[4] # both IP and port number
          try:
              self.wificlient = socket.socket(af, socktype, proto)
              self.wificlient.settimeout(0.5)
              #print("# wifi socket created")
          except OSError as msg:
              self.wificlient = None
              print("# Network connection timeout - retry")
              continue
          try:
              self.wificlient.connect(sa)
              self.connected = True
              self.timeLastCnt = 0
              print("# wifi connected")
          except OSError as msg:
              self.wificlient.close()
              self.wificlient = None
              print("# wifi connect failed")
              continue
          except:
            print("# Network other except")
          break
        pass
      except:
        print("# network address not found")
        self.debug.mainStatus += "Net not found: " + str(self.hopo[0]) + "\n"
        self.debug.mainStatusSet = True
        self.main.message("Net not found: " + str(self.hopo[0]))
    if self.connected:
      print("Network is open")
      self.sendOKTime = time.time()
      self.debug.mainStatus += "Networking on " + str(self.hopo[0]) + ":" + str(self.wifiPort) + "\n"
      self.debug.mainStatusSet = True
      #self.wifiWaiting4reply = False
      setFrameColor(self.ui.frame_wifi, self.dtugreen)
      pass
    pass
  
  
  def close(self):
    if (self.connected):
      print("Network stopping")
      self.connected = False
      #self.info.thisRobot.gotRobotName = False
      # finished sending and receiving (may send HUP?)
      try:
        self.wificlient.shutdown(2);
      except:
        print("socket shutdown error - no connections open - ignoring");
      self.wificlient.close()
      self.ui.statusbar.showMessage("Network client - disconnected", 2000)
      self.debug.mainStatus += "Network " + self.hopo[0] + " disconnected\n"
      self.debug.mainStatusSet = True
      #self.wifiWaiting4reply = False
      #self.talkToBridge = False
      #self.clearLastTab()
    if self.ui.checkBox_connect_wifi.isChecked():
      setFrameColor(self.ui.frame_wifi, self.dtured)
    else:
      setFrameColor(self.ui.frame_wifi, self.dtugrey)
    pass



  def connectClicked_wifi(self):
    if self.ui.checkBox_connect_wifi.isChecked():
      self.open()
    else:
      self.close()


  def run(self):
    count = 0
    m = 0
    n = 0
    c = 0
    b = '\0'
    self.threadRunning = True
    print("# WiFi thread running")
    got = ""
    self.wificlient.settimeout(0.3)
    while (not self.stopThread):
      if self.connected:
        n = 0
        if (b == '\n'):
          got = ""
          b = []
        try: 
          while (b != '\n' and self.connected):
            c = self.wificlient.recv(1)
            if (len(c) > 0):
              b = c.decode('utf-8')
              if (b >= ' ' or b == '\n'):
                # filter all control characters but newline
                got = got + b
          n = len(got)
        except:
          m = m + 1
          time.sleep(0.01)
          if m > 15:
            print("# Read from wifi failed " + str(m) + " times")
            self.close()
        if (n > 0):
          #print("# got (" + str(m) + ", len=" + str(n) + ")=" + got)
          self.rxCnt += 1
          self.main.decodeCommand(got, n, "n")
          m = 0;
      else:
        time.sleep(0.1)
    print("# WiFi read thread ended")
    self.threadRunning = False
    pass

  def stop(self):
    self.close()
    if self.threadRunning:
      self.stopThread = True
      self.thread.join(2)
      print("# wifi thread joined")


    ### send string to wifi socket
  def wifiWrite(self, s):
    self.sendLock.acquire()
    n = len(s)
    d = -1
    #print("# sending " + s)
    if (n > 0):
      #m = 0;
      if True: # try:
        #while m < n:
        #print("# trying to send " + s[m:])
        try:
          d = self.wificlient.send(s.encode())
          self.txCnt += 1
          self.txMiss = 0
        except:
          self.txMiss += 1
          if self.txMiss < 5:
            self.main.message("Net send fail " + str(self.txMiss))
          pass
    self.sendLock.release()
    if (d <= 0):
      dt = time.time() - self.sendOKTime
      #print("# send failed, lastOK " + str(dt) + "s, for " + s)
      if dt > 10:
        self.close()
    else:
     self.sendOKTime = time.time()
      # raise Exception("Write error wifi")
    return self.isOpen()


  def timerUpdate(self, timerCnt):
    if not self.isOpen():
      if self.ui.checkBox_connect_wifi.isChecked():
        self.wifiTimeLastCnt += 1
        setFrameColor(self.ui.frame_wifi, self.dtured)
        if self.wifiTimeLastCnt > 30:
          self.wifiTimeLastCnt = 0
          self.open()
          #print("# Trying to connect to wifi" + str(self.wifiTimeLastCnt))
        pass
      pass
      
