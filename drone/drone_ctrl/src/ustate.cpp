 /***************************************************************************
 * 
 *  Drone state control, 
 *  e.g. Disarmed, Armed
 *  Drone name and ID
 * 
 *   Copyright (C) 2021 by DTU                             *
 *   jca@elektro.dtu.dk                                                    *
 *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/

#include <stdio.h>
#include "control.h"
#include "ustate.h"
#include "command.h"
#include "uesc.h"
#include "upropshield.h"
#include "sensor.h"
#include "uled.h"
#include "eeconfig.h"
#include "mixer.h"
#include "uheight.h"
#include "uusb.h"

UState state;

UState::UState()
{
  /** format
   * hbt 1 : time in seconds, updated every sample time
   *     2 : device ID (probably 1)
   *     3 : software revision number - from SVN * 10 + REV_MINOR
   *     4 : Battery voltage (from AD converter)
   *     5 : arm state =0=init,1=disarmed,2=armed,3=fail
   *     6 : flight state: 0=on ground, 1=starting, 2=flying, 3=landing
   *   7-9 : sensor detect error [gyro, acc+magnetometer, pressure altitude] 0=OK
   * */
  
  addPublishItem("hbt", "Heartbeat: time, ID, ver, battery, arm-state, flight-state, errors...\n");
  addPublishItem("id", "Get drone name\n");
  subscribeDecode("sub hbt 400\n");
}

void UState::setup()
{
  time = 0.0;
  lasthbTime = hb10us;
  deviceID = 1;
  leds.stateSet(armState);
}


void UState::tick()
{ // system time in seconds (float)
  int dt = hb10us - lasthbTime;
  lasthbTime = hb10us;
  // set time in seconds
  if (dt >= 0 and dt < 500)
    time += float(dt) / 100000.0;
  // battery check
  bool lowBattery = sensor.isBatteryCriticallyLow();
  // are set-points controlled from USB connection (else RC)
  if (bypassRC)
    usbCtrl = true;
  else if (not rc.isRcLost())
    usbCtrl = rc.autoCheck();
  //
  // state change check
  switch (armState)
  {
    case Init:
      if (imu.sensorOffsetDone)
      {
        armState = Disarmed;
        leds.stateSet(armState);
      }
      break;
    case Disarmed:
      if (not imu.sensorOffsetDone)
        armState = Init;
      // check RC switches
      if (tryArm or rc.armRequestCheck())
      {
        if (mixer.canArm() and not lowBattery and (bypassRC or not rc.isRcLost()))
        {
          setArmed(true);
          leds.stateSet(armState);
          msgCnt = 0;
        }
        else
        {
          if (msgCnt < 5)
            usb.send("message no Arm (batt or RC or trust)\n");
          msgCnt++;
        }
        tryArm = false;
      }
      break;
    
    case Armed:
      // check remote control is lost
      if (lowBattery or (rc.isRcLost() and not bypassRC))
      {
        setFailed();
        leds.stateSet(armState);
        usb.send("message Bat low or lost RC\n");
      }
      else if (stopNow or (not rc.armRequestCheck() and not bypassRC))
      {
        setArmed(false);
        leds.stateSet(armState);
        stopNow = false;
        usb.send("message stop or disarm request\n");
      } 
      else
      { // still armed set band color dependent on trust
        int u = int(mixer.uHeight);
        leds.setBandColor(u / 4, 255 - u / 4, u / 8);
      }
      break;
    case Fail:
      /// low battery or high temperature
      if (not rc.isRcLost() or bypassRC or flightState == OnGround)
      { // will switch to landing, if in flight
        setArmed(false);
        leds.stateSet(armState);
      }
      break;
    default:
      usb.send("# UState:: unknown arm state\n");
      break;
  }
  switch(flightState)
  {
    case OnGround:
      if (mixer.uHeight > 200 and armState == Armed)
      { // trust is high (starting)
        if (startingCounter > 200)
        {
          flightState = Starting;
          startingCounter = 1000;
//           usb.send("message starting\n");
        }
        startingCounter++;
      }
      else
        startingCounter = 0;
      break;
    case Starting:
      // test if it is likely that we try to fly
      // trust test
      if (mixer.uHeight < 300)
      {
        flightState = OnGround;
        usb.send("message start abort\n");
      }
      // battery test
      if (armState == Armed)
      { // test movement, NB height may be wrong
        if(/*hgt.height > 0.05 and */ hgt.heightVelocity > 0.15 and sensor.batteryVoltage > 10.0)
          startingCounter += 10;
        else
          startingCounter--;
        // takes some time to get started
        if (startingCounter <= 0)
        {
          flightState = OnGround;
//           usb.send("message starting aborted\n");
        }
        else if (startingCounter > 4000 or hgt.heightVelocity >= 0.7)
        {
          flightState = InFlight;
          usb.send("message to in flight\n");
        }
      }
      else
      {
        flightState = OnGround;
//         usb.send("message grounded\n");
      }
      break;
    case InFlight:
      // MC: Added check to see if drone is in auto mode. If not refuse Landing state.
      // TODO: Add check that switches to landing state if prop speed is low for a period of time.
      // Added 1 == 2 to disable auto landing.

      if(rc.autoCheck() == true and 1 == 2 and hgt.height < 0.5 and hgt.heightVelocity < -0.1)
        startingCounter++;
      else
        startingCounter--;

      // Allow pilot in control to disable drone mid air (Not a good idea though).
      if(armState == Disarmed)
      {
        flightState = OnGround;
        usb.send("message emergency stop\n");
      }


      // takes some time to trust
      if (startingCounter <= 0 and rc.autoCheck() == true and 1 == 2)
        startingCounter = 0;
      else if (startingCounter > 1000 and rc.autoCheck() == true and 1 == 2)
      {
        flightState = Landing;
        usb.send("message landing\n");
      }
      // emergency landing
      break;
    case Landing:
      // we are in an auto landing condition
      if (landingCounter == 1)
        usb.send("message *** auto landing ***\n");
      landingCounter++;
      // landing time is 2 seconds
      if (landingCounter > int(landTime/SAMPLETIME))
      { // assume landed - or hanging in net/tree
        flightState = OnGround;
        startingCounter = 0;
        usb.send("message landed\n");
      }
      break;
    default:
      usb.send("# UState:: unknown flight state\n");
      break;      
  }
  // subscription service
    subscribeTick();
}


void UState::sendData(int item)
{
  if (item == 0)
    sendState();
  else if (item == 1)
    sendID();
}


void UState::setArmed(bool value)
{ // change ARM state
  if (value)
  {
    armState = Armed;
    usb.send("message Armed\n");
    // on-board LED
    digitalWriteFast(PIN_LED_ARMED, LED_ON);
  }
  else
  { // return to unarmed and recover from fail mode
    armState = Disarmed;
    esc.stopAllMotors();
    usb.send("message *** Disarmed ***\n");
    // on-board LED
    digitalWriteFast(PIN_LED_ARMED, LED_OFF);
  }
}

void UState::setFailed()
{
  armState = Fail;
  usb.send("message *** State to fail (Battery low or RC lost) ***\n");
  leds.stateSet(armState);
}


void UState::sendState()
{
  const int MRL = 100;
  char reply[MRL];
  /** format
   * hbt 1 : time in seconds, updated every sample time
   *     2 : device ID (probably 1)
   *     3 : software revision number - from SVN * 10 + REV_MINOR
   *     4 : Battery voltage (from AD converter)
   *     5 : arm state =0=init,1=disarmed,2=armed,3=fail
   *     6 : flight state: 0=on ground, 1=starting, 2=flying, 3=landing
   *   7-9 : sensor detect error [gyro, acc+magnetometer, pressure altitude] 0=OK
   * */
  snprintf(reply, MRL,  "hbt %.4f %d %d %.2f %d %d %d %d\r\n",  time, deviceID,
           command.getRevisionNumber()/10, sensor.batteryVoltage, armState, flightState, bypassRC, usbCtrl);
  usb.send(reply);
}


void UState::sendHelp()
{
  const int MRL = 150;
  char reply[MRL];
  usb.send("# ------ State settings -------\r\n");
  snprintf(reply, MRL, "#   setid N      Set drone ID to N (id=%d, part of hbt).\r\n", deviceID);
  usb.send(reply);
  snprintf(reply, MRL, "#   setname string  Set drone name to string (< 32 chars).\r\n");
  usb.send(reply);
  snprintf(reply, MRL, "#   alive        Get time and state (hbt) (try also 'sub state 400')\r\n");
  usb.send(reply);
  snprintf(reply, MRL, "#   stop         Terminate (disarm) now\r\n");
  usb.send(reply);
  snprintf(reply, MRL, "#   arm B        Arm (arm B=1, disarm B=0)\r\n");
  usb.send(reply);
  subscribeSendHelp();
}

bool UState::decode(const char* buf)
{
  bool used = true;
  if (strncmp(buf, "alive ", 5) == 0)
  {
    sendState();
  }
  else if (strncmp(buf, "arm", 3) == 0)
  {
    tryArm = true;
  }
  else if (strncmp(buf, "stop", 6) == 0)
  { // stop in any case
    stopNow = true;;
  }
  else if (strncmp(buf, "bypass", 6) == 0)
  {
    const char * p1 = &buf[6];
    bool oldCtrl = usbCtrl;
    bool oldBypass = bypassRC;
    bypassRC = strtol(p1, (char**)&p1, 10);
    usbCtrl = strtol(p1, (char**)&p1, 10);
    // debug
    const int MSL = 100;
    char s[MSL];
    snprintf(s, MSL, "# bypass=%d, man=%d, %d chars: %s", bypassRC, usbCtrl, strlen(buf), buf);
    usb.send(s);
    // debug_end
    if (bypassRC != oldBypass)
    {
      if (bypassRC)
        usb.send("message RC failsafe bypassed\n");
      else
        usb.send("message RC failsafe NOT bypassed\n");
    }
    if (usbCtrl != oldCtrl)
    {
      if (usbCtrl)
        usb.send("message Auto (USB) control\n");
      else
        usb.send("message RC control\n");
    }
  }
  else if (strncmp(buf, "setid", 5) == 0)
  {
    const char * p1 = &buf[5];
    deviceID = strtol(p1, NULL, 10);
  }
  else if (strncmp(buf, "setname", 7) == 0)
  { // set drone name
    const char * p1 = &buf[7];
    while (isSpace(*p1))
      p1++;
    if (*p1 < ' ')
    {
      droneName[0] = '_';
      droneName[1] = '\0';
      usb.send("# set name to nothing ('_')\n");
    }
    else
    {
      usb.send("# got new name (get with 'id')\n");
      for (int i = 0; i < MAX_NAME_LENGTH-1; i++)
      {
        if (*p1 <= ' ')
        {
          droneName[i] = '\0';
          break;
        }
        if (isalnum(*p1))
          droneName[i] = *p1;
        else
          droneName[i] = '_';
        p1++;
      }
      droneName[MAX_NAME_LENGTH-1] = '\0';
    }
  }
  else if (subscribeDecode(buf)) {}
  else
    used = false;
  return used;
}


void UState::sendID()
{
  const int MRL = 100;
  char reply[MRL];
  snprintf(reply, MRL, "id %s\r\n", droneName);
  usb.send(reply);
}

void UState::eePromLoad()
{
  deviceID = eeConfig.readWord();
  eeConfig.readBlock(droneName, MAX_NAME_LENGTH);
  // ensure it is zero terminated - to avoid garbage
  droneName[MAX_NAME_LENGTH-1] = '\0';
  for (int i = 0; i < MAX_NAME_LENGTH; i++)
  {
    if (droneName[i] != '\0' and not isalnum(droneName[i]))
      droneName[i] = '_';
  }
}

void UState::eePromSave()
{
  eeConfig.pushWord(deviceID);
  eeConfig.pushBlock(droneName, MAX_NAME_LENGTH);
}

