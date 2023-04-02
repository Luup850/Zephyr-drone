 /***************************************************************************
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

#include "usbus.h"
#include "control.h"
#include "eeconfig.h"
#include "main.h"
#include "mixer.h"
#include "ustate.h"
#include "usubss.h"
#include <string> // MC: Added for debugging

USbus rc;

void USbus::setup()
{
  rcbus->begin();
  addPublishItem("rc", "Get RC receiver information (SBUS data).\r\n");
  addPublishItem("rco","Get RC receiver offset and scale.\r\n");
}


void USbus::tick(uint32_t tickCnt)
{
  if (rcbus->read(channel, &rcLost, &lostFrame))
  { // got full package
    if (lostFrame)
    {
      lostFrames++;
      if (lostFrames > 10)
        rcLost = true;
    }
    else
    {
      rcLost = false;
      lostFrames = 0;
      frameCnt++;
      sampleTime = float(ARM_DWT_CYCCNT - lastFrameTime)/(F_CPU / 1000);
      lastFrameTime = ARM_DWT_CYCCNT;
      // set as reference
      stickToRef();
    }
  }
  else
  { // time since last frame
    if (sampleTime < 10)
    {
      sampleTime = 20; // ms 
      lastFrameTime = ARM_DWT_CYCCNT;
    }
    noFrameTime = float(ARM_DWT_CYCCNT - lastFrameTime)/(F_CPU / 1000);
    lostFrames = int(noFrameTime / sampleTime);
    if (lostFrames > 10)
      rcLost = true;
  }
}

void USbus::tick()
{
  subscribeTick();
}

void USbus::sendData(int item)
{
  if (item == 0)
    sendStatus();
  else if (item == 1)
    sendScaleOffset();
}


void USbus::stickToRef()
{
  // stick deliveres between 170 and 1800, midt is 990
  float h = float(channel[0] - offset[0])*scale[0];
  float r = float(channel[1] - offset[1])*scale[1];
  float p = float(channel[2] - offset[2])*scale[2];
  float y = float(channel[3] - offset[3])*scale[3];
  if (not state.isUsbControl())
    // manual flight control using RC, so implement
    control.setRef(h, r, p, y);
}

void USbus::sendHelp()
{
  usb.send("# ------------- SBUS help\r\n");
  subscribeSendHelp();
  usb.send("#   rcos S O S O ...  Set receiver channel scale and offset (7 channels).\r\n");
}

bool USbus::decode(const char* buf)
{
  bool used = true;
  if (subscribeDecode(buf))  {}
  else if (strncmp(buf, "rcos ", 4) == 0)
  {
    const char * p1 = &buf[4];
    const char * p2;
    bool moreParams = false;
    while (isspace(*p1)) p1++;
    p2 = strchr(p1, ' ');
    moreParams = p2 != NULL;
    if (moreParams and strlen(p2) > 15)
    { // further parameters, so set controller
      for (int i = 0; i < 7; i++)
      {
        scale[i] = strtof(p1,(char**)&p1);
        offset[i] = strtol(p1,(char**)&p1,10);
      }
      if (p1 == p2)
        // error in decoding
        usb.send("# error in decoding scale and offset, requires 0 or 14 parameters\r\n");
//       else
//         usb.send("message RC scale and offset decoded\n");
    }
    else
      sendScaleOffset();
  }
  else
    used = false;
  return used;
}


void USbus::sendStatus()
{
  const int MSL = 250;
  char s[MSL];
  snprintf(s, MSL, "rc %d %d %d %.2f  %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d %d\n",
             rcLost, frameCnt, lostFrames, sampleTime,
           channel[0], channel[1], channel[2], channel[3],
           channel[4], channel[5], channel[6], channel[7],
           channel[8], channel[9], channel[10], channel[11],
           channel[12], channel[13], channel[14], channel[15]);
  usb.send(s);
//   usb.send("# send RC\n");
}

void USbus::sendScaleOffset()
{
  const int MSL = 250;
  char s[MSL];
  snprintf(s, MSL, "rco %g %d %g %d %g %d %g %d %g %d %g %d %g %d\n",
           scale[0], offset[0],
           scale[1], offset[1],
           scale[2], offset[2],
           scale[3], offset[3],
           scale[4], offset[4],
           scale[5], offset[5],
           scale[6], offset[6]
  );
  usb.send(s);
  //   usb.send("# send RC scale offset\n");
}

bool USbus::armRequestCheck()
{
  bool armable = false;
  // Arm start is "SC" - right long switch - pushed down (back)
  if (not rcLost)
  {
    int sw = switchToChannel('C');
    if(channel[sw] > 1500)
      armable = true;
  }
  return armable;
}

// MC: Armed(RC) = false. Armed(Auto) = true
bool USbus::autoCheck()
{
  bool aut = false;
  // Arm start is "SC" - right long switch - pushed down (back)
  if (not rcLost)
  {
    int sw = switchToChannel('B');
    if(channel[sw] > 1500)
    {
      aut = true;
//       usb.send("# to auto\n");
    }
  }
  return aut;
}

int USbus::switchToChannel(char sw)
{
  char swU;
  int ch = -1;
  if (isLowerCase(sw))
    swU  = toUpperCase(sw);
  else
    swU = sw;
  if (mixer.droneType == UMixer::drone2x2)
  {
    switch(swU)
    {
      case 'A': ch = 5; break;
      case 'B': ch = 6; break; // auto
      case 'C': ch = 7; break; // arm
      case 'D': ch = 8; break;
      case 'E': ch = 9; break;
      case 'F': ch = 10; break;
      case 'G': ch = 11; break;
      case 'H': ch = 12; break; // log
      default: break;
    };
  }
  else
  { // assume 7-9 channel RX, with 
    // 3 switches configured as
    switch(swU)
    {
      case 'B': ch = 5; break; // auto
      case 'C': ch = 6; break; // arm
      case 'H': ch = 7; break; // log
      default: break;
    };
  }
  // channel is 0-based, so subtract 1 for index
  return ch-1;
}


bool USbus::logswitch()
{
  bool reply = false;
  if (not rcLost)
  { // all switches start being up (< 500), i.e. no action
    int sw = switchToChannel('H');
    if (channel[sw] > 500)
      reply = (channel[sw] - lastLogSwitch) > 500;
    lastLogSwitch = channel[sw];
  }
  return reply;
}

void USbus::eePromSave()
{
  // save desired PWM FRQ
  for (int i = 0; i < 7; i++)
    eeConfig.pushFloat(scale[i]);
  for (int i = 0; i < 7; i++)
    eeConfig.pushWord(offset[i]);
  // debug
  //   usb.send("#saved ESC data from ee (PWM frq, esc count)\r\n");
}

void USbus::eePromLoad()
{
  for (int i = 0; i < 7; i++)
    scale[i] = eeConfig.readFloat();
  for (int i = 0; i < 7; i++)
    offset[i] = eeConfig.readWord();
}

