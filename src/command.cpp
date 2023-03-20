/***************************************************************************
 *   Copyright (C) 2021 by DTU                             *
 *   jca@elektro.dtu.dk                                                    *
 *
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
 

#define REV_ID "$Id: command.cpp 1362 2022-07-04 14:15:18Z jcan $" 

#include <malloc.h>
#include <ADC.h>
#include "IntervalTimer.h"
#include "main.h"
#include "eeconfig.h"
#include "command.h"
#include "uesc.h"
#include "sensor.h"
#include "upropshield.h"
#include "logger.h"
#include "usbus.h"
#include "control.h"
#include "ustate.h"
#include "mixer.h"
#include "uled.h"
#include "ultrasound.h"
#include "uheight.h"
#include "ulaser.h"
#include "uusb.h"

UCommand command;

void UCommand::setup()
{
  addPublishItem("ver", "Get version string\r\n");
}


/**
 * Get SVN revision number */
uint16_t UCommand::getRevisionNumber()
{
  const char * p1 = strstr(REV_ID, ".cpp");
  return strtol(&p1[4], NULL, 10) * 10 + REV_MINOR;
}


//////////////////////////////////////////


void UCommand::sendStatusVersion()
{
  const int MRL = 100;
  char reply[MRL];
  snprintf(reply, MRL, "ver %.1f %s\r\n", (float)getRevisionNumber() / 10.0, REV_ID);
  usb.send(reply);
}


// parse a user command and execute it, or print an error message
//
void UCommand::parse_and_execute_command(char * buf)
{ // command may be preceded by 'robot' or 'teensy'
  if (strncmp(buf, "robot ", 6) == 0)
  {
    buf += 6; // move pointer 7 characters forward to discard
    usb.send("# discarding the 'robot' part\n");
    while (*buf == ' ')
      buf++;
  }
  else if (strncmp(buf, "teensy ", 7) == 0)
  {
    buf += 7; // move pointer 7 characters forward to discard
    while (*buf == ' ')
      buf++;
    usb.send("# discarding the 'teensy' part\n");
  }
  // check if commands are handled by someone (most urgent first)
  if (control.decode(buf)) {} // a control issue - decoded there, handles ref, limit, mix
  else if (rc.decode(buf)) {} // a RC issue - decoded there, handled: rci, rcos
  else if (state.decode(buf)) {} // handled: arm, stop, bypass
  else if (mixer.decode(buf)) {}     // handles mix
//   else if (subscribe->decode(buf)) {} // handles 'sub'
  else if (leds.decode(buf)) {} // handles 'led and leds'
  else if (uhgt.decode(buf)) {} // handles 'uht' (sonar)
  else if (uhlas.decode(buf)) {} // handles lass, lasl and laser (laser height)
  else if (hgt.decode(buf)) {} // handles 'hgt' (combined height), altfilt
  else if (sensor.decode(buf)) {} // handles sensor, temp, batt, seq, amps, time, adctick
  else if (imu.decode(buf)) {} // handles usemag, alt, test, offsetcal, imu, imuzero, imucal, imuraw
  else if (eeConfig.decode(buf)) {} // handles eew and eer
  else if (esc.decode(buf)) {} // decodes esc and esi
  else if (usb.decode(buf)) {} // decodes i (interactive mode) and silent
  else if (subscribeDecode(buf)) {} // decodes for subscriptions in this class
  // commands handled here, e.g. help
  else if (strncmp(buf, "help", 4) == 0)
  {
    const int MRL = 320;
    char reply[MRL];
    snprintf(reply, MRL, "# ------ Commands available for %s ------- \r\n", state.droneName);
    usb.send(reply);
    snprintf(reply, MRL, "#   ver          sends: %s.\r\n", REV_ID);
    usb.send(reply);
    snprintf(reply, MRL, "#   help         This help text.\r\n");
    usb.send(reply);
    snprintf(reply, MRL, "#   publist      List of published data keys.\r\n");
    usb.send(reply);
    // more specific help
    usb.sendHelp();
    imu.sendHelp();
    esc.sendHelp();
    sensor.sendHelp();
    logger.sendHelp();
    control.sendHelp();
    mixer.sendHelp();
    rc.sendHelp();
    state.sendHelp();
    leds.sendHelp();
    uhgt.sendHelp();
    uhlas.sendHelp();
    hgt.sendHelp();
    eeConfig.sendHelp();
    //
  }
  else if (strncmp(buf, "log ", 4) == 0)
  { // log and sub-commands
    logger.decode(&buf[4]);
  }
  else if (strncmp(buf, "publist", 7) == 0)
  {
    int n = 0; // n is updated after each call
    state.sendPublishList(n);
    imu.sendPublishList(n);
    hgt.sendPublishList(n);
    mixer.sendPublishList(n);
  }
  else
    usb.sendInfoAsCommentWithTime("unhandled", buf);
}


void UCommand::tick()
{
  subscribeTick();
}

void UCommand::sendData(int item)
{
  if (item == 0)
    sendStatusVersion();
}



