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

#ifndef COMMAND_H
#define COMMAND_H

#include <string.h>
#include <stdlib.h>
#include <usb_serial.h>
#include <core_pins.h>
#include <HardwareSerial.h>
#include <mutex>
#include "usubss.h"


class UCommand : public USubss
{
public:
//   UCommand()
//   {
//   }
  /**
  * Get revision number from SVN annotation */
  uint16_t getRevisionNumber();
  /**
  * Parse commands from the USB connection and implement those commands.
  * \param buf is string to send
  * The function is served by the main loop, when time allows. */
  void parse_and_execute_command(char *buf);
  /**
  * setup */
  void setup();
  /**
   * subscription service tick */
  void tick();

protected:
  /**
   * send data to subscriber or requester over USB 
   * @param item is the item number corresponding to the added subscription during setup. */
  void sendData(int item);

  
private:
  /**
   * Send version string to client */
  void sendStatusVersion();
};

extern UCommand command;

#endif
