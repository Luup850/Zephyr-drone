 /***************************************************************************
 * 
 *   Copyright (C) 2022 by DTU                             *
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

#ifndef UMIXER_H
#define UMIXER_H

#include <stdint.h>
#include "main.h"
#include "usubss.h"

class UMixer : public USubss
{
public:
  /**
   * Setup */
  void setup();
  /**
   * Decode mixer info command */
  void sendStatus();
  /**
   * Send mixer (drone) configuration and offset */
  void sendMixerConfig();
  
  /**
   * sendHelp */
  void sendHelp();
  /**
   * Decode commands from USB connection 
   * \param buf is the command line string buffer
   * \returns true is command is used. */
  bool decode(const char * buf);
  /**
   * Mixer tick
   * Transfers control output (roll, pitch, yaw, height)
   * to ESC values */
  void tick();
  /**
   * Are we ready to try arming 
   * Requires that trust control is low. */
  bool canArm();
  /**
   * save configuration to EE-prom */
  void eePromSave();
  /**
   * load configuration from EE-prom */
  void eePromLoad();
  
protected:
  /**
   * send data to subscriber or requester over USB 
   * @param item is the item number corresponding to the added subscription during setup. */
  void sendData(int item);

public:
  typedef enum
  { x_config = 1, plus_config, drone2x2, unknown} DroneType;
  DroneType droneType = x_config;
  float uHeight;
  float uRoll;
  float uPitch;
  float uYaw;
};

extern UMixer mixer;

#endif
