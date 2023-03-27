/***************************************************************************
*
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

#include <string.h>
#include <stdlib.h>
#include "main.h"
#include "usubs.h"
#include "uusb.h"


USubs::USubs(const char * key, const char * help)
{
  msgKey = key;
  helpText = help;
  keySize = strlen(msgKey);
}

bool USubs::decode(const char * keyLine, bool newSubscription)
{
  bool used = false;
  if (newSubscription)
  {
    if (strncmp(msgKey, keyLine, keySize) == 0  and keyLine[keySize] == ' ')
    {
  //     usb.send("# USubs:: set subscription\n");
      int n = strtol(&keyLine[keySize],nullptr, 10);
      // convert from ms to tics
      subN = n/int(SAMPLETIME * 1000);
      if (n > 0 and subN == 0)
        // requested faster than sample time, so get sample time.
        subN = 1;
      subCnt = subN;
      used = true;
    }
  }
  else if (strncmp(msgKey, keyLine, keySize) == 0  and keyLine[keySize] == 'i')
  { // one-time request for data
    used = true;
    dataRequest = true;
  }
  return used;
}


bool USubs::tick()
{
  bool isTime = dataRequest;
  if (dataRequest)
    dataRequest = false;
  if (subCnt == 1)
  {
    isTime = true;
    subCnt = subN;
  }
  else if (subN > 1)
    subCnt--;
  return isTime;
}

void USubs::sendHelpLine()
{
  const int MSL = 200;
  char s[MSL];
  snprintf(s, MSL, "#   %si and 'sub %s N', %s\r\n", msgKey, msgKey, helpText);
  usb.send(s);
}

void USubs::sendPublishList(int & listNum)
{
  const int MSL = 200;
  char s[MSL];
  snprintf(s, MSL, "pub %d %s %s\r\n", listNum++, msgKey, helpText);
  usb.send(s);
}
