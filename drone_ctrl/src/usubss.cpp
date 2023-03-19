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
#include "usubss.h"
#include "usubs.h"
#include "uusb.h"


USubss::USubss()
{
}

bool USubss::subscribeDecode(const char * keyLine)
{ // if the subscribe command is for any of my data, then return true.
  bool used = false;
  const char * p1 = keyLine;
  bool newSubscription = strncmp(p1, "sub ", 4) == 0;
  if (newSubscription)
    p1 += 4;
  for (int i = 0; i < (int)subs.size(); i++)
  {
    used = subs[i]->decode(p1, newSubscription);
    if (used)
      break;
  }
  return used;
}

void USubss::subscribeTick()
{ // check for time to send data
  for (int i = 0; i < (int)subs.size(); i++)
  {
    if (subs[i]->tick())
    {
      sendData(i);
    }
  }
}

void USubss::subscribeSendHelp()
{
  for (int i = 0; i < (int)subs.size(); i++)
    subs[i]->sendHelpLine();
}

void USubss::sendPublishList(int & listNum)
{
  for (int i = 0; i < (int)subs.size(); i++)
    subs[i]->sendPublishList(listNum);
}

void USubss::addPublishItem(const char* key, const char * helpLine)
{
  subs.push_back(new USubs(key, helpLine));
}
