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
#ifndef USUBSS_H
#define USUBSS_H

#include "usubs.h"
#include <vector>

class USubs;

class USubss
{
public:
    /**
   * @brief sendHelpLine sends help line for this key
   * \param listNum is the publist list number of last item - to be increased
   */
  void sendPublishList(int & listNum);

protected:
  USubss();
  
  /**
   * @brief Decode a subscription command
   * @param keyline is the keyword for this message and further parameters
   * @returns true if used
   */
  bool subscribeDecode(const char * keyline);
  /**
   * @brief tick is service of subscription
   * \param n is next item to check
   */
  void subscribeTick();
  /**
   * @brief sendHelpLine sends help line for this key
   */
  void subscribeSendHelp();
  /**
   * add subscription key */
  void addPublishItem(const char * key, const char * helpLine);
  /**
   * send data now from one of the subscription items
   * single request or as subscribed */
  virtual void sendData(int item) {};
  
private:
  /// 
  std::vector<USubs*> subs;
};

#endif // USUBS_H
