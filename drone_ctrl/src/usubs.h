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
#ifndef USUBS_H
#define USUBS_H


class USubs
{
public:
  
  USubs(const char * key, const char * help);
  /**
   * @brief Decode a subscription command
   * @param keyline is the keyword for this message and further parameters
   * @param newSubscription is true, if this is a subscription command, else data request
   * @returns true if used
   */
  bool decode(const char * keyline, bool newSubscription);
  /**
   * @brief decode a one time request (no parameters)
   * @param key is the request key, followed by an i
   * @returns true if used
   */
//   bool decode(const char * key);
  /**
   * @brief tick is servise of subscription
   * @returns true if message is to be send
   */
  bool tick();
  /**
   * @brief sendHelpLine sends help line for this key
   */
  void sendHelpLine();
  /**
   * @brief sendHelpLine sends help line for this key
   * \param listNum is the publist list number of last item - to be increased
   */
  void sendPublishList(int & listNum);
  /**
   * @brief MKL and msgKey is the message key for this subscription
   */
  const char * msgKey;
  /**
   * @brief helpText is text to be returned for a help request
   */
  const char * helpText;
  /**
   * @brief keySize is size of key, to make compare faster
   */
  int keySize;
  /// interval count so far
  int subCnt = 0;
  /// interval in ms (0 = not avtive)
  int subN = 0;
  /// explicit request data
  bool dataRequest = false;
  
};

#endif // USUBS_H
