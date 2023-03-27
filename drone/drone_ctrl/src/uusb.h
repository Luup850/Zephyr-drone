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

#ifndef UUSB_H
#define UUSB_H


class UUSB
{
private:
  bool silenceUSBauto = true; // manuel inhibit of USB silent timeout
  uint32_t lastSend = 0;
  // local echo is used if we are talking to e.g. putty
  // and a command prompt would be nice.
  bool localEcho = false;
  bool justSendPrompt = false;
  /**
   * usb command buffer space */
  static const int RX_BUF_SIZE = 200;
  char usbRxBuf[RX_BUF_SIZE];
  int usbRxBufCnt = 0;
  uint32_t rxStartHb = 0;
  bool usbRxBufOverflow = false;
  uint32_t usbTimeoutGotData = 0;
  
public:
  void setup();
  
  void tick();
  /** send message to USB host
   * \param str : string to send, should end with a '\n'
   * \param blocking : if true, then function will not return until send.
   *                   if false, message will be dropped, if no BW is available
   * return true if send. */
  bool send(const char* str); //, bool blocking = false);
  /**
   * Send comment with time and this info.
   * \param info to send - placed after teensy time in seconds, like '# 123.456 info [msg]'
   * \param msg is an optional related message 
   * \returns true if send. */
  bool sendInfoAsCommentWithTime(const char* info, const char * msg);
  /** send to USB channel 
  * \param str is string to send
  * \param n is number of bytes to send
  * \param blocking if false, then send if space only, else don't return until send
  */
  inline bool send_block(const char * str, int n) //, bool blocking)
  {
    return client_send_str(str, n); //, blocking);
  }
  
  /**
   * send help and status */
  void sendHelp();
  /**
   * Decode command for USB settings */
  bool decode(const char * msg);
  // got valid data in recent time (~2 sec)
  bool usbActivity = true;
  
private:
  /// send a string - if possible - of this length to USB
  bool client_send_str(const char * str, int m); //, bool blocking);
  /// received a character from USB
  void receivedCharFromUSB(uint8_t n);
  /// USB incomming port handling
  void handleIncoming();
};
  
extern UUSB usb;

#endif
