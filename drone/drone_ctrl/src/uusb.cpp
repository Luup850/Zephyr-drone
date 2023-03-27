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

#include <core_pins.h>
// #include <Wire.h>
#include <usb_serial.h>
#include "main.h"
#include "uusb.h"
#include "command.h"
#include "uled.h"

UUSB usb;

void UUSB::setup()
{
  // init USB connection (parameter is not used - always 12MB/s)
  Serial.begin ( 115200 ); // USB init serial
  send("# welcome to fejemis \n");
}

void UUSB::tick()
{ // check for messages
  handleIncoming();
}

bool UUSB::send(const char* str) // , bool blocking)
{
  bool sendOK;
  if (localEcho == 1 and justSendPrompt)
  {
    client_send_str("\n\r", 2);
    justSendPrompt = false;
  }
  int n = strlen(str);
  sendOK = client_send_str(str, n);
  return sendOK;
}

void UUSB::sendHelp()
{
  const int MRL = 320;
  char reply[MRL];
  snprintf(reply, MRL, "# ------ USB connection ------- \r\n");
  send(reply);
  snprintf(reply, MRL, "#   i=V          Interactive: V=0: GUI info (require activity), V=1: local echo, V=2:no timeout (i=%d)\r\n", localEcho);
  send(reply);
  snprintf(reply, MRL, "#   silent=1     Should USB be silent, if no communication (1=auto silent) silent=%d\r\n", silenceUSBauto);
  send(reply);
}

bool UUSB::sendInfoAsCommentWithTime(const char* info, const char * msg)
{
  const int MSL = 400;
  char s[MSL];
  bool isOK = false;
  snprintf(s, MSL, "# %.3f %s: '%s'\n", float(hb10us) * SAMPLETIME, info, msg);
  isOK = send(s);
  return isOK;
}


//////////////////////////////////////////////////

bool UUSB::client_send_str(const char * str, int m) // , bool blocking) //, bool toUSB, bool toWifi)
{
  //int n = strlen(str);
  bool okSend = true;
  if (true)
  { // 
    // generate q-code first
    int sum = 0;
    const char * p1 = str;
    for (int i = 0; i < m; i++)
      if (*p1 >= ' ')
        sum += *p1++;
    const int MQL = 4;
    char q[MQL];
    snprintf(q, MQL, ";%02d", (sum % 99) + 1);
//     uint32_t pt = hbTimerCnt;
    usb_serial_write(q, 3);
    usb_serial_write(str, m);
    // usb_serial_flush_output();
  }
  return okSend;
}

////////////////////////////////////////////////////////////////

void UUSB::handleIncoming()
{
  int n = 0, m;
  // get number of available chars in USB buffer
  m = usb_serial_available();
  if (m > 20)
    // limit to no more than 20 chars in one 1ms cycle
    m = 20;
  // 
  if (m > 0)
  { // get characters
    for (int i = 0; i < m; i++)
    { // get pending characters
      n = usb_serial_getchar();
      if (n < 0)
        break;
      if (n >= '\n' and n < 0x80)
      { // there is data from USB, so it is active
        usbTimeoutGotData = hbTimerCnt;
        // command arriving from USB
        //usb_send_str("#got a char from USB\r\n");
        receivedCharFromUSB(n) ;
        if (not usbActivity)
        {
          usbActivity = true;
//           led->setLed(13, 0, 0 ,0,1);
        }
        break;
      }
    }
  }
  // check for activity
//   int dt = hbTimerCnt - usbTimeoutGotData;
//   if (dt > 5000 and usbActivity)
//   { // too long time
//     // turn off all module status LEDs
//     led->setLed(0, 0, 0 ,0, -1);
//     // turn on error LED (lost connection)
//     led->setLed(13, 200, 0, 0);
//     usbActivity = false;
//   }
}


/**
 * Got a new character from USB channel
 * Put it into buffer, and if a full line, then intrepid the result.
 * \param n is the new character */
void UUSB::receivedCharFromUSB(uint8_t n)
{ // got another character from usb host (command)
  if (usbRxBufCnt == 0)
    // got first char in new message
    rxStartHb = hbTimerCnt;
  if (n >= ' ')
  {
    usbRxBuf[usbRxBufCnt] = n;
    if (usbRxBufCnt < RX_BUF_SIZE - 1)
      usbRxBufCnt++;
    else
    {
      usbRxBufOverflow = true;
      usbRxBufCnt = 0;
      usbRxBuf[usbRxBufCnt] = '\0';
    }
  }
  //
  if (localEcho) // and not silentUSB)
    // echo characters back to terminal
    usb_serial_putchar(n);
  if (n == '\n' or n=='\r')
  { // zero terminate
    if (usbRxBufOverflow)
    {
      usbRxBufOverflow = false;
      send("# USB rx-buffer overflow\n");
    }
    else
    {
      if (usbRxBufCnt > 0)
      {
        usbRxBuf[usbRxBufCnt] = '\0';
        // check CRC
        bool crcOK = false;
        if (usbRxBuf[0] == ';')
        {
          const char * p1 = usbRxBuf;
          int crc = int(p1[1] - '0') * 10 + int(p1[2] - '0');
          int sum = 0;
          int sumCnt = 0;
          for (int i = 3; i < usbRxBufCnt; i++)
          {
            if (usbRxBuf[i] < ' ')
              break;
            sum += usbRxBuf[i];
            sumCnt++;
          }
          crcOK = (sum % 99) + 1 == crc;
          if (crcOK or localEcho)
            command.parse_and_execute_command(&usbRxBuf[3]);
          else
          {
            const int MSL = 200;
            char s[MSL];
            snprintf(s, MSL, "# CRC failed (crc=%d, found to be %d, sum=%d, %d chars), for '%s'\n",
                     crc, (sum % 99) + 1, sum, sumCnt, usbRxBuf);
            send(s);
          }
        }
        else
        {
          send("# processing under protest - no CRC\n");
          command.parse_and_execute_command(usbRxBuf);
        }
      }
      if (localEcho == 1)
      {
        send("\r\n>>");
        justSendPrompt = true;
      }
    }
    // flush remaining input
    usbRxBufCnt = 0;
  }
  else if (usbRxBufCnt >= RX_BUF_SIZE - 1)
  { // garbage in buffer, just discard
    usbRxBuf[usbRxBufCnt] = 0;
    const char * msg = "** Discarded (missing \\n)\n";
    send(msg);
    usbRxBufCnt = 0;
  }
}

//////////////////////////////////////////////////////////////

bool UUSB::decode(const char* msg)
{
  bool found = true;
  if (msg[1] == '=')
  { // one character assignment command
    switch (msg[0])
    {
      case 'i':  // interactive mode -1=no info, 0=normal GUI, 1=telnet session (local echo of all)
        localEcho = strtol(&msg[2], NULL, 10);
        break;
      default: 
        sendInfoAsCommentWithTime(" ** unknown one character command!", msg);
        found = false;
        break;
    }
  }
  else if (strncmp(msg, "silent", 6) == 0)
  {
    char * p2 = strchr(&msg[4],'=');
    int v = 1;
    if (p2 != NULL)
    {
      p2++;
      v = strtol(p2, NULL, 10);
    }
    silenceUSBauto = v != 0;
  }
  else
    found = false;
  return found;
}

