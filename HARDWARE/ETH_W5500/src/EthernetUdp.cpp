/*
 *  Udp.cpp: Library to send/receive UDP packets with the Arduino ethernet shield.
 *  This version only offers minimal wrapping of socket.c/socket.h
 *  Drop Udp.h/.cpp into the Ethernet library directory at hardware/libraries/Ethernet/ 
 *
 * MIT License:
 * Copyright (c) 2008 Bjoern Hartmann
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 * bjoern@cs.stanford.edu 12/30/2008
 */

#include "Ethernet_STM32.h"
#include "EthernetUdp.h"


/* Constructor */
EthernetUDP::EthernetUDP() : _sock(MAX_SOCK_NUM) {}

/* Start EthernetUDP socket, listening at local port PORT */
uint8_t EthernetUDP::begin(uint16_t port) {
  if (_sock != MAX_SOCK_NUM)
    return 0;

  for (int i = 0; i < MAX_SOCK_NUM; i++) {
    uint8_t s = socket.status(i);
    if (s == SnSR::CLOSED || s == SnSR::FIN_WAIT) {
      _sock = i;
      break;
    }
  }

  if (_sock == MAX_SOCK_NUM)
    return 0;

  _port = port;
  _remaining = 0;
  socket.open(_sock, SnMR::UDP, _port, 0);

  return 1;
}

/* return number of bytes available in the current packet,
   will return zero if parsePacket hasn't been called yet */
int EthernetUDP::available() {
  return _remaining;
}

/* Release any resources being used by this EthernetUDP instance */
void EthernetUDP::stop()
{
  if (_sock == MAX_SOCK_NUM)
    return;

  socket.close(_sock);

  EthernetClass::server_port[_sock] = 0;
  _sock = MAX_SOCK_NUM;
}

int EthernetUDP::beginPacket(const char *host, uint16_t port)
{
  // Look up the host first
  int ret = 0;
  DNSClient dns;
  IPAddress remote_addr;

  dns.begin(Ethernet.dnsServerIP());
  ret = dns.getHostByName(host, remote_addr);
  if (ret == 1) {
    return beginPacket(remote_addr, port);
  } else {
    return ret;
  }
}

int EthernetUDP::beginPacket(IPAddress ip, uint16_t port)
{
  _offset = 0;
  return socket.startUDP(_sock, rawIPAddress(ip), port);
}

int EthernetUDP::endPacket()
{
  return socket.sendUDP(_sock);
}

size_t EthernetUDP::write(uint8_t byte)
{
  return write(&byte, 1);
}

size_t EthernetUDP::write(const uint8_t *buffer, size_t size)
{
  uint16_t bytes_written = socket.bufferData(_sock, _offset, buffer, size);
  _offset += bytes_written;
  return bytes_written;
}

int EthernetUDP::parsePacket()
{
  // discard any remaining bytes in the last packet
  flush();

  if (socket.recvAvailable(_sock) > 0)
  {
    //HACK - hand-parse the UDP packet using TCP recv method
    uint8_t tmpBuf[8];
    int ret =0; 
    //read 8 header bytes and get IP and port from it
    ret = socket.recv(_sock,tmpBuf,8);
    if (ret > 0)
    {
      _remoteIP = tmpBuf;
      _remotePort = word(tmpBuf[4], tmpBuf[5]);
      _remaining = word(tmpBuf[6], tmpBuf[7]);

      // When we get here, any remaining bytes are the data
      ret = _remaining;
    }
    return ret;
  }
  // There aren't any packets available
  return 0;
}

int EthernetUDP::read()
{
  uint8_t byte;

  if ((_remaining > 0) && (socket.recv(_sock, &byte, 1) > 0))
  {
    // We read things without any problems
    _remaining--;
    return byte;
  }

  // If we get here, there's no data available
  return -1;
}

int EthernetUDP::read(unsigned char* buffer, size_t len)
{

  if (_remaining > 0)
  {

    int got;

    if (_remaining <= len)
    {
      // data should fit in the buffer
      got = socket.recv(_sock, buffer, _remaining);
    }
    else
    {
      // too much data for the buffer, 
      // grab as much as will fit
      got = socket.recv(_sock, buffer, len);
    }

    if (got > 0)
    {
      _remaining -= got;
      return got;
    }

  }

  // If we get here, there's no data available or recv failed
  return -1;

}

int EthernetUDP::peek()
{
  uint8_t b;
  // Unlike recv, peek doesn't check to see if there's any data available, so we must.
  // If the user hasn't called parsePacket yet then return nothing otherwise they
  // may get the UDP header
  if (!_remaining)
    return -1;
  socket.peek(_sock, &b);
  return b;
}

void EthernetUDP::flush()
{
  // could this fail (loop endlessly) if _remaining > 0 and recv in read fails?
  // should only occur if recv fails after telling us the data is there, lets
  // hope the w5100 always behaves :)

  while (_remaining)
  {
    read();
  }
}

/* Start EthernetUDP socket, listening at local port PORT */
uint8_t EthernetUDP::beginMulticast(IPAddress ip, uint16_t port)
{
  if (_sock != MAX_SOCK_NUM)
    return 0;

  for (int i = 0; i < MAX_SOCK_NUM; i++) {
    uint8_t s = socket.status(i);
    if (s == SnSR::CLOSED || s == SnSR::FIN_WAIT) {
      _sock = i;
      break;
    }
  }

  if (_sock == MAX_SOCK_NUM)
    return 0;

  // Calculate MAC address from Multicast IP Address
  byte mac[] = {  0x01, 0x00, 0x5E, 0x00, 0x00, 0x00 };

  mac[3] = ip[1] & 0x7F;
  mac[4] = ip[2];
  mac[5] = ip[3];

  socket.set(_sock, (uint8_t *)mac, rawIPAddress(ip), port);
  //W5100.writeSnDIPR(_sock, rawIPAddress(ip));
  //W5100.writeSnDPORT(_sock, port);
  //W5100.writeSnDHAR(_sock,mac);

  _remaining = 0;
  socket.open(_sock, SnMR::UDP, port, SnMR::MULTI);
  return 1;
}

