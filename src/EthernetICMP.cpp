/*
 *  Icmp.cpp: Library to send/receive ICMP packets with the Arduino ethernet shield.
 *  This version only offers minimal wrapping of socket.cpp
 *  Drop Icmp.h/.cpp into the Ethernet library directory at hardware/libraries/Ethernet/
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

/*
 * Copyright (c) 2010 by Blake Foster <blfoster@vassar.edu>
 *
 * This file is free software; you can redistribute it and/or modify
 * it under the terms of either the GNU General Public License version 2
 * or the GNU Lesser General Public License version 2.1, both as
 * published by the Free Software Foundation.
 */


#include <Arduino.h>
//#include "Ethernet.h"
//#include "utility/w5100.h"
#include "EthernetICMP.h"

#ifdef ICMPPING_INSERT_YIELDS
#define ICMPPING_DOYIELD()		delay(2)
#else
#define ICMPPING_DOYIELD()
#endif

typedef struct {
	uint16_t RX_RSR; // Number of bytes received
	uint16_t RX_RD;  // Address to read
	uint16_t TX_FSR; // Free space ready for transmit
	uint8_t  RX_inc; // how much have we advanced RX_RD
} socketstate_t;

static socketstate_t state[MAX_SOCK_NUM];

static uint16_t getSnTX_FSR(uint8_t s);
static uint16_t getSnRX_RSR(uint8_t s);
static void write_data(uint8_t s, uint16_t offset, const uint8_t *data, uint16_t len);
static void read_data(uint8_t s, uint16_t src, uint8_t *dst, uint16_t len);

/*****************************************/
/*    Socket Data Transmit Functions     */
/*****************************************/

static uint16_t getSnTX_FSR(uint8_t s)
{
        uint16_t val, prev;

        prev = W5100.readSnTX_FSR(s);
        while (1) {
                val = W5100.readSnTX_FSR(s);
                if (val == prev) {
			state[s].TX_FSR = val;
			return val;
		}
                prev = val;
        }
}


static void write_data(uint8_t s, uint16_t data_offset, const uint8_t *data, uint16_t len)
{
	uint16_t ptr = W5100.readSnTX_WR(s);
	ptr += data_offset;
	uint16_t offset = ptr & W5100.SMASK;
	uint16_t dstAddr = offset + W5100.SBASE(s);

	if (W5100.hasOffsetAddressMapping() || offset + len <= W5100.SSIZE) {
		W5100.write(dstAddr, data, len);
	} else {
		// Wrap around circular buffer
		uint16_t size = W5100.SSIZE - offset;
		W5100.write(dstAddr, data, size);
		W5100.write(W5100.SBASE(s), data + size, len - size);
	}
	ptr += len;
	W5100.writeSnTX_WR(s, ptr);
}

/*****************************************/
/*    Socket Data Receive Functions      */
/*****************************************/

static uint16_t getSnRX_RSR(uint8_t s)
{
#if 1
        uint16_t val, prev;

        prev = W5100.readSnRX_RSR(s);
        while (1) {
                val = W5100.readSnRX_RSR(s);
                if (val == prev) {
			return val;
		}
                prev = val;
        }
#else
	uint16_t val = W5100.readSnRX_RSR(s);
	return val;
#endif
}


static void read_data(uint8_t s, uint16_t src, uint8_t *dst, uint16_t len)
{
	uint16_t size;
	uint16_t src_mask;
	uint16_t src_ptr;

	//Serial.printf("read_data, len=%d, at:%d\n", len, src);
	src_mask = (uint16_t)src & W5100.SMASK;
	src_ptr = W5100.RBASE(s) + src_mask;

	if (W5100.hasOffsetAddressMapping() || src_mask + len <= W5100.SSIZE) {
		W5100.read(src_ptr, dst, len);
	} else {
		size = W5100.SSIZE - src_mask;
		W5100.read(src_ptr, dst, size);
		dst += size;
		W5100.read(W5100.RBASE(s), dst, len - size);
	}
}

inline uint16_t _makeUint16(const uint8_t& highOrder, const uint8_t& lowOrder)
{
    // make a 16-bit unsigned integer given the low order and high order bytes.
    // lowOrder first because the Arduino is little endian.
    uint8_t value [] = {lowOrder, highOrder};
    return *(uint16_t *)&value;
}

uint16_t _checksum(const EthernetICMPEcho& echo)
{
    // calculate the checksum of an ICMPEcho with all fields but icmpHeader.checksum populated
    unsigned long sum = 0;

    // add the header, bytes reversed since we're using little-endian arithmetic.
    sum += _makeUint16(echo.icmpHeader.type, echo.icmpHeader.code);

    // add id and sequence
    sum += echo.id + echo.seq;

    // add time, one half at a time.
    uint16_t const * time = (uint16_t const *)&echo.time;
    sum += *time + *(time + 1);
    
    // add the payload
    for (uint8_t const * b = echo.payload; b < echo.payload + sizeof(echo.payload); b+=2)
    {
        sum += _makeUint16(*b, *(b + 1));
    }

    // ones complement of ones complement sum
    sum = (sum >> 16) + (sum & 0xffff);
    sum += (sum >> 16);
    return ~sum;
}

EthernetICMPEcho::EthernetICMPEcho(uint8_t type, uint16_t _id, uint16_t _seq, uint8_t * _payload)
: seq(_seq), id(_id), time(millis())
{
    memcpy(payload, _payload, REQ_DATASIZE);
    icmpHeader.type = type;
    icmpHeader.code = 0;
    icmpHeader.checksum = _checksum(*this);
}

EthernetICMPEcho::EthernetICMPEcho()
: seq(0), id(0), time(0)
{
    memset(payload, 0, sizeof(payload));
    icmpHeader.code = 0;
    icmpHeader.type = 0;
    icmpHeader.checksum = 0;
}

void EthernetICMPEcho::serialize(uint8_t * binData) const
{
    *(binData++) = icmpHeader.type;
    *(binData++) = icmpHeader.code;

    *(uint16_t *)binData = htons(icmpHeader.checksum); binData += 2;
    *(uint16_t *)binData = htons(id);                  binData += 2;
    *(uint16_t *)binData = htons(seq);                 binData += 2;
    *(icmp_time_t *)  binData = htonl(time);                binData += 4;

    memcpy(binData, payload, sizeof(payload));
}

void EthernetICMPEcho::deserialize(uint8_t const * binData)
{
    icmpHeader.type = *(binData++);
    icmpHeader.code = *(binData++);

    icmpHeader.checksum = ntohs(*(uint16_t *)binData); binData += 2;
    id                  = ntohs(*(uint16_t *)binData); binData += 2;
    seq                 = ntohs(*(uint16_t *)binData); binData += 2;

    if (icmpHeader.type != TIME_EXCEEDED)
    {
        time = ntohl(*(icmp_time_t *)binData);   binData += 4;
    }

    memcpy(payload, binData, sizeof(payload));
}


uint16_t EthernetICMPPing::ping_timeout = PING_TIMEOUT;

EthernetICMPPing::EthernetICMPPing(SOCKET socket, uint8_t id) :
#ifdef ICMPPING_ASYNCH_ENABLE
  _curSeq(0), _numRetries(0), _asyncstart(0), _asyncstatus(BAD_RESPONSE),
#endif
  _id(id), _nextSeq(0), _socket(socket),  _attempt(0)
{
    memset(_payload, 0x1A, REQ_DATASIZE);
}


void EthernetICMPPing::setPayload(uint8_t * payload)
{
	memcpy(_payload, payload, REQ_DATASIZE);
}

void EthernetICMPPing::openSocket()
{

    W5100.execCmdSn(_socket, Sock_CLOSE);
    W5100.writeSnIR(_socket, 0xFF);
    W5100.writeSnMR(_socket, SnMR::IPRAW);
    W5100.writeSnPROTO(_socket, IPPROTO::ICMP);
    W5100.writeSnPORT(_socket, 0);
    W5100.execCmdSn(_socket, Sock_OPEN);
}

void EthernetICMPPing::closeSocket()
{
    W5100.execCmdSn(_socket, Sock_CLOSE);
    W5100.writeSnIR(_socket, 0xFF);
}


void EthernetICMPPing::operator()(const IPAddress& addr, int nRetries, EthernetICMPEchoReply& result)
{
	openSocket();

    EthernetICMPEcho echoReq(ICMP_ECHOREQ, _id, _nextSeq++, _payload);

    for (_attempt=0; _attempt<nRetries; ++_attempt)
    {

    	ICMPPING_DOYIELD();

        result.status = sendEchoRequest(addr, echoReq);
        if (result.status == SUCCESS)
        {
            byte replyAddr [4];
        	ICMPPING_DOYIELD();
            receiveEchoReply(echoReq, addr, result);
        }
        if (result.status == SUCCESS)
        {
            break;
        }
    }
   
    W5100.execCmdSn(_socket, Sock_CLOSE);
    W5100.writeSnIR(_socket, 0xFF);
}

EthernetICMPEchoReply EthernetICMPPing::operator()(const IPAddress& addr, int nRetries)
{
    EthernetICMPEchoReply reply;
    operator()(addr, nRetries, reply);
    return reply;
}

void EthernetICMPPing::receiveEchoReply(const EthernetICMPEcho& echoReq, const IPAddress& addr, EthernetICMPEchoReply& echoReply)
{
    icmp_time_t start = millis();
    while (millis() - start < ping_timeout)
    {

        if (getSnRX_RSR(_socket) < 1)
        {
        	// take a break, maybe let platform do
        	// some background work (like on ESP8266)
        	ICMPPING_DOYIELD();
        	continue;
        }

        // ah! we did receive something... check it out.

        uint8_t ipHeader[6];
		uint8_t buffer = W5100.readSnRX_RD(_socket);
		read_data(_socket, (uint16_t) buffer, ipHeader, sizeof(ipHeader));
		buffer += sizeof(ipHeader);
		for (int i = 0; i < 4; ++i)
			echoReply.addr[i] = ipHeader[i];
		uint8_t dataLen = ipHeader[4];
		dataLen = (dataLen << 8) + ipHeader[5];

		uint8_t serialized[sizeof(EthernetICMPEcho)];
		if (dataLen > sizeof(EthernetICMPEcho))
			dataLen = sizeof(EthernetICMPEcho);
		read_data(_socket, (uint16_t) buffer, serialized, dataLen);
		echoReply.data.deserialize(serialized);

		buffer += dataLen;
		W5100.writeSnRX_RD(_socket, buffer);
		W5100.execCmdSn(_socket, Sock_RECV);

		echoReply.ttl = W5100.readSnTTL(_socket);

		// Since there aren't any ports in ICMP, we need to manually inspect the response
		// to see if it originated from the request we sent out.
		switch (echoReply.data.icmpHeader.type) {
		case ICMP_ECHOREP: {
			if (echoReply.data.id == echoReq.id
					&& echoReply.data.seq == echoReq.seq) {
				echoReply.status = SUCCESS;
				return;
			}
			break;
		}
		case TIME_EXCEEDED: {
			uint8_t * sourceIpHeader = echoReply.data.payload;
			unsigned int ipHeaderSize = (sourceIpHeader[0] & 0x0F) * 4u;
			uint8_t * sourceIcmpHeader = echoReply.data.payload + ipHeaderSize;

			// The destination ip address in the originating packet's IP header.
			IPAddress sourceDestAddress(sourceIpHeader + ipHeaderSize - 4);

			if (!(sourceDestAddress == addr))
				continue;

			uint16_t sourceId = ntohs(*(uint16_t * )(sourceIcmpHeader + 4));
			uint16_t sourceSeq = ntohs(*(uint16_t * )(sourceIcmpHeader + 6));

			if (sourceId == echoReq.id && sourceSeq == echoReq.seq) {
				echoReply.status = BAD_RESPONSE;
				return;
			}
			break;
		}
		}


    }
    echoReply.status = NO_RESPONSE;
}

Status EthernetICMPPing::sendEchoRequest(const IPAddress& addr, const EthernetICMPEcho& echoReq)
{
    // I wish there were a better way of doing this, but if we use the uint32_t
    // cast operator, we're forced to (1) cast away the constness, and (2) deal
    // with an endianness nightmare.
    uint8_t addri [] = {addr[0], addr[1], addr[2], addr[3]};
    W5100.writeSnDIPR(_socket, addri);
    W5100.writeSnTTL(_socket, 255);
    // The port isn't used, becuause ICMP is a network-layer protocol. So we
    // write zero. This probably isn't actually necessary.
    W5100.writeSnDPORT(_socket, 0);

    uint8_t serialized [sizeof(EthernetICMPEcho)];
    echoReq.serialize(serialized);

    //W5100.send_data_processing(_socket, serialized, sizeof(EthernetICMPEcho));
    write_data(_socket, 0, serialized, sizeof(EthernetICMPEcho));
    W5100.execCmdSn(_socket, Sock_SEND);

    while ((W5100.readSnIR(_socket) & SnIR::SEND_OK) != SnIR::SEND_OK) 
    {
        if (W5100.readSnIR(_socket) & SnIR::TIMEOUT)
        {
            W5100.writeSnIR(_socket, (SnIR::SEND_OK | SnIR::TIMEOUT));
            return SEND_TIMEOUT;
        }

        ICMPPING_DOYIELD();
    }
    W5100.writeSnIR(_socket, SnIR::SEND_OK);
    return SUCCESS;
}

#ifdef ICMPPING_ASYNCH_ENABLE
/*
 * When ICMPPING_ASYNCH_ENABLE is defined, we have access to the
 * asyncStart()/asyncComplete() methods from the API.
 */
bool EthernetICMPPing::asyncSend(EthernetICMPEchoReply& result)
{
    EthernetICMPEcho echoReq(ICMP_ECHOREQ, _id, _curSeq, _payload);

    Status sendOpResult(NO_RESPONSE);
    bool sendSuccess = false;
    for (uint8_t i=_attempt; i<_numRetries; ++i)
    {
    	_attempt++;

    	ICMPPING_DOYIELD();
    	sendOpResult = sendEchoRequest(_addr, echoReq);
    	if (sendOpResult == SUCCESS)
    	{
    		sendSuccess = true; // it worked
    		sendOpResult = ASYNC_SENT; // we're doing this async-style, force the status
    		_asyncstart = millis(); // not the start time, for timeouts
    		break; // break out of this loop, 'cause we're done.

    	}
    }
    _asyncstatus = sendOpResult; // keep track of this, in case the ICMPEchoReply isn't re-used
    result.status = _asyncstatus; // set the result, in case the ICMPEchoReply is checked
    return sendSuccess; // return success of send op
}
bool EthernetICMPPing::asyncStart(const IPAddress& addr, int nRetries, EthernetICMPEchoReply& result)
{
	openSocket();

	// stash our state, so we can access
	// in asynchSend()/asyncComplete()
	_numRetries = nRetries;
	_attempt = 0;
	_curSeq = _nextSeq++;
	_addr = addr;

	return asyncSend(result);

}

bool EthernetICMPPing::asyncComplete(EthernetICMPEchoReply& result)
{

	if (_asyncstatus != ASYNC_SENT)
	{
		// we either:
		//  - didn't start an async request;
		//	- failed to send; or
		//	- are no longer waiting on this packet.
		// either way, we're done

		// Close RAW socket to allow device being pinged again
		closeSocket();

		return true;
	}


	if (W5100.readSnRX_SIZE(_socket))
	{
		// ooooh, we've got a pending reply
	    EthernetICMPEcho echoReq(ICMP_ECHOREQ, _id, _curSeq, _payload);
		receiveEchoReply(echoReq, _addr, result);
		_asyncstatus = result.status; // make note of this status, whatever it is.

		// Close RAW socket to allow device being pinged again
		closeSocket();

		return true; // whatever the result of the receiveEchoReply(), the async op is done.
	}

	// nothing yet... check if we've timed out
	if ( (millis() - _asyncstart) > ping_timeout)
	{

		// yep, we've timed out...
		if (_attempt < _numRetries)
		{
			// still, this wasn't our last attempt, let's try again
			if (asyncSend(result))
			{
				// another send has succeeded
				// we'll wait for that now...
				return false;
			}

			// this send has failed. too bad,
			// we are done.

			// Close RAW socket to allow device being pinged again
			closeSocket();

			return true;
		}

		// we timed out and have no more attempts left...
		// hello?  is anybody out there?
		// guess not:
	    result.status = NO_RESPONSE;

	    // Close RAW socket to allow device being pinged again
	    closeSocket();

	    return true;
	}

	// have yet to time out, will wait some more:
	return false; // results still not in

}

#endif	/* ICMPPING_ASYNCH_ENABLE */
