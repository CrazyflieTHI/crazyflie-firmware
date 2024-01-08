/**
 *                                             __
 *    __  __ ____      ____  __  __ __________/ /_  __   _____________
 *   / /_/ / ___/____ / __ `/ / / /´__  / ___/ __ \/ /  / / ___/ __  /
 *  / __  (__  )/___// /_/ / /_/ / /_/ (__  ) /_/ / /__/ / /  / /_/ /
 * /_/ /_/____/      \__,_/_____/\__  /____/\____/______/_/   \__  /
 *                              /____/                       /____/
 *
 * Crazyflie control firmware for multi-agent systems
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * macp.h - Multi-Agent Communication Protocol
 */

#ifndef _MACP_H_
#define _MACP_H_

#include <stdint.h>
#include <stdbool.h>

#define MAX_MACP_DATA_SIZE 27

#define MACP_BROADCAST_ADDR 0xF
#define MACP_CLIENT_ADDR 0x0
#define MACP_CRTP_PORT 0x9

#define MACP_HEADER_ADDR(destAddr, sendAddr) (((destAddr & 0x0F) << 4) | (sendAddr & 0x0F))

typedef enum {
  MACP_PORT_RESERVED_0 = 0x00,
  MACP_PORT_RESERVED_1 = 0x01,
  MACP_PORT_RESERVED_2 = 0x02,
  MACP_PORT_RESERVED_3 = 0x03,
  MACP_PORT_RESERVED_4 = 0x04,
  MACP_PORT_RESERVED_5 = 0x05,
} MACPPort;

typedef enum {
  MACP_LOCAL_PORT_RESERVED_0 = 0x10,
  MACP_LOCAL_PORT_RESERVED_1 = 0x11,
  MACP_LOCAL_PORT_RESERVED_2 = 0x12,
  MACP_LOCAL_PORT_RESERVED_3 = 0x13,
  MACP_LOCAL_PORT_RESERVED_4 = 0x14,
  MACP_LOCAL_PORT_RESERVED_5 = 0x15,
} MACPLocalPort;

typedef struct {
  union {
  uint8_t headerAddr;
    struct {
      uint8_t sendAddr : 4;
      uint8_t destAddr : 4;
    };
  };
  uint8_t port;
  uint8_t subPort;
  uint8_t data[MAX_MACP_DATA_SIZE];
} __attribute__((packed)) MACPPacket;

typedef void (*MacpCallback)(MACPPacket *);

void macpInit(void);
bool macpTest(void);

/**
 * Initializes the queue and dispatch of an task.
 *
 * @param[in] taskId The id of the CRTP task
 */
void macpRxTaskInitQueue(MACPPort port);

/**
 * Register a callback to be called for a particular port.
 *
 * @param[in] port MACP port for which the callback is set
 * @param[in] cb Callback that will be called when a packet is received on
 *            'port'.
 *
 * @note Only one callback can be registered per port! The last callback
 *       registered will be the one called
 */
void macpRegisterPortCB(MACPPort port, MacpCallback cb);

/**
 * Send a packet via multi-agent communication protocol
 *
 * If the TX stack is full, the oldest packet is dropped
 *
 * @param[in] destAddr    Destination address
 * @param[in] macpPort    MACP port
 * @param[in] macpSubPort MACP sub port corresponding to a MACP port
 * @param[in] payload     Payload data
 * @param[in] payloadSize Size of the payload in bytes
 *
 * @returns pdTRUE if the packet was successfully handed over to crtp,
 *          otherwise errQUEUE_FULL.
 */
int macpSendPacket(uint8_t destAddr, uint8_t macpPort, uint8_t macpSubPort,
                   uint8_t *payload, uint8_t payloadSize);

/**
 * Get identification number of the Crazyflie for multi-agent communication.
 * It is derived from the Crazyflie address.
 *
 * @returns The identification number for the Crazyflie
 */
uint8_t getOwnMacpId();

#endif /* _MACP_H_ */
