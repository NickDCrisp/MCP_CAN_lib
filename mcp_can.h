/*
  mcp_can.h
  2012 Copyright (c) Seeed Technology Inc.  All right reserved.
  2017 Copyright (c) Cory J. Fowler  All Rights Reserved.

  Author:Loovee
  Contributor: Cory J. Fowler
  2017-09-25
  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-
  1301  USA
*/
#ifndef _MCP2515_H_
#define _MCP2515_H_

// _dfs file contains the #defines for registers and return values.
// by default serial debug mode is active for MCP_CAN so uncomment this next line if you don't need it.
// #define DEBUG_MODE 0
#include "mcp_can_dfs.h"
// typical CAN frame will contain up to 8 data bytes
#define MAX_CHAR_IN_MESSAGE 8

class MCP_CAN
{
    private:
    
    INT8U   m_nExtFlg;                                                  // Identifier Type
                                                                        // Extended (29 bit) or Standard (11 bit)
    INT32U  m_nID;                                                      // CAN ID
    INT8U   m_nDlc;                                                     // Data Length Code
    INT8U   m_nDta[MAX_CHAR_IN_MESSAGE];                                // Data array
    INT8U   m_nRtr;                                                     // Remote request flag
    INT8U   m_nfilhit;                                                  // The number of the filter that matched the message
    INT8U   MCPCS;                                                      // Chip Select pin number
    INT8U   mcpMode;                                                    // Mode to return to after configurations are performed.
    

/*********************************************************************************************************
 *  mcp2515 driver function 
 *********************************************************************************************************/
   // private:
   private:

    void mcp2515_reset(void);                                           // Soft Reset MCP2515

    INT8U mcp2515_readRegister(const INT8U address);                    // Read MCP2515 register
    
    void mcp2515_readRegisterS(const INT8U address,                     // Read MCP2515 successive registers
                                     INT8U values[], 
                               const INT8U n);

    void mcp2515_setRegister(const INT8U address,                       // Set MCP2515 register
                             const INT8U value);

    void mcp2515_setRegisterS(const INT8U address,                      // Set MCP2515 successive registers
                              const INT8U values[],
                              const INT8U n);

    void mcp2515_initCANBuffers(void);

    void mcp2515_modifyRegister(const INT8U address,                    // Set specific bit(s) of a register
                                const INT8U mask,
                                const INT8U data);

    INT8U mcp2515_readStatus(void);                                     // Read MCP2515 Status
    INT8U mcp2515_setCANCTRL_Mode(const INT8U newmode);                 // Set mode
    INT8U mcp2515_requestNewMode(const INT8U newmode);                  // Set mode
    INT8U mcp2515_configRate(const INT8U canSpeed,                      // Set baudrate

                             const INT8U canClock);
                             
    INT8U mcp2515_init(const INT8U canIDMode,                           // Initialize Controller
                       const INT8U canSpeed,
                       const INT8U canClock);

    void mcp2515_write_mf( const INT8U mcp_addr,                        // Write CAN Mask or Filter
                           const INT8U ext,
                           const INT32U id );

    void mcp2515_write_id( const INT8U mcp_addr,                        // Write CAN ID
                           const INT8U ext,
                           const INT32U id );

    void mcp2515_read_id( const INT8U mcp_addr,                         // Read CAN ID
      INT8U* ext,
                                INT32U* id );

    void mcp2515_write_canMsg( const INT8U buffer_sidh_addr );          // Write CAN message
    void mcp2515_read_canMsg( const INT8U buffer_sidh_addr);            // Read CAN message
    INT8U mcp2515_getNextFreeTXBuf(INT8U *txbuf_n);                     // Find empty transmit buffer

/*********************************************************************************************************
 *  CAN operator function
 *********************************************************************************************************/

    INT8U setMsg(INT32U id, INT8U rtr, INT8U ext, INT8U len, INT8U *pData);        // Set message
    INT8U clearMsg();                                                   // Clear all message to zero
    INT8U readMsg();                                                    // Read message
    INT8U sendMsg();                                                    // Send message

public:
    MCP_CAN(INT8U _CS);			    /* Class constructor  - allocates chip select pin but assume HW SPI */	
    INT8U begin(INT8U idmodeset, INT8U speedset, INT8U clockset);    	/* Initialise controller parameters
			resets the MCP, clears all masks and filters, enables RX of ext & normal and RX buffer 0 overflow
			input idmodeset MCP_STDEXT (use filters/masks) or MCP_ANY (disables filters/masks) 
			input speedset a CAN_xxxKBPS speed constant - configures internal clock divider to suit
			input clockset a MCP_20MHZ, 16MHZ or 8MHZ to suit system clock
			then switches to normal mode (Tx & Rx possible)
			result is either CAN_OK [0] or CAN_FAILINIT */
    INT8U init_Mask(INT8U num, INT8U ext, INT32U ulData);               /* Initialise Mask(s)
			sets mask bits of Rx buffer.  Mask defines which filter bits are relevant
			input num = 0 or 1 for Rx buffers 0 or 1
			input ext = CAN_STDID or CAN_EXTID depending on 11-bit or 29-bit ID
			input u1Data = mask : wherever bit is 1 means associated filter bit is relevant.
			returns CAN_OK [0] or relevant fault code */ 
    INT8U init_Mask(INT8U num, INT32U ulData);                          // Initialise Mask(s)
			// as above but the mask format includes coding whether it is extended by OR with CAN_IS_EXTENDED
    INT8U init_Filt(INT8U num, INT8U ext, INT32U ulData);               /* Initialise Filter(s)
			As Mask but setting filters.  E.G. Mask 0xF00 and Filter 0x600 will select all 0x6.. messages
			input num = 0 or 1 for Rx buffer 0; 2..5 for Rx buffer 1
			returns CAN_OK [0] or relevant fault code */
    INT8U init_Filt(INT8U num, INT32U ulData);                          // Initialize Filter(s)
			// as above but ulData includes coding CAN_IS_EXTENDED for extended filters as a short-hand
    void setSleepWakeup(INT8U enable);                                  /* Enable or disable
			the CAN wake interrupt (If enabled the MCP2515 will wake from sleep by CAN bus activity)
			input enable FALSE/TRUE */
    INT8U setMode(INT8U opMode);                                        /* Set operational mode
			Select CAN operating mode from
			MCP_NORMAL   Rx and Tx on CAN
			MCP_SLEEP    Self explanatory.  Can wake if Wake on CAN is enabled?
			MCP_LOOPBACK Internal loop-back for testing
			MCP_LISTENONLY Self explanatory.  For CAN monitoring applications */
    INT8U sendMsgBuf(INT32U id, INT8U ext, INT8U len, INT8U *buf);      /* Send message to transmit buffer
			inputs are
			id message ID to send,
			ext select whether this is an extended ID: CAN_STDID or CAN_EXTID,
			len data length (0...8) or as qualified by MAX_CHAR_IN_MESSAGE
			buf data array of bytes
			returns CAN_OK[0] or CAN_GETTXBFTIMEOUT or CAN_SENDMSGTIMEOUT */
    INT8U sendMsgBuf(INT32U id, INT8U len, INT8U *buf);                 // Send message to transmit buffer
			// as above but ID has internal coding for whether it is extended or not
    INT8U readMsgBuf(INT32U *id, INT8U *ext, INT8U *len, INT8U *buf);   /* Read message from receive buffer
			Reads message from buffer 0 if available otherwise buffer 1 if avail.
			CAN ID, extended/Std, Data length and data bytes returned via arguments
			returns CAN_OK [0] or CAN_NOMSG
			Note: message ID was previously available via getCanId() method which is now deprecated */
	INT8U readMsgBuf(INT32U *id, INT8U *len, INT8U *buf);               // Read message from receive buffer
			// as above but ID has internal coding to indicate whether extended/Std
			// using bit masks CAN_IS_EXTENDED, and CAN_IS_REMOTE_REQUEST
    INT8U checkReceive(void);                                           // Check for received data
			/* if something received	returns CAN_NOMSG or CAN_MSGAVAIL */
    INT8U checkError(void);                                             // Check for errors
			// returns CAN_OK[0] or CAN_CTRLERROR based on EFLG register
    INT8U getError(void);                                               /* Check for errors
			returns the actual error register for you to deal with.
			BitMasks MCP_EFLG_xyz #define to find what type of error
			bits 0...2 are warnings so bitwise AND with MCP_EFLG_ERRORMASK to extract true errors */
    INT8U errorCountRX(void);                                           // Get error count RX register
    INT8U errorCountTX(void);                                           // Get error count TX register
    INT8U enOneShotTX(void);                                            // Enable one-shot transmission
			// One-shot Tx will not attempt to re-send on send failure, use for deterministic bus timing.
    INT8U disOneShotTX(void);                                           // Disable one-shot transmission
    INT8U abortTX(void);                                                // Abort queued transmission(s)
    INT8U setGPO(INT8U data);                                           // Sets GPO
    INT8U getGPI(void);                                                 // Reads GPI
};

#endif
/*********************************************************************************************************
 *  END FILE
 *********************************************************************************************************/
