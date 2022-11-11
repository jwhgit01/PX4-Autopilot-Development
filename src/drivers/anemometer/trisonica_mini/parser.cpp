/****************************************************************************
 *
 *   Copyright (c) 2014 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file parser.cpp
 * @author Jeremy W. Hopwood
 *
 * Parser for the TriSonica Mini 3D sonic anemometer
 */

#include "parser.h"

/*
 * This function parses data from a read buffer one character
 * at a time and returns a valid packet once assembled.
 *
 * This parser relies on the trisonica mini anemometer data
 * packets to be configured as follows:
 *
 * 	S 05.2 D 112 U -01.9 V 04.7 W 01.1 T 22.6 \r\n
 *
 * The tags are in the following format and order:
 *	S = Wind Speed
 *	D = Wind Direction
 *	U = U-Vector
 *	V = V-Vector
 *	W = W-Vector
 *	T = Temperature
 *
 */
int trisonica_mini_parser(char c,char *buffer,int *buffer_index,int *parse_state,float *S,float *D,float *U,float *V,float *W,float *T){

	/* If we are lost, look for a start or end character */
	if ( (*parse_state)<1 ){
		switch (c) {
		case START_PACKET:
			buffer[0] = c;
			(*buffer_index) = 1;
			(*parse_state) = 1;
			break;

		case END_PACKET:
			(*buffer_index)++;
			(*parse_state) = 1;
			break;

		default:
			(*buffer_index)++;
			break;
		}
		return 0;
	}

	/* If we have gotten to here, we are not lost and are either
	   indexed at the start, middle, or end of a packet. */
	switch (c) {

		/* If we get a start-of-packet, begin storing it. */
		case START_PACKET:
			buffer[0] = c;
			(*buffer_index) = 1;
			return 0;
			break;

		/* If we get an end-of-packet character, return the valid buffer. */
		case END_PACKET:
			buffer[*buffer_index] = '\0';
			int scan_result = sscanf(buffer,"S%f D%f U%f V%f W%lf T%lf\r",S,D,U,V,W,T);
			if (scan_result != 6){
				(*parse_state) = 0;
				return 0;
			}
			/* If the result is good, reset the index to zero and return OK */
			(*buffer_index) = 0;
			return 1;
			break;

		/* If it's not a start or end of packet, put it in the buffer to be scanned. */
		default:
			buffer[*buffer_index] = c;
			(*buffer_index)++;
			return 0;
			break;
	}

}
