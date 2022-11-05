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
 * @author Lorenz Meier <lm@inf.ethz.ch>
 * @author Jeremy Hopwood
 *
 * Parser for the TriSonica Mini 3D sonic anemometer
 */

#include "parser.h"
#include <string.h>
#include <stdlib.h>

//#define LW_DEBUG

#ifdef LW_DEBUG
	#include <stdio.h>

	const char *parser_state[] = {
		"0_UNSYNC",
		"1_SYNC",
		"2_GOT_DIGIT0",
		"3_GOT_DOT",
		"4_GOT_DIGIT1",
		"5_GOT_DIGIT2",
		"6_GOT_CARRIAGE_RETURN"
	};
#endif

/*
 * This parser relies on the trisonica mini anemometer to be configured
 * with custom delimiters where a colon ":" comes after the tag and a
 * comma "," comes after the number. An example is given as follows:
 *
 * 	S:05.2,D:112,U:-01.9,V:04.7.W:01.1,T:22.6\r\n
 *
 * The tags are in the following format and order:
 *	S = Wind Speed
 *	D = Wind Direction
 *	U = U-Vector
 *	V = V-Vector
 *	W = W-Vector
 *	T = Temperature
 * V_m_s,&direction_deg,&u_m_s,&v_m_s,&w_m_s,&T_C)
 */
int trisonica_mini_parser(
	char c,
	char *parserbuf,
	unsigned *parserbuf_index,
	enum TRIMINI_PARSE_STATE *state,
	float *S,
	float *D,
	float *U,
	float *V,
	float *W,
	float *T)
{
	int ret = -1;
	char *end;

	// TODO: everything below here that is uncommented is junk.

	switch (*state) {
	case TRIMINI_PARSE_STATE0_UNSYNC:

		/* If we reach a new line, we know where we are */
		if (c == '\n') {

			/* start looking for data */
			*state = TRIMINI_PARSE_STATE1_SYNC;

			/* reset the parsing buffer index */
			(*parserbuf_index) = 0;
		}

		break;

	case TRIMINI_PARSE_STATE1_SYNC:
		if (c == 'S') {
			*state = TRIMINI_PARSE_STATE2_GOT_SPEED;
			parserbuf[*parserbuf_index] = c;
			(*parserbuf_index)++;
		} else if (c == 'D') {
			*state = TRIMINI_PARSE_STATE3_GOT_DIRECTION;
			parserbuf[*parserbuf_index] = c;
			(*parserbuf_index)++;
		} else if (c == 'U') {
			*state = TRIMINI_PARSE_STATE4_GOT_U;
			parserbuf[*parserbuf_index] = c;
			(*parserbuf_index)++;
		} else if (c == 'V') {
			*state = TRIMINI_PARSE_STATE5_GOT_V;
			parserbuf[*parserbuf_index] = c;
			(*parserbuf_index)++;
		} else if (c == 'W') {
			*state = TRIMINI_PARSE_STATE6_GOT_W;
			parserbuf[*parserbuf_index] = c;
			(*parserbuf_index)++;
		} else if (c == 'T') {
			*state = TRIMINI_PARSE_STATE7_GOT_T;
			parserbuf[*parserbuf_index] = c;
			(*parserbuf_index)++;
		} else {
			*state = TRIMINI_PARSE_STATE0_UNSYNC;
		}

		break;

	case TRIMINI_PARSE_STATE2_GOT_SPEED:
		if (c >= '0' && c <= '9') {
			*state = TRIMINI_PARSE_STATE1_SYNC;
			parserbuf[*parserbuf_index] = c;
			(*parserbuf_index)++;

		} else if (c == '.') {
			*state = LW_PARSE_STATE3_GOT_DOT;
			parserbuf[*parserbuf_index] = c;
			(*parserbuf_index)++;

		} else {
			*state = LW_PARSE_STATE0_UNSYNC;
		}

		break;

	case TRIMINI_PARSE_STATE2_GOT_DIGIT0:
		if (c >= '0' && c <= '9') {
			*state = LW_PARSE_STATE2_GOT_DIGIT0;
			parserbuf[*parserbuf_index] = c;
			(*parserbuf_index)++;

		} else if (c == '.') {
			*state = LW_PARSE_STATE3_GOT_DOT;
			parserbuf[*parserbuf_index] = c;
			(*parserbuf_index)++;

		} else {
			*state = LW_PARSE_STATE0_UNSYNC;
		}

		break;

	case TRIMINI_PARSE_STATE3_GOT_DOT:
		if (c >= '0' && c <= '9') {
			*state = LW_PARSE_STATE4_GOT_DIGIT1;
			parserbuf[*parserbuf_index] = c;
			(*parserbuf_index)++;

		} else {
			*state = TRIMINI_PARSE_STATE0_UNSYNC;
		}

		break;

	case TRIMINI_PARSE_STATE4_GOT_DIGIT1:
		if (c >= '0' && c <= '9') {
			*state = LW_PARSE_STATE5_GOT_DIGIT2;
			parserbuf[*parserbuf_index] = c;
			(*parserbuf_index)++;

		} else {
			*state = LW_PARSE_STATE0_UNSYNC;
		}

		break;

	case TRIMINI_PARSE_STATE5_GOT_DIGIT2:
		if (c == '\r') {
			*state = LW_PARSE_STATE6_GOT_CARRIAGE_RETURN;

		} else {
			*state = LW_PARSE_STATE0_UNSYNC;
		}

		break;

	case TRIMINI_PARSE_STATE6_GOT_CARRIAGE_RETURN:
		if (c == '\n') {
			parserbuf[*parserbuf_index] = '\0';
			*dist = strtod(parserbuf, &end);
			*state = TRIMINI_PARSE_STATE1_SYNC;
			*parserbuf_index = 0;
			ret = 0;

		} else {
			*state = TRIMINI_PARSE_STATE0_UNSYNC;
		}

		break;
	}

#ifdef LW_DEBUG
	printf("state: TRIMINI_PARSE_STATE%s\n", parser_state[*state]);
#endif

	return ret;
}
