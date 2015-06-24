﻿#include "robot_interface.h"


const int ROBOT_STATE_MACHINE[ROBOT_CMD_COUNT][ROBOT_STATE_COUNT] =
{
	/* UNKNOWN | UNKNOWN1_HOMED2 | UNKNOWN1_STARTED2 | HOMED1_UNKNOWN2 | HOMED1_STARTED2 | STARTED1_UNKNOWN2 | STARTED1_HOMED2 | STARTED*/
	{ HOMED1_UNKNOWN2, -1, HOMED1_STARTED2, HOMED1_UNKNOWN2, HOMED1_STARTED2, HOMED1_UNKNOWN2, -1, HOMED1_STARTED2 },/* HOME_1 */
	{ UNKNOWN1_HOMED2, UNKNOWN1_HOMED2, UNKNOWN1_HOMED2, -1, -1, STARTED1_HOMED2, STARTED1_HOMED2, STARTED1_HOMED2 },/* HOME_2 */
	{ -1, -1, -1, STARTED1_UNKNOWN2, STARTED, -1, -1, -1 },/* HOME2START_1 */
	{ -1, UNKNOWN1_STARTED2, -1, -1, -1, -1, STARTED, -1 },/* HOME2START_2 */
	{ -1, -1, -1, -1, -1, -1, -1, STARTED },/* MV_FORWARD */
	{ -1, -1, -1, -1, -1, -1, -1, STARTED },/* MV_BACKWARD */
	{ -1, -1, -1, -1, -1, -1, -1, STARTED },/* TURN_LEFT */
	{ -1, -1, -1, -1, -1, -1, -1, STARTED },/* TURN_RIGHT */
};

Aris::Core::CONN control_interface;
Aris::Core::CONN visual_interface;
Aris::Core::CONN data_interface;