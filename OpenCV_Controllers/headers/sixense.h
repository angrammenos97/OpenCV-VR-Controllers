#pragma once
#ifndef _SIXENSE_H_
#define _SIXENSE_H_

#include <array>

#define SIXENSE_BUTTON_BUMPER   (0x01<<7)
#define SIXENSE_BUTTON_JOYSTICK (0x01<<8)
#define SIXENSE_BUTTON_1        (0x01<<5)
#define SIXENSE_BUTTON_2        (0x01<<6)
#define SIXENSE_BUTTON_3        (0x01<<3)
#define SIXENSE_BUTTON_4        (0x01<<4)
#define SIXENSE_BUTTON_START    (0x01<<0)

typedef struct SixenseEmulatedData
{
	float yaw, pitch, roll;
	float x, y, z;
	float joystick_x;
	float joystick_y;
	float trigger;
	unsigned int buttons;
	int enabled;
	int controller_index;
	unsigned char is_docked;
	unsigned char which_hand;
}SixenseEmulatedData;

void initSixense();
void sixenseSetData(std::array<SixenseEmulatedData, 2> input);

#endif /* _SIXENSE_H_ */