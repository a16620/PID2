#include "PlaneControl.h"

Navigator nav;
MasterControl master(&nav);

const unsigned char schedule[] = { 0, 1, 0, 1, 2};
short schedule_counter = 0;

void setup() {
	PlaneController::SetupPin();

	PlaneGyro::getInstance().calibrate();
}

void loop() {
	switch (schedule[schedule_counter]) {
	case 0:
	{
		TimeChecker::getInstance().update();
		PlaneGyro::getInstance().update();
		break;
	}
	case 1:
	{
		nav.update();
		master.process();
		break;
	}
	}

	if (++schedule_counter == sizeof(schedule))
		schedule_counter = 0;
}