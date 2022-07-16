#include "PIDControl.h"

int main() {
	FILE* op;
	fopen_s(&op, "C:\\Users\\Junsung\\Documents\\vscode\\proj\\plot tool\\data.csv", "wt");

	SimpleMotor motor;
	PIDControl pid;
	pid.SetP(0.05);
	pid.SetI(0.01);
	pid.SetD(0.02);

	const double target = 100;
	double loc = 0;
	for (int i = 0; i < 100;  i++) {
		auto obs_loc = loc + 2*sin(rand());
		auto c = pid.GetControlValue(target - obs_loc);
		motor.Set(c);
		auto o = motor.Get();
		loc += (o) * 2-9.8+sin(rand());
		if (loc < 0)
			loc = 0;
		fprintf(op, "%d,%f\n", i, loc);
	}

	fclose(op);
	return 0;
}