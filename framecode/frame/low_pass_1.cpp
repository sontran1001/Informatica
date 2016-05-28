#include "low_pass_1.h"

void Low_pass::init(double f_c, double y_0, double dt) {
	_y_p       = y_0;
	_omega_cut = 6.28318530718 * f_c; // 2 * pi * f_c
	_alpha     = (_omega_cut * dt) / (1+ _omega_cut * dt);
}

Low_pass::Low_pass(double f_c, double dt) {
	init(f_c, 0, dt);
}

Low_pass::Low_pass(double f_c, double y_0, double dt) {
	init(f_c, y_0, dt);
}

double Low_pass::update(double u) {
   	_y_p = (1 - _alpha) * _y_p + _alpha * u;

   return _y_p;
}

