#pragma once

#include "../private_include/pcf8563.h"

void init_pcf8563_interface();

extern struct tm get_current_time();


void update_pcf8586_date(time_t*);