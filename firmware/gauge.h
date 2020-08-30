#ifndef GAUGE_H
#define GAUGE_H

float gauge_bat_v();
float gauge_bat_a();
float gauge_sys_a();

uint8_t gauge_bat_percent(float bat_v);


#endif