#pragma once


void toggleKeymap(char pressed);
void applyKeymap();

void toggleOutputTarget();
std::string getCurrentOutputTarget();   

void on_combo(char held, char pressed);
void on_short_press(char btn);
void on_long_press(char btn);

void on_battery_updated(uint8_t level);

void start_config_mode();
