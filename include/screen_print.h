#ifndef SCREEN_PRINT_H
#define SCREEN_PRINT_H

#include "vector"
#include "vex.h"

void draw_background (void);
void draw_point (double x, double y, double r, color pcolor);
void draw_pixel (double x, double y);
void draw_path (std::vector<std::vector<double>> path);
void draw_obstacles(void);
void obstacle_update (void);

#endif