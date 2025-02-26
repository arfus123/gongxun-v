#pragma once

#include <string>
#include <opencv2/opencv.hpp>

extern std::string camInput;
extern std::string camOutputX;
extern std::string camOutputY;

extern double fx;
extern double fy;
extern double cx;
extern double cy;
extern double k1;
extern double k2;
extern double p1;
extern double p2;
extern double k3;

extern int width;
extern int height;
extern int fps;
extern int RGB;

void readParameters(const std::string &config_file);