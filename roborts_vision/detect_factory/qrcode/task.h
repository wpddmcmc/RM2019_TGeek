#pragma once 

#include <iostream>
#include <opencv2/opencv.hpp>
#include <vector>

bool task1(cv::Mat image, std::vector<int>& key);
void task2(cv::Mat image, std::vector<int>& key);

short get_yaw();
short get_pitch();

void listen2car();

void publish2car(std::vector<int>& key);