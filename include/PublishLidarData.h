#pragma once

#include "common.h"
#include "utils.h"
#include <ros/ros.h>

#include <iostream>
#include <fstream>


int ReadVIOdata(const std::string Path, DataBase *db);
int ReadLidardata(const std::string Path, const std::string LidarBinaryPath, DataBase* db);


