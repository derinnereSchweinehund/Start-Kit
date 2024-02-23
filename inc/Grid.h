#pragma once

#include "common.h"

class Grid
{
public:
    Grid(string fname);

    int rows;
    int cols;
    std::vector<int> map;
    string map_name;

};
