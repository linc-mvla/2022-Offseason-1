#pragma once
#include "AutoState.h"
#include <vector>

class AutoExecutor{
    public:
        void execute();
        void periodic();

    std::vector<AutoState> states;

};