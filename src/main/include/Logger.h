#pragma once

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <frc/Timer.h>

#include "Constants.h"

using namespace std;
class Logger
{
    public:
        Logger(string fileName);
        void openFile();
        void closeFile();
        void addPrint(string print);
        void print();
        void print(double print);
        void print(string print);
    private:
        ofstream outstream_;
        string fileName_;

        vector<pair<double, string>> prints_;

        bool open_;
        double startTime_;

        frc::Timer timer_; //TODO implement, also has red thing weird yeah
};