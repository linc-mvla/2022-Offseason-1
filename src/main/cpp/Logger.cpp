#include "Logger.h"

Logger::Logger(string fileName)
{
    fileName_ = fileName;
    open_ = false;
    startTime_ = 0;
}

void Logger::openFile()
{
    outstream_.open(fileName_, ofstream::out);
    //outstream_.open(fileName_);
    open_ = true;
    startTime_ = timer_.GetFPGATimestamp().value();
    //timer_.Reset();
    //timer_.Start();
}

void Logger::closeFile()
{
    outstream_.close();
    open_ = false;
    //timer_.Stop();
}

void Logger::addPrint(string print)
{
    if(startTime_ == 0)
    {
        startTime_ = timer_.GetFPGATimestamp().value();
    }
    double time = timer_.GetFPGATimestamp().value() - startTime_;
    pair<double, string> printPair(time, print);
    prints_.push_back(printPair);
}

void Logger::print()
{
    ofstream outstream;
    outstream.open(fileName_, ofstream::out);


    for(size_t i = 0; i < prints_.size(); ++i)
    {
        cout << prints_[i].first << ", " << prints_[i].second << endl;
        outstream << prints_[i].first << ", " << prints_[i].second << endl;
    }

    prints_.clear();
    startTime_ = 0;
    outstream.close();
}

void Logger::print(double print)
{
    if(!open_)
    {
        openFile();
    }

    double time = timer_.GetFPGATimestamp().value() - startTime_;

    stringstream stream;
    stream << time << ", " << print;
    //cout << stream.str() << endl;
    outstream_ << stream.str() << endl;

    //cout << stod(print) << endl;
    //outstream_ << time << ", " << print << endl;
}

void Logger::print(string print)
{
    if(!open_)
    {
        openFile();
    }

    double time = timer_.GetFPGATimestamp().value() - startTime_;

    //cout << stod(print) << endl;

    outstream_ << time << ", " << print << endl;
}