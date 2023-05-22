#ifndef DEBUG_DEFINITIONS_H
#define DEBUG_DEFINITIONS_H

#include <array>

struct PidDebugDefinitions
{
    float angle{};
    float desiredAngle{};
    float error{};
    float lastError{};
    float pidOutput{};
};


struct EngineDebugDefinitions
{
    float after_frontLeftEngine{};
    float after_frontRightEngine{};
    float after_backLeftEngine{};
    float after_backRightEngine{};
};

struct DebugInformation
{
    float throttle{};
    std::array<PidDebugDefinitions,3> pid;
    EngineDebugDefinitions engine;
};



 #endif