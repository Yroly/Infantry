#include "gimbal.hpp"

void Gimbal::init(){
    pidMode = PidMode::INIT;
    ctrlMode = CtrlMode::Normal;
    initFlag = 1;
}

void Gimbal::medianInit() {
}