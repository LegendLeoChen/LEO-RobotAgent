/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#include "interface/KeyBoard.h"
#include <iostream>

static ros::NodeHandle* nhPtr = nullptr; 

KeyBoard::KeyBoard(){
    userCmd = UserCommand::NONE;
    userValue.setZero();

    tcgetattr( fileno( stdin ), &_oldSettings );
    _newSettings = _oldSettings;
    _newSettings.c_lflag &= (~ICANON & ~ECHO);
    tcsetattr( fileno( stdin ), TCSANOW, &_newSettings );

    pthread_create(&_tid, NULL, runKeyBoard, (void*)this);

    if(!nhPtr){    // initial once
        int argc = 0; char** argv = nullptr;
        ros::init(argc, argv, "keyboard_node");  // ⭐ init ROS node
        nhPtr = new ros::NodeHandle();
    }
    // sub /dog_cmd
    sub_ = nhPtr->subscribe("/dog_cmd", 10, &KeyBoard::cmdCallback, this);
}

KeyBoard::~KeyBoard(){
    pthread_cancel(_tid);
    pthread_join(_tid, NULL);
    tcsetattr( fileno( stdin ), TCSANOW, &_oldSettings );
}

UserCommand KeyBoard::checkCmd(){
    switch (_c){
    case '1':
        return UserCommand::L2_B;
    case '2':
        return UserCommand::L2_A;
    case '3':
        return UserCommand::L2_X;
    case '4':
        return UserCommand::START;
#ifdef COMPILE_WITH_MOVE_BASE
    case '5':
        return UserCommand::L2_Y;
#endif  // COMPILE_WITH_MOVE_BASE
    case '0':
        return UserCommand::L1_X;
    case '9':
        return UserCommand::L1_A;
    case '8':
        return UserCommand::L1_Y;
    case ' ':
        userValue.setZero();
        return UserCommand::NONE;
    default:
        return UserCommand::NONE;
    }
}

void KeyBoard::changeValue(){
    switch (_c){
    case 'w':case 'W':
        userValue.ly = min<float>(userValue.ly+sensitivityLeft, 1.0);
        break;
    case 's':case 'S':
        userValue.ly = max<float>(userValue.ly-sensitivityLeft, -1.0);
        break;
    case 'd':case 'D':
        userValue.lx = min<float>(userValue.lx+sensitivityLeft, 1.0);
        break;
    case 'a':case 'A':
        userValue.lx = max<float>(userValue.lx-sensitivityLeft, -1.0);
        break;

    case 'i':case 'I':
        userValue.ry = min<float>(userValue.ry+sensitivityRight, 1.0);
        break;
    case 'k':case 'K':
        userValue.ry = max<float>(userValue.ry-sensitivityRight, -1.0);
        break;
    case 'l':case 'L':
        userValue.rx = min<float>(userValue.rx+sensitivityRight, 1.0);
        break;
    case 'j':case 'J':
        userValue.rx = max<float>(userValue.rx-sensitivityRight, -1.0);
        break;
    default:
        break;
    }
}

void KeyBoard::cmdCallback(const std_msgs::String::ConstPtr& msg)
{
    if (msg->data.empty()) return;

    std::string data = msg->data;

    // CMD
    if (data.rfind("CMD:", 0) == 0) {
        _c = data[4];   // CMD:2 -> '2'
        userCmd = checkCmd();
        if(userCmd == UserCommand::NONE)
            changeValue();
        _c = '\0';
        ROS_INFO("Got CMD: %c", data[4]);
    }
    // SPEED
    else if (data.rfind("VEL:", 0) == 0) {
        try {
            float v = std::stof(data.substr(4));
            userValue.ly = userValue.ly / velocity * v;
            userValue.lx = userValue.lx / velocity * v; 
            velocity = v;   // 你需要在类里加一个 float speedScale
            ROS_INFO("Set speed scale = %.2f", velocity);
        } catch (...) {
            ROS_WARN("Invalid VEL input: %s", data.c_str());
        }
    }
    // DIRECT
    else if (data.rfind("DIR:", 0) == 0) {
        // DIR:W=55,A=45
        std::string dirStr = data.substr(4);
        std::stringstream ss(dirStr);
        std::string token;
        while (std::getline(ss, token, ',')) {
            ROS_INFO("Set speed scale = %.2f", velocity);
            if (token.rfind("W=", 0) == 0) {
                userValue.ly = std::stof(token.substr(2)) / 100.0 * velocity;  // 前进分量
            } else if (token.rfind("S=", 0) == 0) {
                userValue.ly = -std::stof(token.substr(2)) / 100.0 * velocity; // 后退分量
            } else if (token.rfind("A=", 0) == 0) {
                userValue.lx = -std::stof(token.substr(2)) / 100.0 * velocity; // 左向分量
            } else if (token.rfind("D=", 0) == 0) {
                userValue.lx = std::stof(token.substr(2)) / 100.0 * velocity;  // 右向分量
            }
        }
        ROS_INFO("Direction set: lx=%.2f, ly=%.2f", userValue.lx, userValue.ly);
    }
}

void* KeyBoard::runKeyBoard(void *arg){
    ((KeyBoard*)arg)->run(NULL);
    return NULL;
}

void* KeyBoard::run(void *arg){
    while(1){
        FD_ZERO(&set);
        FD_SET( fileno( stdin ), &set );

        res = select( fileno( stdin )+1, &set, NULL, NULL, NULL);

        if(res > 0){
            ret = read( fileno( stdin ), &_c, 1 );
            userCmd = checkCmd();
            if(userCmd == UserCommand::NONE)
                changeValue();
            _c = '\0';
        }
        usleep(1000);
    }
    return NULL;
}