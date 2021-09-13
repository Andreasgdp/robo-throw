#ifndef ROBOTCONNECTION_H
#define ROBOTCONNECTION_H

#include <string>

class RobotConnection
{
public:
    RobotConnection();

    void connect(std::string IP);

private:
    std::string IP;
};

#endif // ROBOTCONNECTION_H
