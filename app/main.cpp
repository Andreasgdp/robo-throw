#include <iostream>
#include "./robotConnection/RobotConnection.h"
#include "./simulation/Simulation.h"

using namespace std;

int main()
{
    Simulation sim;
    sim.showImage(sim.loadImage("thyra", "jpeg"));

    RobotConnection conn("127.0.0.1");
    std::vector<double> test = conn.getActualJointPoses();

    for (size_t i = 0; i < test.size(); i++) {
        std::cout << test.at(i);
    }

    return 0;
}
