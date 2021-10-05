#include <iostream>
#include "./robotConnection/RobotConnection.h"
#include "./app/App.h"
#include "./gripperHandling/GripperController.h"
#include "./jointPoseGetter/JointPoseGetter.h"


using namespace std;

int main()
{
    JointPoseGetter j;

    vector<vector<double>> test = j.jacobian(1, 1, 1, 1, 1, 1);

    for (int i = 0; i < test.size(); i++)
    {
        for (int j = 0; j < test[i].size(); j++)
        {
            cout << test[i][j] << ", ";
        }
        cout << endl;
    }

    return 0;
}
