#include <iostream>
#include "./robotConnection/RobotConnection.h"

using namespace std;

int main()
{
    RobotConnection conn;

    conn.connect("127.0.0.1");
    cout << "Hello World!" << endl;
    return 0;
}
