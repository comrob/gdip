#include <iostream>

#include "opendubins/dubins.h"

using namespace std;
using namespace opendubins;

int main() {

    cout << "Hello, I am OpenDubins library!" << endl;
    Dubins d = Dubins(State(0, 0, 0), State(1, 1, 1), 1);
    cout << d << endl;

    return 0;
}