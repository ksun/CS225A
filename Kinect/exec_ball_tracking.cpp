#include "ball_tracking.hpp"
#include <iostream>
#include <stdio.h>


int main(){

    BallTracker BT("Green");
    
    BT.run();
    cout << BT.getPosition() << endl;
    cout << BT.getVelocity() << endl;

}
