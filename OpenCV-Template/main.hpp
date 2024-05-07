#ifndef MAIN_HPP_INCLUDED
#define MAIN_HPP_INCLUDED

// Function declarations
void setup(void);                   //Function to configure the GPIO and devices for operation
int main(int argc, char** argv);
int PID(double setpoint, double pv);
#endif // MAIN_HPP_INCLUDED
