#include "consts.hpp"

///Window managment
const sf::VideoMode WIN_MODE(600,600);
const sf::Time TIME_PER_FRAME = sf::milliseconds(33);
size_t MAX_VERTICES = 500000;
size_t MAX_STEPS = 1000000;
size_t STEPS_PER_LINE = 50;

///Physics
const double G = 6.674 * pow(10, -11); //Newton's universal gravity constant
const double dt = 1; //resolution in time of the simulation

double rescaling = 0.8 * pow(10, -9);
