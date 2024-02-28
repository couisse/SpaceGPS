#ifndef MACROS_HPP_INCLUDED
#define MACROS_HPP_INCLUDED

///standards macros
#define _USE_MATH_DEFINES //constants defined in <cmath>

#ifdef DBG
///Debug configuration

#include <iostream>

#define minLogPriority 1
#define consoleLog(x, priority) { if (priority <= minLogPriority) {std::cout << x << std::endl;} }

#else

///Other config

#define consoleLog(x, y) {}

#endif

#endif // MACROS_HPP_INCLUDED
