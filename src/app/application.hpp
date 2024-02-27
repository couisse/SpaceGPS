#ifndef APPLICATION_HPP_INCLUDED
#define APPLICATION_HPP_INCLUDED

#include <cstring>

#include "../render/window.hpp"
#include "../utils/random.hpp"

#include "../path/thrust_manager.hpp"

/** \brief Main class of the application
 */

class Application{

public:

    /** \brief The constructor also initialize all the environment of execution
     */
    Application();

    ~Application();

    /** \brief Main loop of the app
    */
    void play(int argc, char** argv);

///Member variables
protected:

    //generics
    sf::Clock m_clock;
    //rendering
    Viewer m_viewer;

///Internal
protected:


    struct Performances{
        GPS::GPSPerformances gpsperf;
        ThrustCalculator::ThrustPerformances thrustperf;
    };

    void failmessage(std::string reason);

    Performances resolve(SolarSystem& sys,
                 unsigned short startnumber, double startheight, double starttime,
                 unsigned short targetnumber, double targetheight,
                 double (*scorer)(double, double), double shipacceleration,
                 bool display);

};

#endif // APPLICATION_HPP_INCLUDED
