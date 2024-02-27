#include "application.hpp"

#include <iostream>
#include <fstream>
#include <ctime>
#include <algorithm>

Application::Application(): m_viewer(){
    Random::init();
}

Application::~Application(){
}

double scorerThrust(double time, double deltav){
    return deltav;
}

double scorerTime(double time, double deltav){
    return time;
}

double scorerBalance(double time, double deltav){
    return deltav*time;
}


void Application::play(int argc, char** argv){

    ///Calling a bat file
    if (argc == 2){
        std::ifstream commandstream("in/" + std::string(argv[1]), std::ifstream::in);
        std::string command;

        while (std::getline(commandstream, command)){

            size_t argnum = std::count(command.begin(), command.end(), ' ') + 2;
            /*increasing the number of arguments by 2 to account for
            - the command name argument that doesn't exist in the bat file
            - the last argument has no following space
            */
            char** arguments = new char*[argnum];

            std::string token;
            size_t a(0), b(0);
            for (unsigned int i = 1; i < argnum ; ++i){
                b = command.find_first_of(" ", a);
                if (b == std::string::npos){
                    b = command.length();
                }
                token = command.substr(a, b-a);
                arguments[i] = new char[token.length()+1];
                strcpy(arguments[i], token.c_str());
                a = b+1;
            }

            std::cout << "\nExecuting new command:\n" << std::endl;

            this->play(argnum, arguments);

            for (unsigned int i = 1; i < argnum+1; ++i){
                delete[] arguments[i];
            }
            delete[] arguments;
        }
        return;
    }

    ///direct command
    if (argc == 11){
        SolarSystem sys(argv[1]);
        unsigned short startnumber; double startheight; double starttime;
        unsigned short targetnumber; double targetheight;
        double (*scorer)(double, double); double shipacceleration;
        bool display;
        std::string outputname;


        //the start planet number
        std::string name(argv[2]);
        startnumber = sys.search_planet(name);
        if (startnumber == sys.size()){
            this->failmessage("Wrong planet name: " + name); return;
        }

        //the start height
        if (strcmp(argv[3], "auto") == 0){
            startheight = 2 * sys[startnumber].get_rad();
        }else {
            startheight = strtod(argv[3], NULL) * 1000;
            if (startheight == 0.d){
                this->failmessage("Wrong start heigth"); return;
            }
        }

        //the start time
        starttime = strtod(argv[4], NULL);

        //the target planet number
        name = std::string(argv[5]);
        targetnumber = sys.search_planet(name);
        if (targetnumber == sys.size()){
            this->failmessage("Wrong planet name: " + name); return;
        }

        //the target heigth
        if (strcmp(argv[6], "auto") == 0){
            targetheight = 2 * sys[startnumber].get_rad();
        }else {
            targetheight = strtod(argv[6], NULL) * 1000;
            if (targetheight == 0.d){
                this->failmessage("Wrong target height"); return;
            }
        }

        //the scorer method
        if (strcmp(argv[7], "balance") == 0){
            scorer = scorerBalance;
        }else if (strcmp(argv[7], "time") == 0){
            scorer = scorerTime;
        }else if (strcmp(argv[7], "deltav") == 0){
            scorer = scorerThrust;
        }else {
            this->failmessage("Wrong scoring method"); return;
        }

        //the ship acceleration
        shipacceleration = strtod(argv[8], NULL);
        if (shipacceleration == 0.d){
            this->failmessage("Wrong acceleration"); return;
        }

        //display
        display = argv[9][0] == '1';

        //outputfile
        outputname = "out/" + std::string(argv[10]);

        Performances perfs = this->resolve(sys, startnumber, startheight, starttime, targetnumber, targetheight, scorer, shipacceleration, display);

        std::ofstream output(outputname, std::ios::app);

        output << perfs.gpsperf.considered << ";" << perfs.gpsperf.discarded << ";" << perfs.gpsperf.generated << ";"
            << perfs.thrustperf.steps_number << ";" << perfs.thrustperf.time << ";" << perfs.thrustperf.total_delta_v
            << ";" << perfs.thrustperf.success << std::endl;

        return;
    }

    this->failmessage("Wrong argument number");
    std::cout << "\n\n\n" << std::endl;
    return;
}


void Application::failmessage(std::string reason){
    std::cout << reason << "\nExpected:\n"
        << "- sgps <commandsfilename>\n"
        << "- sgps <systemfilename> <startname> <startheight(auto/double)> <starttime(double)> "
        << "<targetname> <targetheight(auto/double)> "
        << "<scoring(balance/time/deltav)>  "
        << "<shipacceleration(double)> "
        << "<shoulddisplay(1/0)> <outputfile>"
        << std::endl;
}

Application::Performances Application::resolve(SolarSystem& sys,
                 unsigned short startnumber, double startheight, double starttime,
                 unsigned short targetnumber, double targetheight,
                 double (*scorer)(double, double), double shipacceleration,
                 bool display){

    std::cout << "Using following solar system:\n\n" << sys << std::endl;



    //calculating the path
    GPS my_gps(sys);

    std::vector<GPS::Step> manouvers;

    sf::Clock chrono;

    manouvers = my_gps.path(startnumber, starttime, targetnumber,  scorer);

    std::cout << "\n\n" << chrono.getElapsedTime().asSeconds() << " seconds to calculate the flight plan" << std::endl;

    GPS::GPSPerformances perfs = my_gps.get_performances();
    std::cout << "With " << perfs.generated << " generated steps, " << perfs.considered << " considered and " << perfs.discarded << " discarded\n\n" << std::endl;


    for (unsigned int i = 0; i < manouvers.size(); ++i){
        std::cout << "Step " << i << " " << sys[manouvers[i].planet].get_name() << " on time " << manouvers[i].fly_by_time << std::endl;
    }

    std::cout << "\n" << manouvers.back().fly_by_time / (3600.d * 24.d * 365.d) << " years of travel" << std::endl;
    std::cout << "For " << manouvers.back().total_deltav << "m/s of delta-v\n\n\n" << std::endl;

    //simulating the travel
    VisualTrajectory* traj = nullptr;
    if (display){
        traj = m_viewer.newTrajectory(sf::Color::Yellow);
    }

    ThrustCalculator simulator(sys, shipacceleration);

    ThrustCalculator::ThrustPerformances tperfs = simulator.simulate(manouvers, startheight, targetheight, traj);

    std::cout << "Simulating required " << tperfs.steps_number << " steps, " << tperfs.total_delta_v << " deltav and "
     << tperfs.time / (3600 * 24 * 365) << " year(s)" << std::endl;


    /* Graphics */
    if (display){
        //displaying the solar system
        std::vector<Ellipse> ellipses;
        ellipses.reserve(sys.size());
        for (unsigned int i = 0; i < sys.size(); ++i){
            ellipses.emplace_back(sys[i], sf::Color::Black);
            m_viewer.addOverlay(&ellipses[i]);
        }
        Points sun(sf::Color(255, 140, 0));
        sun.addPoint(NullCoords);
        m_viewer.addOverlay(&sun);

        //displaying the transfer orbits
        std::vector<Ellipse> path_ellipses;
        Points cyan_marker(sf::Color(0, 255, 255));
        path_ellipses.reserve(manouvers.size());
        for (unsigned int i = 0; i < manouvers.size(); ++i){
            cyan_marker.addPoint(sys[manouvers[i].planet].get_cartesian_position_on_time(manouvers[i].fly_by_time));
            path_ellipses.emplace_back(manouvers[i].incomming_orbit, sf::Color::Red);
            m_viewer.addOverlay(&path_ellipses.back());
        }


        //launching the display. Blocks the execution until the display is closed
        m_viewer.play();

        //cleaning for a potential next step
        m_viewer.clearTrajectories();
        m_viewer.clearOverlay();
    }


    return {perfs, tperfs};
}
