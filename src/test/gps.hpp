#ifndef GPS_HPP_INCLUDED
#define GPS_HPP_INCLUDED

#include <queue>

#include "solarsystem.hpp"

/** \brief This class generates an array of transfer orbits between a start and a finish planet
 *
 */


class GPS {

public:

    struct Step{
        unsigned int planet;
        double fly_by_time;
        double total_deltav;
        Elliptic_Orbit incomming_orbit;
    };

    struct GPSPerformances{
        unsigned int generated, considered, discarded;
    };

public:

    GPS(SolarSystem& system);

    std::vector<Step> path(unsigned int start, double start_time, unsigned int finish, double (*scorer)(double, double));

    inline GPSPerformances get_performances() {return {m_generated, m_considered, m_discarded};}

///Members
protected:
    SolarSystem* m_sys;
    unsigned int m_start, m_finish;
    std::vector<Step> m_steps;

    double (*m_scorer)(double, double);

    struct Candidate {
        Step manouver;
        double score;
        int previous;
    };

    //declared as static to avoid having multiple arrays in the memory
    //(is always reset at the end of the execution)
    static std::vector<Candidate> s_pool;

    class Compare {
    public:
        bool operator()(int left, int right){
            return GPS::s_pool[left].score >= GPS::s_pool[right].score;
        }
    };

    //the candidate queue
    std::priority_queue<int, std::vector<int>, Compare> m_queue;

    //the best so far. Used to prun not good enough branches
    int m_best;
    double m_best_score;

    double m_max_docking_time;
    double m_docking_step;

    //performance counters
    unsigned int m_generated;
    unsigned int m_considered;
    unsigned int m_discarded;

///Internal
protected:

    void clear();

    bool add(Candidate potential);

    void extract(int current);

    bool forward();
};

#endif // GPS_HPP_INCLUDED
