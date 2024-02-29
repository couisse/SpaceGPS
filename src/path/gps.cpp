#include <cfloat>
#include <algorithm>

#include "gps.hpp"

std::vector<GPS::Candidate> GPS::s_pool;

GPS::GPS(SolarSystem& sys){
    m_sys = &sys;
}

std::vector<GPS::Step> GPS::path(unsigned int start, double start_time, unsigned int finish, double (*scorer)(double, double)){

    //loading the arguments
    m_start = start;
    m_finish = finish;
    m_scorer = scorer;
    m_best = -1;
    m_best_score = DBL_MAX;

    //resetting the counters
    m_generated = 0;
    m_considered = 0;
    m_discarded = 0;

    //reserving a chunk of memory. Probably not enough, but will avoid multiple low-size
    //memory allocations
    s_pool.reserve(100);

    //calculating the docking times
    double T1 = m_sys->at(start).period();
    double T2 = m_sys->at(finish).period();
    m_docking_step = std::min(T1 * 0.25, T2 * 0.25 );
    //a quarter of a period is a good balance between orbital change and number of points
    m_max_docking_time = start_time + ceil(std::max(T1, T2) / std::abs(T1 - T2)) * T1;

    //loading the first step
    Candidate first;
    first.manouver.total_deltav = 0;
    first.manouver.fly_by_time = start_time;
    first.manouver.planet = start;
    first.manouver.incomming_orbit = m_sys->at(start);
    first.score = 0;
    first.previous = -1;
    this->add(first);

    //launching the pathfinding

    while (this->forward()){}

    //retrieving the final steps
    consoleLog("\nGenerating the path...", 1);
    this->extract(m_best);

    //creates a copy before resetting
    std::vector<Step> res(m_steps.size());
    std::reverse_copy(std::begin(m_steps), std::end(m_steps), std::begin(res));

    //clearing the mess
    this->clear();

    return std::move(res);
}

void GPS::clear(){
    s_pool.clear();
    while (!m_queue.empty()){ //priority queue has no clear() method
        m_queue.pop();
    }
    m_steps.clear();
}


bool GPS::add(Candidate potential){

    m_generated++;

    if (potential.score >= m_best_score){
            m_discarded++;
            return false;
    } //discard the ones that are not interesting

    s_pool.push_back(potential);
    m_queue.push(s_pool.size() - 1 );

    if (potential.manouver.planet == m_finish){
        m_best_score = potential.score;
        m_best = s_pool.size() - 1;
    }

    return true;
}

void GPS::extract(int current){
    if (current < 0){return;}
    consoleLog("Adding step #" << current, 4);
    m_steps.push_back(s_pool[current].manouver);
    this->extract(s_pool[current].previous);
}

bool GPS::forward(){
    if (m_queue.empty()){return false;} //something went wrong

    if (m_considered % 50 == 0 && m_considered != 0){
        std::cout << m_considered << " steps considered thus far. " << m_queue.size() << " pending" << std::endl;
    }
    m_considered++;

    int n_current = m_queue.top();
    Candidate* current = &s_pool[n_current];
    m_queue.pop();

    unsigned int n_planet = current->manouver.planet;
    double time = current->manouver.fly_by_time;

    consoleLog("\n\n" << "Currently on " << m_sys->at(n_planet).get_name() << ", t = " << time << " #" << n_current <<  "\t\t**************", 1);
    consoleLog("Previous being #" << current->previous << "\n", 4);
    consoleLog(m_queue.size() << " steps pending", 4);

    if ( n_planet == m_finish){return false;} //we arrived


    Coords incomming_speed = current->manouver.incomming_orbit.get_cartesian_speed_on_time(time);
    Coords place = current->manouver.incomming_orbit.get_cartesian_position_on_time(time);
    Coords planet_speed = m_sys->at(n_planet).get_cartesian_speed_on_time(time);


    ///going for the next planet
    for (unsigned int i = 0; i < m_sys->size(); ++i){

        if (i == n_planet){continue;}

        consoleLog("Trying to get to " << m_sys->at(i).get_name() << "...", 2);

        Elliptic_Orbit::RDV transfer = Elliptic_Orbit::fly_by_problem(
                                                m_sys->get_sun_mass(),
                                                place, incomming_speed, planet_speed, time,
                                                m_sys->at(i),
                                                m_scorer, 128, 128);

        if (!transfer.transfer.is_valid()){continue;}

        Coords needed_speed = transfer.transfer.get_cartesian_speed_on_time(time);

        if ((dot(needed_speed, incomming_speed) <= 0) && current->previous >= 0){continue;}
        //discarding the ones that need a too sharp turn, but only if it is not a initial burst

        Candidate next;
        next.manouver.fly_by_time = transfer.contactTime;
        next.manouver.planet = i;
        next.manouver.incomming_orbit = transfer.transfer;

        next.manouver.total_deltav = current->manouver.total_deltav + fly_by_cost(incomming_speed, needed_speed, planet_speed);

        next.previous = n_current;

        ///If this is the final planet, we take in account the docking price
        if (i == m_finish){
            Coords targetSpeed = m_sys->at(i).get_cartesian_speed_on_time(transfer.contactTime);
            Coords finalSpeed = transfer.transfer.get_cartesian_speed_on_time(transfer.contactTime);
            next.manouver.total_deltav += length(finalSpeed - targetSpeed);
        }

        next.score = m_scorer(next.manouver.fly_by_time, next.manouver.total_deltav);

        if(this->add(next)){
            consoleLog("Success", 3);
        }
        current = &s_pool[n_current]; //adding the new orbit in the pool may have changed
        //the object place, so the pointer has to be refreshed each time.

    }

    ///If we are still on the first planet, we can wait a bit on the planet's orbit
    if (current->previous < 0){
        time = current->manouver.fly_by_time + m_docking_step;
        if (time <= m_max_docking_time){
            Candidate wait;
            wait.manouver.total_deltav = 0;
            wait.manouver.fly_by_time = time;
            wait.manouver.planet = m_start;
            wait.manouver.incomming_orbit = m_sys->at(m_start);
            wait.previous = -1;
            wait.score = m_scorer(time, 0);
            if(this->add(wait)){
                consoleLog("Waiting on orbit", 2);
            }
        }
    }

    return true;
}
