#include "thrust_manager.hpp"

const ThrustCalculator::RegulationParameters ThrustCalculator::s_orbit_reg(0.0001, 0.1);
const ThrustCalculator::RegulationParameters ThrustCalculator::s_docking_reg(0.01, 0.1);

ThrustCalculator::RegulationParameters::RegulationParameters(double pulsation, double quality){
    omega_2 = pulsation * pulsation;
    omega_q = pulsation / quality;
}

ThrustCalculator::ThrustCalculator(SolarSystem& system, double max_acceleration){
    m_sys = &system;
    m_max_acceleration = max_acceleration;
}

ThrustCalculator::ThrustPerformances ThrustCalculator::simulate(const std::vector<GPS::Step> path, double initial_orbit_height,
                                                                  double final_height, VisualTrajectory* traj) {


    bool succeded = true;

    m_target = traj;
    m_mod = s_sampleFrequency;

    m_path = path;
    m_current_step = 0;
    m_consumed_delta_v = 0;

    m_next_to = path[0].planet;

    m_time = path[0].fly_by_time;
    Coords targetSpeed = path[1].incomming_orbit.get_cartesian_speed_on_time(m_time);

    m_steps = 0;

    //we cheat a bit on the initial position to be in the correct spot.
    Coords ur = normalize(targetSpeed);
    double r = initial_orbit_height + m_sys->at(m_next_to).get_rad();
    m_position = m_sys->at(m_next_to).get_cartesian_position_on_time(m_time) + r * ur;
    m_speed = path[0].incomming_orbit.get_cartesian_speed_on_time(m_time) + turn_direct(ur) * sqrt(G * m_sys->at(m_next_to).get_mass() / initial_orbit_height);

    m_state = FlyBy;

    if (traj){
        traj->changeColor(sf::Color::Magenta);
    }

    if (!(this->unengage() && this->travel() && this->park(final_height))){
        consoleLog("Failed to simulate the trajectory", 1);
        succeded = false;
    }

    this->clear();
    return {m_steps, m_consumed_delta_v, m_time, succeded};
}


void ThrustCalculator::clear(){
    m_path.clear();
}

bool ThrustCalculator::unengage(){


    SolarSystem::GlobalSituation situation;

    while (m_state == ThrustCalculator::FlyBy){

        situation = m_sys->get_situation(m_position, m_time);

        this->followOrbit(situation, m_path[1].incomming_orbit, s_docking_reg);
    }

    ++m_current_step;

    if (m_state == ThrustCalculator::SunOrbit){
        consoleLog("Finished to escape the gravitational well of the initial planet on t=" << m_time , 1);
        return true;
    }

    consoleLog("Crashed on time t=" << m_time, 1);
    return false;
}

bool ThrustCalculator::travel(){

    consoleLog("Starting the interplanetary ride...", 1);

    SolarSystem::GlobalSituation situation;

    double finish_time;

    while (m_current_step < m_path.size()){

        finish_time = m_path[m_current_step].fly_by_time;

        while ( m_state != Crashed
               && (m_state == ThrustCalculator::SunOrbit || m_next_to != m_path[m_current_step].planet)
               && m_time <= finish_time){

            situation = m_sys->get_situation(m_position, m_time);


            this->followOrbit(situation, m_path[m_current_step].incomming_orbit, s_orbit_reg);

        }

        consoleLog("Entering the field of step " << m_current_step, 1);

        //diving in the gravity field of the planet. If it is not the final planet,
        //performing a slingshot
        if (m_state == ThrustCalculator::Crashed || (m_current_step < m_path.size() - 1 && !this->slingshot())){
            consoleLog("Failed on t = " << m_time, 1);
            return false;
        }

        consoleLog("Successfully passed step " << m_current_step << " on t = " << m_time, 1);
        consoleLog(finish_time << " " << m_consumed_delta_v << std::endl, 1);
        ++m_current_step;

    }

    return m_state != Crashed;
}

bool ThrustCalculator::slingshot(){

    if (m_target){
        m_target->changeColor(sf::Color::Green);
    }

    const unsigned int nplanet = m_path[m_current_step].planet;

    SolarSystem::GlobalSituation situation;
    Coords targetSpeed = m_path[m_current_step + 1].incomming_orbit.get_cartesian_speed_on_time(m_time);
    Planet::Situation fly_sit;
    Coords error;



    double well_radius = m_sys->at(nplanet).get_influence_rad();

    do {
        situation = m_sys->get_situation(m_position, m_time);
        fly_sit = situation.details[nplanet];

        error = targetSpeed - m_speed;

        if (dot(fly_sit.gravitationnal_acceleration, m_speed) >= 0){
            this->m_target->changeColor(sf::Color::Green);
        }else {
            this->m_target->changeColor(sf::Color::Blue);
        }


        this->updateState(1, situation, NullCoords);


    }while (m_state != Crashed && std::abs(dot(normalize(targetSpeed), fly_sit.relative_pos)) < well_radius);

    if (m_target){
        m_target->changeColor(sf::Color::Magenta);
    }

    return m_state != Crashed;
}

bool ThrustCalculator::park(double final_height){


    SolarSystem::GlobalSituation situation;
    Planet::Situation fly_sit;


    m_current_step--;

    unsigned int planet_nb = m_path[m_current_step].planet;
    const Planet* planet = &m_sys->at(planet_nb);

    consoleLog("Trying to dock around " << planet->get_name() << "\n", 1);

    double finaldistance = final_height + planet->get_rad();

    Coords planetSpeed, planetPos;
    double dockingSpeed = sqrt(G * planet->get_mass() / finaldistance);

    Coords acc, ur, ut, targetPos, relativeSpeed;

    double mindistance, spd;

    do {
        situation = m_sys->get_situation(m_position, m_time);
        fly_sit = situation.details[planet_nb];

        planetSpeed = planet->get_cartesian_speed_on_time(m_time);
        relativeSpeed = m_speed - planetSpeed;
        ur = normalize(fly_sit.relative_pos);
        ut = turn_direct(ur);
        if (dot(relativeSpeed, ut) < 0){
            ut *= -1.d;
        }

        mindistance = minrad(relativeSpeed, fly_sit.relative_pos, planet->get_mass());

        acc = - situation.sun_acceleration;
        if (length(acc) > m_max_acceleration){
            acc = normalize(acc) * m_max_acceleration;
        }

        if (mindistance < finaldistance - 10 * s_epsilonOrbitPos){
            if (m_target){
                m_target->changeColor(sf::Color::Magenta);
            }
        }else {
            if (m_target){
                m_target->changeColor(sf::Color::Blue);
            }
            if (dot(ut, relativeSpeed) > m_max_acceleration){
                acc =  - ut * m_max_acceleration;
            }

        }

        this->updateState(5, situation, acc);


    }while (m_state!= Crashed && fly_sit.distance > finaldistance + s_epsilonOrbitPos);

    consoleLog("Desired orbit height reached", 1);

    if (m_target){
        m_target->changeColor(sf::Color::Green);
    }

    //now trying to dock properly
    double stime = m_time;
    do {
        situation = m_sys->get_situation(m_position, m_time);
        fly_sit = situation.details[planet_nb];

        planetSpeed = planet->get_cartesian_speed_on_time(m_time);
        relativeSpeed = m_speed - planetSpeed;
        planetPos = m_position - fly_sit.relative_pos;
        ur = normalize(fly_sit.relative_pos);
        ut = turn_direct(ur);

        if (dot(relativeSpeed, ut) < 0){
            ut = - ut;
        }

        spd = length(relativeSpeed);

        this->regulate(1, situation, planetSpeed + dockingSpeed * ut, planetPos + finaldistance * ur, s_docking_reg);

        if (m_time > stime + 24 * 3600){
            consoleLog("Timeout on docking: unable to dock properly", 0);
            return false;
        }


    }while (m_state!= Crashed && (std::abs(fly_sit.distance - finaldistance) > s_epsilonOrbitPos ||
            std::abs(spd - dockingSpeed) > dockingSpeed * s_epsilonSpeedFactor));

    consoleLog("Docked on time t = " << m_time, 0);

    return m_state!= Crashed;
}


void ThrustCalculator::followOrbit(SolarSystem::GlobalSituation& situation, Elliptic_Orbit& target, RegulationParameters reg){

    double angle = vector_angle(m_position);
    Coords t_pos = target.get_cartesian_position(angle);
    Coords t_spd = target.get_cartesian_speed_on_angle(angle);
    this->regulate(s_interplanetaryTimeStep, situation, t_spd, t_pos, reg);

}


void ThrustCalculator::regulate(double stepping_time, SolarSystem::GlobalSituation& situation, Coords targetSpeed, Coords targetPos, RegulationParameters settings){

    m_steps++;

    //retrieving the current acceleration
    Coords error = targetPos - m_position;
    Coords derreor = targetSpeed - m_speed;
    Coords acc = NullCoords;

    if (targetSpeed != NullCoords) {

        if (targetPos == NullCoords){ //simple regulation of speed

            if (length(derreor) > s_epsilonSpeedFactor * length(targetSpeed)){
                acc = derreor;
            }

        }else { //regulation of position and speed

            if (length(error) > s_epsilonPos || length(derreor) > s_epsilonSpeedFactor * length(targetSpeed)){
                acc = settings.omega_2 * error  + settings.omega_q * derreor;
            }
        }

    }else if (targetPos != NullCoords){ //regulation of only position
        if (abs_vector_angle(m_speed, error) > s_epsilonAngle){
            acc = error;
        }
    }

    acc /= stepping_time;

    if (length(acc) > m_max_acceleration){
        acc = normalize(acc) * m_max_acceleration;
    }

    this->updateState(stepping_time, situation, acc);
}


void ThrustCalculator::updateState(double stepping_time, SolarSystem::GlobalSituation& situation, Coords my_acceleration){

    ///integrating position and speed
    m_speed += (situation.gravitational_acceleration + my_acceleration) * stepping_time;
    m_position += m_speed * stepping_time;
    m_consumed_delta_v += length(my_acceleration) * stepping_time;

    ///increasing the time
    m_time += stepping_time;

    ///the debugging traj
    if (m_target != nullptr && m_mod >= s_sampleFrequency){
        m_target->addPoint(m_position);
        m_mod = 0;
    }
    ++m_mod;

    ///state updating

    m_state = situation.isInSun ? ThrustCalculator::Crashed : ThrustCalculator::SunOrbit;

    for (unsigned int i = 0; i < situation.details.size(); ++i){
        switch (situation.details[i].interaction){
        case Planet::Situation::Crashed:
            m_state = ThrustCalculator::Crashed;
            return;
        case Planet::Situation::inField:
            m_state = ThrustCalculator::FlyBy;
            m_next_to = i;
            return;
        default:
            break;
        }
    }
}
