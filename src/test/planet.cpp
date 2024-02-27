#include "planet.hpp"

Planet::Planet(): Elliptic_Orbit() {
    mass = 0;
    influence_radius = 0;
}

Planet::Planet(double a_, double e__, double omega_, double t_p_, double attractorMass, std::string name_, double mass_, double radius_, Coords attractorPos, bool spin)
    : Elliptic_Orbit(a_, e__, omega_, t_p_, attractorMass, attractorPos, spin) {
    mass = mass_;
    radius = radius_;
    name = name_;

    Coords perihelion = this->get_cartesian_perihelion();
    double aphelion = 2 * a - perihelion.x;
    influence_radius = 10 *  sqrt( mass / attractorMass) * aphelion;
}

const Coords Planet::get_gravitational_acceleration(Coords bodyPos, double time) const {
    Coords relativePos = this->get_relative_pos(bodyPos, time);
    double r = length(relativePos);
    if (r > influence_radius) {return NullCoords;}
    r = 1/r;
    Coords radial_vector = - (relativePos * r);
    return radial_vector * G * mass * r * r;
}

const bool Planet::does_crash(Coords bodyPos, double time) const {
    return length(this->get_relative_pos(bodyPos, time)) <= radius;
}

const Planet::Situation Planet::get_situation(Coords bodyPos, double time) const {
    Situation result;
    result.relative_pos = this->get_relative_pos(bodyPos, time);
    result.distance = length(result.relative_pos);
    result.gravitationnal_acceleration = -(G * mass / (result.distance * result.distance * result.distance) ) * result.relative_pos;
    result.interaction = (result.distance >= influence_radius) ? Planet::Situation::Away
                                : ((result.distance >= radius) ? Planet::Situation::inField : Planet::Situation::Crashed);
    return result;
}

std::ostream& operator<< (std::ostream &out, const Planet &source){
    out << source.name << std::endl;
    out << "-Radius: " << source.radius << std::endl;
    out << "-Gravity well radius: " << source.influence_radius << std::endl;
    out << "-Mass: " << source.mass << std::endl;
    out << "-Semi major axis: " << source.a << std::endl;
    out << "-Eccentricity: " << source.e << std::endl;
    out << "-Argument of perihelion: " << source.omega << std::endl;
    out << "-Time of perihelion: " << source.t_p << std::endl;
    out << "-Spin direction: " << source.spin << std::endl;
    out << "-Period: " << source.T << std::endl;
    return out;
}
