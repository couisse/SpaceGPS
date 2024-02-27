#ifndef PLANET_HPP_INCLUDED
#define PLANET_HPP_INCLUDED

#include <string>

#include "orbit.hpp"

class Planet: public Elliptic_Orbit {

public:
    Planet();
    Planet(double a, double e, double omega, double t_p, double attractorMass, std::string name, double mass, double radius, Coords attractorPos = NullCoords, bool spin = true);
    Planet(const Planet& source) = default;

    /* Retrieves the acceleration generated by the planet on the body on the
       given position and time */
    const Coords get_gravitational_acceleration(Coords bodyPos, double time) const;

    /* Indicates if a body would crash on the planet at a given point in time,
       aka if it is closer than the radius of the planet */
    const bool does_crash(Coords bodyPos, double time) const;

    /* This more general purpose function is designed to retrieve all the information
       of an object relative to the planet, without having to call multiple functions that
       may perform same calculations (eg calculating the the position of the planet at a given
       point in time)
    */
    struct Situation{
        enum Type{Away, inField, Crashed};

        Type interaction;
        Coords relative_pos;
        double distance; //can be calculated from the one above, but avoids sqrt
        Coords gravitationnal_acceleration;
    };

    const Situation get_situation(Coords bodyPos, double time) const;

    /* Lets you retrieve the name of the planet */
    inline const std::string get_name() const {return name;}

    inline const double get_mass() const {return mass;}

    inline const double get_rad() const {return radius;}

    inline const double get_influence_rad() const {return influence_radius;}

    /* Lets you stream the values characteristics of the planet to perform debugging */
    friend std::ostream& operator<< (std::ostream &out, const Planet &source);

///Member variables
protected:
    /* Mass and radius of the planet */
    double mass;
    double radius;

    /* The radius in which the planet's gravitational pull is
       NOT negligible compared to the attractor's one */
    double influence_radius;

    /* The name of the planet */
    std::string name;

};

#endif // PLANET_HPP_INCLUDED