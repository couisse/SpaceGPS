#ifndef SOLARSYSTEM_HPP_INCLUDED
#define SOLARSYSTEM_HPP_INCLUDED


#include "planet.hpp"

class SolarSystem {

public:
    SolarSystem() = default;
    SolarSystem(const SolarSystem& source) = default;
    SolarSystem(std::string filename);

    /* Calculates the acceleration of a body in the solar system at a point in space and time*/
    const Coords gravity_acceleration(Coords body, double time) const;

    /* Retrieves the global situation of an object*/
    struct GlobalSituation {
        double distance;
        Coords gravitational_acceleration;
        Coords sun_acceleration;
        bool isInSun; //tells if the object ends up in the sun (which may be bad)
        std::vector<Planet::Situation> details;
    };

    const GlobalSituation get_situation(Coords bodyPos, double time) const;

    /* Retrieves the mass of the sun */
    inline const double get_sun_mass() const {return sun_mass;}

    /* Accessors to a planet */
    const Planet& operator[](unsigned int index) const;
    const Planet& at(unsigned int index) const;

    //returns the index of a planet from its name
    //returns the planet number if no such planet exists
    const unsigned short search_planet(std::string name) const;


    /* Accessor to the planet array */
    const std::vector<Planet>& get_planets() const;

    /* Number of planets */
    const size_t size() const;

    /* Lets you stream the values of the solar system to debug */
    friend std::ostream& operator<< (std::ostream &out, const SolarSystem &source);

protected:
    std::vector<Planet> planets;
    double sun_mass;
    double sun_radius;
};

#endif // SOLARSYSTEM_HPP_INCLUDED
