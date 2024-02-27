#include <fstream>
#include <stdexcept>
#include <sstream>
#include <algorithm>
#include "solarsystem.hpp"

SolarSystem::SolarSystem(std::string filename){
    std::fstream reader(("systems/" + filename).c_str(), std::fstream::in);
    if (!reader.is_open()){
        throw std::invalid_argument("Cannot open file");
    }
    std::string token;
    std::stringstream parser;

    //checking the data format
    std::getline(reader, token);
    if (token != "Astre,Masse (kg),Rayon (m),a (m),e,w (rad),t0 (s)"){
        throw std::invalid_argument("Bad csv data format");
    }

    //getting the sun
    std::getline(reader, token);
    if (token == ""){
        throw std::invalid_argument("Expected sun data");
    }
    parser << token;
    std::getline(parser, token, ',');
    parser >> sun_mass;
    parser.ignore();
    parser >> sun_radius;
    parser.str(std::string()); //clearing the buffer
    std::string name;

    //getting the planets
    double arg[6];
    while (std::getline(reader, token)) {

        std::stringstream parser_; //creating a parser each time to avoid error character propagation

        parser_.str(token); //loading the token

        std::getline(parser_, name, ','); //retrieving the name


        for (int j = 0; j < 6; ++j){
            parser_ >> arg[j];
            parser_.ignore();
        }

        //(double a_, double e__, double omega_, double t_p_, double attractorMass, std::string name_, double mass_, double radius_, Coords attractorPos, bool spin)
        planets.emplace_back(arg[2], arg[3], arg[4], arg[5], sun_mass, name, arg[0], arg[1]);
    }

    reader.close();
}

const Coords SolarSystem::gravity_acceleration(Coords body, double time) const {
    double r = length(body);
    Coords acc = body * G * sun_mass / (r * r * r);
    for (unsigned int i = 0; i < planets.size(); ++i){
        acc += planets[i].get_gravitational_acceleration(body, time);
    }
    return acc;
}

const Planet& SolarSystem::operator[](unsigned int index) const{
    return planets[index];
}

const Planet& SolarSystem::at(unsigned int index) const {
    return planets.at(index);
}

const std::vector<Planet>& SolarSystem::get_planets() const{
    return planets;
}

 const unsigned short SolarSystem::search_planet(std::string name) const{
    return std::find_if(planets.begin(), planets.end(),
                    [&name](const Planet& p){return p.get_name() == name;})
            - planets.begin();
 }

const SolarSystem::GlobalSituation SolarSystem::get_situation(Coords bodyPos, double time) const {
    GlobalSituation result;
    result.details.reserve(planets.size());
    result.distance = length(bodyPos);
    result.sun_acceleration = -(G * sun_mass / (result.distance * result.distance * result.distance) ) * bodyPos;
    result.gravitational_acceleration = result.sun_acceleration;
    for (unsigned int i =0; i < planets.size(); ++i){
        result.details.push_back(planets[i].get_situation(bodyPos, time));
        result.gravitational_acceleration += result.details[i].gravitationnal_acceleration;
    }
    result.isInSun = result.distance <= sun_radius;
    return result;
}

const size_t SolarSystem::size() const {
    return planets.size();
}

std::ostream& operator<< (std::ostream &out, const SolarSystem &source){
    out << "Sun\n-Mass: " << source.sun_mass << std::endl;
    out << "-Radius: " << source.sun_radius << std::endl;
    for (unsigned int i = 0; i < source.size(); ++i){
        out << "\n" << source[i];
    }
    return out;
}
