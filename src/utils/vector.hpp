#ifndef VECTOR_HPP_INCLUDED
#define VECTOR_HPP_INCLUDED

#include <cmath>
#include <iostream>

#include <SFML/System.hpp>

#include "consts.hpp"
#include "macros.hpp"


//those typedef lets mutate easily the dimensions of the space.
//To change the base, you also need to change below functions to match
//the new space you are considering

typedef sf::Vector2<double> Coords;
typedef Coords PolarCoords;
extern const Coords NullCoords;

class Movement{
public:
    Coords start;
    Coords stop;
    Movement(Coords first = NullCoords, Coords last = NullCoords): start(first), stop(last) {}
};

inline double dot(const Coords& a, const Coords& b){
    return a.x * b.x + a.y * b.y;
}

inline double length(const Coords& source){
    return sqrt(dot(source, source));
}

inline double distance(const Coords& left, const Coords& right){
    return length(left - right);
}

inline Coords normalize(const Coords& source){
    return source/length(source);
}

inline bool areColinear(const Coords a, const Coords b){
    return (a.x * b.y - a.y * b.x) == 0;
}

inline Coords direction(const Coords& source, const Coords target){
    return normalize(target-source);
}

inline sf::Vector2f project(Coords source){
    return sf::Vector2f(source.x * rescaling, -source.y*rescaling);
}

inline Coords orientedCoords(const double angle, const double magnitude){
    return Coords(cos(angle), sin(angle)) * magnitude;
}

inline double vector_angle(const Coords v){
    return atan2(v.y, v.x);
}

inline double vector_angle(const Coords v1, const Coords v2){
    double angle = vector_angle(v2) - vector_angle(v1);
    if (angle > M_PI) {return angle - 2.d * M_PI;}
    if (angle <= -M_PI){return angle + 2.d * M_PI;}
    return angle;
}

inline double abs_vector_angle(const Coords v1, const Coords v2){
    return std::abs(vector_angle(v1, v2));
}

inline Coords polar_to_cartesian(const PolarCoords point){
    return Coords(point.x * cos(point.y), point.x * sin(point.y));
}

inline Coords turn_direct(const Coords v){
    return Coords(-v.y, v.x);
}

//other stuff

template<class T>
inline bool inRange(const sf::Vector2<T> range, T value){
    return value >= range.x && value <= range.y;
}

template<class T>
inline sf::Vector3<T> crossprod(const sf::Vector3<T> a, const sf::Vector3<T> b){
    return sf::Vector3<T>( a.y * b.z - a.z * b.y,
                           a.z * b.x - a.x * b.z,
                           a.x * b.y - a.y * b.x
                          );
}

template<class T>
//expressing a 2D vector on the main plain in 3D
inline sf::Vector3<T> in3DSpace(const sf::Vector2<T> v){
    return sf::Vector3<T>(v.x, v.y, 0);
}

template<class T>
inline sf::Vector2<T> flatten(const sf::Vector3<T> v){
    return sf::Vector2<T>(v.x, v.y);
}

template<class T>
//Performing a crossprod on vector in 2d. The z coordinate is
//set to zero for the two vector. it is a convenience shortcut,
//even thought it has no real mathematical meaning
inline sf::Vector3<T> crossprod(const sf::Vector2<T> a, const sf::Vector2<T> b){
    return sf::Vector3<T>(0, 0, a.x * b.y - a.y * b.x);
}

template<class T>
double dot(const sf::Vector3<T> a, const sf::Vector3<T> b){
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

template<class T>
inline double length(const sf::Vector3<T> a){
    return sqrt(dot(a, a));
}

inline std::ostream& operator<< (std::ostream &out, const Coords &source) {
    out << "( " << source.x << ", " << source.y << ")";
    return out;
}


#endif // VECTOR_HPP_INCLUDED
