#ifndef ORBIT_HPP_INCLUDED
#define ORBIT_HPP_INCLUDED

#include <iostream>
#include <vector>
#include <exception>

#include "../utils/macros.hpp"
#include "../utils/vector.hpp"


/** \brief This struct represents an elliptic orbit in 2D with
 *      the standard parameters, aka:
 *      - a the semi major axis
 *      - e the eccentricity
 *      - omega the argument of perihelion
 *      - t_p the time of perihelion passage
 *      - spin the way the object spins: true for direct, false for hour
 */
class Elliptic_Orbit{

///Constructors, accessors
public:
    Elliptic_Orbit();
    Elliptic_Orbit(double a, double e, double omega, double t_p, double attractorMass, Coords attractorPos = NullCoords, bool spin = true);
    Elliptic_Orbit(const Elliptic_Orbit& source) = default;

    /* Retrieves the position of the object, given the angle */
    const PolarCoords get_polar_position(double angle) const;
    const Coords get_cartesian_position(double angle) const;

    /* Retrieves the position of the object, given the time */
    const PolarCoords get_polar_position_on_time(double time) const;
    const Coords get_cartesian_position_on_time(double time) const;

    /* Retrieves the lowest time superior to min_time, when the angle
       of the body's position relative to the attractor, will be angle*/
    const double get_time_on_angle(double angle, double min_time = 0) const;

    /* Retrieves the speed of the object, given the angle */
    const Coords get_cartesian_speed_on_angle(double angle) const;

    /* Retrieves the speed of the object, given the time */
    const Coords get_cartesian_speed_on_time(double time) const;

    /* Retrieves the perihelion vector*/
    const PolarCoords get_polar_perihelion() const;
    const Coords get_cartesian_perihelion() const;

    /* Retrieves the position of a body relative to the object at a point in time */
    const Coords get_relative_pos(Coords bodyPos, double time) const;

    /* performs a quick check on Kepler values to assert the object is well
       defined. Indeed, the generating methods does sometimes fail, returning
       what should be an hyperbola for example. In that case the object is corrupted */
    const bool is_valid() const;

    /* Given another orbit, computes the intersections on the orbital paths */
    const std::vector<Coords> intersection_points(const Elliptic_Orbit& other) const;

    /* Checks if two objects will collide between the time t1 and t2, aka get closer than
        the precision parameter (a distance in m) */
    const bool collides(const Elliptic_Orbit& other, double t1, double t2, double precision) const;

    /* Raw parameters accessors */
    inline const double period() const {return T;}


///Methods of generation of an orbit. These are not constructors, because they need each others and can also
///return more than a single orbit
public:

    /* Given a position and speed of a body, constructs the orbit the body is running through */
    static Elliptic_Orbit point_problem(double attractorMass, Coords pos, Coords speed, double t = 0, Coords attractorPos = NullCoords);

    /* Given three positions observed and the time of the first position, constructs the orbit the body is running through
        It is assumed that the positions are given in the increasing time of observation*/
    static Elliptic_Orbit gauss_problem(double attractorMass, Coords p1, Coords p2, Coords p3, double t = 0, Coords attractorPos = NullCoords);

    /* Represents an orbital rendez-vous, with the transfer orbit, the time and the position
       of the rendez-vous (cf declaration under)*/
    struct RDV;

    /* Given two positions and time in space, generates a transfer orbit between those two points
       Since it is an iterative resolution (algebraic is not possible), the maximum number of iteration and
       the maximum error in time must be specified. The rendez-vous format is thus used to allow retrieving the time error */
    static RDV lambert_problem(double attractorMass, Coords p1, double t1, Coords p2, double t2, double time_precision, unsigned int maxIter = 30, Coords attractorPos = NullCoords);

    /* Given a position, a velocity and a target orbit in space, generates a RDV between the studied body and
       the targeted body. It is assumed (hence the name) that the object in already in a gravitational field
       that helps him turn and accelerate, the field moving at fieldSpeed.
       The transfer orbit goes trough the given point and meets the target. However, the initial
       speed will probably be changed. Since there's an infinite number of orbits that satisfies those condition, the goal
       of this function is to find one that MINIMIZE the evaluation function, scorer
       scorer must be the form double scorer(double time, double deltav); and gives a score to
       the transfer time and the amount of deltav necessary in the initial position*/
    static RDV fly_by_problem(double attractorMass, Coords p1, Coords v1, Coords fieldSpeed, double t1, Elliptic_Orbit target,
                                            double (*scorer)(double, double), unsigned int maxIter = 30,
                                            unsigned int innerMaxIter = 30, Coords attractorPos = NullCoords);

///Friends
public:
    /* Lets you stream the values characteristics of the orbit to perform debugging */
    friend std::ostream& operator<< (std::ostream &out, const Elliptic_Orbit &source);

///Parameters
protected:

    /* Kepler's parameters */
    double a, e, omega, t_p;
    bool spin;
    Coords center;

    /*other useful parameters, precomputed to avoid redundant computations
        - period is the orbital period
        - w is the orbital pulsation (w = 2pi/T)
        - e_ is the factor that links the angle and the true anomaly: e_ = sqrt( 1+e / 1-e )
        - area_constant is r² * d_theta, and is constant in time according to laws of central force motion
    */
    double T, w, e_, area_constant;

    /* number of iterations used to calculate E */
    const static unsigned int iteration_number = 20;

///internal functions
protected:

    /* Retrieves the distance from the center of the object, given the angle */
    const double get_radius(double angle) const;

    /* Retrieves the angle of the object, given the time */
    const double get_angle_on_time(double time) const;

    /* Computes an approximation of E, given the time*/
    const double E(double time) const;

    /* converts the angle between global coordinate system and intern coordinate system */
    inline const double relative_angle(double absolute_angle) const {return spin ? absolute_angle - omega : omega - absolute_angle;}
    inline const double absolute_angle(double relative_angle) const {return (spin ? relative_angle: - relative_angle) + omega;}
};

//is not declared inside the Elliptic_Orbit class definition to avoid
//an incomplete type error
struct Elliptic_Orbit::RDV {
    Elliptic_Orbit transfer;
    Coords contactPoint;
    double contactTime;
};


/** These are functions meant to calculate a specific information about a trajectory
    without having to calculate a whole orbit*/
//the deltav price of entering a gravity well and changing speed, while being helped by gravity
double fly_by_cost(Coords initialSpeed, Coords neededSpeed, Coords planetSpeed);

double minrad(Coords relativeSpeed, Coords relativePos, double planetMass);


#endif // ORBIT_HPP_INCLUDED
