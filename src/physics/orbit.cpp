#include "orbit.hpp"

/* Constructors and accessors *****************************************************************************************************************/

Elliptic_Orbit::Elliptic_Orbit(): a(0), e(0), omega(0), t_p(0), T(0), w(0), e_(0) {}

Elliptic_Orbit::Elliptic_Orbit(double a_, double e__, double omega_, double t_p_, double attractorMass, Coords attractorPos, bool spin_){
    a = a_;
    e = e__;
    omega = omega_;
    t_p = t_p_;
    center = attractorPos;
    spin = spin_;

    e_ = sqrt((1 + e) / (1 - e));
    T = 2 * M_PI * sqrt(a * a * a / (G * attractorMass));
    w = 2 * M_PI / T;

    //according to Kepler's laws, the parameter of an ellipse is
    //p = C² / Gm. We also have p = a(1-e²)
    area_constant = sqrt(G * attractorMass  * a * ( 1 - e * e)) * (spin ? 1 : - 1);
}

const Coords Elliptic_Orbit::get_relative_pos(Coords body, double time) const {
    Coords my_pos = this->get_cartesian_position_on_time(time);
    return body - my_pos;
}


const bool Elliptic_Orbit::is_valid() const{
    return ! (std::isnan(a) || (a <= 0) || std::isnan(e) || std::isnan(omega) || std::isnan(t_p));
    //if the main parameters are valid, the other ones too
}

/** Following function are using results presented in
    https://femto-physique.fr/mecanique/forces-centrales.php#MathJax-Element-219-Frame
**/

const double Elliptic_Orbit::E(double time) const{
    ///generates an approximation of the true anomaly using single point recursion
    ///(https://en.wikipedia.org/wiki/Kepler%27s_equation#Fixed-point_iteration)
    double wt = time * w;
    double E = wt;
    for (unsigned int i = 0; i < iteration_number; ++i){
        E = wt + e * sin(E);
    }
    return E;
}

const double Elliptic_Orbit::get_angle_on_time(double time) const{
    ///tan(theta/2) = sqrt( 1+e / 1-e) * tan(E/2)
    double E = this->E(time - t_p);
    double angle = 2 * atan(e_ * tan(0.5 * E));
    return this->absolute_angle(angle);
}

const double Elliptic_Orbit::get_radius(double angle) const{
    /// r = a(1-e^2) / (1 + e cos(theta))
    return a * (1 - e * e) / (1 + e * cos(angle - omega));
}

const Coords Elliptic_Orbit::get_polar_position(double angle) const{
    return Coords(this->get_radius(angle), angle);
}

const Coords Elliptic_Orbit::get_cartesian_position(double angle) const{
    return polar_to_cartesian(this->get_polar_position(angle));
}

const Coords Elliptic_Orbit::get_polar_position_on_time(double time) const{
    return this->get_polar_position(this->get_angle_on_time(time));
}

const Coords Elliptic_Orbit::get_cartesian_position_on_time(double time) const{
    return this->get_cartesian_position(this->get_angle_on_time(time));
}

const double Elliptic_Orbit::get_time_on_angle(double angle, double min_time) const{
    ///E - e sinE = wt
    ///tan(theta/2) = sqrt( 1+e / 1-e) * tan(E/2)
    angle = this->relative_angle(angle);
    double E = 2 * atan(tan(angle * 0.5)/e_);
    double time = (E - e * sin(E)) / w + t_p;
    if (time < min_time){
        time += ceil((min_time - time) / T) * T;
    }
    return time;
}

const Coords Elliptic_Orbit::get_cartesian_speed_on_angle(double angle) const{
    /// v = dr * ur + r * dtheta * utheta
    PolarCoords place = this->get_polar_position(angle);
    Coords c_place = polar_to_cartesian(place);
    Coords u_r = c_place / place.x;
    Coords u_theta =  turn_direct(u_r);

    double d_theta = area_constant / (place.x * place.x);
    angle = this->relative_angle(angle);
    double dividing = (1 + e * cos(angle));
    double d_radius = a * (1 - e * e) * e * d_theta * sin(angle) / (dividing * dividing);

    return (spin ? 1. : -1.) * d_radius * u_r + place.x * d_theta * u_theta;
}

const Coords Elliptic_Orbit::get_cartesian_speed_on_time(double time) const{
    return this->get_cartesian_speed_on_angle(this->get_angle_on_time(time));
}

const PolarCoords Elliptic_Orbit::get_polar_perihelion() const {
    return PolarCoords(a * (1 - e), omega);
}

const Coords Elliptic_Orbit::get_cartesian_perihelion() const {
    return polar_to_cartesian(this->get_polar_perihelion());
}

const std::vector<Coords> Elliptic_Orbit::intersection_points(const Elliptic_Orbit& other) const{
    std::vector<Coords> result;

    double A = a * (1 - e * e) / (other.a * (1 - other.e * other.e));
    double gamma = e * cos(omega) - A * other.e * cos(other.omega);
    double lambda = e * sin(omega) - A * other.e * sin(other.omega);
    double a_minus_one = A - 1;

    if (A + gamma - 1 == 0){
        throw 0;
    }

    double delta = 4 * (lambda * lambda + gamma * gamma - a_minus_one * a_minus_one);

    if (delta == 0){
        double t = lambda / (a_minus_one + gamma);
        result.push_back(this->get_cartesian_position(2 * atan(t)));
    }else if (delta > 0) {
        double h_plus = 0.5 * (2 * lambda + sqrt(delta)) / (a_minus_one + gamma);
        double h_minus = 0.5 * (2 * lambda - sqrt(delta)) / (a_minus_one + gamma);
        result.push_back(this->get_cartesian_position(2 * atan(h_plus)));
        result.push_back(this->get_cartesian_position(2 * atan(h_minus)));
    }

    return std::move(result);
}

std::ostream& operator<< (std::ostream &out, const Elliptic_Orbit &source){
    out << "-Semi major axis: " << source.a << std::endl;
    out << "-Eccentricity: " << source.e << std::endl;
    out << "-Argument of perihelion: " << source.omega << std::endl;
    out << "-Time of perihelion: " << source.t_p << std::endl;
    out << "-Spin direction: " << source.spin << std::endl;
    out << "-Period: " << source.T << std::endl;
    return out;
}

const bool Elliptic_Orbit::collides(const Elliptic_Orbit& other, double t1, double t2, double precision) const {
    std::vector<Coords> collisions = this->intersection_points(other);
    double time, angle;
    Coords c2;
    if (!(this->is_valid() && other.is_valid())){return false;}
    for (Coords c : collisions) {
        angle = vector_angle(c);
        time = this->get_time_on_angle(angle, t1);
        while (time < t2){
            c2 = other.get_cartesian_position_on_time(time);
            if (distance(c, c2) <= precision){
                return true;
            }
            time += T;
        }
    }
    return false;
}


/* The multiples orbit generation problems ****************************************************************************************************/

/** \brief The calculations are based on the book
 *          CELESTIAL MECHANICS Jeremy Tatum University of Victoria
 *         https://phys.libretexts.org/Bookshelves/Astronomy__Cosmology/Celestial_Mechanics_(Tatum)
 *         p 252
 *         and the methods to solve the quadrant problem for E is inspired from
 *         en.m.wikipedia.org/wiki/Eccentric_anomaly
 *
 *         It assumes that the ship mass is negligible compared to the attractor's one
 */
Elliptic_Orbit Elliptic_Orbit::point_problem(double attractorMass, Coords pos, Coords speed, double t, Coords attractorPos){

    Elliptic_Orbit response;

    response.center = attractorPos;
    pos -= attractorPos;

    ///position and speed magnitudes
    double r = length(pos);
    double v = length(speed);

    ///Vectors of motion
    Coords ur = pos / r;
    Coords utheta = turn_direct(ur);

    ///derivatives of motion
    double dr_dt = dot(speed, ur); //v = dr * ur + r * dtheta * utheta
    double dtheta_dt = dot(speed, utheta) / r;

    ///convenience shortcut
    double GM = G * attractorMass;

    ///semi-major axis
    response.a = 1 / (2/r - v*v / GM);

    ///theta
    double theta = vector_angle(pos);

    ///psi
    double psi = vector_angle(speed);

    ///squared angular momentum
    double h_2 = r * v * sin(psi-theta);
    h_2 *= h_2;

    ///eccentricity
    response.e = sqrt( 1 - h_2/(GM*response.a));
    response.e_ = sqrt((1 + response.e) / (1 - response.e));

    ///the area constant
    response.area_constant = r * r * dtheta_dt;

    ///The spin way
    response.spin = dtheta_dt > 0;

    ///argument of perihelion
    double theta_minus_omega = acos((response.a / r * (1 - response.e * response.e) -1)/response.e); //we have here two solutions. the one calculated, and its opposite

    /*we choose using the variation of radius
    theta - omega is in [0, pi], (because of arccos), so sin(theta - omega)
    is positive.
    -> if r is decreasing, the perihelion is right in front of us.
    and thus sin(theta - omega) should be negative. The sign shall be changed
    -> if r is increasing, the perihelion is right behind us, and thus
    sin(theta - omega) should be positive, which it is

    /!\ this argument assumes that the object is spinning in the direct direction
    If not, positive and negative have to be inverted.
    */

    if ( ( (dr_dt < 0) && response.spin  ) || ( (dr_dt > 0) && !response.spin  ) ){
         theta_minus_omega *= -1;
    }

    //we can finally calculate the argument of perihelion
    response.omega = theta - theta_minus_omega;


    ///period and pulse
    response.T = 2 * M_PI * sqrt(response.a * response.a * response.a / GM);
    response.w = 2 * M_PI / response.T;

    ///time of perihelion passage

    //anomaly
    double E = acos( (response.e + cos(theta_minus_omega)) / (1 + response.e * cos(theta_minus_omega)) ); //E without the sign
    /*we know that sinE is sqrt(1 - e²)sin(theta - omega)/ (1 + e cos(theta - omega))
      so sin E should be the same sign of sin(theta - omega).
      sin E > 0 because currently E is in [0, pi]. We thus have to check sin(theta - omega)
      Again, this reasoning must be flipped in sign of the spin direction is not direct
    */
    if (( (sin(theta_minus_omega) < 0) && response.spin  ) || ( (sin(theta_minus_omega) > 0) && !response.spin  )){
        E = -E;
    }


    ///time of perihelion
    response.t_p = t - (E - response.e * sin(E)) / response.w;

    ///finally answering
    return response;
}


/** \brief Solves the Gauss problem. The calculations are based on
 *         https://fr.wikipedia.org/wiki/Orbitographie
 *
 *         It assumes that the ship mass is negligible compared to the attractor's one
 */
Elliptic_Orbit Elliptic_Orbit::gauss_problem(double attractorMass, Coords p1, Coords p2, Coords p3, double t, Coords attractorPos){
    p1-= attractorPos;
    p2-= attractorPos;
    p3-= attractorPos;
    sf::Vector3<double> P1(in3DSpace(p1)), P2(in3DSpace(p2)), P3(in3DSpace(p3));
    sf::Vector3<double> J1(crossprod(P1, P2)), J2(crossprod(P2, P3)), J3(crossprod(P3, P1));
    double r1(length(p1)), r2(length(p2)), r3(length(p3));
    sf::Vector3<double> G_ = in3DSpace( (r2 - r3) * p1 + (r3 - r1) * p2 + (r1 - r2) * p3 ); //Gauss-Gibbs vector
    sf::Vector3<double> A = J1 + J2 + J3; //area vector
    sf::Vector3<double> V = r3 * J1 + r1 * J2 + r2 * J3; //volume vector

    Coords v = flatten( sqrt(G * attractorMass / (length(V) * length(A))) * (G_ + 1 / r1 * crossprod(A, P1)) );
    return point_problem(attractorMass, p1 + attractorPos, v, t, attractorPos);
}

/** \brief Solves the Lambert problem. The calculations are base on
 *         https://archive.wikiwix.com/cache/index2.php?url=http%3A%2F%2Fartemmis.univ-mrs.fr%2Fcybermeca%2FFormcont%2Fmecaspa%2FPROJETS%2FSATELITT%2FSatel_3.htm#federation=archive.wikiwix.com&tab=url
 *         which is an archive of http://artemmis.univ-mrs.fr/cybermeca/Formcont/mecaspa/PROJETS/SATELITT/Satel_3.htm
 */

class LambertRunner{
public:
    Elliptic_Orbit best;
    double error;
    unsigned int iterations;
    LambertRunner(double attractorMass, Coords p1, double t1, Coords p2, double t2, double time_precision, unsigned int maxIter, Coords attractorPos);

private:
    struct Orbit_candidate {
        Elliptic_Orbit orb;
        double error;
    };

private:
    double m_attractorMass;
    Coords m_attractorPos;
    Coords m_p1, m_p3;
    double m_p3Angle;
    double m_t1, m_t2;
    double m_precision;
    unsigned int m_maxIter;
    unsigned int m_maxBranchingDepth;

    bool m_foundValid;


private:

    Orbit_candidate generate_candidate(double lambda){
        Orbit_candidate result;
        Coords p2 = lambda * (m_p1 + m_p3);
        result.orb = Elliptic_Orbit::gauss_problem(m_attractorMass, m_p1, p2, m_p3, m_t1, m_attractorPos);
        if (result.orb.is_valid()){
            double time = result.orb.get_time_on_angle(m_p3Angle, m_t1);
            result.error = m_t2 - time;
        }
        return result;
    }

    int pick_best(Orbit_candidate& left, Orbit_candidate& right){
        int chosen;
        Orbit_candidate* best_one;
        bool valid_left = left.orb.is_valid();
        bool valid_right = right.orb.is_valid();
        if (valid_left || valid_right){
            m_foundValid = true;
        }else {
            return -1;
        }
        if (!valid_right || (valid_left && std::abs(left.error) <= std::abs(right.error))){
            best_one = &left;
            chosen = 0;
        }else {
            best_one = &right;
            chosen = 1;
        }
        if ( !best.is_valid() || std::abs(best_one->error) < std::abs(error)){
            error = best_one->error;
            best = best_one->orb;
        }
        return chosen;
    }

    void run_iter(double lambda, double step, unsigned int depth){
        if ((m_foundValid && std::abs(error) < m_precision) || m_maxIter < iterations || (!m_foundValid && depth > m_maxBranchingDepth) ){
            return;
        }
        ++iterations;
        Orbit_candidate left = this->generate_candidate(lambda - step);
        Orbit_candidate right = this->generate_candidate(lambda + step);

        int chosen = this->pick_best(left, right);

        if (chosen == 0){
            this->run_iter(lambda - step, step * 0.5, depth + 1);
        }else if (chosen == 1){
            this->run_iter(lambda + step, step * 0.5, depth + 1);
        }else{
            if (!m_foundValid){
                this->run_iter(lambda - step, step * 0.5, depth + 1);
                this->run_iter(lambda + step, step * 0.5, depth + 1);
            }
        }
    }
};


LambertRunner::LambertRunner(double attractorMass, Coords p1, double t1, Coords p2, double t2, double time_precision, unsigned int maxIter, Coords attractorPos){
    m_attractorMass = attractorMass;
    m_attractorPos = attractorPos;
    m_p1 = p1;
    m_p3 = p2;
    m_p3Angle = vector_angle(p2);
    m_t1 = t1;
    m_t2 = t2;
    m_precision = time_precision;
    m_maxIter = maxIter;
    m_maxBranchingDepth = std::log2(maxIter);
    error = 0;
    iterations = 0;
    m_foundValid = false;
    this->run_iter(0.75, 0.125, 0);
}

Elliptic_Orbit::RDV Elliptic_Orbit::lambert_problem(double attractorMass, Coords p1, double t1, Coords p2, double t2, double time_precision, unsigned int maxIter, Coords attractorPos){
    if (t2 < t1){
        return lambert_problem(attractorMass, p2, t2, p1, t1, time_precision, maxIter, attractorPos);
    }
    LambertRunner runner(attractorMass, p1, t1, p2, t2, time_precision, maxIter, attractorPos);
    //std::cout << "It took " << runner.iterations << " iterations to find the tranfert orbit with a time error of " << runner.error << std::endl;
    RDV result;
    result.transfer = runner.best;
    result.contactTime = t2 + runner.error;
    result.contactPoint = p2;
    return result;
}


/** \brief Solves the transfert problem
 */

 class TransfertRunner{
public:
    Elliptic_Orbit::RDV best;
    double best_score;
    unsigned int iterations;
    TransfertRunner(double attractorMass, Coords p1, Coords v1, Coords fieldSpeed, double t1, Elliptic_Orbit& target, double (*scorer)(double, double), unsigned int maxIter, unsigned int innerMaxIter, Coords attractorPos);

private:
    struct Transfert_candidate {
        Elliptic_Orbit::RDV orb;
        double score;
        bool valid;
    };

private:
    double m_precision = 60;
    double m_attractorMass;
    Coords m_attractorPos;
    Coords m_p1, m_v1;
    Coords m_fieldSpeed;
    double m_angleP1;
    double m_t1;
    Elliptic_Orbit* m_target;
    double (*m_scorer)(double, double);

    unsigned int m_maxIter;
    unsigned int m_innerMaxIter;
    unsigned int m_maxBranchingDepth;

    bool m_foundValid;


private:

    Transfert_candidate generate_candidate(double time){
        double t2 = m_t1 + time;
        Transfert_candidate candidate;
        Coords rdv = m_target->get_cartesian_position_on_time(t2);
        candidate.orb = Elliptic_Orbit::lambert_problem(m_attractorMass,
                                            m_p1, m_t1, rdv, t2, m_precision, m_innerMaxIter, m_attractorPos);
        //if the transfer is valid and good enough
        if (candidate.orb.transfer.is_valid() && candidate.orb.contactTime > m_t1  && std::abs(candidate.orb.contactTime - t2) < m_precision){
            Coords newSpd = candidate.orb.transfer.get_cartesian_speed_on_angle(m_angleP1);
            double deltav = fly_by_cost(m_v1, newSpd, m_fieldSpeed);
            candidate.score = m_scorer(time, deltav);
            candidate.valid = true;
        } else {
            candidate.valid = false;
        }
        return candidate;
    }

    int pick_best(Transfert_candidate& left, Transfert_candidate& right){
        int chosen;
        Transfert_candidate* best_one;
        if (left.valid || right.valid){
            m_foundValid = true;
        }else {
            return -1;
        }
        if (!right.valid || (left.valid && left.score <= right.score)){
            best_one = &left;
            chosen = 0;
        }else {
            best_one = &right;
            chosen = 1;
        }
        if ( !best.transfer.is_valid() || best_one->score < best_score){
            best_score = best_one->score;
            best = best_one->orb;
        }
        return chosen;
    }

    void run_iter(double time, double timeStep, unsigned int depth, bool found_previously = false){
        if (time < 0 || timeStep < m_precision || m_maxIter < iterations || (!m_foundValid && depth > m_maxBranchingDepth) ){
            return;
        }
        ++iterations;
        Transfert_candidate left = this->generate_candidate(time - timeStep);
        Transfert_candidate right = this->generate_candidate(time + timeStep);

        int chosen = this->pick_best(left, right);

        if (chosen == 0){
            this->run_iter(time - timeStep, timeStep * 0.5, depth + 1, true);
        }else if (chosen == 1){
            this->run_iter(time + timeStep, timeStep * 0.5, depth + 1, true);
        }else{
            if (!m_foundValid){
                this->run_iter(time + timeStep, timeStep, depth + 1);
                this->run_iter(time - timeStep, timeStep * 0.5, depth + 1);
            }else if (found_previously) {
                this->run_iter(time, timeStep * 0.8, depth + 1, found_previously);
            }
        }
    }
};


TransfertRunner::TransfertRunner(double attractorMass, Coords p1, Coords v1, Coords fieldSpeed, double t1, Elliptic_Orbit& target, double (*scorer)(double, double), unsigned int maxIter, unsigned int innerMaxIter, Coords attractorPos){
    m_attractorMass = attractorMass;
    m_p1 = p1;
    m_angleP1 = vector_angle(m_p1);
    m_v1 = v1;
    m_fieldSpeed = fieldSpeed;
    m_t1 = t1;
    m_target = &target;
    m_scorer = scorer;
    m_maxIter = maxIter;
    m_innerMaxIter = innerMaxIter;
    m_maxBranchingDepth = std::log2(maxIter);
    best_score = 0;
    iterations = 0;
    m_foundValid = false;

    Coords currentPlace = m_target->get_cartesian_position_on_time(m_t1);

    double guess = length(currentPlace - m_p1) / length(m_v1);

    this->run_iter( 2 * guess, guess, 0);
}


Elliptic_Orbit::RDV Elliptic_Orbit::fly_by_problem(double attractorMass, Coords p1, Coords v1, Coords fieldSpeed, double t1, Elliptic_Orbit target, double (*scorer)(double, double),
                                                      unsigned int maxIter, unsigned int innerMaxIter,  Coords attractorPos){
    TransfertRunner runner(attractorMass, p1, v1, fieldSpeed, t1, target, scorer, maxIter, innerMaxIter, attractorPos);
    return runner.best;
}

double fly_by_cost(Coords initialSpeed, Coords neededSpeed, Coords planetSpeed){
    double enteringSpeed = length(initialSpeed - planetSpeed);
    double outSpeed = length(neededSpeed - planetSpeed);
    return std::abs(outSpeed - enteringSpeed);
}

double minrad(Coords relativeSpeed, Coords relativePos, double planetMass){

    Coords ur = normalize(relativePos);
    Coords utheta = turn_direct(ur);
    double r = dot(relativePos, ur);
    double rdtheta = dot(relativeSpeed, utheta);

    double gm = G * planetMass;

    double ec = 0.5 * dot(relativeSpeed, relativeSpeed);
    double ep = - gm / r;

    double em = ep + ec;

    double c = r * rdtheta;

    double delta = gm * gm + 2 * em * c * c;

    if (delta < 0){
        return 0;
    }

    return 0.5 * (sqrt(delta) - gm) / em;
}
