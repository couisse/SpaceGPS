#ifndef GAME_HPP_INCLUDED
#define GAME_HPP_INCLUDED

#include "trajectory_vertices.hpp"
#include "../physics/orbit.hpp"

/** \brief Displayer of the app and overlay classes
 */

 class UpdatingDrawable: public sf::Drawable{
 public:
    UpdatingDrawable(): m_running(false){}
    virtual ~UpdatingDrawable() {}
    virtual void update(bool forced_update = false) {}; //the forced_update parameter lets you bypass the running condition for update
    inline virtual void toggle_run() {m_running = !m_running;}
    virtual void reset() {}

 protected:
    bool m_running;
    virtual void draw(sf::RenderTarget& target, sf::RenderStates states) const {}
 };

 class Points: public UpdatingDrawable {
protected:
    sf::VertexArray m_vertices;
    sf::Color m_color;

public:

    Points(sf::Color color): m_color(color) {
        m_vertices.setPrimitiveType(sf::Quads);
    }

    virtual ~Points(){}

    void addPoint(Coords where){
        sf::Vector2f w = project(where);
        m_vertices.append(sf::Vertex(w + sf::Vector2f(0,-3), m_color));
        m_vertices.append(sf::Vertex(w + sf::Vector2f(3,0), m_color));
        m_vertices.append(sf::Vertex(w + sf::Vector2f(0,3), m_color));
        m_vertices.append(sf::Vertex(w + sf::Vector2f(-3,0), m_color));
    }

    void changePoint(unsigned int idx, Coords where){
        sf::Vector2f translation = project(where) - m_vertices[4 * idx].position + sf::Vector2f(0,-3);
        for (unsigned int i = 0; i < 4; ++i){
            m_vertices[4 * idx + i].position += translation;
        }
    }

protected:

    virtual void draw(sf::RenderTarget& target, sf::RenderStates states) const
    {
        target.draw(m_vertices, states);
    }

};

class Ellipse: public UpdatingDrawable {
protected:
    sf::VertexArray m_vertices;
    Points m_orbitting;
    Elliptic_Orbit m_orbit;
    double m_time;

    static const short m_points = 1000;
    static constexpr double time_step = 24*3600;
    static constexpr double s_origin = 0;

public:

    Ellipse(Elliptic_Orbit orbit, sf::Color color): m_orbitting(color), m_orbit(orbit), m_time(s_origin){
        m_vertices.setPrimitiveType(sf::LineStrip);
        double theta;

        for (short points = 0; points <= m_points; points++){
            theta = 2 * M_PI * points / m_points;
            m_vertices.append(sf::Vertex(project(orbit.get_cartesian_position(theta)), color));
        }
        Coords p = m_orbit.get_cartesian_position_on_time(m_time);
        m_orbitting.addPoint(p);

    }

    virtual void reset(){
        m_time = s_origin;
        this->update(true);
    }

    virtual ~Ellipse() {}

protected:

    virtual void draw(sf::RenderTarget& target, sf::RenderStates states) const
    {
        target.draw(m_vertices, states);
        target.draw(m_orbitting);
    }

    virtual void update(bool forced_update = false){
        if (!m_running && !forced_update){return;}
        m_orbitting.changePoint(0, m_orbit.get_cartesian_position_on_time(m_time));
        m_time += time_step;
    }

};

class Viewer: public sf::RenderWindow {

public:
    Viewer();
    ~Viewer();

    /** \brief Main loop of the rendering
    */
    void play();


    VisualTrajectory* newTrajectory(sf::Color color);
    void clearTrajectories();

    /** \brief only saves a pointer on the Drawable, does not creates a copy of the drawable
     *  keeping the object alive is thus necessary
     */
    void addOverlay(UpdatingDrawable* extra);
    void clearOverlay();

protected:

    ///Internal methods
    /** \brief calls for all rendering-related method and display the frame
    */
    void rendering();

    /** \brief processes the events pile
    */
    void manageEvents();


    ///attributes
    sf::Clock m_clock;
    std::vector<VisualTrajectory> m_trajectories;

    size_t current_vertice;
    sf::View m_view;
    float m_movingOffset;

    std::vector<UpdatingDrawable*> m_overlay;

};

#endif // GAME_HPP_INCLUDED
