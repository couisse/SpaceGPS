#ifndef TRAJECTORY_VERTICES_HPP_INCLUDED
#define TRAJECTORY_VERTICES_HPP_INCLUDED

#include <vector>

#include <SFML/System.hpp>
#include <SFML/Graphics.hpp>

#include "../utils/macros.hpp"
#include "../utils/consts.hpp"
#include "../utils/vector.hpp"

/** \brief This class is intended to draw a trajectory, storing points in 2D space
 *      It has a hard cap on the number of points that can be stored. If too many points
 *      are added, this beginning of the curve will be discarded to save the end.
 */


class VisualTrajectory{
protected:
    sf::VertexArray m_vertices;
    size_t m_currentIdx;
    size_t m_previous;
    sf::Color m_color;

    bool increaseIdx();

public:
    VisualTrajectory(sf::Color color, size_t vertices_number = MAX_VERTICES);
    ~VisualTrajectory();

    inline sf::VertexArray& getVertices() {return m_vertices;}

    void addPoint(Coords position);

    /** changes the color of the points TO COME. Existing points are not modified.*/
    void changeColor(sf::Color color);

};

#endif // TRAJECTORY_VERTICES_HPP_INCLUDED
