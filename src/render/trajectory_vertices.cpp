#include "trajectory_vertices.hpp"

VisualTrajectory::VisualTrajectory(sf::Color color, size_t vertices_number){

    m_vertices.resize(vertices_number);
    m_vertices.setPrimitiveType(sf::LinesStrip);

    m_color = color;
    m_previous = 0;
    m_currentIdx = 1;
}

VisualTrajectory::~VisualTrajectory(){
}

bool VisualTrajectory::increaseIdx(){
    m_currentIdx++;
    if (m_currentIdx>=MAX_VERTICES){
        m_currentIdx = 0;
        return true;
    }
    return false;
}

void VisualTrajectory::addPoint(Coords position){
    sf::Vector2f projection = project(position);

    do {
        m_vertices[m_previous].color = m_color;
        m_vertices[m_previous].position = projection;
        m_vertices[m_currentIdx].position = projection;
        m_previous = m_currentIdx;
    }while (this->increaseIdx()); //doing it a second time to connect end and begin point
    m_vertices[m_currentIdx].color = sf::Color(0,0,0,0);

}

void VisualTrajectory::changeColor(sf::Color color){
    m_color = color;
}
