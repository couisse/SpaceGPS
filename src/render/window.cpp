#include "window.hpp"

Viewer::Viewer(){

    //configuring the view
    m_view.reset(sf::FloatRect(0, 0, WIN_MODE.width, WIN_MODE.height));
    m_view.setCenter(sf::Vector2f(0,0));
    m_movingOffset = ((float) WIN_MODE.width) / 100.f;
    this->setView(m_view);
}

Viewer::~Viewer(){
}

void Viewer::play(){
    this->create(WIN_MODE, "Balistique spatiale");
    while (this->isOpen()){
        m_clock.restart();

        this->manageEvents();
        this->rendering();

        sf::sleep(TIME_PER_FRAME-m_clock.getElapsedTime());
    }
}

void Viewer::manageEvents(){
    //window events
    sf::Event event;
    while (this->pollEvent(event)){
        if (event.type==sf::Event::Closed){
            this->close();
        }else if (event.type == sf::Event::Resized){
            m_view.setSize(sf::Vector2f(event.size.width, event.size.height));
            m_movingOffset = ((float) event.size.width) / 100.f;
        }else if (event.type == sf::Event::MouseWheelScrolled && event.mouseWheelScroll.wheel == sf::Mouse::VerticalWheel){
            float factor = pow(0.9, event.mouseWheelScroll.delta);
            m_view.zoom(factor);
            m_movingOffset *= factor;
        }else if (event.type == sf::Event::KeyPressed){
            if (event.key.code == sf::Keyboard::Space){
                for (unsigned int i = 0; i < m_overlay.size(); ++i){
                    m_overlay[i]->toggle_run();
                }
            }else if (event.key.code == sf::Keyboard::R){
                for (unsigned int i = 0; i < m_overlay.size(); ++i){
                    m_overlay[i]->reset();
                }
            }
        }
    }

    //moving the view
    sf::Vector2f offset(0,0);
    if (sf::Keyboard::isKeyPressed(sf::Keyboard::Z)){
        offset.y = -m_movingOffset;
    }
    if (sf::Keyboard::isKeyPressed(sf::Keyboard::S)){
        offset.y = m_movingOffset;
    }
    if (sf::Keyboard::isKeyPressed(sf::Keyboard::Q)){
        offset.x = -m_movingOffset;
    }
    if (sf::Keyboard::isKeyPressed(sf::Keyboard::D)){
        offset.x = m_movingOffset;
    }
    m_view.setCenter(m_view.getCenter() + offset);
    this->setView(m_view);
}


VisualTrajectory* Viewer::newTrajectory(sf::Color color){
    m_trajectories.emplace_back(color);
    return &m_trajectories[m_trajectories.size()-1];
}

void Viewer::clearTrajectories(){
    m_trajectories.clear();
}

void Viewer::rendering(){

    this->clear(sf::Color::White);

    for (size_t i = 0; i < m_overlay.size(); i++){
        m_overlay[i]->update();
        this->draw(*m_overlay[i]);
    }

    for (size_t i = 0; i < m_trajectories.size(); i++){
        this->draw(m_trajectories[i].getVertices());
    }

    this->display();
}

void Viewer::addOverlay(UpdatingDrawable* extra){
    m_overlay.push_back(extra);
}

void Viewer::clearOverlay(){
    m_overlay.clear();
}
