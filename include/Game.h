#ifndef WHACKAMOLE_GAME_H
#define WHACKAMOLE_GAME_H

#include <iostream>
#include <osg/BoundingBox>
#include <osg/Callback>
#include <osg/MatrixTransform>
#include <osg/Referenced>
#include <osgText/Text>
#include <BaseGame.h>

namespace osg
{
class MatrixTransform;
}

namespace toy
{

#define sgg ::toy::Game::instance()

struct Burrow
{
    bool active = false;
    int index = -1;
    osg::Vec3 pos = osg::Vec3();
    osg::Node* node;
};

class Mole : public osg::MatrixTransform
{
public:
    Mole(Burrow* burrow);

    Burrow* getBurrow() const { return _burrow; }
    void setBurrow(Burrow* v) { _burrow = v; }

    bool getKicked() const { return _kicked; }
    void setKicked(bool v) { _kicked = v; }

    int getScore() const { return _score; }
    void setScore(int v) { _score = v; }

    static const osg::BoundingBox& getDrawableBoundingBox();
    static osg::Node* getDrawable();

private:
    static osg::ref_ptr<osg::Node> _drawable;
    static osg::BoundingBox _boundingbox;

    bool _kicked = false;
    int _score = 100;
    Burrow* _burrow = 0;
};

class Game
    : public BaseGame
    , public osg::Callback
{
public:
    Game(const Game&) = delete;
    Game& operator=(const Game&) = delete;

    static Game& instance()
    {
        static osg::ref_ptr<Game> instance = new Game;
        return *instance;
    }

    enum game_status
    {
        gs_init,
        gs_running,
        gs_timeout
    };

    bool run(osg::Object* object, osg::Object* data) override;

    void createScene() override;

    void popMole();

    void kickMole(Mole* mole);

    void removeMole(Mole* mole);

    void highLightMole(Mole* mole);

    void updateScore(const osg::Vec3& pos, int score);

    void restart();

    void timeout();

    game_status getStatus() const { return _status; }
    void setStatus(game_status v) { _status = v; }

private:
    osg::Node* createLawn();

    void createBurrows();

    Burrow createBurrow(const osg::Vec3& pos);

    osg::Node* createUI();

    Game() = default;
    ~Game() = default;

    int _score = 0;
    float _lastTime = 0;
    float _timer = 30;
    game_status _status = gs_init;
    osg::ref_ptr<osgText::Text> _scoreText;
    osg::ref_ptr<osgText::Text> _msg;
    osg::ref_ptr<osgText::Text> _timerText;
    std::vector<Burrow> _burrowList;
};

}  // namespace toy

#endif  // WHACKAMOLE_GAME_H
