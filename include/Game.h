#ifndef WHACKAMOLE_GAME_H
#define WHACKAMOLE_GAME_H

#include <iostream>
#include <osg/BoundingBox>
#include <osg/Callback>
#include <osg/MatrixTransform>
#include <osg/Referenced>
#include <BaseGame.h>

namespace osg
{
class MatrixTransform;
class HeightField;
}

namespace osgTerrain
{
class Terrain;
}

namespace osgText
{
class Text;
}

namespace toy
{

struct Burrow
{
    bool active = false;
    osg::Vec3 normal = osg::Vec3(0, 0, 1);
    osg::MatrixTransform* node;

    osg::Vec3 getTopCenter();
};

class Mole : public osg::MatrixTransform
{
public:
    Mole(Burrow* burrow);

    Burrow* getBurrow() const { return _burrow; }
    void setBurrow(Burrow* v) { _burrow = v; }

    bool getKicked() const { return _kicked; }
    void setKicked(bool v); 

    int getScore() const { return _score; }
    void setScore(int v) { _score = v; }

    static osg::Node* getModel();

    static osg::Node* getBurnedModel();

private:
    static osg::ref_ptr<osg::Node> _model;
    static osg::ref_ptr<osg::Node> _burnedModel;
    static osg::BoundingBox _boundingbox;

    bool _kicked = false;
    int _score = 100;
    Burrow* _burrow = 0;
    osg::ref_ptr<osg::Switch> _switch;
};

enum node_bit
{
    nb_visible = 1 << 0,
    nb_raytest = 1 << 1
};

#define sgg ::toy::Game::instance()

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

    void whackMole(Mole* mole);

    void removeMole(Mole* mole);

    void highLightMole(Mole* mole);

    void updateScore(const osg::Vec3& pos, int score);

    void restart();

    void timeout();

    game_status getStatus() const { return _status; }
    void setStatus(game_status v) { _status = v; }

    double getDeltaTime() const { return _deltaTime; }

    float getTotalTime() const { return _totalTime; }
    void setTotalTime(float v) { _totalTime = v; }

    float getPercentTime() const { return _timer / _totalTime; }

    void hide(osg::Node* node);

    void show(osg::Node* node);

private:
    Game();

    ~Game();

    osg::Node* createTerrain();

    osg::Node* createMeadow();

    osg::Node* createOverallMeadow();

    std::pair<osg::Vec3, osg::Vec3> getTerrainPoint(float x, float y);

    void createBurrows();

    Burrow createBurrow(const osg::Vec3& pos, const osg::Vec3& normal);

    osg::Node* createUI();

    void playWhackAnimation(const osg::Vec3& pos);

    void popScore(const osg::Vec3& pos, int score);

    void createStartAnimation();

    game_status _status = gs_init;

    double _lastTime = 0;
    double _deltaTime = 0;
    float _timer = 30;
    float _totalTime = 30;
    osg::ref_ptr<osgText::Text> _timerText;
    osg::ref_ptr<osg::Node> _timerBar;

    int _score = 0;
    osg::ref_ptr<osgText::Text> _scoreText;

    osg::ref_ptr<osgText::Text> _msg;
    osg::ref_ptr<osg::HeightField> _heightField;
    osg::ref_ptr<osgTerrain::Terrain> _terrain;

    std::vector<Burrow> _burrowList;
};

}  // namespace toy

#endif  // WHACKAMOLE_GAME_H
