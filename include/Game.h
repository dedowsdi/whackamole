#ifndef WHACKAMOLE_GAME_H
#define WHACKAMOLE_GAME_H

#include <iostream>
#include <osg/BoundingBox>
#include <osg/Callback>
#include <osg/Referenced>
#include <osg/MatrixTransform>
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

    bool run(osg::Object* object, osg::Object* data) override;

    void createScene() override;

    void popMole();

    void kickMole(Mole* mole);

    void removeMole(Mole* mole);

    void highLightMole(Mole* mole);

    void updateScore(const osg::Vec3& pos, int score);

private:
    osg::Node* createLawn();

    void createBurrows();

    Burrow createBurrow(const osg::Vec3& pos);

    osg::Node* createUI();

    Game() = default;
    ~Game() = default;

    int _score;
    osg::ref_ptr<osgText::Text> _scoreText;
    std::vector<Burrow> _burrowList;
};

}  // namespace toy

#endif  // WHACKAMOLE_GAME_H
