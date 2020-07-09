#ifndef WHACKAMOLE_GAME_H
#define WHACKAMOLE_GAME_H

#include <iostream>
#include <osg/BoundingBox>
#include <osg/Callback>
#include <osg/Referenced>
#include <BaseGame.h>

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

    void kickMole(osg::Node* mole);

    void hideMole(osg::Node* mole);

private:
    osg::Node* createLawn();

    void createBurrows();

    Burrow createBurrow(const osg::Vec3& pos);

    osg::Node* createUI();

    Game() = default;
    ~Game() = default;

    osg::ref_ptr<osg::Node> _mole;
    osg::BoundingBox _moleBB;
    std::vector<Burrow> _burrowList;
};

}  // namespace toy

#endif  // WHACKAMOLE_GAME_H
