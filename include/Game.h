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
class Texture3D;
}  // namespace osg

namespace osgTerrain
{
class Terrain;
}

namespace osgText
{
class Text;
}

namespace osgGA
{
class KeySwitchMatrixManipulator;
}

namespace toy
{

struct Burrow
{
    bool active = false; osg::Vec3 normal = osg::Vec3(0, 0, 1);
    osg::MatrixTransform* node = 0;

    osg::Vec3 getTopCenter();
};

// see
// https://developer.nvidia.com/gpugems/gpugems/part-i-natural-effects/chapter-1-effective-water-simulation-physical-models
// for detail
class Wind
{
public:
    Wind();

    void update(double dt);

    void updateUniform(osg::StateSet& ss, int index);

    float frequence() { return osg::PIf * 2 / _length; }
    float phi() { return frequence() * _speed; }

    float getStrength() const { return _strength; }
    float getSpeed() const { return _speed; }

private:
    void mutate();

    float _length = 0; // distance between two crest
    float _amplitude = 0;
    float _strength = 0; // current amplitude
    float _speed = 0; // distance per second
    float _exponent = 0;
    float _time = 0; // left duration
    float _duration = 0;

    osg::Vec2 _direction;
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

    bool getHighlighted() const { return _highlighted; }
    void setHighlighted(bool v);

    static osg::Node* getModel();

    static osg::Node* getBurnedModel();

private:
    static osg::ref_ptr<osg::Node> _model;
    static osg::ref_ptr<osg::Node> _burnedModel;
    static osg::BoundingBox _boundingbox;

    bool _kicked = false;
    bool _highlighted = false;
    int _score = 100;
    Burrow* _burrow = 0;
    osg::ref_ptr<osg::Switch> _switch;
};

enum node_bit
{
    nb_invisible = 0,
    nb_visible = 1 << 0,
    nb_above_waterline = 1 << 1,
    nb_below_waterline = 1 << 2,
    nb_raytest = 1 << 3,
    nb_ui = nb_visible,
    nb_terrain = nb_above_waterline | nb_below_waterline | nb_raytest,
    nb_real_object = nb_above_waterline | nb_raytest,
    nb_unreal_object = nb_above_waterline,
    nb_fish = nb_below_waterline
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

    void preInit() override;

    void postInit() override;

    void createScene() override;

    void popMole();

    void whackMole(Mole* mole);

    void highlightMole(Mole* mole);

    void spawnMeteor();

    void highlightCursor(bool b);

    void removeMole(Mole* mole);

    void updateScore(const osg::Vec3& pos, int score);

    void restart();

    void timeout();

    void resize(int width, int height);

    void moveCursor(float x, float y);

    void flashCursor(bool v);

    game_status getStatus() const { return _status; }
    void setStatus(game_status v) { _status = v; }

    double getDeltaTime() const { return _deltaTime; }

    float getTotalTime() const { return _totalTime; }
    void setTotalTime(float v) { _totalTime = v; }

    float getPercentTime() const { return _timer / _totalTime; }

    void hide(osg::Node* node);

    // interpolated point
    osg::Vec3 getTerrainPoint(float x, float y);

    // 4 point normal of the closest control point
    osg::Vec3 getTerrainNormal(float x, float y);

    float getSceneRadius() const { return _sceneRadius; }

private:
    Game();

    ~Game();

    void clear();

    void resetUI();

    void createTerrain();

    void createPool();

    void createTrees();

    void createMeadow();

    void createBurrows();

    void createLights();

    void setupCameraAndManipulator();

    Burrow createBurrow(const osg::Vec3& pos, const osg::Vec3& normal);

    osg::Node* createUI();

    void createStarfield();

    void playWhackAnimation(const osg::Vec3& pos);

    void popScore(const osg::Vec3& pos, int score);

    void explode(const osg::Vec3& pos);

    void createStartAnimation();

    game_status _status = gs_init;

    double _lastTime = 0;
    double _deltaTime = 0;
    float _timer = 30;
    float _totalTime = 30;

    float _sceneRadius = 128;
    float _sceneHeight = 128;
    float _popRate = 0.002;

    osg::ref_ptr<osgText::Text> _timerText;
    osg::ref_ptr<osg::Geometry> _timerBar;

    int _score = 0;
    osg::ref_ptr<osgText::Text> _scoreText;

    osg::ref_ptr<Mole> _cursorMole;

    osg::ref_ptr<osg::Node> _fish;

    osg::ref_ptr<osg::MatrixTransform> _cursorFrame;
    osg::ref_ptr<osg::Geometry> _cursorGeom;
    osg::ref_ptr<osg::Program> _cursorProgram;

    osg::ref_ptr<osg::Node> _cursor;

    osg::ref_ptr<osgText::Text> _msg;
    osg::ref_ptr<osg::HeightField> _heightField;
    osg::ref_ptr<osgTerrain::Terrain> _terrain;

    osg::ref_ptr<osg::Geometry> _pool;
    osg::ref_ptr<osg::Texture2D> _reflectMap;
    osg::ref_ptr<osg::Texture2D> _refractMap;
    osg::ref_ptr<osg::Texture2D> _depthMap;
    osg::ref_ptr<osg::Texture2D> _dudvMap;
    osg::ref_ptr<osg::Texture2D> _normalMap;
    osg::ref_ptr<osg::Camera> _reflectRttCamera;
    osg::ref_ptr<osg::Camera> _refractRttCamera;

    osg::ref_ptr<osgGA::KeySwitchMatrixManipulator> _manipulator;

    osg::ref_ptr<osg::Group> _starfield;
    osg::ref_ptr<osg::StateSet> _meteorStateSet;
    osg::ref_ptr<osg::Texture3D> _noiseTexture3D;

    std::vector<osg::Vec4> _explosions;
    std::vector<Wind> _winds;
    std::vector<Burrow> _burrowList;

    std::map<Mole*, osg::Callback*> _removeMoleCallbacks;
};

}  // namespace toy

#endif  // WHACKAMOLE_GAME_H
