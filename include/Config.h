#ifndef UFO_CONFIG_H
#define UFO_CONFIG_H

#include <map>
#include <string>
#include <osg/Vec2s>
#include <osg/Vec2>
#include <osg/Vec3>
#include <osg/Vec4>
#include <osg/Matrixf>
#include <osg/Matrixd>


namespace toy
{

#define sgc ::toy::Config::instance()

class Config
{
public:
    Config(const Config&) = delete;
    Config& operator=(const Config&) = delete;

    static Config& instance()
    {
        static Config instance;
        return instance;
    }

    void reload();

    void clear();

    template<typename T>
    T get(const std::string& key);

    bool getBool(const std::string& key) { return get<bool>(key); }
    char getChar(const std::string& key) { return get<char>(key); }
    short getShort(const std::string& key) { return get<short>(key); }
    int getInt(const std::string& key) { return get<int>(key); }
    unsigned char getUnsignedChar(const std::string& key) { return get<unsigned char>(key); }
    unsigned short getUnsignedShort(const std::string& key) { return get<unsigned short>(key); }
    unsigned getUnsigned(const std::string& key) { return get<unsigned>(key); }
    float getFloat(const std::string& key) { return get<float>(key); }
    double getDouble(const std::string& key) { return get<double>(key); }
    osg::Vec2s getVec2s(const std::string& key) { return get<osg::Vec2s>(key); }
    osg::Vec2 getVec2(const std::string& key) { return get<osg::Vec2>(key); }
    osg::Vec3 getVec3(const std::string& key) { return get<osg::Vec3>(key); }
    osg::Vec4 getVec4(const std::string& key) { return get<osg::Vec4>(key); }
    osg::Matrixf getMatrixf(const std::string& key) { return get<osg::Matrixf>(key); }
    osg::Matrixd getMatrixd(const std::string& key) { return get<osg::Matrixd>(key); }
    std::string getString(const std::string& key) { return get<std::string>(key); }

    // return "" if not found, no complain
    std::string search(const std::string& key);

private:
    Config();
    ~Config() = default;

    using ConfigMap = std::map<std::string, std::string>;
    ConfigMap _dict;
};

}  // namespace galaxy

#endif  // UFO_CONFIG_H
