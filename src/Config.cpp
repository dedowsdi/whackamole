#include <Config.h>

#include <iomanip>
#include <sstream>

#include <osg/io_utils>
#include <osg/os_utils>
#include <osgDB/Registry>
#include <ToyMath.h>

namespace toy
{

void Config::reload()
{
    _dict.clear();

    auto configPath = osgDB::Registry::instance()->findDataFile(
        "whackamole.config", 0, osgDB::CASE_SENSITIVE);

    if (configPath.empty())
    {
        OSG_FATAL << "missing config file" << std::endl;
        return;
    }

    std::ifstream ifs(configPath);

    std::string line;
    while (std::getline(ifs, line))
    {
        std::string key;
        std::istringstream iss(line);
        iss >> key;

        if (key.empty() || key.front() == '#')
            continue;

        std::string value;

        if (line.size() > key.size())
        {
            value = line.substr(line.find_first_not_of(" \t", key.size()));
        }

        if (value.empty())
        {
            OSG_WARN << "missing value for key : " << key << "\n";
            continue;
        }

        _dict.insert(std::make_pair(key, value));
    }
}

void Config::clear() {}

inline std::istream& operator>>(std::istream& input, osg::Matrixf& m)
{
    for (auto row = 0; row < 4; ++row)
    {
        for (auto col = 0; col < 4; ++col)
        {
            input >> m(row, col);
            if (row != 3 || col != 3)
                input >> std::ws;
        }
    }
    return input;
}

inline std::istream& operator>>(std::istream& input, osg::Matrixd& m)
{
    for (auto row = 0; row < 4; ++row)
    {
        for (auto col = 0; col < 4; ++col)
        {
            input >> m(row, col);
            if (row != 3 || col != 3)
                input >> std::ws;
        }
    }
    return input;
}

template<typename T>
T Config::get(const std::string& key)
{
    auto iter = _dict.find(key);
    if (iter == _dict.end())
    {
        OSG_NOTICE << key << " not found." << std::endl;
        return T();
    }

    T t;
    std::istringstream iss(iter->second);
    iss >> t;

    if (!iss)
    {
        OSG_WARN << "Failed to convert " << key << "\n";
        return T();
    }

    return t;
}

osg::Vec4 Config::getColor(const std::string& key)
{
    return htmlColorToVec4(getString(key));
}

std::string Config::search(const std::string& key)
{
    auto iter = _dict.find(key);
    return iter == _dict.end() ? "" : iter->second;
}

Config::Config()
{
    reload();
}

#define INSTANTIATE_get(T) template T Config::get(const std::string& key);

INSTANTIATE_get(bool);
INSTANTIATE_get(char);
INSTANTIATE_get(short);
INSTANTIATE_get(int);
INSTANTIATE_get(unsigned char);
INSTANTIATE_get(unsigned short);
INSTANTIATE_get(unsigned);
INSTANTIATE_get(float);
INSTANTIATE_get(double);
INSTANTIATE_get(osg::Vec2s);
INSTANTIATE_get(osg::Vec2);
INSTANTIATE_get(osg::Vec3);
INSTANTIATE_get(osg::Vec4);
INSTANTIATE_get(osg::Matrixf);
INSTANTIATE_get(osg::Matrixd);
INSTANTIATE_get(std::string);

}  // namespace toy
