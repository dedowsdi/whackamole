#ifndef BOUNCINGBALL_DB_H
#define BOUNCINGBALL_DB_H

#include <osgDB/Registry>

namespace toy
{
class ALBuffer;
}

namespace osgDB
{

toy::ALBuffer* readALBufferFile(const std::string& filename, const Options* options);

toy::ALBuffer* readALBufferFile(const std::string& filename);

}  // namespace osgDB

#endif  // BOUNCINGBALL_DB_H
