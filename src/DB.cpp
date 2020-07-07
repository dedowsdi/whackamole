#include <DB.h>

#include <osgDB/Registry>
#include <ALBuffer.h>

namespace osgDB
{

toy::ALBuffer* readALBufferFile(const std::string& filename, const Options* options)
{
    auto rr = osgDB::Registry::instance()->readObject(filename, options);
    if (!rr.success())
    {
        OSG_WARN << "Error reading file " << filename << ": " << rr.statusMessage()
                 << std::endl;
        return nullptr;
    }

    auto alBuf = dynamic_cast<toy::ALBuffer*>(rr.takeObject());
    if (!alBuf)
    {
        OSG_WARN << "Failed to cast Object* to ALBuffer*" << std::endl;
    }

    return alBuf;
}

toy::ALBuffer* readALBufferFile(const std::string& filename)
{
    return readALBufferFile(filename, osgDB::Registry::instance()->getOptions());
}

}  // namespace osgDB
