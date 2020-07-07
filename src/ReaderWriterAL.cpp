#include <algorithm>
#include <fstream>
#include <memory>

#include <osgDB/FileUtils>
#include <osgDB/ReaderWriter>
#include <osgDB/Registry>
#include <ALBuffer.h>

class ReaderWriterAL : public osgDB::ReaderWriter
{
public:
    ReaderWriterAL() { supportsExtension("wav", "WAV audio format"); }

    const char* className() const override { return "WAV audio Reader/Writer"; }

    ReadResult readObject(const std::string& file,
        const osgDB::ReaderWriter::Options* options = NULL) const override
    {
        auto filepath = osgDB::findDataFile(file);
        if (filepath.empty())
            return ReadResult(ReadResult::FILE_NOT_FOUND);

        std::ifstream ifs(filepath, std::ios::in | std::ios::binary);
        if (!ifs)
            return ReadResult::FILE_NOT_HANDLED;

        ifs.seekg(0, std::ios::end);
        auto size = ifs.tellg();
        ifs.seekg(0, std::ios::beg);

        std::vector<unsigned char> data;
        data.reserve(size);

        std::copy(std::istreambuf_iterator<char>(ifs), std::istreambuf_iterator<char>(),
            std::back_inserter(data));

        auto buf = new toy::ALBuffer();
        buf->setData(move(data));

        return ReadResult(buf);
    }
};

REGISTER_OSGPLUGIN(al, ReaderWriterAL)
