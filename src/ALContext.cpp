#include <AL.h>
#include <ALContext.h>

namespace toy
{

ALContext::ALContext(int argc, char* argv[])
{
    initOpenAL(argc, argv);
}

ALContext::~ALContext()
{
    closeOpenAL();
}

}  // namespace toy
