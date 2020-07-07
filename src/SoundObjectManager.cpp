#include <SoundObjectManager.h>

namespace toy
{

void ALObjectManager::scheduleGLObjectForDeletion(ALuint obj)
{
    _deleteALObjectHandles.push_back(obj);
}

void ALObjectManager::flushAllDeletedALObjects()
{
    for (auto obj: _deleteALObjectHandles)
    {
        deleteALObject(obj);
    }
    _deleteALObjectHandles.clear();
}

void SoundObjectManager::flushAllDeletedALObjects()
{
    for (auto& p : _managerMap)
    {
        dynamic_cast<ALObjectManager&>(*p.second).flushAllDeletedALObjects();
    }
}

}  // namespace toy

