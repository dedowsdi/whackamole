#ifndef BOUNCINGBALL_SOUNDOBJECTMANAGER_H
#define BOUNCINGBALL_SOUNDOBJECTMANAGER_H

#include <list>
#include <map>
#include <typeindex>

#include <AL/al.h>
#include <osg/ref_ptr>
#include <osg/Referenced>

namespace toy
{

class ALObjectManager : public osg::Referenced
{
public:
    void scheduleGLObjectForDeletion(ALuint obj);

    void flushAllDeletedALObjects();

    virtual void deleteALObject(ALuint obj) = 0;

    virtual ~ALObjectManager() = default;

private:
    using ALObjectHandleList = std::list<ALuint>;
    ALObjectHandleList _deleteALObjectHandles;
};

#define sgsom SoundObjectManager::instance()

class SoundObjectManager
{
public:
    SoundObjectManager(const SoundObjectManager&) = delete;
    SoundObjectManager& operator=(const SoundObjectManager&) = delete;

    static SoundObjectManager& instance()
    {
        static SoundObjectManager* instance = new SoundObjectManager;
        return *instance;
    }

    void flushAllDeletedALObjects();

    template<typename T>
    T* get()
    {
        auto& ptr = _managerMap[std::type_index(typeid(T))];
        if (!ptr)
        {
            ptr = new T();
        }
        return static_cast<T*>(ptr.get());
    }

    template<typename T>
    const T* get() const
    {
        auto itr = _managerMap.find(std::type_index(typeid(T)));
        if (itr == _managerMap.end())
            return 0;
        else
            return itr->second.get();
    }

private:
    SoundObjectManager() = default;
    ~SoundObjectManager() = default;

    typedef std::map<std::type_index, osg::ref_ptr<osg::Referenced>> ManagerMap;
    ManagerMap _managerMap;
};

}  // namespace toy

#endif  // BOUNCINGBALL_SOUNDOBJECTMANAGER_H
