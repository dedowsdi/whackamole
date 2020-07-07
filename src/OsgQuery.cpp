#include <OsgQuery.h>

#include <algorithm>
#include <climits>

#include <osg/Camera>
#include <osgAnimation/Timeline>
#include <osgUtil/LineSegmentIntersector>
#include <osgViewer/Viewer>

namespace osgq
{

Segment getCameraRay(
    osg::Camera& camera, double winX, double winY, float startDepth, float endDepth)
{
    auto vpwMat = camera.getViewMatrix() * camera.getProjectionMatrix() *
                  camera.getViewport()->computeWindowMatrix();
    auto vpwMatI = osg::Matrix::inverse(vpwMat);

    auto start = osg::Vec3(winX, winY, startDepth) * vpwMatI;
    auto end = osg::Vec3(winX, winY, endDepth) * vpwMatI;

    return std::make_pair(start, end);
}

bool contains(osgAnimation::Timeline& timeline, osgAnimation::Action& action)
{
    auto actions = timeline.getActionLayer(1);
    auto iter = std::find_if(actions.begin(), actions.end(),
        [&action](osgAnimation::FrameAction& fa) { return fa.second == &action; });
    return iter != actions.end();
}

class SearchNodeVisitor : public osg::NodeVisitor
{
public:
    SearchNodeVisitor(const std::string& name, int maxDepth)
        : _maxDepth(maxDepth), _name(name)
    {
        setTraversalMode(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN);
    }

    void apply(osg::Node& node) override
    {
        if (_depth >= 0 && node.getName() == _name)
        {
            _node = &node;
        }
        else
        {
            if (_depth < _maxDepth)
            {
                ++_depth;
                traverse(node);
                --_depth;
            }
        }
    }

    osg::Node* getNode() const { return _node; }

private:
    // Use the same convention as find, depth 0 means search direct children
    int _maxDepth = 0;
    int _depth = -1;
    osg::Node* _node = 0;
    std::string _name;
};

osg::Node* searchNode(osg::Group& node, const std::string& name, int maxDepth)
{
    SearchNodeVisitor visitor(name, maxDepth < 0 ? INT_MAX : maxDepth);
    visitor.setNodeMaskOverride(-1);
    node.accept(visitor);
    return visitor.getNode();
}

namespace
{

template<typename T>
class SearchNodeTypeVisitor : public osg::NodeVisitor
{
public:
    using AsFunc = T* (osg::Node::*)();

    SearchNodeTypeVisitor(AsFunc asFunc) : _asFunc(asFunc)
    {
        setTraversalMode(osg::NodeVisitor::TRAVERSE_ALL_CHILDREN);
    }

    void apply(osg::Node& node) override
    {
        T* t = (node.*_asFunc)();
        if (t)
        {
            _nodePaths.push_back(getNodePath());
        }
        traverse(node);
    }

    const osg::NodePathList& getNodePathList() const { return _nodePaths; }
    osg::NodePathList&& takeNodePathList() { return move(_nodePaths); }

private:
    std::vector<T*> _nodes;
    osg::NodePathList _nodePaths;
    AsFunc _asFunc;
};

}  // namespace

template<typename T>
osg::NodePathList searchNodes(osg::Node& node, T* (osg::Node::*asFunc)(), int traversalMask)
{
    SearchNodeTypeVisitor<T> visitor(asFunc);
    visitor.setTraversalMask(traversalMask);
    node.accept(visitor);
    return visitor.takeNodePathList();
}

#define INSTANTIATE_searchNodes(T)                                                         \
    template osg::NodePathList searchNodes<T>(osg::Node&, T * (osg::Node::*)(), int);

INSTANTIATE_searchNodes(osg::Drawable);
INSTANTIATE_searchNodes(osg::Geometry);
INSTANTIATE_searchNodes(osg::Group);
INSTANTIATE_searchNodes(osg::Transform);
INSTANTIATE_searchNodes(osg::Switch);
INSTANTIATE_searchNodes(osg::Geode);
INSTANTIATE_searchNodes(osgTerrain::Terrain);

void* getGraphicsWindow(const osgViewer::Viewer& viewer)
{
    osgViewer::Viewer::Windows windows;
    const_cast<osgViewer::Viewer&>(viewer).getWindows(windows);
    if (windows.empty())
    {
        throw std::runtime_error("Failed to get window");
    }
    return windows[0];
}

const void* getGraphicsContextTraits(const osgViewer::Viewer& viewer)
{
    return static_cast<const osgViewer::GraphicsWindow*>(getGraphicsWindow(viewer))
        ->getTraits();
}

osg::Vec4i getWindowRect(const osgViewer::Viewer& viewer)
{
    auto traits =
        static_cast<const osg::GraphicsContext::Traits*>(getGraphicsContextTraits(viewer));
    return osg::Vec4i(traits->x, traits->y, traits->width, traits->height);
}

osg::Vec2i getScreenSize(int identifier)
{
    auto wsi = osg::GraphicsContext::getWindowingSystemInterface();
    if (!wsi)
    {
        throw std::runtime_error("failed to get window system interface");
    }

    osg::GraphicsContext::ScreenIdentifier si(identifier);
    si.readDISPLAY();
    si.setUndefinedScreenDetailsToDefaultScreen();

    unsigned width, height;
    wsi->getScreenResolution(si, width, height);

    return osg::Vec2i(width, height);
}

}  // namespace osgq
