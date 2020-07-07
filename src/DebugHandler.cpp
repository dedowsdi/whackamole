#include "DebugHandler.h"

#include <bitset>

#include <osgViewer/Viewer>
#include <OsgQuery.h>

namespace toy
{

std::string to_string(osg::StateSet::RenderBinMode mode)
{
    switch (mode)
    {
        case osg::StateSet::INHERIT_RENDERBIN_DETAILS:
            return "INHERIT_RENDERBIN_DETAILS";
        case osg::StateSet::OVERRIDE_PROTECTED_RENDERBIN_DETAILS:
            return "OVERRIDE_PROTECTED_RENDERBIN_DETAILS";
        case osg::StateSet::OVERRIDE_RENDERBIN_DETAILS:
            return "OVERRIDE_RENDERBIN_DETAILS";
        case osg::StateSet::PROTECTED_RENDERBIN_DETAILS:
            return "PROTECTED_RENDERBIN_DETAILS";
        case osg::StateSet::USE_RENDERBIN_DETAILS:
            return "USE_RENDERBIN_DETAILS";
        default:
            return "";
    }
}

std::string to_string(osgUtil::RenderBin::SortMode mode)
{
    switch (mode)
    {
        case osgUtil::RenderBin::SORT_BACK_TO_FRONT:
            return "SORT_BACK_TO_FRONT";
        case osgUtil::RenderBin::SORT_BY_STATE:
            return "SORT_BY_STATE";
        case osgUtil::RenderBin::SORT_BY_STATE_THEN_FRONT_TO_BACK:
            return "SORT_BY_STATE_THEN_FRONT_TO_BACK";
        case osgUtil::RenderBin::SORT_FRONT_TO_BACK:
            return "SORT_FRONT_TO_BACK";
        case osgUtil::RenderBin::TRAVERSAL_ORDER:
            return "TRAVERSAL_ORDER";
        default:
            return "";
    }
}

RenderStagePrinter::RenderStagePrinter(std::ostream& out) : _out(out) {}

void RenderStagePrinter::drawImplementation(
    osgUtil::RenderBin* bin, osg::RenderInfo& renderInfo, osgUtil::RenderLeaf*& previous)
{
    if (_enabled)
    {
        auto stage = dynamic_cast<osgUtil::RenderStage*>(bin);
        if (stage)
        {
            _out << std::string(60, '-') << "\n";
            _out << "Frame " << renderInfo.getState()->getFrameStamp()->getFrameNumber()
                 << "\n";
            pushRenderStage(0, 0, stage);
            printRenderStage(stage);
            popRenderStage();
        }
        _enabled = false;
    }

    bin->drawImplementation(renderInfo, previous);
}

void RenderStagePrinter::printRenderStage(const osgUtil::RenderStage* stage)
{
    auto& preStages = stage->getPreRenderList();
    for (auto& item: preStages)
    {
        pushRenderStage(-1, item.first, item.second);
        printRenderStage(item.second);
        popRenderStage();
    }

    std::cout << std::string(40, '=') << std::endl;
    pushRenderBin(stage);
    printRenderBin(stage);
    popRenderBin();

    auto& postStages = stage->getPostRenderList();
    for (auto& item: postStages)
    {
        pushRenderStage(1, item.first, item.second);
        printRenderStage(item.second);
        popRenderStage();
    }
}

void RenderStagePrinter::pushRenderStage(
    int renderOrder, int order, const osgUtil::RenderStage* stage)
{
    _stages.push_back({renderOrder, order, stage});
}

void RenderStagePrinter::popRenderStage()
{
    _stages.pop_back();
}

void RenderStagePrinter::pushRenderBin(const osgUtil::RenderBin* bin)
{
    _bins.push_back(bin);
}

void RenderStagePrinter::popRenderBin()
{
    _bins.pop_back();
}

void RenderStagePrinter::printRenderBin(const osgUtil::RenderBin* bin)
{
    // print child bins with binNum <0
    auto& bins = bin->getRenderBinList();
    for (auto it = bins.begin(); it->first < 0 && it != bins.end(); ++it)
    {
        pushRenderBin(it->second);
        printRenderBin(it->second);
        popRenderBin();
    }

    // print stage path, bin path
    std::cout << std::string(20, '*') << std::endl;
    printPath();

    auto& fineGrained = bin->getRenderLeafList();
    if (!fineGrained.empty())
    {
        _out << "fine grained :\n";
        printLeaves(fineGrained);
    }

    _out << "coarse grained :\n";
    for (auto graph: bin->getStateGraphList())
    {
        _out << "StageGraph : " << graph << "\n";
        printLeaves(graph->_leaves);
    }

    // print child bins with binNum >0
    for (auto it = bins.begin(); it->first > 0 && it != bins.end(); ++it)
    {
        pushRenderBin(it->second);
        printRenderBin(it->second);
        popRenderBin();
    }
}

void RenderStagePrinter::printPath()
{
    for (auto i = 0; i < _stages.size(); ++i)
    {
        auto& stageNode = _stages[i];
        if (stageNode.renderOrder == 0)
        {
            _out << ":" << stageNode.stage;
        }
        else
        {
            _out << (stageNode.renderOrder < 0 ? "<--" : "-->");
            _out << stageNode.order << "-" << stageNode.stage;
        }

        auto camera = stageNode.stage->getCamera();
        if (camera && !camera->getName().empty())
        {
            _out << "(camera : " << camera->getName() << ")";
        }
        _out << "\n";
    }
    _out << "\n";
    std::string binIndent = "    ";

    for (auto i = 0; i < _bins.size(); ++i)
    {
        auto bin = _bins[i];
        _out << binIndent << bin->getBinNum() << "-" << bin << "("
             << to_string(bin->getSortMode()) << ")";

        if (bin->getStateSet())
        {
            _out << ", StateSet : " << bin->getStateSet();
        }
        _out << "\n";
    }
    _out << "\n";
}

namespace
{

std::ostream& printByte(std::ostream& os, unsigned char byte)
{
    if (byte == 0xff)
    {
        os << "ff";
    }
    else
    {
        os << std::bitset<8>(byte);
    }
    return os;
}

}  // namespace

void VerbosePrintVisitor::apply(osg::Node& node)
{
    output() << node.libraryName() << "::" << node.className() << " ( " << &node << " ) "
             << "\"" << node.getName() << "\" ";
    auto mask = node.getNodeMask();
    printByte(_out, mask >> 24 & 0xff) << " ";
    printByte(_out, mask >> 16 & 0xff) << " ";
    printByte(_out, mask >> 8 & 0xff) << " ";
    printByte(_out, mask >> 0 & 0xff);

    auto ss = node.getStateSet();
    if (ss)
    {
        _out << " StateSet( " << ss;
        if (ss->getRenderBinMode() != osg::StateSet::INHERIT_RENDERBIN_DETAILS)
        {
            _out << " " << to_string(ss->getRenderBinMode()) << " " << ss->getBinNumber()
                 << " \"" << ss->getBinName() << "\"";
        }

        _out << " )";
    }
    _out << "\n";
    enter();
    traverse(node);
    leave();
}

DebugHandler::DebugHandler(osg::Camera* camera)
{
    _renderStagePrinter = new RenderStagePrinter(std::cout);
    setCamera(camera);
}

bool DebugHandler::handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa)
{
    auto view = aa.asView();
    switch (ea.getEventType())
    {
        case osgGA::GUIEventAdapter::KEYDOWN:
            switch (ea.getKey())
            {
                case osgGA::GUIEventAdapter::KEY_F2:
                {
                    OSG_NOTICE << std::string(60, '-') << "\n";
                    VerbosePrintVisitor visitor(std::cout);
                    _camera->accept(visitor);
                    break;
                }

                case osgGA::GUIEventAdapter::KEY_F3:
                    _renderStagePrinter->setEnabled(true);
                    break;

                case osgGA::GUIEventAdapter::KEY_F4:
                    osgDB::writeNodeFile(*_camera, "main.osgt",
                        new osgDB::Options("WriteImageHint=UseExternal"));
                    break;

                default:
                    break;
            }
            break;
        default:
            break;
    }
    return false;  // return true will stop event
}

void DebugHandler::setCamera(osg::Camera* v)
{
    _camera = v;
    auto renderer = dynamic_cast<osgViewer::Renderer*>(_camera->getRenderer());
    if (renderer)
    {
        renderer->getSceneView(0)->getRenderStage()->setDrawCallback(_renderStagePrinter);
        renderer->getSceneView(1)->getRenderStage()->setDrawCallback(_renderStagePrinter);
    }

    // TODO slave cameras?
}

}  // namespace toy
