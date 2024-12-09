#include <cnoid/Plugin>
#include <cnoid/ItemManager>

#include <choreonoid_viewer/choreonoid_viewer.h>

namespace reachability_map_visualizer_sample{
  void jaxon();
  class jaxonItem : public choreonoid_viewer::ViewerBaseItem {
  public:
    static void initializeClass(cnoid::ExtensionManager* ext){ ext->itemManager().registerClass<jaxonItem>("jaxonItem"); }
  protected:
    virtual void main() override{ jaxon(); return;}
  };
  typedef cnoid::ref_ptr<jaxonItem> jaxonItemPtr;

  void visualize();
  class visualizeItem : public choreonoid_viewer::ViewerBaseItem {
  public:
    static void initializeClass(cnoid::ExtensionManager* ext){ ext->itemManager().registerClass<visualizeItem>("visualizeItem"); }
  protected:
    virtual void main() override{ visualize(); return;}
  };
  typedef cnoid::ref_ptr<visualizeItem> visualizeItemPtr;

  class ReachabilityMapVisualizerSamplePlugin : public cnoid::Plugin
  {
  public:
    ReachabilityMapVisualizerSamplePlugin() : Plugin("ReachabilityMapVisualizerSamplePlugin")
    {
      require("Body");
    }
    virtual bool initialize() override
    {
      jaxonItem::initializeClass(this);
      visualizeItem::initializeClass(this);
      return true;
    }
  };
}

CNOID_IMPLEMENT_PLUGIN_ENTRY(reachability_map_visualizer_sample::ReachabilityMapVisualizerSamplePlugin)
