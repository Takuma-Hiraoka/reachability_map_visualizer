#ifndef REACHABILITY_MAP_VISUALIZER_H
#define REACHABILITY_MAP_VISUALIZER_H

#include <choreonoid_viewer/choreonoid_viewer.h>
#include <prioritized_inverse_kinematics_solver2/prioritized_inverse_kinematics_solver2.h>
#include <ik_constraint2/ik_constraint2.h>

namespace reachability_map_visualizer {
  class EndEffector {
  public:
    cnoid::LinkPtr parent;
    cnoid::Isometry3 localPose;
  };
  class ReachabilityMapParam {
  public:
    cnoid::BodyPtr robot;
    std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > constraints;
    std::vector<cnoid::LinkPtr> variables;
    std::vector<EndEffector> endEffectors;
    double posResolution = 0.1;
    int testPerGrid = 100;
    cnoid::Vector3 size = cnoid::Vector3(2.0,2.0,2.0);
    cnoid::Vector3 origin = cnoid::Vector3::Zero();
    prioritized_inverse_kinematics_solver2::IKParam pikParam;
    double transparency = 0.5;
    std::shared_ptr<choreonoid_viewer::Viewer> viewer = nullptr;
    ReachabilityMapParam(){
      pikParam.maxIteration = 30;
    };
  };
  // origin中心としたsizeの立方体をposResolutionで区切ったグリッドそれぞれの位置について
  // ランダム姿勢を生成し、EndEffectorがそのPoseに到達するかどうかを調べる．これを全EndEffectorで調べる.
  // ランダム姿勢をtestPerGrid回生成して解けた割合を記録し、その割合に応じてvisualizeする.
  void visualize(const std::shared_ptr<ReachabilityMapParam>& param);
}

#endif
