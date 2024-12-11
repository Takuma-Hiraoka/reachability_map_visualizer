#include <reachability_map_visualizer/reachability_map_visualizer.h>
#include <choreonoid_viewer/choreonoid_viewer.h>
#include <cnoid/Body>
#include <cnoid/BodyLoader>
#include <iostream>
#include <ros/package.h>

namespace reachability_map_visualizer_sample{
  void jaxon_hand(){
    std::shared_ptr<choreonoid_viewer::Viewer> viewer = std::make_shared<choreonoid_viewer::Viewer>();
    std::shared_ptr<reachability_map_visualizer::ReachabilityMapParam> param = std::make_shared<reachability_map_visualizer::ReachabilityMapParam>();

    cnoid::BodyLoader bodyLoader;
    param->robot = bodyLoader.load(ros::package::getPath("jvrc_models") + "/JAXON_JVRC/JAXON_JVRCmain.wrl");
    if(!(param->robot)) std::cerr << "!robot" << std::endl;

    param->robot->rootLink()->p() = cnoid::Vector3(0,0,0.0);
    param->robot->rootLink()->v().setZero();
    param->robot->rootLink()->R() = cnoid::Matrix3::Identity();
    param->robot->rootLink()->w().setZero();
    std::vector<double> reset_manip_pose{
      0.0, 0.0, -0.349066, 0.698132, -0.349066, 0.0,// rleg
        0.0, 0.0, -0.349066, 0.698132, -0.349066, 0.0,// lleg
        0.0, 0.0, 0.0, // torso
        0.0, 0.0, // head
        0.0, 0.959931, -0.349066, -0.261799, -1.74533, -0.436332, 0.0, -0.785398,// rarm
        0.0, 0.959931, 0.349066, 0.261799, -1.74533, 0.436332, 0.0, -0.785398,// larm
        };

    for(int j=0; j < param->robot->numJoints(); ++j){
      param->robot->joint(j)->q() = reset_manip_pose[j];
    }
    param->robot->calcForwardKinematics();
    param->robot->calcCenterOfMass();
    // joint limit
    for(int i=0;i<param->robot->numJoints();i++){
      std::shared_ptr<ik_constraint2::JointLimitConstraint> constraint = std::make_shared<ik_constraint2::JointLimitConstraint>();
      constraint->joint() = param->robot->joint(i);
      param->constraints.push_back(constraint);
    }
    param->variables.push_back(param->robot->joint("LARM_JOINT0"));
    param->variables.push_back(param->robot->joint("LARM_JOINT1"));
    param->variables.push_back(param->robot->joint("LARM_JOINT2"));
    param->variables.push_back(param->robot->joint("LARM_JOINT3"));
    param->variables.push_back(param->robot->joint("LARM_JOINT4"));
    param->variables.push_back(param->robot->joint("LARM_JOINT5"));
    param->variables.push_back(param->robot->joint("LARM_JOINT6"));
    param->variables.push_back(param->robot->joint("LARM_JOINT7"));
    reachability_map_visualizer::EndEffector ee;
    ee.parent = param->robot->link("LARM_JOINT7");
    ee.localPose.translation() = cnoid::Vector3(-0.03,0.0,-0.15);
    ee.localPose.linear() = cnoid::rotFromRpy(0.0,M_PI/2,0.0);
    param->endEffectors.push_back(ee);
    param->posResolution = 0.05;
    param->pikParam.maxIteration = 30;
    param->testPerGrid = 50;
    param->origin = cnoid::Vector3(0.0, 0.1,0.5); // IKが解けない時に無駄な計算をしてしまうので、ぎりぎりのサイズにしたほうが速い
    param->size = cnoid::Vector3(2.0,2.0,2.0);
    param->weight[5] = 0.0;
    std::shared_ptr<reachability_map_visualizer::ReachabilityMap> map = std::make_shared<reachability_map_visualizer::ReachabilityMap>();
    reachability_map_visualizer::createMap(param, map);

    reachability_map_visualizer::writeMap(ros::package::getPath("reachability_map_visualizer_sample") + "/config/jaxon_hand.yaml",map);
    viewer->objects(param->robot);
    reachability_map_visualizer::visualizeMap(map, viewer);
    viewer->drawObjects();
  }
}
