#include <reachability_map_visualizer/reachability_map_visualizer.h>
#include <cnoid/MeshGenerator>
#include <cnoid/YAMLWriter>
#include <cnoid/YAMLReader>

namespace reachability_map_visualizer {
  void frame2Link(const std::vector<double>& frame, const std::vector<cnoid::LinkPtr>& links){
    unsigned int i=0;
    for(int l=0;l<links.size();l++){
      if(links[l]->isRevoluteJoint() || links[l]->isPrismaticJoint()) {
        links[l]->q() = frame[i];
        i+=1;
      }else if(links[l]->isFreeJoint()) {
        links[l]->p()[0] = frame[i+0];
        links[l]->p()[1] = frame[i+1];
        links[l]->p()[2] = frame[i+2];
        links[l]->R() = cnoid::Quaternion(frame[i+6],
                                          frame[i+3],
                                          frame[i+4],
                                          frame[i+5]).toRotationMatrix();
        i+=7;
      }
    }
  }
  void link2Frame(const std::vector<cnoid::LinkPtr>& links, std::vector<double>& frame){
    frame.clear();
    for(int l=0;l<links.size();l++){
      if(links[l]->isRevoluteJoint() || links[l]->isPrismaticJoint()) {
        frame.push_back(links[l]->q());
      }else if(links[l]->isFreeJoint()) {
        frame.push_back(links[l]->p()[0]);
        frame.push_back(links[l]->p()[1]);
        frame.push_back(links[l]->p()[2]);
        cnoid::Quaternion q(links[l]->R());
        frame.push_back(q.x());
        frame.push_back(q.y());
        frame.push_back(q.z());
        frame.push_back(q.w());
      }
    }
  }
  void createMap(const std::shared_ptr<ReachabilityMapParam>& param, const std::shared_ptr<ReachabilityMap>& map) {
    map->reachabilityMap.clear();
    map->origin = param->origin;
    map->posResolution = param->posResolution;
    std::vector<double> initPose;
    link2Frame(param->variables, initPose);

    for (double grid_x = - param->size[0]/2; grid_x <= param->size[0] / 2; grid_x += param->posResolution) {
      for (double grid_y = - param->size[1]/2; grid_y <= param->size[1] / 2; grid_y += param->posResolution) {
        for (double grid_z = - param->size[2]/2; grid_z <= param->size[2] / 2; grid_z += param->posResolution) {
          double solveCount = 0.0;
          cnoid::Isometry3 targetPose;
          targetPose.translation() = param->origin + cnoid::Vector3(grid_x, grid_y, grid_z);
          for (int i=0; i<param->testPerGrid; i++) {
            bool solved = false;
            targetPose.linear() = Eigen::Quaterniond::UnitRandom().matrix();
            for (int e=0; e< param->endEffectors.size() && !solved; e++) {
              std::shared_ptr<ik_constraint2::PositionConstraint> constraint = std::make_shared<ik_constraint2::PositionConstraint>();
              constraint->A_link() = param->endEffectors[e].parent;
              constraint->A_localpos() = param->endEffectors[e].localPose;
              constraint->B_link() = nullptr;
              constraint->B_localpos() = targetPose;
              constraint->eval_link() = nullptr;
              constraint->eval_localR() = targetPose.linear();
              for (int i=0;i<3;i++) {
                constraint->weight()[i] = param->weight[i] * constraint->precision() / param->posResolution;
                constraint->maxError()[i] = constraint->maxError()[i] * param->posResolution / constraint->precision();
              }
              for (int i=0;i<3;i++) {
                constraint->weight()[i+3] = param->weight[i+3];
              }
              std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > > constraints{param->constraints, std::vector<std::shared_ptr<ik_constraint2::IKConstraint>>{constraint} };
              std::vector<std::shared_ptr<prioritized_qp_base::Task> > prevTasks;
              solved = prioritized_inverse_kinematics_solver2::solveIKLoop(param->variables,
                                                                           constraints,
                                                                           prevTasks,
                                                                           param->pikParam
                                                                           );
              frame2Link(initPose,param->variables);
              param->robot->calcForwardKinematics();
              param->robot->calcCenterOfMass();
            }
            if (solved) solveCount += 1;
          }
          double solvability = solveCount / param->testPerGrid;
          std::cerr << "x : " << grid_x << " y : " << grid_y << " z : " << grid_z << " solvability : " << solvability << std::endl;
          if (solvability>0.0){
            map->reachabilityMap.push_back(std::pair<cnoid::Vector3, double>(cnoid::Vector3(grid_x, grid_y, grid_z), solvability));
          }
        }
      }
    }
  }
  void visualizeMap(const std::shared_ptr<ReachabilityMap>& map, const std::shared_ptr<choreonoid_viewer::Viewer>& viewer) {
    cnoid::BodyPtr reachabilityMap = new cnoid::Body();
    cnoid::MeshGenerator meshGenerator;
    cnoid::LinkPtr rootLink = new cnoid::Link();
    for (int i=0;i<map->reachabilityMap.size();i++) {
      double solvability = map->reachabilityMap[i].second;
      float red = std::max(1.0 - 2*solvability, 0.0);
      float green = std::max(1.0 - std::abs(1 - 2*solvability), 0.0);
      float blue = std::max(2*solvability - 1.0, 0.0);

      cnoid::SgShapePtr shape = new cnoid::SgShape();
      if (map->boxSize == cnoid::Vector3::Zero()) shape->setMesh(meshGenerator.generateBox(cnoid::Vector3(map->posResolution,map->posResolution,map->posResolution)));
      else shape->setMesh(meshGenerator.generateBox(map->boxSize));
      cnoid::SgMaterialPtr material = new cnoid::SgMaterial();
      material->setTransparency(map->transparency);
      material->setDiffuseColor(cnoid::Vector3f(red, green, blue));
      shape->setMaterial(material);
      cnoid::SgPosTransformPtr posTransform = new cnoid::SgPosTransform();
      posTransform->translation() = map->origin + map->reachabilityMap[i].first;;
      posTransform->addChild(shape);
      rootLink->addShapeNode(posTransform);
    }
    reachabilityMap->setRootLink(rootLink);
    viewer->objects(reachabilityMap);
  }
  void writeMap(std::string outputFilePath, const std::shared_ptr<ReachabilityMap>& map) {
    cnoid::YAMLWriter writer(outputFilePath);
    writer.startMapping();
    writer.putKey("reachabilityMapParam");
    writer.startListing();
    {
      writer.startMapping();
      writer.putKey("origin");
      writer.startFlowStyleListing();
      for (int i=0; i<3; i++) writer.putScalar(map->origin[i]);
      writer.endListing();
      writer.putKeyValue("posResolution",map->posResolution);
      writer.endMapping();
    }
    writer.endListing();
    writer.endMapping();
    writer.startMapping();
    writer.putKey("reachabilityMap");
    writer.startListing();
    for (int i=0; i<map->reachabilityMap.size(); i++) {
      writer.startMapping();
      writer.startListing();
      {
        writer.putKey("translation");
        writer.startFlowStyleListing();
        for (int j=0; j<3; j++) writer.putScalar(map->reachabilityMap[i].first[j]);
        writer.endListing();
        writer.putKeyValue("solvability",map->reachabilityMap[i].second);
      }
      writer.endListing();
      writer.endMapping();
    }
    writer.endListing();
    writer.endMapping();

  }
  void readMap(std::string inputFilePath, const std::shared_ptr<ReachabilityMap>& map) {
    map->reachabilityMap.clear();
    cnoid::YAMLReader reader;
    cnoid::MappingPtr node;
    try {
      node = reader.loadDocument(inputFilePath)->toMapping();
    } catch(const cnoid::ValueNode::Exception& ex) {
      std::cerr << "cannot load config file" << std::endl;;
      return;
    }
    // load
    cnoid::Listing* reachabilityMapParam = node->findListing("reachabilityMapParam");
    {
      cnoid::Mapping* info = reachabilityMapParam->at(0)->toMapping();
      cnoid::ValueNodePtr translation_ = info->extract("origin");
      if(translation_){
        cnoid::ListingPtr translationTmp = translation_->toListing();
        if(translationTmp->size()==3){
          map->origin = cnoid::Vector3(translationTmp->at(0)->toDouble(), translationTmp->at(1)->toDouble(), translationTmp->at(2)->toDouble());
        }
      }
      cnoid::ValueNodePtr posResolution_ = info->extract("posResolution");
      if (posResolution_ && posResolution_->isScalar()){
        map->posResolution = posResolution_->toDouble();
      }
    }

    cnoid::Listing* reachabilityMap = node->findListing("reachabilityMap");
    if (!reachabilityMap->isValid()) {
      std::cerr << "cannot load config file value" << std::endl;;
      return;
    } else {
      for (int i=0; i<reachabilityMap->size();i++) {
        cnoid::Mapping* info = reachabilityMap->at(i)->toMapping();
        std::pair<cnoid::Vector3,double> param;
        cnoid::ValueNodePtr translation_ = info->extract("translation");
        if(translation_){
          cnoid::ListingPtr translationTmp = translation_->toListing();
          if(translationTmp->size()==3){
            param.first = cnoid::Vector3(translationTmp->at(0)->toDouble(), translationTmp->at(1)->toDouble(), translationTmp->at(2)->toDouble());
          }
        }
        cnoid::ValueNodePtr solvability_ = info->extract("solvability");
        if (solvability_ && solvability_->isScalar()){
          param.second = solvability_->toDouble();
        }
        map->reachabilityMap.push_back(param);
      }
    }
  }
}
