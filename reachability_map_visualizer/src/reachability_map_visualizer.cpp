#include <reachability_map_visualizer/reachability_map_visualizer.h>
#include <cnoid/MeshGenerator>

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
  void visualize(const std::shared_ptr<ReachabilityMapParam>& param) {
    std::vector<double> initPose;
    link2Frame(param->variables, initPose);
    cnoid::BodyPtr reachabilityMap = new cnoid::Body();
    cnoid::MeshGenerator meshGenerator;
    cnoid::LinkPtr rootLink = new cnoid::Link();

    for (double grid_x = - param->size[0]/2; grid_x <= param->size[0] / 2; grid_x += param->posResolution) {
      for (double grid_y = - param->size[1]/2; grid_y <= param->size[1] / 2; grid_y += param->posResolution) {
        for (double grid_z = - param->size[2]/2; grid_z <= param->size[2] / 2; grid_z += param->posResolution) {
          std::cerr << "x : " << grid_x << " y : " << grid_y << " z : " << grid_z << std::endl;
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
              constraint->weight() << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;
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
            float red = std::max(1.0 - 2*solvability, 0.0);
            float green = std::max(1.0 - std::abs(1 - 2*solvability), 0.0);
            float blue = std::max(2*solvability - 1.0, 0.0);

            cnoid::SgShapePtr shape = new cnoid::SgShape();
            shape->setMesh(meshGenerator.generateBox(cnoid::Vector3(param->posResolution,param->posResolution,param->posResolution)));
            cnoid::SgMaterialPtr material = new cnoid::SgMaterial();
            material->setTransparency(param->transparency);
            material->setDiffuseColor(cnoid::Vector3f(red, green, blue));
            shape->setMaterial(material);
            cnoid::SgPosTransformPtr posTransform = new cnoid::SgPosTransform();
            posTransform->translation() = targetPose.translation();
            posTransform->addChild(shape);
            rootLink->addShapeNode(posTransform);
          }
        }
      }
    }
    reachabilityMap->setRootLink(rootLink);
    param->viewer->objects(reachabilityMap);
  }
}
