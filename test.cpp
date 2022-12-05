// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Luning Fang
// =============================================================================
//
// todo: motor at revolute joint
// gaits
// record contact force (Callback)
// use 13cm ones
// =============================================================================

#include "chrono/core/ChStream.h"
#include "chrono/core/ChRealtimeStep.h"
#include "chrono/utils/ChUtilsInputOutput.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChLinkMotorRotationAngle.h"



#include "chrono/utils/ChUtilsInputOutput.h"
#include "chrono_thirdparty/filesystem/path.h"
#include "Skeleton.cpp"

#include <cmath>
using namespace chrono;
using namespace chrono::irrlicht;

collision::ChCollisionSystemType collision_type = collision::ChCollisionSystemType::BULLET;

void AddContainerWall(std::shared_ptr<ChBody> body,
                      std::shared_ptr<ChMaterialSurface> mat,
                      std::shared_ptr<ChVisualMaterial> vis_mat,
                      const ChVector<>& size,
                      const ChVector<>& pos,
                      bool visible = true) {
    ChVector<> hsize = 0.5 * size;

    body->GetCollisionModel()->AddBox(mat, hsize.x(), hsize.y(), hsize.z(), pos);
    // body->GetCollisionModel()->SetFamily(1);
    if (visible) {
        auto box = chrono_types::make_shared<ChBoxShape>();
        box->GetBoxGeometry().Size = hsize;
        box->SetMaterial(0, vis_mat);
        body->AddVisualShape(box, ChFrame<>(pos, QUNIT));
    }
}

void AddContainer(ChSystemNSC& sys) {
    // The fixed body (5 walls)
    auto fixedBody = chrono_types::make_shared<ChBody>(collision_type);

    fixedBody->SetMass(1.0);
    fixedBody->SetBodyFixed(true);
    fixedBody->SetPos(ChVector<>());
    fixedBody->SetCollide(true);

    // Contact material for container
    auto fixed_mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();
    auto fixed_mat_vis = chrono_types::make_shared<ChVisualMaterial>(*ChVisualMaterial::Default());
    std::cout << "data path: " << GetChronoDataPath() << std::endl;
    fixed_mat_vis->SetKdTexture(GetChronoDataFile("textures/concrete.jpg"));

    fixedBody->GetCollisionModel()->ClearModel();
    // ground
    AddContainerWall(fixedBody, fixed_mat, fixed_mat_vis, ChVector<>(5.0, 0.02, 5.0), ChVector<>(0, -0.025, 0));
    // lwall
    AddContainerWall(fixedBody, fixed_mat, fixed_mat_vis, ChVector<>(0.02, 5.0, 5.0), ChVector<>(-5.0, 2.0, 0.0));
    // rwall
    AddContainerWall(fixedBody, fixed_mat, fixed_mat_vis, ChVector<>(0.02, 5.0, 5.0), ChVector<>(5.0, 2.0, 0.0));

    AddContainerWall(fixedBody, fixed_mat, fixed_mat_vis, ChVector<>(5.0, 5.0, 0.02), ChVector<>(0, 2.0, -5.0), false);

    AddContainerWall(fixedBody, fixed_mat, fixed_mat_vis, ChVector<>(5.0, 5.0, 0.02), ChVector<>(0, 2, 5.0));
    fixedBody->GetCollisionModel()->BuildModel();

    sys.AddBody(fixedBody);

}

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";
    ChSystemNSC sys;
    AddContainer(sys);

    

    // ChVector<float> gravity(0, 0, 9.81);
    ChVector<float> gravity(0, -9.81, 0);
    sys.Set_G_acc(gravity);
    double step_size = 1e-3; // 1e-3 in world.skel which one?

    // actuator (type: servo not velocity)
    // force uppser limit (0, 3.0e-2)
    // force lower limit (0, -3.0e-2)
    // position limit enforced
    // point limit: angle: (0, 1.5708 to 0, -1.5708) 

 
    Skeleton skeleton1(ChVector<double>(0.0f, 0.0f, 0.0f), 0, -CH_C_PI_2, -CH_C_PI_2, 11);
    skeleton1.Initialize();
    skeleton1.AddSkeleton(sys);


    Skeleton skeleton2(ChVector<double>(-0.108, 0.0f, 0), 0, -CH_C_PI_2, -CH_C_PI_2, 11);
    skeleton2.Initialize();
    skeleton2.AddSkeleton(sys);


    // Skeleton skeleton2(ChVector<double>(0.05, 0.f, -0.06), 0, 3.f * CH_C_PI/2.0f, -CH_C_PI/2.0f, 11);
    // Skeleton skeleton2(ChVector<double>(-0.0474, 0.f, -0.0069),  2.4399 - (EIGEN_PI), CH_C_PI_2, CH_C_PI_2, 10);
    // Skeleton skeleton2(ChVector<double>(-0.02, 0.f, -0.03),  0, CH_C_PI, 0, 10);

    // skeleton2.Initialize();
    // skeleton2.AddSkeleton(sys);

    // AddSkeleton(sys, ChVector<double>(0.05, 0.f, -0.06), 0, 3.f * CH_C_PI/2.0f, -CH_C_PI/2.0f, 11);

    // ChVectorDynamic<double> initPose1(6);
    // initPose1 << 0.0309, -3.6426e-4, 5.3186 - (EIGEN_PI), 1.5720, 1.57690;


    // Create the Irrlicht visualization system

    auto vis = chrono_types::make_shared<ChVisualSystemIrrlicht>();
    vis->AttachSystem(&sys);
    vis->SetWindowSize(800, 600);
    vis->SetWindowTitle("smarticle test");
    vis->Initialize();
    vis->AddLogo();
    vis->AddSkyBox();
    vis->AddCamera(ChVector<>(0, 0.5, 0));
    vis->AddTypicalLights();

    int frame = 0;

    while (vis->Run()) {
//    while (true){
        
        vis->BeginScene();
        vis->Render();
        vis->ShowInfoPanel(true);
        vis->EndScene();
        
        sys.DoStepDynamics(step_size);


        if (frame % 100 == 0){
            char filename[300];
            sprintf(filename, "screenshot_%04d.png", int(frame/100));
            vis->WriteImageToFile(filename);

        }

        // std::cout << sys.GetChTime() << "," << skeleton1.GetPos().x() << ", " << skeleton1.GetPos().z() << std::endl;

        if (frame % 20 == 0){
            std::cout << sys.GetChTime() << "," << skeleton1.GetPos().x() << ", " << skeleton1.GetPos().y() << ", " << skeleton1.GetPos().z() << ", " << skeleton1.GetAlpha1() << ", " << skeleton1.GetAlpha2() << ", " << skeleton2.GetPos().x() << ", " << skeleton2.GetPos().y() << ", "  << skeleton2.GetPos().z() << ", " << skeleton2.GetAlpha1() << ", " << skeleton2.GetAlpha2() << std::endl;

        }

        frame++;



        if (sys.GetChTime()>180){
            break;
        }

    }


    return 0;
}
