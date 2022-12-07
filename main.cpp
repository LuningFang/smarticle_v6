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
#include "chrono/physics/ChSystemSMC.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChLinkMotorRotationAngle.h"
#include "chrono_irrlicht/ChIrrApp.h"


#include "chrono/utils/ChUtilsInputOutput.h"
#include "chrono_thirdparty/filesystem/path.h"
#include "Skeleton.cpp"

#include <cmath>
using namespace chrono;
using namespace chrono::irrlicht;

using namespace irr;
using namespace irr::core;


void AddContainerWall(std::shared_ptr<ChBody> body,
                      std::shared_ptr<ChMaterialSurface> mat,
                      const ChVector<>& size,
                      const ChVector<>& pos,
                      bool visible = true) {
    ChVector<> hsize = 0.5 * size;

    body->GetCollisionModel()->AddBox(mat, hsize.x(), hsize.y(), hsize.z(), pos);
    // body->GetCollisionModel()->SetFamily(1);
    if (visible) {
        auto box = chrono_types::make_shared<ChBoxShape>();
        box->GetBoxGeometry().Size = hsize;
        body->AddAsset(box);
        // body->AddVisualShape(box, ChFrame<>(pos, QUNIT));
    }
}

void AddContainer(ChSystem* sys) {

// void AddContainer(ChSystemNSC& sys) {
    // The fixed body (5 walls)
    auto fixedBody = chrono_types::make_shared<ChBody>();

    fixedBody->SetMass(1.0);
    fixedBody->SetBodyFixed(true);
    fixedBody->SetPos(ChVector<>());
    fixedBody->SetCollide(true);

    // Contact material for container
    // auto fixed_mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();
    ChContactMethod contact_method = sys->GetContactMethod();

    auto fixed_mat = DefaultContactMaterial(contact_method);


    fixedBody->GetCollisionModel()->ClearModel();
    // ground
    AddContainerWall(fixedBody, fixed_mat, ChVector<>(5.0, 0.02, 5.0), ChVector<>(0, -0.025, 0), false);
    // lwall
    AddContainerWall(fixedBody, fixed_mat, ChVector<>(0.02, 5.0, 5.0), ChVector<>(-5.0, 2.0, 0.0), false);
    // rwall
    AddContainerWall(fixedBody, fixed_mat, ChVector<>(0.02, 5.0, 5.0), ChVector<>(5.0, 2.0, 0.0), false);

    AddContainerWall(fixedBody, fixed_mat, ChVector<>(5.0, 5.0, 0.02), ChVector<>(0, 2.0, -5.0), false);

    AddContainerWall(fixedBody, fixed_mat, ChVector<>(5.0, 5.0, 0.02), ChVector<>(0, 2, 5.0), false);
    fixedBody->GetCollisionModel()->BuildModel();


    auto texture = chrono_types::make_shared<ChTexture>();
    texture->SetTextureFilename(GetChronoDataFile("textures/concrete.jpg"));
    fixedBody->AddAsset(texture);

    sys->AddBody(fixedBody);

}



int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";
    SetChronoDataPath(CHRONO_DATA_DIR);

    bool useSMC;
    if (argc == 1){
        std::cout << "use NSC contact. \n";
        useSMC = false;
    }

    if (argc == 2){
        useSMC = atoi(argv[1]);
        std::cout << "useSMC = " << useSMC << std::endl;
    }

    if (argc > 2){
        std::cout << "useage: ./main <0/1 - NSC/SMC> \n";
        return 0;
    }

    ChSystem *sys;
    double step_size;

    if (useSMC){
        ChSystemSMC *my_sys = new ChSystemSMC();
        sys = my_sys;
        step_size = 1e-4;
    } else {
        ChSystemNSC *my_sys = new ChSystemNSC();
        sys = my_sys;
        step_size = 1e-3;    
    }

    double time_end = 1;


    AddContainer(sys);


    double output_frame_step = 0.01; // write image every 0.01 second
    

    // ChVector<float> gravity(0, 0, 9.81);
    ChVector<float> gravity(0, -9.81, 0);
    sys->Set_G_acc(gravity);
    // double step_size = 1e-3; // 1e-3 in world.skel which one?

    // actuator (type: servo not velocity)
    // force uppser limit (0, 3.0e-2)
    // force lower limit (0, -3.0e-2)
    // position limit enforced
    // point limit: angle: (0, 1.5708 to 0, -1.5708) 

    // Parameters;
    // location of the belly at -0.02, 0, -0.006
    // theta, oritentation of the belly
    // alpha1 and alpha2
    // body id, make sure it's different for every body
    Skeleton skeleton1(ChVector<double>(-0.02f, 0.0f, -0.006f), 0, -CH_C_PI_2, -CH_C_PI_2, 11);
    skeleton1.SetContactMethod(sys->GetContactMethod());
    skeleton1.Initialize();
    
    // Add body skeleton 1 to sys
    skeleton1.AddSkeleton(sys);


    // Test one skeleton first ... then writes API that sets phase shift
    Skeleton skeleton2(ChVector<double>(0.02, 0.0f, 0.06f), CH_C_PI, -CH_C_PI_2, -CH_C_PI_2, 12);
    skeleton2.SetContactMethod(sys->GetContactMethod());

    skeleton2.Initialize();
    skeleton2.AddSkeleton(sys);

    // Create the Irrlicht visualization system
    ChIrrApp application(sys, L"smarticle demo", core::dimension2d<u32>(800, 600));
    // Add camera, lights, logo and sky in Irrlicht scene
    application.AddTypicalLogo();
    application.AddTypicalSky();
    application.AddTypicalLights();
    application.AddTypicalCamera(core::vector3df(0, 0.2, 0));



    // Complete asset specification: convert all assets to Irrlicht
    application.AssetBindAll();
    application.AssetUpdateAll();

    int frame = 0;

    // initialize the csv writer to write the output file
    utils::CSV_writer txt(" ");
    txt.stream().setf(std::ios::scientific | std::ios::showpos);
    txt.stream().precision(8);


    // while (vis->Run()) {
    while (true){
        // visualization
        // compute dynamics
        sys->DoStepDynamics(step_size);


        if (frame % int(output_frame_step/step_size) == 0){
            char filename[100];
            sprintf(filename, "img_%04d.jpg", int(frame/100));
            irr::video::IImage* image = application.GetVideoDriver()->createScreenShot();
            application.GetVideoDriver()->writeImageToFile(image, filename);
            image->drop();

        }

        if (frame % 20 == 0){

            application.BeginScene();
            application.DrawAll();
            application.EndScene();
            

            txt << sys->GetChTime() << skeleton1.GetPos().x() << skeleton1.GetPos().y() << skeleton1.GetPos().z() << skeleton1.GetTheta() << skeleton1.GetAlpha1() << skeleton1.GetAlpha2();


            txt << skeleton2.GetPos().x() << skeleton2.GetPos().y() << skeleton2.GetPos().z() << skeleton2.GetTheta() << skeleton2.GetAlpha1() << skeleton2.GetAlpha2();

            txt << std::endl;

        }

        frame++;



        if (sys->GetChTime()>time_end){

            break;
        }

    }

    std::string txt_output;
    if (useSMC){
        txt_output = "SMC_result.txt";
    }
    else {
        txt_output = "NSC_result.txt";

    }
    txt.write_to_file(txt_output);


    return 0;
}
