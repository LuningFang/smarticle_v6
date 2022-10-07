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

#include <cmath>
using namespace chrono;
using namespace chrono::irrlicht;

collision::ChCollisionSystemType collision_type = collision::ChCollisionSystemType::BULLET;

double m_small = 0.003186;
double m_large = 0.0248;

class ChFunction_MyTest : public ChFunction {
    public:
    ChFunction_MyTest() : m_amp(1), m_phase(0), m_omg(2 * CH_C_PI), m_offset(0) {}

    ChFunction_MyTest(double amp, double omg, double phase, double offset):
    m_amp(amp), m_omg(omg), m_phase(phase), m_offset(offset) {}


    virtual ChFunction_MyTest* Clone() const override { return new ChFunction_MyTest(); }

    virtual double Get_y(double x) const override { return (m_amp * sin( m_omg*x + m_phase) + m_offset); }  // just for test: simple cosine

    virtual double Get_y_dx(double x) const override{ return m_omg * m_amp * cos(m_omg * x + m_phase);}
    virtual double Get_y_dxdx(double x) const override{ return -m_amp * m_omg * m_omg * sin(m_omg * x + m_phase);}

    private:
    double m_omg;
    double m_phase;
    double m_amp;
    double m_offset;

};


void AddSkeleton(ChSystemNSC& sys, 
                 ChVector<double> pos_center, 
                 double body_rotation, 
                 double left_angle,   // left arm angle 
                 double right_angle,  // right arm angle
                 int no_contact_family_id){

    // link1
    ChVector<double> arm_size(0.05, 0.023, 0.003);
    ChVector<double> body_pos = pos_center;
    ChVector<double> body_size(0.054, 0.027, 0.022);



    ChVector<double> arm1_pos(-body_size.x()/2.0f + arm_size.x()/2.0f * std::cos(left_angle), 0, - arm_size.x()/2.0f * std::sin(left_angle));
    ChQuaternion<double> arm1_rot(Q_from_AngAxis(left_angle, VECT_Y));

    auto link_mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();
    link_mat->SetFriction(0.3);
    auto link_mat_vis = chrono_types::make_shared<ChVisualMaterial>(*ChVisualMaterial::Default());

    std::shared_ptr<collision::ChCollisionModel> collision_model =
                      chrono_types::make_shared<collision::ChCollisionModelBullet>();


    auto arm1 = chrono_types::make_shared<ChBodyEasyBox>(arm_size.x(), 
                                                          arm_size.y(), 
                                                          arm_size.z(),  // x,y,z size
                                                          100,        // density
                                                          true,       // visualization?
                                                          true,
                                                          link_mat,
                                                          collision_model);     // collision?
    arm1->SetPos(arm1_pos + pos_center);
    arm1->SetRot(arm1_rot);
    arm1->SetBodyFixed(false);
    arm1->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(no_contact_family_id);
    sys.Add(arm1);


    // ChQuaternion<double> link2_rot = Q_from_AngAxis(rotation, VECT_Y);

    std::shared_ptr<collision::ChCollisionModel> collision_model2 =
                      chrono_types::make_shared<collision::ChCollisionModelBullet>();

    auto body = chrono_types::make_shared<ChBodyEasyBox>(body_size.x(), 
                                                         body_size.y(), 
                                                         body_size.z(),  // x,y,z size
                                                         100,        // density
                                                         true,       // visualization?
                                                         true,
                                                         link_mat,
                                                         collision_model2);     // collision?
    body->SetPos(pos_center);
    // link2->SetRot(link2_rot);
    body->SetBodyFixed(false);  // the truss does not move!
    body->SetMass(m_large);
    body->GetCollisionModel()->SetFamily(no_contact_family_id);
    
    sys.Add(body);

    ChVector<double> arm2_pos(body_size.x()/2.0f + arm_size.x()/2.0f * std::cos(right_angle), 0, - arm_size.x()/2.0f * std::sin(right_angle));
    ChQuaternion<double> arm2_rot(Q_from_AngAxis(right_angle, VECT_Y));

    std::shared_ptr<collision::ChCollisionModel> collision_model3 =
                      chrono_types::make_shared<collision::ChCollisionModelBullet>();

    auto arm2 = chrono_types::make_shared<ChBodyEasyBox>(arm_size.x(), 
                                                         arm_size.y(), 
                                                         arm_size.z(),  // x,y,z size
                                                         100,        // density
                                                         true,       // visualization?
                                                         true,
                                                         link_mat,
                                                         collision_model3);     // collision?
    arm2->SetPos(arm2_pos + pos_center);
    arm2->SetRot(arm2_rot);
    arm2->SetBodyFixed(false);  // the truss does not move!
    arm2->SetMass(m_small);
    arm2->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(no_contact_family_id);
    sys.Add(arm2);



    // -----------------------------------------------------
    // Create a motor between link1 and link2
    // -----------------------------------------------------
    auto my_motor_1 = chrono_types::make_shared<ChLinkMotorRotationAngle>();
    my_motor_1->Initialize(arm1, body, ChFrame<>(ChVector<>(-body_size.x()/2.0, 0, 0) + body_pos, Q_from_AngAxis(-CH_C_PI_2, VECT_X)));

    // compute initial phase angle
    double amplitude = CH_C_PI_2;
    double freq = 2 * CH_C_PI * 20;
    double offset_left = CH_C_PI - left_angle;
    double phase_angle = -std::asin(offset_left/amplitude);

    // Let the motor use this motion function as a motion profile:
    auto test_left = chrono_types::make_shared<ChFunction_MyTest>(amplitude, freq, phase_angle, offset_left);

    my_motor_1->SetAngleFunction(test_left);    
    sys.AddLink(my_motor_1);

    // -----------------------------------------------------
    // Create a motor between link2 and link3
    // -----------------------------------------------------
    auto my_motor_2 = chrono_types::make_shared<ChLinkMotorRotationAngle>();
    // -90 degree w.r.t. global x, rotation axis, z, pointing in y direction, opposite of gravity
    my_motor_2->Initialize(arm2, body, ChFrame<>(ChVector<>(body_size.x()/2.0f, 0, 0) + body_pos, Q_from_AngAxis(-CH_C_PI_2, VECT_X)));

    // compute initial phase angle
    freq = 2 * CH_C_PI * 25;
    double offset_right = - right_angle;
    phase_angle = -std::asin(offset_right/amplitude);

    // Let the motor use this motion function as a motion profile:
    auto f_test = chrono_types::make_shared<ChFunction_MyTest>(amplitude, freq, phase_angle, offset_right);

    my_motor_2->SetAngleFunction(f_test);    
    sys.AddLink(my_motor_2);

}

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
    double step_size = 1e-4; // 1e-3 in world.skel which one?

    // actuator (type: servo not velocity)
    // force uppser limit (0, 3.0e-2)
    // force lower limit (0, -3.0e-2)
    // position limit enforced
    // point limit: angle: (0, 1.5708 to 0, -1.5708)

    // arm friction 0.37 w/ ground? 

    ChVector<double> skeleton_center(0.0f, 0.0f, 0.0f);
    double left_angle_initial = CH_C_PI_2 + CH_C_PI/6.0f;
    double right_angle_initial = CH_C_PI/2.0f;
    // double right_angle_initial = 0;
 

 
    AddSkeleton(sys, skeleton_center, 0, left_angle_initial, right_angle_initial, 10);


    AddSkeleton(sys, ChVector<double>(0.05, 0.f, -0.06), 0, 3.f * CH_C_PI/2.0f, -CH_C_PI/2.0f, 11);

    ChVectorDynamic<double> initPose1(6);
    initPose1 << 0.0309, 0.0135+0.01, -3.6426e-4, 5.3186 - (EIGEN_PI), 1.5720, 1.57690;

    ChVectorDynamic<double> initPose2(6);
    initPose2 << -0.0474, 0.0135+0.01, -0.0069, 2.4399 - (EIGEN_PI), 1.5719, 1.57691;

    // Create the Irrlicht visualization system
    auto vis = chrono_types::make_shared<ChVisualSystemIrrlicht>();
    vis->AttachSystem(&sys);
    vis->SetWindowSize(800, 600);
    vis->SetWindowTitle("NSC collision demo");
    vis->Initialize();
    vis->AddLogo();
    vis->AddSkyBox();
    vis->AddCamera(ChVector<>(0, 0.5, 0));
    vis->AddTypicalLights();

    int frame = 0;

    while (vis->Run()) {
        vis->BeginScene();
        vis->Render();
        vis->EndScene();

        sys.DoStepDynamics(step_size);


        if (frame % 5 == 0){
            std::cout << sys.GetChTime() << std::endl;
            std::string filename = "screenshot_" + std::to_string(int(frame/100)) + ".png";
            vis->WriteImageToFile(filename);

        }

        frame++;


        // auto link1 = sys.Get_bodylist().at(1);
        // std::cout << "link1 position: " << link1->GetPos().z() << std::endl;

        // std::cout << "link1 velo: " << link1->GetPos_dt().z() << std::endl;

        // std::cout << "link1 acc: " << link1->GetPos_dtdt().z() << std::endl;

    }


    return 0;
}
