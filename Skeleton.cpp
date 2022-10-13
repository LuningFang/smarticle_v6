#include "chrono/physics/ChSystemNSC.h"
#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChLinkMotorRotationAngle.h"



#include "chrono/utils/ChUtilsInputOutput.h"
#include "chrono_thirdparty/filesystem/path.h"

#include <cmath>
using namespace chrono;




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

class Skeleton{
    public:
        Skeleton(ChVector<double> skeleton_center,
                 double body_rotation,
                 double left_angle_initial,
                 double right_angle_initial,
                 int id){

            m_skeleton_center = skeleton_center;
            m_body_rotation = body_rotation;
            m_left_angle = left_angle_initial;
            m_right_angle = right_angle_initial;
            no_contact_family_id = id;        
        };

        void Initialize(){

            double m_small = 0.003186;
            double m_large = 0.0248;

            auto link_mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();
            link_mat->SetFriction(0.3);
            auto link_mat_vis = chrono_types::make_shared<ChVisualMaterial>(*ChVisualMaterial::Default());


            ChVector<double> arm_size(0.05, 0.023, 0.003);
            ChVector<double> body_size(0.054, 0.027, 0.022);

            ChVector<double>  left_motor_pos(-body_size.x()/2 * std::cos(m_body_rotation), 0,  body_size.x()/2 * std::sin(m_body_rotation));
            ChVector<double> right_motor_pos( body_size.x()/2 * std::cos(m_body_rotation), 0, -body_size.x()/2 * std::sin(m_body_rotation));



            ChVector<double> arm1_pos(left_motor_pos.x() + arm_size.x()/2.0f * std::cos(m_left_angle + m_body_rotation), 0, left_motor_pos.z() - arm_size.x()/2.0f * std::sin(m_left_angle + m_body_rotation));

            ChQuaternion<double> arm1_rot(Q_from_AngAxis(m_left_angle + m_body_rotation, VECT_Y));


            std::shared_ptr<collision::ChCollisionModel> collision_model =
                            chrono_types::make_shared<collision::ChCollisionModelBullet>();



            // left arm
            left_arm = chrono_types::make_shared<ChBodyEasyBox>(arm_size.x(), 
                                                                arm_size.y(), 
                                                                arm_size.z(),  // x,y,z size
                                                                100,        // density
                                                                true,       // visualization?
                                                                true,
                                                                link_mat,
                                                                collision_model);     // collision?
            left_arm->SetPos(arm1_pos + m_skeleton_center);
            left_arm->SetRot(arm1_rot);
            left_arm->SetBodyFixed(false);
            left_arm->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(no_contact_family_id);


            ChQuaternion<double> center_body_rotation = Q_from_AngAxis(m_body_rotation, VECT_Y);

            std::shared_ptr<collision::ChCollisionModel> collision_model2 =
                            chrono_types::make_shared<collision::ChCollisionModelBullet>();

            center_body = chrono_types::make_shared<ChBodyEasyBox>(body_size.x(), 
                                                                body_size.y(), 
                                                                body_size.z(),  // x,y,z size
                                                                100,        // density
                                                                true,       // visualization?
                                                                true,
                                                                link_mat,
                                                                collision_model2);     // collision?
            center_body->SetPos(m_skeleton_center);
            center_body->SetRot(center_body_rotation);
            center_body->SetBodyFixed(false);  // the truss does not move!
            center_body->SetMass(m_large);
            center_body->GetCollisionModel()->SetFamily(no_contact_family_id);



            // -----------------------------------------------------
            // Create a motor between left arm and center body
            // -----------------------------------------------------
            left_motor = chrono_types::make_shared<ChLinkMotorRotationAngle>();
            // left_motor->Initialize(left_arm, center_body, ChFrame<>(left_motor_pos + m_skeleton_center, Q_from_AngAxis(-CH_C_PI_2, VECT_X)));

            left_motor->Initialize(left_arm, center_body, ChFrame<>(left_motor_pos + m_skeleton_center, Q_from_AngAxis(CH_C_PI_2, VECT_X)));

            // compute initial phase angle
            double amplitude = CH_C_PI_2;
            double freq = 2 * CH_C_PI * 0.625;
            double offset_left = CH_C_PI - m_left_angle;
            double phase_angle = -std::asin(offset_left/amplitude);

            // Let the motor use this motion function as a motion profile:
            auto test_left = chrono_types::make_shared<ChFunction_MyTest>(amplitude, freq, phase_angle, offset_left);

            left_motor->SetAngleFunction(test_left);


            ChVector<double> arm2_pos(right_motor_pos.x() + arm_size.x()/2.0f * std::cos(m_right_angle + m_body_rotation), 0, right_motor_pos.z() - arm_size.x()/2.0f * std::sin(m_right_angle + m_body_rotation));
            ChQuaternion<double> arm2_rot(Q_from_AngAxis(m_right_angle + m_body_rotation, VECT_Y));

            std::shared_ptr<collision::ChCollisionModel> collision_model3 =
                            chrono_types::make_shared<collision::ChCollisionModelBullet>();

            right_arm = chrono_types::make_shared<ChBodyEasyBox>(arm_size.x(), 
                                                                arm_size.y(), 
                                                                arm_size.z(),  // x,y,z size
                                                                100,        // density
                                                                true,       // visualization?
                                                                true,
                                                                link_mat,
                                                                collision_model3);     // collision?
            right_arm->SetPos(arm2_pos + m_skeleton_center);
            right_arm->SetRot(arm2_rot);
            right_arm->SetBodyFixed(false);  // the truss does not move!
            right_arm->SetMass(m_small);
            right_arm->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(no_contact_family_id);

            // -----------------------------------------------------
            // Create a motor between right arm and center body
            // -----------------------------------------------------
            right_motor = chrono_types::make_shared<ChLinkMotorRotationAngle>();
            // -90 degree w.r.t. global x, rotation axis, z, pointing in y direction, opposite of gravity
            right_motor->Initialize(right_arm, center_body, ChFrame<>(right_motor_pos + m_skeleton_center, Q_from_AngAxis(-CH_C_PI_2, VECT_X)));

            // compute initial phase angle
            freq = 2 * CH_C_PI * 0.625;
            double offset_right = - m_right_angle;
            phase_angle = -std::asin(offset_right/amplitude);

            // Let the motor use this motion function as a motion profile:
            auto f_test = chrono_types::make_shared<ChFunction_MyTest>(amplitude, freq, phase_angle, offset_right);

            right_motor->SetAngleFunction(f_test);

        };

        void AddSkeleton(ChSystemNSC& sys){
            sys.AddBody(left_arm);
            sys.AddBody(center_body);
            sys.Add(right_arm);

            sys.AddLink(left_motor);            
            sys.AddLink(right_motor);

        }
    
        ChVector<double> GetLeftArmPos(){
            return left_arm->GetPos();
        };

    private:
        ChVector<double> m_skeleton_center;
        double m_body_rotation;
        double m_left_angle;
        double m_right_angle;
        int no_contact_family_id;
        std::shared_ptr<ChBodyEasyBox> left_arm;
        std::shared_ptr<ChBodyEasyBox> center_body;
        std::shared_ptr<ChBodyEasyBox> right_arm;
        std::shared_ptr<ChLinkMotorRotationAngle> left_motor;
        std::shared_ptr<ChLinkMotorRotationAngle> right_motor;

};

