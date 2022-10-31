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

class myGaits : public ChFunction {
    public:
    myGaits() : m_amp(CH_C_PI) {}

    myGaits(double amp): m_amp(amp) {}


    virtual myGaits* Clone() const override { return new myGaits(); }

    virtual double Get_y(double x) const override { 
        int period = std::ceil(x/0.8);
        if( period % 2 == 1){
            return 0;
        } 
        else return m_amp;}  // just for test: simple cosine

    virtual double Get_y_dx(double x) const override{ return 0;}
    virtual double Get_y_dxdx(double x) const override{ return 0;}

    private:
    double m_amp;

};

class Skeleton{
    public:
        Skeleton(ChVector<double> skeleton_center,
                 double belly_rotation,
                 double alpha1,  // see Akash paper
                 double alpha2,  // see Akash paper for how alpha defined definition
                 int id){

            m_skeleton_center = skeleton_center;
            m_belly_rotation = belly_rotation;


            m_left_angle = CH_C_PI - alpha1;
            m_right_angle = alpha2;
            no_contact_family_id = id;        
        };

        void Initialize(){

            double mass_small = 0.003186;
            double mass_large = 0.0248;

            auto link_mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();
            link_mat->SetFriction(0.37);
            auto link_mat_vis = chrono_types::make_shared<ChVisualMaterial>(*ChVisualMaterial::Default());


            ChVector<double> arm_size(0.05, 0.023, 0.003);
            ChVector<double> belly_size(0.054, 0.027, 0.022);

            ChVector<double>  left_motor_pos(-belly_size.x()/2 * std::cos(m_belly_rotation), 0,  belly_size.x()/2 * std::sin(m_belly_rotation));
            ChVector<double> right_motor_pos( belly_size.x()/2 * std::cos(m_belly_rotation), 0, -belly_size.x()/2 * std::sin(m_belly_rotation));



            ChVector<double> arm1_pos(left_motor_pos.x() + arm_size.x()/2.0f * std::cos(m_left_angle + m_belly_rotation), 0, left_motor_pos.z() - arm_size.x()/2.0f * std::sin(m_left_angle + m_belly_rotation));

            ChQuaternion<double> arm1_rot(Q_from_AngAxis(m_left_angle + m_belly_rotation, VECT_Y));


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


            ChQuaternion<double> center_body_rotation = Q_from_AngAxis(m_belly_rotation, VECT_Y);

            std::shared_ptr<collision::ChCollisionModel> collision_model2 =
                            chrono_types::make_shared<collision::ChCollisionModelBullet>();

            center_body = chrono_types::make_shared<ChBodyEasyBox>(belly_size.x(), 
                                                                  belly_size.y(), 
                                                                  belly_size.z(),  // x,y,z size
                                                                  100,        // density
                                                                  true,       // visualization?
                                                                  true,
                                                                  link_mat,
                                                                  collision_model2);     // collision?
            center_body->SetPos(m_skeleton_center);
            center_body->SetRot(center_body_rotation);
            center_body->SetBodyFixed(false);  // the truss does not move!
            center_body->SetMass(mass_large);
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

            auto my_functsequence = chrono_types::make_shared<ChFunction_Sequence>();
            auto my_funcpause1 = chrono_types::make_shared<ChFunction_Const>(0);
            auto my_funcsigma1 = chrono_types::make_shared<ChFunction_Sigma>(CH_C_PI, 0, 0.2);  // diplacement, t_start, t_end
            auto my_funcsigma2 = chrono_types::make_shared<ChFunction_Sigma>(-CH_C_PI, 0, 0.2);  // diplacement, t_start, t_end
            auto my_funcpause2 = chrono_types::make_shared<ChFunction_Const>(0);


            auto my_funcpause3 = chrono_types::make_shared<ChFunction_Const>(m_left_angle - CH_C_PI_2);

            std::cout << "set angle left: " << m_left_angle - CH_C_PI_2 << std::endl;
            my_functsequence->InsertFunct(my_funcpause1, 0.6, 1.0, true);  // fx, duration, weight, enforce C0 continuity
            my_functsequence->InsertFunct(my_funcsigma1, 0.2, 1.0, true);  // fx, duration, weight, enforce C0 continuity
            my_functsequence->InsertFunct(my_funcpause2, 0.6, 1.0, true);  // fx, duration, weight, enforce C0 continuity
            my_functsequence->InsertFunct(my_funcsigma2, 0.2, 1.0, true);  // fx, duration, weight, enforce C0 continuity
            auto my_functangle = chrono_types::make_shared<ChFunction_Repeat>();
            my_functangle->Set_fa(my_functsequence);
            my_functangle->Set_window_length(1.6);
            left_motor->SetAngleFunction(my_functangle);


            ChVector<double> arm2_pos(right_motor_pos.x() + arm_size.x()/2.0f * std::cos(m_right_angle + m_belly_rotation), 0, right_motor_pos.z() - arm_size.x()/2.0f * std::sin(m_right_angle + m_belly_rotation));
            ChQuaternion<double> arm2_rot(Q_from_AngAxis(m_right_angle + m_belly_rotation, VECT_Y));

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
            right_arm->SetMass(mass_small);
            right_arm->GetCollisionModel()->SetFamilyMaskNoCollisionWithFamily(no_contact_family_id);

            // -----------------------------------------------------
            // Create a motor between right arm and center body
            // -----------------------------------------------------
            right_motor = chrono_types::make_shared<ChLinkMotorRotationAngle>();
            // -90 degree w.r.t. global x, rotation axis, z, pointing in y direction, opposite of gravity
            right_motor->Initialize(right_arm, center_body, ChFrame<>(right_motor_pos + m_skeleton_center, Q_from_AngAxis(-CH_C_PI_2, VECT_X)));

            auto right_functsequence = chrono_types::make_shared<ChFunction_Sequence>();
            right_functsequence->InsertFunct(my_funcpause1, 0.4, 1.0, true);  // fx, duration, weight, enforce C0 continuity
            right_functsequence->InsertFunct(my_funcsigma1, 0.2, 1.0, true);  // fx, duration, weight, enforce C0 continuity
            right_functsequence->InsertFunct(my_funcpause2, 0.4, 1.0, true);  // fx, duration, weight, enforce C0 continuity
            right_functsequence->InsertFunct(my_funcsigma2, 0.2, 1.0, true);  // fx, duration, weight, enforce C0 continuity
            right_functsequence->InsertFunct(my_funcpause1, 0.4, 1.0, true);  // fx, duration, weight, enforce C0 continuity

            auto right_func_angle = chrono_types::make_shared<ChFunction_Repeat>();
            right_func_angle->Set_fa(right_functsequence);
            right_func_angle->Set_window_length(1.6);


            right_motor->SetAngleFunction(right_func_angle);

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

        std::shared_ptr<ChLinkMotorRotationAngle> GetRightMotor(){return right_motor;};

        std::shared_ptr<ChLinkMotorRotationAngle> GetLeftMotor(){return left_motor;};

    private:
        ChVector<double> m_skeleton_center;
        double m_belly_rotation;
        double m_left_angle;
        double m_right_angle;
        int no_contact_family_id;
        std::shared_ptr<ChBodyEasyBox> left_arm;
        std::shared_ptr<ChBodyEasyBox> center_body;
        std::shared_ptr<ChBodyEasyBox> right_arm;
        std::shared_ptr<ChLinkMotorRotationAngle> left_motor;
        std::shared_ptr<ChLinkMotorRotationAngle> right_motor;

};

