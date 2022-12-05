#include "chrono/physics/ChSystemNSC.h"
#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChLinkMotorRotationAngle.h"
#include "chrono/physics/ChLinkMotorRotationSpeed.h"


#include "chrono/utils/ChUtilsInputOutput.h"
#include "chrono_thirdparty/filesystem/path.h"
#include <cmath>
using namespace chrono;

// left motor function 
class ChFunction_LeftMotor : public ChFunction {
    public:
    ChFunction_LeftMotor() : m_speed(CH_C_1_PI/0.4f) {}

    ChFunction_LeftMotor(double speed): m_speed(speed){}


    virtual ChFunction_LeftMotor* Clone() const override { return new ChFunction_LeftMotor(); }

    // define your own speed function ( x is time, Get_y return speed)
    virtual double Get_y(double x) const override { 

        int g = int((x+0.2f)/0.4f) % 4;
        double velo;
        switch (g) {
            case 0:
                return 0;
            case 1:
                return m_speed;
            case 2:
                return 0;
            case 3:
                return -m_speed;
        }
    }


    private:
    double m_speed;
};

class ChFunction_RightMotor : public ChFunction {
    public:
    ChFunction_RightMotor() : m_speed(CH_C_1_PI/0.4f) {}

    ChFunction_RightMotor(double speed): m_speed(speed){}


    virtual ChFunction_RightMotor* Clone() const override { return new ChFunction_RightMotor(); }

    virtual double Get_y(double x) const override { 

        int g = int((x + 0.2f)/0.4f) % 4;
        double velo;
        switch (g) {
            case 0:
                return -m_speed;
            case 1:
                return 0;
            case 2:
                return m_speed;
            case 3:
                return 0;
        }
    }


    private:
    double m_speed;
};


class Skeleton{
    public:
        Skeleton(ChVector<double> skeleton_center,
                 double belly_rotation,
                 double alpha1,  
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
            ChVector<double> arm_size(0.05, 0.023, 0.003);
            ChVector<double> belly_size(0.054, 0.027, 0.022);


            auto link_mat = chrono_types::make_shared<ChMaterialSurfaceNSC>();
            link_mat->SetFriction(0.37);
            auto link_mat_vis = chrono_types::make_shared<ChVisualMaterial>(*ChVisualMaterial::Default());


            ChVector<double>  left_motor_pos(-belly_size.x()/2 * std::cos(m_belly_rotation), 0,  belly_size.x()/2 * std::sin(m_belly_rotation));
            ChVector<double> right_motor_pos( belly_size.x()/2 * std::cos(m_belly_rotation), 0, -belly_size.x()/2 * std::sin(m_belly_rotation));



            ChVector<double> arm1_pos(left_motor_pos.x() + arm_size.x()/2.0f * std::cos(m_left_angle + m_belly_rotation), 0, left_motor_pos.z() - arm_size.x()/2.0f * std::sin(m_left_angle + m_belly_rotation));

            // rotate angle of theta+alpha_1 with respect to y axis
            ChQuaternion<double> arm1_rot(Q_from_AngAxis(m_left_angle + m_belly_rotation, VECT_Y));


            std::shared_ptr<collision::ChCollisionModel> collision_model =
                            chrono_types::make_shared<collision::ChCollisionModelBullet>();


            // -----------------------------------------------------
            // left arm
            // -----------------------------------------------------
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


            // -----------------------------------------------------
            // center belly
            // -----------------------------------------------------
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
            center_body->SetBodyFixed(false);
            center_body->SetMass(mass_large);
            center_body->GetCollisionModel()->SetFamily(no_contact_family_id);



            // -----------------------------------------------------
            // Create a motor between left arm and center body
            // -----------------------------------------------------
            left_motor = chrono_types::make_shared<ChLinkMotorRotationSpeed>();
            left_motor->Initialize(left_arm, center_body, ChFrame<>(left_motor_pos + m_skeleton_center, Q_from_AngAxis(CH_C_PI_2, VECT_X)));
            // initialize motor velocity function, speed of pi/4
            auto left_motor_velo_func = chrono_types::make_shared<ChFunction_LeftMotor>(CH_C_PI/0.4f);
            left_motor->SetSpeedFunction(left_motor_velo_func);


            ChVector<double> arm2_pos(right_motor_pos.x() + arm_size.x()/2.0f * std::cos(m_right_angle + m_belly_rotation), 0, right_motor_pos.z() - arm_size.x()/2.0f * std::sin(m_right_angle + m_belly_rotation));
            ChQuaternion<double> arm2_rot(Q_from_AngAxis(m_right_angle + m_belly_rotation, VECT_Y));

            std::shared_ptr<collision::ChCollisionModel> collision_model3 =
                            chrono_types::make_shared<collision::ChCollisionModelBullet>();

            // -----------------------------------------------------
            // right arm
            // -----------------------------------------------------
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
            right_motor = chrono_types::make_shared<ChLinkMotorRotationSpeed>();
            right_motor->Initialize(right_arm, center_body, ChFrame<>(right_motor_pos + m_skeleton_center, Q_from_AngAxis(-CH_C_PI_2, VECT_X)));

            auto right_motor_velo_func = chrono_types::make_shared<ChFunction_RightMotor>(CH_C_PI/0.4f);
            right_motor->SetSpeedFunction(right_motor_velo_func);

        };

        void AddSkeleton(ChSystemNSC& sys){
            sys.AddBody(left_arm);
            sys.AddBody(center_body);
            sys.AddBody(right_arm);

            sys.AddLink(left_motor);            
            // sys.AddLink(right_motor);

        }
    
        ChVector<double> GetPos(){
            return center_body->GetPos();
        };

        std::shared_ptr<ChLinkMotorRotationSpeed> GetRightMotor(){return right_motor;};

        std::shared_ptr<ChLinkMotorRotationSpeed> GetLeftMotor(){return left_motor;};

        // return alpha1 angle 
        double GetAlpha1() {return left_motor->GetMotorRot();};

        // return alpha2 angle 
        double GetAlpha2() {return right_motor->GetMotorRot();};

        // return theta


    private:
        ChVector<double> m_skeleton_center;
        double m_belly_rotation;
        double m_left_angle;
        double m_right_angle;
        int no_contact_family_id;
        std::shared_ptr<ChBodyEasyBox> left_arm;
        std::shared_ptr<ChBodyEasyBox> center_body;
        std::shared_ptr<ChBodyEasyBox> right_arm;
        std::shared_ptr<ChLinkMotorRotationSpeed> left_motor;
        std::shared_ptr<ChLinkMotorRotationSpeed> right_motor;

};

