#ifndef REALROBOT_INTERFACE_H
#define REALROBOT_INTERFACE_H

#include <iostream>
#include <thread>

#include <Eigen/Dense>
#include <vector>

#include "ethercattype.h"
#include "nicdrv.h"
#include "ethercatbase.h"
#include "ethercatmain.h"
#include "ethercatdc.h"
#include "ethercatcoe.h"
#include "ethercatfoe.h"
#include "ethercatconfig.h"
#include "ethercatprint.h"

//#include "ethercat.h"

#include "tocabi_controller/state_manager.h"
#include "tocabi_controller/GainCommand.h"

#define NSEC_PER_SEC 1000000000

#define CNT_TO_RAD_46 (3.141592 * 2 / 819200) //819200
#define RAD_TO_CNT_46 (1 / (CNT_TO_RAD_46))
/*
#define Kp_Yaw1 150000   //Hip
#define Kp_Roll1 500000  //Hip
#define Kp_Pitch1 500000 //Hip
#define Kp_Pitch2 500000 //Knee
#define Kp_Pitch3 500000 //Ankle
#define Kp_Roll2 650000  //Ankle

#define Kv_Yaw1 3000   //Hip
#define Kv_Roll1 3000  //Hip
#define Kv_Pitch1 3000 //Hip
#define Kv_Pitch2 2000 //Knee
#define Kv_Pitch3 3000 //Ankle
#define Kv_Roll2 5000  //Ankle
*/
#define Kp_Yaw1 1500   //Hip
#define Kp_Roll1 5000  //Hip
#define Kp_Pitch1 5000 //Hip
#define Kp_Pitch2 5000 //Knee
#define Kp_Pitch3 5000 //Ankle
#define Kp_Roll2 6500  //Ankle

#define Kv_Yaw1 50   //Hip
#define Kv_Roll1 50  //Hip
#define Kv_Pitch1 50 //Hip
#define Kv_Pitch2 40 //Knee
#define Kv_Pitch3 30 //Ankle
#define Kv_Roll2 60  //Ankle

#define EC_TIMEOUTMON 500

const double CNT2RAD[MODEL_DOF] =
    {
        CNT_TO_RAD_46, CNT_TO_RAD_46, CNT_TO_RAD_46, CNT_TO_RAD_46, CNT_TO_RAD_46, CNT_TO_RAD_46,
        CNT_TO_RAD_46, CNT_TO_RAD_46, CNT_TO_RAD_46, CNT_TO_RAD_46, CNT_TO_RAD_46, CNT_TO_RAD_46,
        CNT_TO_RAD_46, CNT_TO_RAD_46, CNT_TO_RAD_46, CNT_TO_RAD_46, CNT_TO_RAD_46, CNT_TO_RAD_46,
        CNT_TO_RAD_46, CNT_TO_RAD_46, CNT_TO_RAD_46, CNT_TO_RAD_46, CNT_TO_RAD_46, CNT_TO_RAD_46,
        CNT_TO_RAD_46, CNT_TO_RAD_46, CNT_TO_RAD_46, CNT_TO_RAD_46, CNT_TO_RAD_46, CNT_TO_RAD_46,
        CNT_TO_RAD_46, CNT_TO_RAD_46, CNT_TO_RAD_46};

const double RAD2CNT[MODEL_DOF] =
    {
        RAD_TO_CNT_46, RAD_TO_CNT_46, RAD_TO_CNT_46, RAD_TO_CNT_46, RAD_TO_CNT_46, RAD_TO_CNT_46,
        RAD_TO_CNT_46, RAD_TO_CNT_46, RAD_TO_CNT_46, RAD_TO_CNT_46, RAD_TO_CNT_46, RAD_TO_CNT_46,
        RAD_TO_CNT_46, RAD_TO_CNT_46, RAD_TO_CNT_46, RAD_TO_CNT_46, RAD_TO_CNT_46, RAD_TO_CNT_46,
        RAD_TO_CNT_46, RAD_TO_CNT_46, RAD_TO_CNT_46, RAD_TO_CNT_46, RAD_TO_CNT_46, RAD_TO_CNT_46,
        RAD_TO_CNT_46, RAD_TO_CNT_46, RAD_TO_CNT_46, RAD_TO_CNT_46, RAD_TO_CNT_46, RAD_TO_CNT_46,
        RAD_TO_CNT_46, RAD_TO_CNT_46, RAD_TO_CNT_46};

const double CNT2NM[MODEL_DOF] =
    {             //Elmo 순서
        0.010526, //head
        0.010526,
        0.010526, //wrist
        0.010526,
        0.010526,
        0.010526,
        0.064516, //shoulder3
        0.064516, //arm
        0.064516, //arm
        0.064516, //shoulder3
        0.02381,  //Elbow
        0.02381,  //Forearm
        0.02381,  //Forearm
        0.02381,  //Elbow
        0.064516, //shoulder1
        0.064516, //shoulder2
        0.064516, //shoulder2
        0.064516, //shoulder1
        0.30303,  //Waist
        0.30303,
        0.1724, //rightLeg
        0.2307,
        0.2635,
        0.2890,
        0.2834,
        0.0811,
        0.30303, //upperbody
        0.1724,  //leftLeg
        0.2307,
        0.2635,
        0.2890,
        0.2834,
        0.0811};

const double NM2CNT[MODEL_DOF] =
    {       //Elmo 순서
        95, //head
        95,
        95, //wrist
        95,
        95,
        95,
        15.5, //shoulder3
        15.5, //arm
        15.5, //arm
        15.5, //shoulder3
        42,   //Elbow
        42,   //Forearm
        42,   //Forearm
        42,   //Elbow
        15.5, //shoulder1
        15.5, //shoulder2
        15.5, //shoulder2
        15.5, //shoulder1
        3.3,  //Waist
        3.3,
        5.8, //rightLeg
        4.3,
        3.8,
        3.46,
        3.52,
        3.3,
        3.3, //upperbody
        5.8, //leftLeg
        4.3,
        3.8,
        3.46,
        3.52,
        3.3};

const int positionExternalModElmo[MODEL_DOF] =
    {
        0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0,
        0, 3942, 5148, 3234, 7499, 4288,
        0,
        0, 2522, 735, 8132, 2127, 7155};

const double MOTORCONTSTANT[MODEL_DOF] =
    {
        0.11495,
        0.1748,
        0.14915,
        0.14915,
        0.14915,
        0.05795,
        0.11495,
        0.1748,
        0.14915,
        0.14915,
        0.14915,
        0.05795};

const double Kp[MODEL_DOF] =
    {
        Kp_Yaw1,
        Kp_Roll1,
        Kp_Pitch1,
        Kp_Pitch2,
        Kp_Pitch3,
        Kp_Roll2,
        Kp_Yaw1,
        Kp_Roll1,
        Kp_Pitch1,
        Kp_Pitch2,
        Kp_Pitch3,
        Kp_Roll2};

const double Kv[MODEL_DOF] =
    {
        Kv_Yaw1,
        Kv_Roll1,
        Kv_Pitch1,
        Kv_Pitch2,
        Kv_Pitch3,
        Kv_Roll2,
        Kv_Yaw1,
        Kv_Roll1,
        Kv_Pitch1,
        Kv_Pitch2,
        Kv_Pitch3,
        Kv_Roll2};

//Axis correction parameter.
const double Dr[MODEL_DOF] =
    {1, 1, 1, 1, 1, 1,
     1, 1, 1, 1, 1, 1,
     1, 1, 1, 1, 1, 1,
     1, 1, 1, 1, 1, 1,
     1, 1, 1, 1, 1, 1,
     1, 1, 1};

//homingsensorcheck
const int FILE_CNT = 1;
const std::string FILE_NAMES[FILE_CNT] =
    {
        "/home/dyros/homing/0_homing.txt"};

using namespace std;

namespace EtherCAT_Elmo
{
enum MODE_OF_OPERATION
{
    ProfilePositionmode = 1,
    ProfileVelocitymode = 3,
    ProfileTorquemode = 4,
    Homingmode = 6,
    InterpolatedPositionmode = 7,
    CyclicSynchronousPositionmode = 8,
    CyclicSynchronousVelocitymode = 9,
    CyclicSynchronousTorquemode = 10,
    CyclicSynchronousTorquewithCommutationAngle = 11
};

struct ElmoGoldDevice
{
    struct elmo_gold_tx
    {
        int32_t targetPosition;
        int32_t targetVelocity;
        int16_t targetTorque;
        uint16_t maxTorque;
        uint16_t controlWord;
        int8_t modeOfOperation;
    };
    struct elmo_gold_rx
    {
        int32_t positionActualValue;
        //int32_t positionFollowingErrrorValue;
        uint32_t hommingSensor;
        uint16_t statusWord;
        //int8_t modeOfOperationDisplay;
        int32_t velocityActualValue;
        int16_t torqueActualValue;
        //int16_t torqueDemandValue;
        int32_t positionExternal;
    };
};
} // namespace EtherCAT_Elmo

namespace ElmoHommingStatus
{
enum FZResult
{
    SUCCESS,
    FAILURE
};
}; // namespace ElmoHommingStatus

struct ElmoHomming
{
    bool hommingElmo;
    bool hommingElmo_before;
    bool startFound = false;
    bool endFound = false;
    int findZeroSequence = 0;
    double initTime;
    double initPos;
    double posStart;
    double posEnd;
    double req_length = 0.2;
    double firstPos;

    int result;
};

class RealRobotInterface : public StateManager
{
    fstream file[FILE_CNT];

public:
    RealRobotInterface(DataContainer &dc_global);
    virtual ~RealRobotInterface() {}

    //update state of Robot from mujoco
    virtual void updateState() override;

    virtual void sendCommand(Eigen::VectorQd command, double sim_time) override;

    //connect to ethercat
    //virtual void connect() override;

    //void ethercatCheck_();
    //void ethercatThread_();
    void ethercatThread();
    void ethercatCheck();
    void imuThread();
    void ftsensorThread();

    int checkTrajContinuity(int slv_number);
    void checkSafety(int slv_number, double max_vel, double max_dis);
    void findZeroPoint(int slv_number);
    void findZeroLeg();

    bool controlWordGenerate(const uint16_t statusWord, uint16_t &controlWord);

    const int FAULT_BIT = 3;
    const int OPERATION_ENABLE_BIT = 2;
    const int SWITCHED_ON_BIT = 1;
    const int READY_TO_SWITCH_ON_BIT = 0;

    enum
    {
        CW_SHUTDOWN = 6,
        CW_SWITCHON = 7,
        CW_ENABLEOP = 15,
        CW_DISABLEOP = 7,
    };

    enum
    {
        FZ_CHECKHOMMINGSTATUS,
        FZ_FINDHOMMINGSTART,
        FZ_FINDHOMMINGEND,
        FZ_FINDHOMMING,
        FZ_GOTOZEROPOINT,
        FZ_HOLDZEROPOINT,
        FZ_FAILEDANDRETURN,
        FZ_MANUALDETECTION,
        FZ_TORQUEZERO,
    };

    ElmoHomming elmofz[MODEL_DOF];

    int expectedWKC;
    boolean needlf;
    volatile int wkc;
    boolean inOP;
    uint8 currentgroup = 0;

    int Walking_State;
    int ElmoMode[MODEL_DOF];
    bool checkPosSafety[MODEL_DOF];
    //int ElmoState[MODEL_DOF];
    //int ElmoState_before[MODEL_DOF];

    enum
    {
        EM_POSITION = 11,
        EM_TORQUE = 22,
        EM_DEFAULT = 33,
    };

    Eigen::VectorQd positionElmo;
    Eigen::VectorQd positionExternalElmo;
    Eigen::VectorQd velocityElmo;
    Eigen::VectorQd torqueElmo;
    Eigen::VectorQd torqueDemandElmo;
    Eigen::VectorQd velocityDesiredElmo;
    Eigen::VectorQd torqueDesiredElmo;
    Eigen::VectorQd torqueDesiredController;
    Eigen::VectorQd positionDesiredElmo;
    Eigen::VectorQd positionDesiredElmo_Before;
    Eigen::VectorQd positionDesiredController;
    Eigen::VectorQd positionInitialElmo;
    Eigen::VectorQd positionZeroElmo;
    Eigen::VectorQd positionZeroModElmo;
    Eigen::VectorQd initTimeElmo;
    Eigen::VectorQd positionSafteyHoldElmo;

    Eigen::VectorQd rq_;
    Eigen::VectorQd rq_dot_;

    int stateElmo[MODEL_DOF];
    int stateElmo_before[MODEL_DOF];
    int positionExternalModElmo[MODEL_DOF];

    bool hommingElmo[MODEL_DOF];
    bool hommingElmo_before[MODEL_DOF];

    int ElmoSafteyMode[MODEL_DOF];

    EtherCAT_Elmo::ElmoGoldDevice::elmo_gold_rx *rxPDO[MODEL_DOF];
    EtherCAT_Elmo::ElmoGoldDevice::elmo_gold_tx *txPDO[MODEL_DOF];

    bool ElmoConnected = false;
    bool ElmoTerminate = false;

    std::vector<int> fz_group1;
    std::vector<int> fz_group2;
    std::vector<int> fz_group3;

private:
    DataContainer &dc;

    Eigen::VectorQd getCommand();
    void ImuCallback(const sensor_msgs::ImuConstPtr &msg);

    ros::Subscriber imuSubscriber;
    ros::Subscriber gainSubscriber;
    Eigen::VectorQd CustomGain;
    void gainCallbak(const tocabi_controller::GainCommandConstPtr &msg);

    double elmoJointMove(double init, double angle, double start_time, double traj_time);

    mujoco_ros_msgs::JointSet mujoco_joint_set_msg_;

    Eigen::Vector6d RealConstant;

    bool sim_runnung;

    std::string joint_name_mj[MODEL_DOF];
    //ros::Rate rate_;
    int dyn_hz;
};

#endif
