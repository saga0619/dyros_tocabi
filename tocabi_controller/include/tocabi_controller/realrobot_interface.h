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

#include <std_msgs/Float32MultiArray.h>
#include "tocabi_controller/mx5_imu.h"
//#include "ethercat.h"

#include "tocabi_controller/state_manager.h"
#include "tocabi_controller/GainCommand.h"

#define NSEC_PER_SEC 1000000000

#define CNT_TO_RAD_46 (3.141592 * 2 / (8192 * 100)) //819200
#define CNT_TO_RAD_80 (3.141592 * 2 / (8000 * 100)) //819200

#define EXT_CNT_TO_RAD_46 (3.141592 * 2 / 8192) //819200
#define EXT_CNT_TO_RAD_80 (3.141592 * 2 / 8192) //819200

#define RAD_TO_CNT_46 (1 / (CNT_TO_RAD_46))
#define RAD_TO_CNT_80 (1 / (CNT_TO_RAD_80))

#define EXT_RAD_TO_CNT_46 (1 / (EXT_CNT_TO_RAD_46))
#define EXT_RAD_TO_CNT_80 (1 / (EXT_CNT_TO_RAD_80))

#define ELMO_DOF 33

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

extern volatile bool shutdown_tocabi_bool;

const double CNT2RAD[ELMO_DOF] =
    {
        CNT_TO_RAD_46, CNT_TO_RAD_46, CNT_TO_RAD_46, CNT_TO_RAD_46, CNT_TO_RAD_46, CNT_TO_RAD_46,
        CNT_TO_RAD_46, CNT_TO_RAD_46, CNT_TO_RAD_46, CNT_TO_RAD_46, CNT_TO_RAD_46, CNT_TO_RAD_46,
        CNT_TO_RAD_46, CNT_TO_RAD_46, CNT_TO_RAD_46, CNT_TO_RAD_46, CNT_TO_RAD_46, CNT_TO_RAD_46,
        CNT_TO_RAD_46, CNT_TO_RAD_46, CNT_TO_RAD_80, CNT_TO_RAD_46, CNT_TO_RAD_46, CNT_TO_RAD_46,
        CNT_TO_RAD_46, CNT_TO_RAD_46, CNT_TO_RAD_80, CNT_TO_RAD_46, CNT_TO_RAD_46, CNT_TO_RAD_46,
        CNT_TO_RAD_46, CNT_TO_RAD_46, CNT_TO_RAD_46};

const double EXTCNT2RAD[ELMO_DOF] =
    {
        EXT_CNT_TO_RAD_46, EXT_CNT_TO_RAD_46, EXT_CNT_TO_RAD_46, EXT_CNT_TO_RAD_46, EXT_CNT_TO_RAD_46, EXT_CNT_TO_RAD_46,
        EXT_CNT_TO_RAD_46, EXT_CNT_TO_RAD_46, EXT_CNT_TO_RAD_46, EXT_CNT_TO_RAD_46, EXT_CNT_TO_RAD_46, EXT_CNT_TO_RAD_46,
        EXT_CNT_TO_RAD_46, EXT_CNT_TO_RAD_46, EXT_CNT_TO_RAD_46, EXT_CNT_TO_RAD_46, EXT_CNT_TO_RAD_46, EXT_CNT_TO_RAD_46,
        EXT_CNT_TO_RAD_46, EXT_CNT_TO_RAD_46, EXT_CNT_TO_RAD_46, EXT_CNT_TO_RAD_46, EXT_CNT_TO_RAD_46, EXT_CNT_TO_RAD_46,
        EXT_CNT_TO_RAD_46, EXT_CNT_TO_RAD_46, EXT_CNT_TO_RAD_46, EXT_CNT_TO_RAD_46, EXT_CNT_TO_RAD_46, EXT_CNT_TO_RAD_46,
        EXT_CNT_TO_RAD_46, EXT_CNT_TO_RAD_46, EXT_CNT_TO_RAD_46};

const double RAD2CNT[ELMO_DOF] =
    {
        RAD_TO_CNT_46, RAD_TO_CNT_46, RAD_TO_CNT_46, RAD_TO_CNT_46, RAD_TO_CNT_46, RAD_TO_CNT_46,
        RAD_TO_CNT_46, RAD_TO_CNT_46, RAD_TO_CNT_46, RAD_TO_CNT_46, RAD_TO_CNT_46, RAD_TO_CNT_46,
        RAD_TO_CNT_46, RAD_TO_CNT_46, RAD_TO_CNT_46, RAD_TO_CNT_46, RAD_TO_CNT_46, RAD_TO_CNT_46,
        RAD_TO_CNT_46, RAD_TO_CNT_46, RAD_TO_CNT_80, RAD_TO_CNT_46, RAD_TO_CNT_46, RAD_TO_CNT_46,
        RAD_TO_CNT_46, RAD_TO_CNT_46, RAD_TO_CNT_80, RAD_TO_CNT_46, RAD_TO_CNT_46, RAD_TO_CNT_46,
        RAD_TO_CNT_46, RAD_TO_CNT_46, RAD_TO_CNT_46};

const double EXTRAD2CNT[ELMO_DOF] =
    {
        EXT_RAD_TO_CNT_46, EXT_RAD_TO_CNT_46, EXT_RAD_TO_CNT_46, EXT_RAD_TO_CNT_46, EXT_RAD_TO_CNT_46, EXT_RAD_TO_CNT_46,
        EXT_RAD_TO_CNT_46, EXT_RAD_TO_CNT_46, EXT_RAD_TO_CNT_46, EXT_RAD_TO_CNT_46, EXT_RAD_TO_CNT_46, EXT_RAD_TO_CNT_46,
        EXT_RAD_TO_CNT_46, EXT_RAD_TO_CNT_46, EXT_RAD_TO_CNT_46, EXT_RAD_TO_CNT_46, EXT_RAD_TO_CNT_46, EXT_RAD_TO_CNT_46,
        EXT_RAD_TO_CNT_46, EXT_RAD_TO_CNT_46, EXT_RAD_TO_CNT_46, EXT_RAD_TO_CNT_46, EXT_RAD_TO_CNT_46, EXT_RAD_TO_CNT_46,
        EXT_RAD_TO_CNT_46, EXT_RAD_TO_CNT_46, EXT_RAD_TO_CNT_46, EXT_RAD_TO_CNT_46, EXT_RAD_TO_CNT_46, EXT_RAD_TO_CNT_46,
        EXT_RAD_TO_CNT_46, EXT_RAD_TO_CNT_46, EXT_RAD_TO_CNT_46};

const double NM2CNT[ELMO_DOF] =
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
        4.5,
        12.33,
        3.3, //upperbody
        5.8, //leftLeg
        4.3,
        3.8,
        3.46,
        4.5,
        12.33};

const double jointLimitUp[ELMO_DOF] =
    {
        0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0};

const double jointLimitLow[ELMO_DOF] =
    {
        0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0, 0};

const int positionExternalModElmo[ELMO_DOF] =
    {
        0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0, 0, 0, 0, 0,
        0, 0, 0, 0,
        5370, 3942, 5148, 3234, 7499, 4288,
        0,
        3799, 2522, 735, 8132, 2127, 7155};

const double MOTORCONTSTANT[ELMO_DOF] =
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

const double Kp[ELMO_DOF] =
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

const double Kv[ELMO_DOF] =
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
const double Dr[ELMO_DOF] =
    {1, -1, 1, 1, 1, 1,
     1, 1, 1, -1, -1, 1,
     1, -1, 1, 1, 1, 1,
     1, 1, -1, -1, -1, -1,
     1, 1, 1, 1, -1, 1,
     1, -1, 1};

const double EXTDr[ELMO_DOF] =
    {1, -1, 1, 1, 1, 1,
     1, 1, 1, -1, -1, 1,
     1, -1, 1, 1, 1, 1,
     1, 1, 1, 1, 1, 1,
     -1, -1, 1, 1, 1, -1,
     -1, 1, -1};

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
    SUCCESS = 11,
    FAILURE = 22
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
    double init_direction = 1;

    int result;
};

class RealRobotInterface : public StateManager
{
public:
    RealRobotInterface(DataContainer &dc_global);
    virtual ~RealRobotInterface() {}

    //update state of Robot from mujoco
    virtual void updateState() override;

    virtual void sendCommand(Eigen::VectorQd command, double sim_time, int control_mode = Torquemode) override;

    //connect to ethercat
    //virtual void connect() override;

    //void ethercatCheck_();
    //void ethercatThread_();
    void ethercatThread();
    void ethercatCheck();
    void imuThread();
    void ftsensorThread();
    void handftsensorThread();

    void ethercatThreadLower();
    void ethercatThreadUpper();
    void ethercatCheckLower();
    void ethercatCheckUpper();

    int checkTrajContinuity(int slv_number);
    void checkSafety(int slv_number, double max_vel, double max_dis);

    void checkJointLimit(int slv_number);
    void findZeroPoint(int slv_number);
    void findZeroLeg();
    void findZeroPointlow(int slv_number);

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

    ElmoHomming elmofz[ELMO_DOF];

    int expectedWKC;
    boolean needlf;
    volatile int wkc;
    boolean inOP;
    uint8 currentgroup = 0;

    int ElmoMode[ELMO_DOF];
    bool checkPosSafety[ELMO_DOF];
    //int ElmoState[ELMO_DOF];
    //int ElmoState_before[ELMO_DOF];
    fstream file_homming;
    fstream elmo_zp;
    fstream elmo_zp_log;
    std::string zp_path, zplog_path, pack_path;

    int checkfirst = -1;
    int checkfirst_before = -1;
    int al = 111;
    int fr_count = 0;

    enum
    {
        EM_POSITION = 11,
        EM_TORQUE = 22,
        EM_DEFAULT = 33,
        EM_COMMUTATION = 44,
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
    Eigen::VectorQd rq_ext_;
    Eigen::VectorQd rq_dot_;

    int stateElmo[ELMO_DOF];
    int stateElmo_before[ELMO_DOF];

    bool hommingElmo[ELMO_DOF];
    bool hommingElmo_before[ELMO_DOF];

    int ElmoSafteyMode[ELMO_DOF];

    EtherCAT_Elmo::ElmoGoldDevice::elmo_gold_rx *rxPDO[ELMO_DOF];
    EtherCAT_Elmo::ElmoGoldDevice::elmo_gold_tx *txPDO[ELMO_DOF];

    bool ElmoConnected = false;
    bool ElmoTerminate = false;

    std::vector<int> fz_group1;
    bool fz_group1_check = false;
    std::vector<int> fz_group2;
    bool fz_group2_check = false;
    std::vector<int> fz_group3;
    bool fz_group3_check = false;
    int fz_group = 0;

    bool ConnectionUnstableBeforeStart = false;

    int bootseq = 0;
    //int bootseq
    const int firstbootseq[5] = {0, 33, 35, 8, 64};
    const int secondbootseq[4] = {0, 33, 35, 39};

    bool ecat_connection_ok = false;

    bool ecat_number_ok = false;
    bool ecat_WKC_ok = false;
    bool commutation_check = true;
    bool commutation_ok = false;
    bool commutation_fail = false;

    bool zp_waiting_low_switch = false;
    bool zp_waiting_upper_switch = false;

    bool zp_init_check = true;
    bool zp_low_check = false;
    bool zp_upper_check = false;
    bool zp_ok = false;
    bool zp_fail = false;

    bool zp_load_ok = true;

    bool operation_ready = false;

private:
    DataContainer &dc;

    Eigen::VectorQd getCommand();

    ros::Subscriber gainSubscriber;
    Eigen::VectorQd CustomGain;
    void gainCallbak(const std_msgs::Float32MultiArrayConstPtr &msg);

    double elmoJointMove(double init, double angle, double start_time, double traj_time);

    Eigen::Vector6d RealConstant;

    bool sim_runnung;

    std::string joint_name_mj[MODEL_DOF];
    //ros::Rate rate_;
    int dyn_hz;
};

#endif
