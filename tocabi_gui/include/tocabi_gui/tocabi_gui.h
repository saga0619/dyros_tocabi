#ifndef tocabi_gui__TocabiGUI_H
#define tocabi_gui__TocabiGUI_H

#include <rqt_gui_cpp/plugin.h>
#include <tocabi_gui/ui_tocabi_gui.h>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <ros/macros.h>

#include <QList>
#include <QString>
#include <QSize>
#include <QWidget>
#include <QObject>
#include <QStateMachine>
#include <QState>
#include <QEventTransition>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Int8.h>
#include <geometry_msgs/PolygonStamped.h>
#include <std_msgs/String.h>
#include <QMetaType>

#include <QGraphicsRectItem>
#include <QGraphicsSceneWheelEvent>

#include <QStringListModel>

#include "tocabi_controller/TaskCommand.h"
#include "tocabi_controller/TaskCommandQue.h"
#include "tocabi_controller/TaskGainCommand.h"
#include "tocabi_controller/VelocityCommand.h"
#include "tocabi_controller/positionCommand.h"

const double NM2CNT[33] =
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
        12.33,
        3.3, //upperbody
        5.8, //leftLeg
        4.3,
        3.8,
        3.46,
        3.52,
        12.33};

//leftleg rightleg waist leftarm head rightarm 


const double posStandard[33] = {0.0, 0.0, -0.24, 0.6, -0.36, 0.0,
								0.0, 0.0, -0.24, 0.6, -0.36, 0.0,
								0, 0, 0, 
								0.3, 0.3, 1.5, -1.27, -1, 0, -1, 0,
                                0, 0,
								-0.3, -0.3, -1.5, 1.27, 1, 0, 1, 0};
const double posStandard3[33] = {0.0, 0.0, -0.93, 1.24, -0.5, 0.0, 
								0.0, 0.0, -0.93, 1.24, -0.5, 0.0,
								0, 0.44, 0, 
								-0.39, -1.205, 1, -0.375, -1.18, 2.31, -1.176, -0.178,
                                0, 0,
								0.39, 1.205, -1, 0.375, 1.18, -2.31, 1.176, 0.178};
const double posStandard2[33] = {0.01920543546875, 0.014871940644527501, -0.9358508043751563, 1.338121842500375, -0.5674653948051875, 0.050828442832027504, 
0.0705680103, -0.06239465556640125, -0.8990736226364687, 1.1964587462107499, -0.4385266665035625, 0.0016720387109400003, 
0.00085875627915, 0.4494140928808593, 0.04510635186519375, 
-0.5609804923725624, -1.1437550122563438, 0.9443183765642187, -0.34260309411968737, -1.0029662848342813, 2.2997703634082023, -1.0719877360214063, 0.1035897010547, 
0.0015608251269812537, 0.014461600869112495, 
0.5184163693160313, 1.1251018097560312, -0.931678377501875, 0.36133671315124993, 1.0657751151269375, -2.643443347626953, 1.121776907089875, 0.23148915758789498};


struct task_que
{
    std::string task_title;
    tocabi_controller::TaskCommand tc_;
};

namespace tocabi_gui
{

class MyQGraphicsScene : public QGraphicsScene
{
    Q_OBJECT
public:
    explicit MyQGraphicsScene(QWidget *parent = 0);
    virtual void wheelEvent(QGraphicsSceneWheelEvent *event);
    //virtual void mousePressEvent(QGraphicsSceneMouseEvent *mouseEvent);

public slots:

private:
};

class MyQGraphicsView : public QGraphicsView
{
    Q_OBJECT
public:
    explicit MyQGraphicsView(QWidget *parent = 0);
    //virtual void wheelEvent(QGraphicsViewWheelEvent *event);
    //virtual void mousePressEvent(QGraphicsSceneMouseEvent *mouseEvent);

public slots:

private:
};

class TocabiGui : public rqt_gui_cpp::Plugin
{
    Q_OBJECT
public:
    TocabiGui();

    virtual void initPlugin(qt_gui_cpp::PluginContext &context);
    virtual void shutdownPlugin();
    virtual void saveSettings(qt_gui_cpp::Settings &plugin_settings, qt_gui_cpp::Settings &instance_settings) const;
    virtual void restoreSettings(const qt_gui_cpp::Settings &plugin_settings, const qt_gui_cpp::Settings &instance_settings);

protected slots:
    //virtual void updateTopicList();
    virtual void torqueoncb();
    virtual void torqueoffcb();
    virtual void emergencyoffcb();
    //virtual void logbtn();
    virtual void plainTextEditcb(const std_msgs::StringConstPtr &msg);
    virtual void ecatpbtn();
    virtual void statpbtn();
    virtual void commandpbtn();
    virtual void initializebtncb();
    virtual void safetyresetbtncb();
    virtual void mtunebtn();
    virtual void walkinginitbtncb();
    virtual void walkingstartbtncb();
    virtual void walkingbtn();
    virtual void dgbtn();
    virtual void sendtunebtn();
    virtual void resettunebtn();
    virtual void pointcb(const geometry_msgs::PolygonStampedConstPtr &msg);
    virtual void imucb(const sensor_msgs::ImuConstPtr &msg);
    virtual void timercb(const std_msgs::Float32ConstPtr &msg);
    virtual void ftcalibbtn();
    virtual void tasksendcb();
    virtual void stateestimationcb();
    virtual void torquerediscb();
    virtual void qp2ndcb();
    virtual void customtaskgaincb(int state);
    virtual void fixedgravcb();
    virtual void gravcompcb();
    virtual void posconcb();
    virtual void posgravconcb();
    virtual void dshowbtn();
    virtual void ecatinitlow();
    virtual void safety2btncb();
    virtual void que_downbtn();
    virtual void que_upbtn();
    virtual void que_deletebtn();
    virtual void que_resetbtn();
    virtual void que_sendbtn();
    virtual void que_addquebtn();
    virtual void shutdown_robot();
    virtual void sysstatecb(const std_msgs::Int32MultiArrayConstPtr &msg);
    virtual void solvermode_cb(int state);
    virtual void inityaw();
    virtual void simvj();
    virtual void igimubtn();
    virtual void imureset();
    virtual void printdata();
    virtual void enablelpf();
    virtual void sendtaskgaincommand();
    virtual void resettaskgaincommand();
    virtual void sebyftbtn();
    virtual void slidervelcommand();
    virtual void sliderrel1();
    virtual void sliderrel2();
    virtual void sliderrel3();
    virtual void disablelower();
    virtual void positionCommand();
    virtual void positionPreset1();
    virtual void positionPreset2();

    virtual void taskmodecb(int index);

    //dg
    virtual void walkingspeedcb(int value);
    virtual void walkingdurationcb(int value);
    virtual void walkingangvelcb(int value);
    virtual void kneetargetanglecb(int value);
    virtual void footheightcb(int value);
    virtual void vr_eye_depth_cb(int value);
    virtual void vr_eye_distance_cb(int value);

    virtual void sendupperbodymodecb();
    virtual void sendnextswinglegcb();

    virtual void sendcomposgaincb();
    virtual void sendpelvorigaincb();
    virtual void sendsupportfootdampinggaincb();
    virtual void sendleggaincb();
    virtual void sendalphaxcb();
    virtual void sendalphaycb();
    virtual void sendstepwidthcb();
    virtual void sendtest1cb();
    virtual void sendtest2cb();

    virtual void sendarmgaincb();
    virtual void sendwaistgaincb();

    virtual void sendstillposecalibration();
    virtual void sendtposecalibration();
    virtual void sendforwardposecalibration();
    virtual void sendresetposecalibration();
    virtual void sendloadsavedcalibration();


    virtual void torqueCommand();
    void handletaskmsg();

private:
    //ROS_DEPRECATED virtual QList<QString>
    std::vector<task_que> tq_;

    Ui::TocabiGuiWidget ui_;
    QWidget *widget_;

    //QStringListModel *model;
    //QStringList list;

    std::vector<QLabel *> ecatlabels;
    std::vector<QLabel *> safetylabels;
    std::vector<QLineEdit *> ecattexts;
    MyQGraphicsScene *scene;
    MyQGraphicsView *view;

    QGraphicsEllipseItem *com_d;
    QGraphicsRectItem *rfoot_d;
    QGraphicsLineItem *rfoot_l1;
    QGraphicsLineItem *rfoot_l2;
    QGraphicsRectItem *lfoot_d;
    QGraphicsLineItem *lfoot_l1;
    QGraphicsLineItem *lfoot_l2;
    QGraphicsRectItem *Pelv;
    QGraphicsEllipseItem *zmp;

    QGraphicsEllipseItem *rfoot_c;
    QGraphicsEllipseItem *lfoot_c;

    double robot_time;

    ros::NodeHandle nh_;

public:
    ros::Subscriber timesub;
    ros::Subscriber jointsub;
    ros::Subscriber pointsub;
    ros::Subscriber guilogsub;
    ros::Publisher gain_pub;
    std_msgs::Float32MultiArray gain_msg;
    ros::Publisher com_pub;
    std_msgs::String com_msg;

    ros::Publisher poscom_pub;
    tocabi_controller::positionCommand poscom_msg;

    ros::Publisher task_pub;
    tocabi_controller::TaskCommand task_msg;

    ros::Publisher task_que_pub;
    tocabi_controller::TaskCommandQue task_que_msg;

    ros::Publisher taskgain_pub;
    tocabi_controller::TaskGainCommand taskgain_msg;

    ros::Publisher velcommand_pub;
    tocabi_controller::VelocityCommand velcmd_msg;

    //dg
    ros::Publisher walkingslidercommand_pub;
    std_msgs::Float32MultiArray walkingslidercommand_msg;

    ros::Publisher upperbodymode_pub;
    std_msgs::Float32 upperbodymode_msg;
    ros::Publisher nextswingleg_pub;
    std_msgs::Float32 nextswingleg_msg;

    ros::Publisher com_walking_pd_gain_pub;
    std_msgs::Float32MultiArray com_walking_pd_gain_msg;
    ros::Publisher pelv_ori_pd_gain_pub;
    std_msgs::Float32MultiArray pelv_ori_pd_gain_msg;
    ros::Publisher support_foot_damping_gain_pub;
    std_msgs::Float32MultiArray support_foot_damping_gain_msg;
    ros::Publisher dg_leg_pd_gain_pub;
    std_msgs::Float32MultiArray dg_leg_pd_gain_msg;
    
    ros::Publisher alpha_x_pub;
    std_msgs::Float32 alpha_x_msg;
    ros::Publisher alpha_y_pub;
    std_msgs::Float32 alpha_y_msg;
    ros::Publisher step_width_pub;
    std_msgs::Float32 step_width_msg;
    
    ros::Publisher test1_pub;
    std_msgs::Float32 test1_msg;
    ros::Publisher test2_pub;
    std_msgs::Float32 test2_msg;

    ros::Publisher arm_pd_gain_pub;
    std_msgs::Float32MultiArray arm_pd_gain_msg;
    ros::Publisher waist_pd_gain_pub;
    std_msgs::Float32MultiArray waist_pd_gain_msg;

    ros::Publisher pose_calibration_pub;
    std_msgs::Int8 pose_calibration_msg;

    ros::Subscriber sysstatesub;

    ros::Subscriber imusub;

    ros::Publisher vr_slider_pub;
    std_msgs::Float32MultiArray vr_slider_msg;

    //void guiLogCallback(const std_msgs::StringConstPtr &msg);
    std::string logtext;

    double com_height = 0;

signals:
    void guiLogCallback(const std_msgs::StringConstPtr &msg);
    void pointCallback(const geometry_msgs::PolygonStampedConstPtr &msg);
    void timerCallback(const std_msgs::Float32ConstPtr &msg);
    void imuCallback(const sensor_msgs::ImuConstPtr &msg);
    void sysstateCallback(const std_msgs::Int32MultiArrayConstPtr &msg);
    virtual void guiLogSignal();
};

} // namespace tocabi_gui
Q_DECLARE_METATYPE(std_msgs::StringConstPtr);
Q_DECLARE_METATYPE(geometry_msgs::PolygonStampedConstPtr);
Q_DECLARE_METATYPE(std_msgs::Float32ConstPtr);
Q_DECLARE_METATYPE(sensor_msgs::ImuConstPtr);
Q_DECLARE_METATYPE(std_msgs::Int32MultiArrayConstPtr);

#endif
