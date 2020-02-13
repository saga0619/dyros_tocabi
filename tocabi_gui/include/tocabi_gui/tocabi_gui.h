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
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/PolygonStamped.h>
#include <std_msgs/String.h>
#include <QMetaType>

#include <QGraphicsRectItem>

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

namespace tocabi_gui
{

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
    virtual void sendtunebtn();
    virtual void resettunebtn();
    virtual void pointcb(const geometry_msgs::PolygonStampedConstPtr &msg);


private:
    //ROS_DEPRECATED virtual QList<QString>

    Ui::TocabiGuiWidget ui_;
    QWidget *widget_;

    std::vector<QLabel *> ecatlabels;
    std::vector<QLineEdit *> ecattexts;

    QGraphicsScene *scene;

    QGraphicsEllipseItem *com_d;
    QGraphicsRectItem *rfoot_d;
    QGraphicsRectItem *lfoot_d;

    
    

    ros::NodeHandle nh_;

public:
    ros::Subscriber timesub;
    void timerCallback(const std_msgs::Float32ConstPtr &msg);

    ros::Subscriber pointsub;

    ros::Subscriber guilogsub;
    ros::Publisher gain_pub;
    std_msgs::Float32MultiArray gain_msg;
    ros::Publisher com_pub;
    std_msgs::String com_msg;

    //void guiLogCallback(const std_msgs::StringConstPtr &msg);
    std::string logtext;

signals:
    void guiLogCallback(const std_msgs::StringConstPtr &msg);
    void pointCallback(const geometry_msgs::PolygonStampedConstPtr &msg);
    virtual void guiLogSignal();
};

} // namespace tocabi_gui
Q_DECLARE_METATYPE(std_msgs::StringConstPtr);
Q_DECLARE_METATYPE(geometry_msgs::PolygonStampedConstPtr);
#endif