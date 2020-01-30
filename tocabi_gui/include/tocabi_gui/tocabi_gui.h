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
#include <std_msgs/String.h>

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

private:
    //ROS_DEPRECATED virtual QList<QString>

    Ui::TocabiGuiWidget ui_;
    QWidget *widget_;

    ros::NodeHandle nh_;

public:
    ros::Subscriber timesub;
    void timerCallback(const std_msgs::Float32ConstPtr &msg);

    ros::Publisher com_pub;
    std_msgs::String com_msg;
};

} // namespace tocabi_gui

#endif