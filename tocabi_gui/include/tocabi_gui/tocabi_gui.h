#ifndef tocabi_gui__TocabiGUI_H
#define tocabi_gui__TocabiGUI_H


#include <rqt_gui_cpp/plugin.h>
#include <ui_tocabi_gui.h>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <ros/macros.h>

#include <QList>
#include <QString>
#include <QSize>
#include <QWidget>
#include <QObject>

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

protected:
    //ROS_DEPRECATED virtual QList<QString>

    Ui::TocabiGuiWidget ui_;

    QWidget* widget_;

};

} // namespace tocabi_gui

#endif