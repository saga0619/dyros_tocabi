

#include "tocabi_gui/tocabi_gui.h"
#include <ros/master.h>
#include <pluginlib/class_list_macros.h>
#include <QStringList>
#include <iostream>
#include <QString>

namespace tocabi_gui
{

TocabiGui::TocabiGui()
    : rqt_gui_cpp::Plugin(), widget_(0)
{
    setObjectName("TocabiGui");

    //initPlugin()
    timesub = nh_.subscribe("/tocabi/time", 1, &TocabiGui::timerCallback, this);
    com_pub = nh_.advertise<std_msgs::String>("/tocabi/command", 1);
}

void TocabiGui::initPlugin(qt_gui_cpp::PluginContext &context)
{
    widget_ = new QWidget();
    ui_.setupUi(widget_);

    if (context.serialNumber() > 1)
    {
        widget_->setWindowTitle(widget_->windowTitle() + " (" + QString::number(context.serialNumber()) + ")");
    }
    context.addWidget(widget_);

    connect(ui_.torqueon_button, SIGNAL(pressed()), this, SLOT(torqueoncb()));
    connect(ui_.torqueoff_button, SIGNAL(pressed()), this, SLOT(torqueoffcb()));
    connect(ui_.emergencyoff_button, SIGNAL(pressed()), this, SLOT(emergencyoffcb()));
}
void TocabiGui::shutdownPlugin()
{
}

void TocabiGui::saveSettings(qt_gui_cpp::Settings &plugin_settings, qt_gui_cpp::Settings &instance_settings) const
{
}

void TocabiGui::restoreSettings(const qt_gui_cpp::Settings &plugin_settings, const qt_gui_cpp::Settings &instance_settings)
{
}

void TocabiGui::torqueoncb()
{
    com_msg.data = std::string("torqueon");
    com_pub.publish(com_msg);
}

void TocabiGui::torqueoffcb()
{
    com_msg.data = std::string("torqueoff");
    com_pub.publish(com_msg);
}
void TocabiGui::emergencyoffcb()
{
    com_msg.data = std::string("emergencyoff");
    com_pub.publish(com_msg);
}

void TocabiGui::timerCallback(const std_msgs::Float32ConstPtr &msg)
{
    ui_.currenttime->setText(QString::number(msg->data));
}

} // namespace tocabi_gui

PLUGINLIB_EXPORT_CLASS(tocabi_gui::TocabiGui, rqt_gui_cpp::Plugin)