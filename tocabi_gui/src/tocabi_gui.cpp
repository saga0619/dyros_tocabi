

#include "tocabi_gui/tocabi_gui.h"
#include <ros/master.h>
#include <pluginlib/class_list_macros.h>
#include <iostream>



namespace tocabi_gui
{

TocabiGui::TocabiGui()
    : rqt_gui_cpp::Plugin(), widget_(0)
{
    setObjectName("TocabiGui");
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

    connect(ui_.torqueon_button,SIGNAL(pressed()),this,SLOT(torqueoncb()));
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
    std::cout<<"hello world"<<std::endl;
}

} // namespace tocabi_gui