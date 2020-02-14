

#include "tocabi_gui/tocabi_gui.h"
#include <pluginlib/class_list_macros.h>
#include <iostream>
#include <vector>

int elng[33] = {0, 1, 16, 17, 9, 8, 4, 5, 13, 12, 14, 15, 7, 6, 2, 3, 11, 10, 18, 19, 27, 28, 29, 30, 31, 32, 20, 21, 22, 23, 24, 25, 26};

namespace tocabi_gui
{

TocabiGui::TocabiGui()
    : rqt_gui_cpp::Plugin(), widget_(0)
{
    qRegisterMetaType<std_msgs::StringConstPtr>();
    qRegisterMetaType<geometry_msgs::PolygonStampedConstPtr>();
    setObjectName("TocabiGui");

    //initPlugin()
    pointsub = nh_.subscribe("/tocabi/point", 1, &TocabiGui::pointCallback, this);
    timesub = nh_.subscribe("/tocabi/time", 1, &TocabiGui::timerCallback, this);
    com_pub = nh_.advertise<std_msgs::String>("/tocabi/command", 1);
    guilogsub = nh_.subscribe("/tocabi/guilog", 1000, &TocabiGui::guiLogCallback, this);
    gain_pub = nh_.advertise<std_msgs::Float32MultiArray>("/tocabi/gain_command", 100);

    gain_msg.data.resize(33);
    //ecatlabels = {ui_.}
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

    std::string mode;
    nh_.param<std::string>("/tocabi_controller/run_mode", mode, "default");

    connect(ui_.torqueon_button, SIGNAL(pressed()), this, SLOT(torqueoncb()));
    connect(ui_.torqueoff_button, SIGNAL(pressed()), this, SLOT(torqueoffcb()));
    connect(ui_.emergencyoff_button, SIGNAL(pressed()), this, SLOT(emergencyoffcb()));

    ui_.torqueon_button->setShortcut(QKeySequence(Qt::Key_E));
    ui_.torqueoff_button->setShortcut(QKeySequence(Qt::Key_C));
    ui_.emergencyoff_button->setShortcut(QKeySequence(Qt::Key_Escape));

    connect(ui_.ecat_btn, SIGNAL(pressed()), this, SLOT(ecatpbtn()));
    connect(ui_.stat_btn, SIGNAL(pressed()), this, SLOT(statpbtn()));
    connect(ui_.command_btn, SIGNAL(pressed()), this, SLOT(commandpbtn()));
    connect(ui_.mtunebtn, SIGNAL(pressed()), this, SLOT(mtunebtn()));

    connect(ui_.sendtunebtn, SIGNAL(pressed()), this, SLOT(sendtunebtn()));
    connect(ui_.resettunebtn, SIGNAL(pressed()), this, SLOT(resettunebtn()));
    connect(ui_.ftcalibbtn, SIGNAL(pressed()), this, SLOT(ftcalibbtn()));

    ui_.stackedWidget->setCurrentIndex(0);

    ui_.ecat_btn->setShortcut(QKeySequence(Qt::Key_1));
    ui_.stat_btn->setShortcut(QKeySequence(Qt::Key_2));
    ui_.command_btn->setShortcut(QKeySequence(Qt::Key_3));
    ui_.mtunebtn->setShortcut(QKeySequence(Qt::Key_4));

    scene = new QGraphicsScene(this);
    ui_.graphicsView->setScene(scene);
    QBrush redbrush(Qt::red);
    QPen blackpen(Qt::black);

    lfoot_d = new QGraphicsRectItem(QRectF(-85 / 4, -120 / 4, 170 / 4, 300 / 4));
    scene->addItem(lfoot_d);

    rfoot_d = new QGraphicsRectItem(QRectF(-85 / 4, -120 / 4, 170 / 4, 300 / 4));
    scene->addItem(rfoot_d);

    com_d = scene->addEllipse(-10, -10, 20, 20, blackpen, redbrush);
    rfoot_c = scene->addEllipse(-5, -5, 10, 10, blackpen, redbrush);
    lfoot_c = scene->addEllipse(-5, -5, 10, 10, blackpen, redbrush);

    scene->addLine(-20, 0, 40, 0, blackpen);
    scene->addLine(0, -20, 0, 40, blackpen);

    QGraphicsTextItem *front = scene->addText("front");
    front->setPos(0, 100);

    connect(this, &TocabiGui::guiLogCallback, this, &TocabiGui::plainTextEditcb);
    connect(this, &TocabiGui::pointCallback, this, &TocabiGui::pointcb);

    //connect(ui_)
    connect(ui_.initializebtn, SIGNAL(pressed()), this, SLOT(initializebtncb()));
    connect(ui_.safetyresetbtn, SIGNAL(pressed()), this, SLOT(safetyresetbtncb()));

    if (mode == "simulation")
    {
        ui_.label_zpstatus->setStyleSheet("QLabel { background-color : yellow; color : black; }");
        ui_.label_imustatus->setStyleSheet("QLabel { background-color : yellow; color : black; }");
        ui_.label_ecatstatus->setStyleSheet("QLabel { background-color : yellow; color : black; }");
        ui_.label_ftstatus->setStyleSheet("QLabel { background-color : yellow; color : black; }");

        ui_.label_zpstatus->setText(QString::fromUtf8("SIM MODE"));
        ui_.label_imustatus->setText(QString::fromUtf8("SIM MODE"));
        ui_.label_ecatstatus->setText(QString::fromUtf8("SIM MODE"));
        ui_.label_ftstatus->setText(QString::fromUtf8("SIM MODE"));
    }
    else
    {
        ui_.label_zpstatus->setStyleSheet("QLabel { background-color : red; color : white; }");
        ui_.label_imustatus->setStyleSheet("QLabel { background-color : red; color : white; }");
        ui_.label_ecatstatus->setStyleSheet("QLabel { background-color : red; color : white; }");
        ui_.label_ftstatus->setStyleSheet("QLabel { background-color : red; color : white; }");
    }

    ecatlabels.resize(33);
    //head
    for (int i = 0; i < 2; i++)
    {
        ecatlabels[i] = new QLabel(ui_.head_layout->parentWidget());
        ui_.head_layout->addWidget(ecatlabels[i]);
        ecatlabels[i]->setFrameShape(QFrame::Panel);
    }

    for (int i = 2; i < 10; i++)
    {
        ecatlabels[i] = new QLabel(ui_.leftarm_layout->parentWidget());
        ui_.leftarm_layout->addWidget(ecatlabels[i]);
        ecatlabels[i]->setFrameShape(QFrame::Panel);
    }

    for (int i = 10; i < 18; i++)
    {
        ecatlabels[i] = new QLabel(ui_.rightarm_layout->parentWidget());
        ui_.rightarm_layout->addWidget(ecatlabels[i]);
        ecatlabels[i]->setFrameShape(QFrame::Panel);
    }

    for (int i = 18; i < 21; i++)
    {
        ecatlabels[i] = new QLabel(ui_.waist_layout->parentWidget());
        ui_.waist_layout->addWidget(ecatlabels[i]);
        ecatlabels[i]->setFrameShape(QFrame::Panel);
    }

    for (int i = 21; i < 27; i++)
    {
        ecatlabels[i] = new QLabel(ui_.leftleg_layout->parentWidget());
        ui_.leftleg_layout->addWidget(ecatlabels[i]);
        ecatlabels[i]->setFrameShape(QFrame::Panel);
    }

    for (int i = 27; i < 33; i++)
    {
        ecatlabels[i] = new QLabel(ui_.rightleg_layout->parentWidget());
        ui_.rightleg_layout->addWidget(ecatlabels[i]);
        ecatlabels[i]->setFrameShape(QFrame::Panel);
    }

    //ecat constant tune
    ecattexts.resize(33);
    for (int i = 0; i < 2; i++)
    {
        ecattexts[i] = new QLineEdit(ui_.head_layout_2->parentWidget());
        ui_.head_layout_2->addWidget(ecattexts[i]);
    }
    for (int i = 2; i < 10; i++)
    {
        ecattexts[i] = new QLineEdit(ui_.leftarm_layout_2->parentWidget());
        ui_.leftarm_layout_2->addWidget(ecattexts[i]);
    }
    for (int i = 10; i < 18; i++)
    {
        ecattexts[i] = new QLineEdit(ui_.rightarm_layout_2->parentWidget());
        ui_.rightarm_layout_2->addWidget(ecattexts[i]);
    }
    for (int i = 18; i < 21; i++)
    {
        ecattexts[i] = new QLineEdit(ui_.waist_layout_2->parentWidget());
        ui_.waist_layout_2->addWidget(ecattexts[i]);
    }
    for (int i = 21; i < 27; i++)
    {
        ecattexts[i] = new QLineEdit(ui_.leftleg_layout_2->parentWidget());
        ui_.leftleg_layout_2->addWidget(ecattexts[i]);
    }
    for (int i = 27; i < 33; i++)
    {
        ecattexts[i] = new QLineEdit(ui_.rightleg_layout_2->parentWidget());
        ui_.rightleg_layout_2->addWidget(ecattexts[i]);
    }

    for (int i = 0; i < 33; i++)
    {
        ecatlabels[i]->setAlignment(Qt::AlignCenter);
        ecattexts[i]->setText(QString::fromUtf8("0.0"));
        ecattexts[i]->setValidator(new QDoubleValidator(0, 1000, 3, this));
    }

    //for(int i=0;i<)

    //    label_40 = new QLabel(verticalLayoutWidget);
    //    label_40->setObjectName(QStringLiteral("label_40"));

    //    leftarm_layout->addWidget(label_40);

    //widget_->s
    //connect(ui_.log_btn,SIGNAL(pressed()),this,SLOR(ui_.))
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
    ui_.currenttime->setText(QString::number(msg->data, 'f', 3));
}

void TocabiGui::ecatpbtn()
{
    ui_.stackedWidget->setCurrentIndex(0);
}
void TocabiGui::statpbtn()
{
    ui_.stackedWidget->setCurrentIndex(1);
}
void TocabiGui::commandpbtn()
{
    ui_.stackedWidget->setCurrentIndex(2);
}
void TocabiGui::mtunebtn()
{
    ui_.stackedWidget->setCurrentIndex(3);
}

/*
void TocabiGui::guiLogCallback(const std_msgs::StringConstPtr &msg)
{
    guiLogSignal();
    logtext = msg->data;
}*/

void TocabiGui::plainTextEditcb(const std_msgs::StringConstPtr &msg)
{
    //std::cout << msg->data << std::endl;
    std::string rcv_msg;
    rcv_msg = msg->data;
    std::string word;
    std::vector<std::string> words;
    for (auto x : rcv_msg)
    {
        if (x == ' ')
        {
            words.push_back(word);
            word.erase();
        }
        else
            word = word + x;
    }
    words.push_back(word);
    if (words[0] == "jointzp")
    {
        int num = elng[atoi(words[1].c_str())];
        if (atoi(words[2].c_str()) == 0) //zp started
        {
            ecatlabels[num]->setText(QString::fromUtf8("zp"));
            ecatlabels[num]->setStyleSheet("QLabel { background-color : yellow ; color : black; }");
        }
        else if (atoi(words[2].c_str()) == 1) //zp success
        {
            ecatlabels[num]->setText(QString::fromUtf8("ok"));
            ecatlabels[num]->setStyleSheet("QLabel { background-color : rgb(138, 226, 52) ; color : black; }");
        }
        else if (atoi(words[2].c_str()) == 2) //zp manual
        {
            ecatlabels[num]->setText(QString::fromUtf8("manual"));
            ecatlabels[num]->setStyleSheet("QLabel { background-color : orange ; color : black; }");
        }
        else if (atoi(words[2].c_str()) == 3) //failed
        {
            ecatlabels[num]->setText(QString::fromUtf8("fail"));
            ecatlabels[num]->setStyleSheet("QLabel { background-color : red ; color : white; }");
        }
    }
    else if (msg->data == "imuvalid")
    {
        ui_.label_imustatus->setStyleSheet("QLabel { background-color : rgb(138, 226, 52) ; color : black; }");
        ui_.label_imustatus->setText(QString::fromUtf8("OK"));
    }
    else if (msg->data == "imunotvalid")
    {
        ui_.label_imustatus->setStyleSheet("QLabel { background-color : red; color : white; }");
        ui_.label_imustatus->setText(QString::fromUtf8("NOT OK"));
    }
    else if (msg->data == "zpgood")
    {
        ui_.label_zpstatus->setStyleSheet("QLabel { background-color : rgb(138, 226, 52) ; color : black; }");
        ui_.label_zpstatus->setText(QString::fromUtf8("OK"));
    }
    else if (msg->data == "ftgood")
    {
        ui_.label_ftstatus->setStyleSheet("QLabel { background-color : rgb(138, 226, 52) ; color : black; }");
        ui_.label_ftstatus->setText(QString::fromUtf8("OK"));
    }
    else if (msg->data == "ecatgood")
    {
        ui_.label_ecatstatus->setStyleSheet("QLabel { background-color : rgb(138, 226, 52) ; color : black; }");
        ui_.label_ecatstatus->setText(QString::fromUtf8("OK"));
    }
    else if (msg->data == "zpnotgood")
    {
        ui_.label_zpstatus->setStyleSheet("QLabel { background-color : red; color : white; }");
        ui_.label_zpstatus->setText(QString::fromUtf8("NOT OK"));
    }
    else if (msg->data == "ecatcommutationdone")
    {
        ui_.label_zpstatus->setStyleSheet("QLabel { background-color : rgb(138, 226, 52) ; color : black; }");
        ui_.label_zpstatus->setText(QString::fromUtf8("OK"));
    }
    else if (msg->data == "ecatcommutation")
    {
        ui_.label_zpstatus->setStyleSheet("QLabel { background-color : red; color : white; }");
        ui_.label_zpstatus->setText(QString::fromUtf8("COMMUTATION"));
    }
    else
    {
        ui_.plainTextEdit->appendPlainText(QString::fromStdString(msg->data));
    }
}

void TocabiGui::pointcb(const geometry_msgs::PolygonStampedConstPtr &msg)
{
    //msg->polygon.points[0].x;
    //msg->polygon.points[0].y;

    //std::cout<<msg->polygon.points[0].x*25<<std::endl;

    com_d->setPos(QPointF(msg->polygon.points[0].y * 250, msg->polygon.points[0].x * 250));

    ui_.label->setText(QString::number(msg->polygon.points[0].x, 'f', 5));
    ui_.label_2->setText(QString::number(msg->polygon.points[0].y, 'f', 5));

    rfoot_d->setPos(QPointF(msg->polygon.points[1].y * 250, msg->polygon.points[1].x * 250));
    rfoot_d->setRotation(msg->polygon.points[1].z * -180.0 / 3.141592);
    rfoot_c->setPos(QPointF(msg->polygon.points[1].y * 250, msg->polygon.points[1].x * 250));

    ui_.label_73->setText(QString::number(msg->polygon.points[1].x, 'f', 5));
    ui_.label_74->setText(QString::number(msg->polygon.points[1].y, 'f', 5));

    lfoot_d->setPos(QPointF(msg->polygon.points[2].y * 250, msg->polygon.points[2].x * 250));
    lfoot_d->setRotation(msg->polygon.points[2].z * -180.0 / 3.141592);
    lfoot_c->setPos(QPointF(msg->polygon.points[2].y * 250, msg->polygon.points[2].x * 250));

    ui_.label_64->setText(QString::number(msg->polygon.points[2].x, 'f', 5));
    ui_.label_65->setText(QString::number(msg->polygon.points[2].y, 'f', 5));

    //zmp by ft
}

void TocabiGui::initializebtncb()
{
    com_msg.data = std::string("ecatinit");
    com_pub.publish(com_msg);
}

void TocabiGui::safetyresetbtncb()
{
    com_msg.data = std::string("safetyreset");
    com_pub.publish(com_msg);
}

void TocabiGui::sendtunebtn()
{
    for (int i = 0; i < 33; i++)
    {
        gain_msg.data[i] = ecattexts[elng[i]]->text().toFloat();
    }
    gain_pub.publish(gain_msg);
}

void TocabiGui::resettunebtn()
{
    for (int i = 0; i < 33; i++)
    {
        ecattexts[elng[i]]->setText(QString::number(NM2CNT[i], 'f', 3));
    }
}

void TocabiGui::ftcalibbtn()
{
    com_msg.data = std::string("ftcalib");
    com_pub.publish(com_msg);
}

} // namespace tocabi_gui

PLUGINLIB_EXPORT_CLASS(tocabi_gui::TocabiGui, rqt_gui_cpp::Plugin)