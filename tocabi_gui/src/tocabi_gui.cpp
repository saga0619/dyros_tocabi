
#include "tocabi_gui/tocabi_gui.h"
#include <pluginlib/class_list_macros.h>
#include <iostream>
#include <vector>

int elng[33] = {0, 1, 16, 17, 9, 8, 4, 5, 13, 12, 14, 15, 7, 6, 2, 3, 11, 10, 18, 19, 27, 28, 29, 30, 31, 32, 20, 21, 22, 23, 24, 25, 26};

namespace tocabi_gui
{
MyQGraphicsScene::MyQGraphicsScene(QWidget *parent) : QGraphicsScene(parent)
{
}
/*
void MyQGraphicsScene::mousePressEvent(QGraphicsSceneMouseEvent *event)
{
    std::cout << "press" << std::endl;
}
*/
void MyQGraphicsScene::wheelEvent(QGraphicsSceneWheelEvent *event)
{
    //std::cout << "wheel" << event->delta() << std::endl;
}

TocabiGui::TocabiGui()
    : rqt_gui_cpp::Plugin(), widget_(0)
{
    qRegisterMetaType<std_msgs::StringConstPtr>();
    qRegisterMetaType<geometry_msgs::PolygonStampedConstPtr>();
    qRegisterMetaType<std_msgs::Float32ConstPtr>();
    qRegisterMetaType<sensor_msgs::ImuConstPtr>();
    setObjectName("TocabiGui");

    //initPlugin()
    pointsub = nh_.subscribe("/tocabi/point", 1, &TocabiGui::pointCallback, this);
    timesub = nh_.subscribe("/tocabi/time", 1, &TocabiGui::timerCallback, this);
    com_pub = nh_.advertise<std_msgs::String>("/tocabi/command", 1);
    guilogsub = nh_.subscribe("/tocabi/guilog", 1000, &TocabiGui::guiLogCallback, this);
    gain_pub = nh_.advertise<std_msgs::Float32MultiArray>("/tocabi/gain_command", 100);
    imusub = nh_.subscribe("/tocabi/imu", 1, &TocabiGui::imuCallback, this);
    task_pub = nh_.advertise<tocabi_controller::TaskCommand>("/tocabi/taskcommand", 100);

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
    ui_.safetyresetbtn->setShortcut(QKeySequence(Qt::Key_R));
    ui_.emergencyoff_button->setShortcut(QKeySequence(Qt::Key_Escape));

    connect(ui_.ecat_btn, SIGNAL(pressed()), this, SLOT(ecatpbtn()));
    connect(ui_.stat_btn, SIGNAL(pressed()), this, SLOT(statpbtn()));
    connect(ui_.command_btn, SIGNAL(pressed()), this, SLOT(commandpbtn()));
    connect(ui_.mtunebtn, SIGNAL(pressed()), this, SLOT(mtunebtn()));
    connect(ui_.walkingbtn, SIGNAL(pressed()), this, SLOT(walkingbtn()));

    connect(ui_.sendtunebtn, SIGNAL(pressed()), this, SLOT(sendtunebtn()));
    connect(ui_.resettunebtn, SIGNAL(pressed()), this, SLOT(resettunebtn()));
    connect(ui_.ftcalibbtn, SIGNAL(pressed()), this, SLOT(ftcalibbtn()));

    connect(ui_.customtaskgain, SIGNAL(stateChanged(int)), this, SLOT(customtaskgaincb(int)));

    ui_.stackedWidget->setCurrentIndex(0);

    ui_.ecat_btn->setShortcut(QKeySequence(Qt::Key_1));
    ui_.stat_btn->setShortcut(QKeySequence(Qt::Key_2));
    ui_.command_btn->setShortcut(QKeySequence(Qt::Key_3));
    ui_.mtunebtn->setShortcut(QKeySequence(Qt::Key_4));
    ui_.walkingbtn->setShortcut(QKeySequence(Qt::Key_5));

    scene = new MyQGraphicsScene(widget_);
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
    front->setPos(0, 50);

    //ui_.graphicsView->setSceneRect(-210, -260, 421, 521);

    connect(this, &TocabiGui::timerCallback, this, &TocabiGui::timercb);
    connect(this, &TocabiGui::guiLogCallback, this, &TocabiGui::plainTextEditcb);
    connect(this, &TocabiGui::pointCallback, this, &TocabiGui::pointcb);
    connect(this, &TocabiGui::imuCallback, this, &TocabiGui::imucb);

    //connect(ui_)
    connect(ui_.initializebtn, SIGNAL(pressed()), this, SLOT(initializebtncb()));
    connect(ui_.safetyresetbtn, SIGNAL(pressed()), this, SLOT(safetyresetbtncb()));

    connect(ui_.task_send_button, SIGNAL(pressed()), this, SLOT(tasksendcb()));
    connect(ui_.walkinginit_btn, SIGNAL(pressed()), this, SLOT(walkinginitbtncb()));
    connect(ui_.walkingstart_btn, SIGNAL(pressed()), this, SLOT(walkingstartbtncb()));

    connect(ui_.sebutton, SIGNAL(pressed()), this, SLOT(stateestimationcb()));
    connect(ui_.torqueredis, SIGNAL(pressed()), this, SLOT(torquerediscb()));
    connect(ui_.qp2nd, SIGNAL(pressed()), this, SLOT(qp2ndcb()));

    connect(ui_.gravity_button_4, SIGNAL(pressed()), this, SLOT(gravcompcb()));
    connect(ui_.task_button_4, SIGNAL(pressed()), this, SLOT(posconcb()));
    connect(ui_.contact_button_4, SIGNAL(pressed()), this, SLOT(fixedgravcb()));

    //connect(ui_.)

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
    ui_.taskgain->setDisabled(true);  

    //for(int i=0;i<)

    //    label_40 = new QLabel(verticalLayoutWidget);
    //    label_40->setObjectName(QStringLiteral("label_40"));

    //    leftarm_layout->addWidget(label_40);

    //widget_->s
    //connect(ui_.log_btn,SIGNAL(pressed()),this,SLOR(ui_.))
    /*
    line = new QLineSeries();
    chart = new QChart();
    QFont labelsFont;
    labelsFont.setPixelSize(8);

    chart->legend()->hide();
    chart->addSeries(line);
    chart->createDefaultAxes();
    chart->setContentsMargins(-20, -20, -20, -20);
    chart->layout()->setContentsMargins(0, 0, 0, 0);
    chart->setBackgroundRoundness(0);
    chart->axisX()->setLabelsFont(labelsFont);
    chart->axisY()->setLabelsVisible(false);
    chart->axisY()->setLabelsFont(labelsFont);

    line_roll = new QLineSeries();
    chart_roll = new QChart();
    line_pitch = new QLineSeries();
    chart_pitch = new QChart();
    line_yaw = new QLineSeries();
    chart_yaw = new QChart();

    chart_roll->legend()->hide();
    chart_roll->addSeries(line_roll);
    chart_roll->createDefaultAxes();
    chart_roll->setContentsMargins(-20, -20, -20, -20);
    chart_roll->layout()->setContentsMargins(0, 0, 0, 0);
    chart_roll->setBackgroundRoundness(0);
    chart_roll->axisX()->setLabelsFont(labelsFont);
    chart_roll->axisY()->setLabelsFont(labelsFont);

    chart_pitch->legend()->hide();
    chart_pitch->addSeries(line_pitch);
    chart_pitch->createDefaultAxes();
    chart_pitch->setContentsMargins(-20, -20, -20, -20);
    chart_pitch->layout()->setContentsMargins(0, 0, 0, 0);
    chart_pitch->setBackgroundRoundness(0);
    chart_pitch->axisX()->setLabelsFont(labelsFont);
    chart_pitch->axisY()->setLabelsFont(labelsFont);

    chart_yaw->legend()->hide();
    chart_yaw->addSeries(line_yaw);
    chart_yaw->createDefaultAxes();
    chart_yaw->setContentsMargins(-20, -20, -20, -20);
    chart_yaw->layout()->setContentsMargins(0, 0, 0, 0);
    chart_yaw->setBackgroundRoundness(0);
    chart_yaw->axisX()->setLabelsFont(labelsFont);
    chart_yaw->axisY()->setLabelsFont(labelsFont);

    ui_.imu_roll->setChart(chart_roll);
    ui_.imu_roll->setRenderHint(QPainter::Antialiasing);

    ui_.imu_pitch->setChart(chart_pitch);
    ui_.imu_pitch->setRenderHint(QPainter::Antialiasing);

    ui_.imu_yaw->setChart(chart_yaw);
    ui_.imu_yaw->setRenderHint(QPainter::Antialiasing);*/

    //QChartView *chartView = new QChartView(chart, ui_.widget);
    //chartView->setRenderHint(QPainter::Antialiasing);
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

void TocabiGui::customtaskgaincb(int state)
{
    if (ui_.customtaskgain->isChecked())
    {
        ui_.taskgain->setEnabled(true);
    }
    else
    {
        ui_.taskgain->setDisabled(true);
    }
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

void TocabiGui::stateestimationcb()
{
    com_msg.data = std::string("stateestimation");
    com_pub.publish(com_msg);
}
void TocabiGui::torquerediscb()
{
    com_msg.data = std::string("torqueredis");
    com_pub.publish(com_msg);
}
void TocabiGui::qp2ndcb()
{
    com_msg.data = std::string("qp2nd");
    com_pub.publish(com_msg);
}

void TocabiGui::timercb(const std_msgs::Float32ConstPtr &msg)
{
    robot_time = msg->data;
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
void TocabiGui::walkingbtn()
{
    ui_.stackedWidget->setCurrentIndex(4);
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
    else if (msg->data == "initreq")
    {
        ui_.label_ftstatus->setStyleSheet("QLabel { background-color : yellow ; color : black; }");
        ui_.label_ftstatus->setText(QString::fromUtf8("INIT REQ"));
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

    ui_.label_14->setText(QString::number(msg->polygon.points[4].x, 'f', 5));
    ui_.label_15->setText(QString::number(msg->polygon.points[4].y, 'f', 5));
    ui_.label_16->setText(QString::number(msg->polygon.points[4].z, 'f', 5));

    //zmp by ft

    ui_.label_22->setText(QString::number(msg->polygon.points[12].x, 'f', 5));
    ui_.label_23->setText(QString::number(msg->polygon.points[12].y, 'f', 5));
    
    double com_x = msg->polygon.points[0].x;
    double com_y = msg->polygon.points[0].y;

    double left_x = msg->polygon.points[2].x;
    double left_y = msg->polygon.points[2].y;

    double right_x = msg->polygon.points[1].x;
    double right_y = msg->polygon.points[1].y;

    double a, b, c;

    a = (right_y - left_y) / (right_x - left_x);
    b = -1;
    c = -(a * left_x + b * left_y);

    double dis = ((a * com_x + b * com_y + c)) / sqrt(a * a + b * b);

    //com distance from both foot

    dis = msg->polygon.points[0].z;
    ui_.label_3->setText(QString::number(dis, 'f', 5));


    com_x = msg->polygon.points[12].x;
    com_y = msg->polygon.points[12].y;

    a = (right_y - left_y) / (right_x - left_x);
    b = -1;
    c = -(a * left_x + b * left_y);

    dis = ((a * com_x + b * com_y + c)) / sqrt(a * a + b * b);




    ui_.label_42->setText(QString::number(dis, 'f', 5));

    ui_.graphicsView->setSceneRect(0, 0, 0, 0);
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

void TocabiGui::tasksendcb()
{
    task_msg.angle = ui_.com_angle->text().toFloat();
    task_msg.ratio = ui_.com_pos->text().toFloat();
    task_msg.height = ui_.com_height->text().toFloat();


    task_msg.l_x = ui_.text_l_x->text().toFloat();
    task_msg.l_y = ui_.text_l_y->text().toFloat();
    task_msg.l_z = ui_.text_l_z->text().toFloat();
    task_msg.l_roll = ui_.text_l_roll->text().toFloat();
    task_msg.l_pitch = ui_.text_l_pitch->text().toFloat();
    task_msg.l_yaw = ui_.text_l_yaw->text().toFloat();

    task_msg.r_x = ui_.text_r_x->text().toFloat();
    task_msg.r_y = ui_.text_r_y->text().toFloat();
    task_msg.r_z = ui_.text_r_z->text().toFloat();
    task_msg.r_roll = ui_.text_r_roll->text().toFloat();
    task_msg.r_pitch = ui_.text_r_pitch->text().toFloat();
    task_msg.r_yaw = ui_.text_r_yaw->text().toFloat();

    task_msg.time = ui_.text_traj_time->text().toFloat();
    task_msg.mode = ui_.task_mode->currentIndex();

    task_msg.customTaskGain = ui_.customtaskgain->isChecked();
    if (task_msg.customTaskGain)
    {
        task_msg.pos_p = ui_.pospgain->text().toFloat();
        task_msg.pos_d = ui_.posdgain->text().toFloat();
        task_msg.ang_p = ui_.angpgain->text().toFloat();
        task_msg.ang_d = ui_.angdgain->text().toFloat();
    }

    task_pub.publish(task_msg);

    ui_.text_l_x->setText(QString::number(0.0, 'f', 1));
    ui_.text_l_y->setText(QString::number(0.0, 'f', 1));
    ui_.text_l_z->setText(QString::number(0.0, 'f', 1));
    ui_.text_l_roll->setText(QString::number(0.0, 'f', 1));
    ui_.text_l_pitch->setText(QString::number(0.0, 'f', 1));
    ui_.text_l_yaw->setText(QString::number(0.0, 'f', 1));

    ui_.text_r_x->setText(QString::number(0.0, 'f', 1));
    ui_.text_r_y->setText(QString::number(0.0, 'f', 1));
    ui_.text_r_z->setText(QString::number(0.0, 'f', 1));
    ui_.text_r_roll->setText(QString::number(0.0, 'f', 1));
    ui_.text_r_pitch->setText(QString::number(0.0, 'f', 1));
    ui_.text_r_yaw->setText(QString::number(0.0, 'f', 1));
}


void TocabiGui::imucb(const sensor_msgs::ImuConstPtr &msg)
{ /*
    //std::cout<<robot_time<<"msg->linacc"<<msg->linear_acceleration.x<<std::endl;
    line_roll->append(robot_time, msg->linear_acceleration.x);
    chart_roll->axisY()->setRange(-5, 5);
    chart_roll->axisX()->setRange(robot_time - 5, robot_time);
    if (line_roll->at(0).x() < (robot_time - 5))
        line_roll->remove(0);

    line_pitch->append(robot_time, msg->linear_acceleration.y);
    chart_pitch->axisY()->setRange(-5, 5);
    chart_pitch->axisX()->setRange(robot_time - 5, robot_time);
    if (line_pitch->at(0).x() < (robot_time - 5))
        line_pitch->remove(0);

    line_yaw->append(robot_time, msg->linear_acceleration.z);
    chart_yaw->axisY()->setRange(-5, 5);
    chart_yaw->axisX()->setRange(robot_time - 5, robot_time);
    if (line_yaw->at(0).x() < (robot_time - 5))
        line_yaw->remove(0);*/
}

void TocabiGui::walkinginitbtncb()
{   
    task_msg.walking_enable = 2.0;
    task_msg.ik_mode = ui_.ik_mode->currentIndex();
    
    if(ui_.walking_pattern->currentIndex() == 0)
    {
        task_msg.pattern = 0;
    }
    else if(ui_.walking_pattern->currentIndex() == 1)
    {
        task_msg.pattern = 1;
    }
    else
    {  
        task_msg.pattern = 2;
    }

    if(ui_.checkBox_dob->isChecked() == true)
    {
        task_msg.dob = true;
    }
    else
    {
        task_msg.dob = false;
    }
    
    task_msg.first_foot_step = ui_.step_mode->currentIndex();
    
    task_msg.x = ui_.text_walking_x->text().toFloat();
    task_msg.y = ui_.text_walking_y->text().toFloat();
    task_msg.z = ui_.text_walking_z->text().toFloat();
    task_msg.walking_height=ui_.text_walking_height->text().toFloat();
    task_msg.theta=ui_.text_walking_theta->text().toFloat();
    task_msg.step_length_x=ui_.text_walking_steplengthx->text().toFloat();
    task_msg.step_length_y=ui_.text_walking_steplengthy->text().toFloat();

    task_pub.publish(task_msg);
}

void TocabiGui::walkingstartbtncb()
{   
    task_msg.walking_enable = 1.0;
    task_msg.ik_mode = ui_.ik_mode->currentIndex();
    
    if(ui_.walking_pattern->currentIndex() == 0)
    {
        task_msg.pattern = 0;
    }
    else if(ui_.walking_pattern->currentIndex() == 1)
    {
        task_msg.pattern = 1;
    }
    else
    {  
        task_msg.pattern = 2;
    }

    if(ui_.checkBox_dob->isChecked() == true)
    {
        task_msg.dob = true;
    }
    else
    {
        task_msg.dob = false;
    }
    
    task_msg.first_foot_step = ui_.step_mode->currentIndex();
    
    task_msg.x = ui_.text_walking_x->text().toFloat();
    task_msg.y = ui_.text_walking_y->text().toFloat();
    task_msg.z = ui_.text_walking_z->text().toFloat();
    task_msg.walking_height=ui_.text_walking_height->text().toFloat();
    task_msg.theta=ui_.text_walking_theta->text().toFloat();
    task_msg.step_length_x=ui_.text_walking_steplengthx->text().toFloat();
    task_msg.step_length_y=ui_.text_walking_steplengthy->text().toFloat();

    task_pub.publish(task_msg);
}


/*
void TocabiGui::wheelEvent(QWheelEvent *event)
{
    std::cout << "wheel event" << std::endl;
}*/

void TocabiGui::fixedgravcb()
{
    com_msg.data = std::string("fixedgravity");
    com_pub.publish(com_msg);
}

void TocabiGui::gravcompcb()
{
    com_msg.data = std::string("gravity");
    com_pub.publish(com_msg);
}

void TocabiGui::posconcb()
{
    com_msg.data = std::string("positioncontrol");
    com_pub.publish(com_msg);
}

} // namespace tocabi_gui

PLUGINLIB_EXPORT_CLASS(tocabi_gui::TocabiGui, rqt_gui_cpp::Plugin)
