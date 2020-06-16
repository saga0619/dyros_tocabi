
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

MyQGraphicsView::MyQGraphicsView(QWidget *parent) : QGraphicsView(parent)
{
}
/*
void MyQGraphicsScene::mousePressEvent(QGraphicsSceneMouseEvent *event)
{
    std::cout << "press" << std::endl;
}*/
void MyQGraphicsScene::wheelEvent(QGraphicsSceneWheelEvent *event)
{
    //std::cout << parent()->findChild<QObject *>("graphicsViewCustom")->objectName().toStdString() << std::endl;

    QGraphicsView *view_ = parent()->findChild<QGraphicsView *>("graphicsViewCustom");

    //view_->setViewport();

    double scaleFactor = 0.1;
    const qreal minFactor = 1.0;
    const qreal maxFactor = 10.0;
    static qreal h11 = 1.0;
    static qreal h22 = 1.0;

    if (event->delta() == 120)
    {
        h11 = (h11 >= maxFactor) ? h11 : (h11 + scaleFactor);
        h22 = (h22 >= maxFactor) ? h22 : (h22 + scaleFactor);
    }
    else if (event->delta() == -120)
    {
        h11 = (h11 <= minFactor) ? minFactor : (h11 - scaleFactor);
        h22 = (h22 <= minFactor) ? minFactor : (h22 - scaleFactor);
    }
    view_->setTransformationAnchor(QGraphicsView::AnchorUnderMouse);
    view_->setTransform(QTransform(h11, 0, 0, 0, h22, 0, 0, 0, 1));
}

TocabiGui::TocabiGui()
    : rqt_gui_cpp::Plugin(), widget_(0)
{
    qRegisterMetaType<std_msgs::StringConstPtr>();
    qRegisterMetaType<geometry_msgs::PolygonStampedConstPtr>();
    qRegisterMetaType<std_msgs::Float32ConstPtr>();
    qRegisterMetaType<sensor_msgs::ImuConstPtr>();
    qRegisterMetaType<std_msgs::Int32MultiArrayConstPtr>();
    setObjectName("TocabiGui");

    //initPlugin()
    pointsub = nh_.subscribe("/tocabi/point", 1, &TocabiGui::pointCallback, this);
    timesub = nh_.subscribe("/tocabi/time", 1, &TocabiGui::timerCallback, this);
    sysstatesub = nh_.subscribe("/tocabi/systemstate", 1, &TocabiGui::sysstateCallback, this);
    com_pub = nh_.advertise<std_msgs::String>("/tocabi/command", 1);
    guilogsub = nh_.subscribe("/tocabi/guilog", 1000, &TocabiGui::guiLogCallback, this);
    gain_pub = nh_.advertise<std_msgs::Float32MultiArray>("/tocabi/gain_command", 100);
    imusub = nh_.subscribe("/tocabi/imu", 1, &TocabiGui::imuCallback, this);
    task_pub = nh_.advertise<tocabi_controller::TaskCommand>("/tocabi/taskcommand", 100);
    task_que_pub = nh_.advertise<tocabi_controller::TaskCommandQue>("/tocabi/taskquecommand", 100);
    
    walkingspeed_pub = nh_.advertise<std_msgs::Float32 >("/tocabi/walkingspeedcommand", 100);
    walkingduration_pub = nh_.advertise<std_msgs::Float32>("/tocabi/walkingdurationcommand", 100);
    walkingangvel_pub = nh_.advertise<std_msgs::Float32>("/tocabi/walkingangvelcommand", 100);
    kneetargetangle_pub = nh_.advertise<std_msgs::Float32>("/tocabi/kneetargetanglecommand", 100);
    footheight_pub = nh_.advertise<std_msgs::Float32>("/tocabi/footheightcommand", 100);
    
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
    connect(ui_.data_button_4, SIGNAL(pressed()), this, SLOT(dshowbtn()));

    connect(ui_.customtaskgain, SIGNAL(stateChanged(int)), this, SLOT(customtaskgaincb(int)));
    connect(ui_.solver_mode, SIGNAL(currentIndexChanged(int)), this, SLOT(solvermode_cb(int)));

    connect(ui_.emergencyoff_button_2, SIGNAL(pressed()), this, SLOT(shutdown_robot()));

    ui_.stackedWidget->setCurrentIndex(0);

    ui_.ecat_btn->setShortcut(QKeySequence(Qt::Key_1));
    ui_.stat_btn->setShortcut(QKeySequence(Qt::Key_2));
    ui_.command_btn->setShortcut(QKeySequence(Qt::Key_3));
    ui_.mtunebtn->setShortcut(QKeySequence(Qt::Key_4));
    ui_.walkingbtn->setShortcut(QKeySequence(Qt::Key_5));

    connect(this, &TocabiGui::timerCallback, this, &TocabiGui::timercb);
    connect(this, &TocabiGui::guiLogCallback, this, &TocabiGui::plainTextEditcb);
    connect(this, &TocabiGui::pointCallback, this, &TocabiGui::pointcb);
    connect(this, &TocabiGui::imuCallback, this, &TocabiGui::imucb);
    connect(this, &TocabiGui::sysstateCallback, this, &TocabiGui::sysstatecb);

    //connect(ui_)
    connect(ui_.initializebtn, SIGNAL(pressed()), this, SLOT(initializebtncb()));
    connect(ui_.initializebtn_2, SIGNAL(pressed()), this, SLOT(ecatinitlow()));
    connect(ui_.safetyresetbtn, SIGNAL(pressed()), this, SLOT(safetyresetbtncb()));
    connect(ui_.safetyresetbtn_2, SIGNAL(pressed()), this, SLOT(safety2btncb()));

    connect(ui_.task_send_button, SIGNAL(pressed()), this, SLOT(tasksendcb()));
    connect(ui_.walkinginit_btn, SIGNAL(pressed()), this, SLOT(walkinginitbtncb()));
    connect(ui_.walkingstart_btn, SIGNAL(pressed()), this, SLOT(walkingstartbtncb()));

    connect(ui_.sebutton, SIGNAL(pressed()), this, SLOT(stateestimationcb()));
    connect(ui_.torqueredis, SIGNAL(pressed()), this, SLOT(torquerediscb()));
    connect(ui_.qp2nd, SIGNAL(pressed()), this, SLOT(qp2ndcb()));

    connect(ui_.gravity_button_4, SIGNAL(pressed()), this, SLOT(gravcompcb()));
    connect(ui_.task_button_4, SIGNAL(pressed()), this, SLOT(posconcb()));
    connect(ui_.task_button_5, SIGNAL(pressed()), this, SLOT(posgravconcb()));
    connect(ui_.contact_button_4, SIGNAL(pressed()), this, SLOT(fixedgravcb()));

    //connect(ui_.)

    connect(ui_.que_add, SIGNAL(pressed()), this, SLOT(que_addquebtn()));
    connect(ui_.que_delete, SIGNAL(pressed()), this, SLOT(que_deletebtn()));
    connect(ui_.que_down, SIGNAL(pressed()), this, SLOT(que_downbtn()));
    connect(ui_.que_up, SIGNAL(pressed()), this, SLOT(que_upbtn()));
    connect(ui_.que_reset, SIGNAL(pressed()), this, SLOT(que_resetbtn()));
    connect(ui_.que_send, SIGNAL(pressed()), this, SLOT(que_sendbtn()));
    connect(ui_.iybtn, SIGNAL(pressed()), this, SLOT(inityaw()));

    connect(ui_.vjbtn, SIGNAL(pressed()), this, SLOT(simvj()));
    connect(ui_.imuresetbtn, SIGNAL(pressed()), this, SLOT(imureset()));

    connect(ui_.walking_speed_slider, SIGNAL(valueChanged(int)), this, SLOT(walkingspeedcb(int) ));
    connect(ui_.walking_duration_slider, SIGNAL(valueChanged(int)), this, SLOT(walkingdurationcb(int) ));
    connect(ui_.walking_angvel_slider, SIGNAL(valueChanged(int)), this, SLOT(walkingangvelcb(int) ));
    connect(ui_.knee_target_angle_slider, SIGNAL(valueChanged(int)), this, SLOT(kneetargetanglecb(int) ));
    connect(ui_.foot_height_slider, SIGNAL(valueChanged(int)), this, SLOT(footheightcb(int) ));

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
    safetylabels.resize(33);

    //head
    for (int i = 0; i < 2; i++)
    {
        ecatlabels[i] = new QLabel(ui_.head_layout->parentWidget());
        ui_.head_layout->addWidget(ecatlabels[i]);
        ecatlabels[i]->setFrameShape(QFrame::Panel);

        safetylabels[i] = new QLabel(ui_.head_safety->parentWidget());
        ui_.head_safety->addWidget(safetylabels[i]);
        safetylabels[i]->setFrameShape(QFrame::Panel);
    }

    for (int i = 2; i < 10; i++)
    {
        ecatlabels[i] = new QLabel(ui_.leftarm_layout->parentWidget());
        ui_.leftarm_layout->addWidget(ecatlabels[i]);
        ecatlabels[i]->setFrameShape(QFrame::Panel);

        safetylabels[i] = new QLabel(ui_.leftarm_safety->parentWidget());
        ui_.leftarm_safety->addWidget(safetylabels[i]);
        safetylabels[i]->setFrameShape(QFrame::Panel);
    }

    for (int i = 10; i < 18; i++)
    {
        ecatlabels[i] = new QLabel(ui_.rightarm_layout->parentWidget());
        ui_.rightarm_layout->addWidget(ecatlabels[i]);
        ecatlabels[i]->setFrameShape(QFrame::Panel);

        safetylabels[i] = new QLabel(ui_.rightarm_safety->parentWidget());
        ui_.rightarm_safety->addWidget(safetylabels[i]);
        safetylabels[i]->setFrameShape(QFrame::Panel);
    }

    for (int i = 18; i < 21; i++)
    {
        ecatlabels[i] = new QLabel(ui_.waist_layout->parentWidget());
        ui_.waist_layout->addWidget(ecatlabels[i]);
        ecatlabels[i]->setFrameShape(QFrame::Panel);

        safetylabels[i] = new QLabel(ui_.waist_safety->parentWidget());
        ui_.waist_safety->addWidget(safetylabels[i]);
        safetylabels[i]->setFrameShape(QFrame::Panel);
    }

    for (int i = 21; i < 27; i++)
    {
        ecatlabels[i] = new QLabel(ui_.leftleg_layout->parentWidget());
        ui_.leftleg_layout->addWidget(ecatlabels[i]);
        ecatlabels[i]->setFrameShape(QFrame::Panel);

        safetylabels[i] = new QLabel(ui_.leftleg_safety->parentWidget());
        ui_.leftleg_safety->addWidget(safetylabels[i]);
        safetylabels[i]->setFrameShape(QFrame::Panel);
    }

    for (int i = 27; i < 33; i++)
    {
        ecatlabels[i] = new QLabel(ui_.rightleg_layout->parentWidget());
        ui_.rightleg_layout->addWidget(ecatlabels[i]);
        ecatlabels[i]->setFrameShape(QFrame::Panel);

        safetylabels[i] = new QLabel(ui_.rightleg_safety->parentWidget());
        ui_.rightleg_safety->addWidget(safetylabels[i]);
        safetylabels[i]->setFrameShape(QFrame::Panel);
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

        safetylabels[i]->setStyleSheet("QLabel { background-color : rgb(138, 226, 52) ; color : black; }");
    }
    ui_.taskgain->setDisabled(true);

    ui_.graphicsView->setDisabled(true);
    ui_.graphicsView->setHidden(true);

    view = new MyQGraphicsView(widget_);
    view->setObjectName(QStringLiteral("graphicsViewCustom"));
    view->setGeometry(QRect(1180, 80, 411, 371));
    view->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    view->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    view->setSizeAdjustPolicy(QAbstractScrollArea::AdjustToContentsOnFirstShow);
    view->setInteractive(true);
    view->setDragMode(QGraphicsView::ScrollHandDrag);
    view->setTransformationAnchor(QGraphicsView::NoAnchor);
    view->setResizeAnchor(QGraphicsView::NoAnchor);

    scene = new MyQGraphicsScene(widget_);
    //scene->setSceneRect(0, 0, view->width(), view->height());

    view->setScene(scene);

    //std::cout << widget_->findChild<QObject *>("graphicsViewCustom")->objectName().toStdString() << std::endl;

    QBrush redbrush(Qt::red);
    QBrush bluebrush(Qt::blue);
    QBrush yellowbrush(Qt::yellow);
    QBrush blackbrush(Qt::black);
    QPen blackpen(Qt::black);
    Pelv = new QGraphicsRectItem(QRectF(-150 / 4, -40 / 4, 300 / 4, 80 / 4));
    scene->addItem(Pelv);

    lfoot_d = new QGraphicsRectItem(QRectF(-85 / 4, -120 / 4, 170 / 4, 300 / 4));
    scene->addItem(lfoot_d);

    rfoot_d = new QGraphicsRectItem(QRectF(-85 / 4, -120 / 4, 170 / 4, 300 / 4));
    scene->addItem(rfoot_d);

    com_d = scene->addEllipse(-10, -10, 20, 20, blackpen, yellowbrush);
    rfoot_c = scene->addEllipse(-2, -2, 4, 4, blackpen, blackbrush);
    lfoot_c = scene->addEllipse(-2, -2, 4, 4, blackpen, blackbrush);

    zmp = scene->addEllipse(-5, -5, 10, 10, blackpen, redbrush);

    scene->addLine(-20, 0, 40, 0, blackpen);
    scene->addLine(0, -20, 0, 40, blackpen);

    QGraphicsTextItem *front = scene->addText("front");
    front->setPos(0, 50);

    //ui_.graphicsView->scale(10, 10);

    //ui_.graphicsView->setSceneRect(-210, -260, 421, 521);

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

void TocabiGui::solvermode_cb(int state)
{
    if (state == 0)
    {
        ui_.cr_mode->setEnabled(true);
        ui_.cr_mode->setCurrentIndex(0);
    }
    else
    {
        ui_.cr_mode->setDisabled(true);
        ui_.cr_mode->setCurrentIndex(2);
    }
}

void TocabiGui::sysstatecb(const std_msgs::Int32MultiArrayConstPtr &msg)
{
    if (msg->data[0] == 0) //imu
    {
        ui_.label_imustatus->setStyleSheet("QLabel { background-color : red ; color : white; }");
        ui_.label_imustatus->setText(QString::fromUtf8("NOT OK"));
    }
    else if (msg->data[0] == 1)
    {
        ui_.label_imustatus->setStyleSheet("QLabel { background-color : yellow; color : black; }");
        ui_.label_imustatus->setText(QString::fromUtf8("waiting"));
    }
    else if (msg->data[0] == 2)
    {
        ui_.label_imustatus->setStyleSheet("QLabel { background-color : rgb(138, 226, 52) ; color : black; }");
        ui_.label_imustatus->setText(QString::fromUtf8("OK"));
    }

    if (msg->data[1] == 0) // zp
    {
        ui_.label_zpstatus->setStyleSheet("QLabel { background-color : red ; color : white; }");
        ui_.label_zpstatus->setText(QString::fromUtf8("NOT OK"));
    }
    else if (msg->data[1] == 1)
    {
        ui_.label_zpstatus->setStyleSheet("QLabel { background-color : yellow ; color : black; }");
        ui_.label_zpstatus->setText(QString::fromUtf8("FIND REQ"));
    }
    else if (msg->data[1] == 2)
    {
        ui_.label_zpstatus->setStyleSheet("QLabel { background-color : rgb(138, 226, 52) ; color : black; }");
        ui_.label_zpstatus->setText(QString::fromUtf8("OK"));
    }

    if (msg->data[2] == 0) //ft
    {
        ui_.label_ftstatus->setStyleSheet("QLabel { background-color : red ; color : white; }");
        ui_.label_ftstatus->setText(QString::fromUtf8("NOT OK"));
    }
    else if (msg->data[2] == 1)
    {
        ui_.label_ftstatus->setStyleSheet("QLabel { background-color : yellow ; color : black; }");
        ui_.label_ftstatus->setText(QString::fromUtf8("INIT REQ"));
    }
    else if (msg->data[2] == 2)
    {
        ui_.label_ftstatus->setStyleSheet("QLabel { background-color : rgb(138, 226, 52) ; color : black; }");
        ui_.label_ftstatus->setText(QString::fromUtf8("OK"));
    }

    if (msg->data[3] == 0) //ecat
    {
        ui_.label_ecatstatus->setStyleSheet("QLabel { background-color : red ; color : white; }");
        ui_.label_ecatstatus->setText(QString::fromUtf8("NOT OK"));
    }
    else if (msg->data[3] == 1)
    {
        ui_.label_ecatstatus->setStyleSheet("QLabel { background-color : rgb(138, 226, 52) ; color : black; }");
        ui_.label_ecatstatus->setText(QString::fromUtf8("OK"));
    }
    else if (msg->data[3] == 2)
    {
        ui_.label_ecatstatus->setStyleSheet("QLabel { background-color : yellow ; color : black; }");
        ui_.label_ecatstatus->setText(QString::fromUtf8("COMMUTATION"));
    }
}

void TocabiGui::que_downbtn()
{
}
void TocabiGui::que_upbtn()
{
}
void TocabiGui::que_deletebtn()
{
    std::cout << ui_.que_listwidget->currentIndex().row() << std::endl;
    QListWidgetItem *item = ui_.que_listwidget->takeItem(ui_.que_listwidget->currentIndex().row());

    ui_.que_listwidget->removeItemWidget(item);
}
void TocabiGui::que_resetbtn()
{
    ui_.que_listwidget->clear();
    tq_.clear();
}

void TocabiGui::que_sendbtn()
{
    task_que_msg.tque.resize(tq_.size());
    for (int i = 0; i < tq_.size(); i++)
    {
        task_que_msg.tque[i] = tq_[i].tc_;
    }
    task_que_pub.publish(task_que_msg);
}

void TocabiGui::shutdown_robot()
{
    com_msg.data = std::string("terminate");
    com_pub.publish(com_msg);
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

void TocabiGui::simvj()
{
    com_msg.data = std::string("simvirtualjoint");
    com_pub.publish(com_msg);
}
void TocabiGui::imureset()
{
    com_msg.data = std::string("imureset");
    com_pub.publish(com_msg);
}

void TocabiGui::ecatinitlow()
{
    com_msg.data = std::string("ecatinitlower");
    com_pub.publish(com_msg);
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

void TocabiGui::inityaw()
{
    com_msg.data = std::string("inityaw");
    com_pub.publish(com_msg);
}

void TocabiGui::dshowbtn()
{
    com_msg.data = std::string("showdata");
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
    else if (words[0] == "Lock")
    {
        int num = elng[atoi(words[1].c_str())];
        safetylabels[num]->setStyleSheet("QLabel { background-color : red ; color : white; }");
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
    rfoot_d->setRotation(msg->polygon.points[9].z * -180.0 / 3.141592);
    rfoot_c->setPos(QPointF(msg->polygon.points[1].y * 250, msg->polygon.points[1].x * 250));

    ui_.label_73->setText(QString::number(msg->polygon.points[1].x, 'f', 5));
    ui_.label_74->setText(QString::number(msg->polygon.points[1].y, 'f', 5));

    lfoot_d->setPos(QPointF(msg->polygon.points[2].y * 250, msg->polygon.points[2].x * 250));
    lfoot_d->setRotation(msg->polygon.points[8].z * -180.0 / 3.141592);
    lfoot_c->setPos(QPointF(msg->polygon.points[2].y * 250, msg->polygon.points[2].x * 250));

    Pelv->setPos(QPointF(msg->polygon.points[3].y * 250, msg->polygon.points[3].x * 250));
    Pelv->setRotation(msg->polygon.points[4].z * -180.0 / 3.141592);

    zmp->setPos(QPointF(msg->polygon.points[7].y * 250, msg->polygon.points[7].x * 250));

    ui_.label_64->setText(QString::number(msg->polygon.points[2].x, 'f', 5));
    ui_.label_65->setText(QString::number(msg->polygon.points[2].y, 'f', 5));

    //pelvis rpy
    ui_.label_14->setText(QString::number(msg->polygon.points[4].x * 180.0 / 3.141592, 'f', 5));
    ui_.label_15->setText(QString::number(msg->polygon.points[4].y * 180.0 / 3.141592, 'f', 5));
    ui_.label_16->setText(QString::number(msg->polygon.points[4].z * 180.0 / 3.141592, 'f', 5));

    ui_.label_13->setText(QString::number(msg->polygon.points[3].x, 'f', 5));
    ui_.label_21->setText(QString::number(msg->polygon.points[3].y, 'f', 5));
    ui_.label_91->setText(QString::number(msg->polygon.points[3].z, 'f', 5));

    //zmp by ft

    ui_.label_22->setText(QString::number(msg->polygon.points[7].x, 'f', 5));
    ui_.label_23->setText(QString::number(msg->polygon.points[7].y, 'f', 5));

    //Right hand
    ui_.label_126->setText(QString::number(msg->polygon.points[5].x, 'f', 5));
    ui_.label_127->setText(QString::number(msg->polygon.points[5].y, 'f', 5));
    ui_.label_128->setText(QString::number(msg->polygon.points[5].z, 'f', 5));

    //Left hand
    ui_.label_119->setText(QString::number(msg->polygon.points[6].x, 'f', 5));
    ui_.label_120->setText(QString::number(msg->polygon.points[6].y, 'f', 5));
    ui_.label_121->setText(QString::number(msg->polygon.points[6].z, 'f', 5));

    //LF orient
    ui_.label_105->setText(QString::number(msg->polygon.points[8].x * 180.0 / 3.141592, 'f', 5));
    ui_.label_106->setText(QString::number(msg->polygon.points[8].y * 180.0 / 3.141592, 'f', 5));
    ui_.label_107->setText(QString::number(msg->polygon.points[8].z * 180.0 / 3.141592, 'f', 5));

    //RF orient
    ui_.label_112->setText(QString::number(msg->polygon.points[9].x * 180.0 / 3.141592, 'f', 5));
    ui_.label_113->setText(QString::number(msg->polygon.points[9].y * 180.0 / 3.141592, 'f', 5));
    ui_.label_114->setText(QString::number(msg->polygon.points[9].z * 180.0 / 3.141592, 'f', 5));

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

    //ui_.graphicsView->setSceneRect(0, 0, 0, 0);
}

void TocabiGui::initializebtncb()
{
    com_msg.data = std::string("ecatinit");
    com_pub.publish(com_msg);
}

void TocabiGui::safetyresetbtncb()
{
    for (int i = 0; i < 33; i++)
    {
        safetylabels[i]->setStyleSheet("QLabel { background-color : rgb(138, 226, 52) ; color : black; }");
    }
    com_msg.data = std::string("safetyreset");
    com_pub.publish(com_msg);
}

void TocabiGui::safety2btncb()
{
    for (int i = 0; i < 33; i++)
    {
        safetylabels[i]->setStyleSheet("QLabel { background-color : yellow ; color : black; }");
    }
    com_msg.data = std::string("safetydisable");
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

void TocabiGui::handletaskmsg()
{
    task_msg.pelv_pitch = ui_.pelv_pitch->text().toFloat();
    task_msg.roll = ui_.com_roll->text().toFloat();
    task_msg.pitch = ui_.com_pitch->text().toFloat();
    task_msg.yaw = ui_.com_yaw->text().toFloat();
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

    task_msg.solver = ui_.solver_mode->currentIndex();

    task_msg.contactredis = ui_.cr_mode->currentIndex();

    task_msg.acc_p = ui_.accgain->text().toFloat();

    if (task_msg.customTaskGain)
    {
        task_msg.pos_p = ui_.pospgain->text().toFloat();
        task_msg.pos_d = ui_.posdgain->text().toFloat();
        task_msg.ang_p = ui_.angpgain->text().toFloat();
        task_msg.ang_d = ui_.angdgain->text().toFloat();
    }
}

void TocabiGui::que_addquebtn()
{
    task_que tq_temp;

    handletaskmsg();
    tq_temp.tc_ = task_msg;
    //ui_.text_que->text().toStdString();

    std::stringstream ss;
    ss << "task " << tq_.size() + 1 << " : ";

    ui_.que_listwidget->addItem(ss.str().c_str() + ui_.text_que->text());

    tq_.push_back(tq_temp);

    //list.append(ui_.text_que->text());

    //ui_.que_list->set
}

void TocabiGui::tasksendcb()
{
    handletaskmsg();

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
    task_msg.walking_enable = 3.0;
    task_msg.ik_mode = ui_.ik_mode->currentIndex();

    if (ui_.walking_pattern->currentIndex() == 0)
    {
        task_msg.pattern = 0;
    }
    else if (ui_.walking_pattern->currentIndex() == 1)
    {
        task_msg.pattern = 1;
    }
    else
    {
        task_msg.pattern = 2;
    }

    if (ui_.checkBox_dob->isChecked() == true)
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
    task_msg.walking_height = ui_.text_walking_height->text().toFloat();
    task_msg.theta = ui_.text_walking_theta->text().toFloat();
    task_msg.step_length_x = ui_.text_walking_steplengthx->text().toFloat();
    task_msg.step_length_y = ui_.text_walking_steplengthy->text().toFloat();

    task_pub.publish(task_msg);
}

void TocabiGui::walkingstartbtncb()
{
    task_msg.walking_enable = 1.0;
    task_msg.ik_mode = ui_.ik_mode->currentIndex();

    if (ui_.walking_pattern->currentIndex() == 0)
    {
        task_msg.pattern = 0;
    }
    else if (ui_.walking_pattern->currentIndex() == 1)
    {
        task_msg.pattern = 1;
    }
    else
    {
        task_msg.pattern = 2;
    }

    if (ui_.checkBox_dob->isChecked() == true)
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
    task_msg.walking_height = ui_.text_walking_height->text().toFloat();
    task_msg.theta = ui_.text_walking_theta->text().toFloat();
    task_msg.step_length_x = ui_.text_walking_steplengthx->text().toFloat();
    task_msg.step_length_y = ui_.text_walking_steplengthy->text().toFloat();

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

void TocabiGui::posgravconcb()
{
    com_msg.data = std::string("positiongravcontrol");
    com_pub.publish(com_msg);
}

void TocabiGui::walkingspeedcb(int value)
{
    double max_speed = 0.6;
    double min_speed = -0.4;
    double scale = value;
    
    walkingspeed_msg.data = scale/100*(max_speed - min_speed) + min_speed;
    walkingspeed_pub.publish(walkingspeed_msg);
}

void TocabiGui::walkingdurationcb(int value)
{
    double max_duration = 1;
    double min_duration = 0.2;
    double scale = value;

    walkingduration_msg.data = scale/100*(max_duration - min_duration) + min_duration;
    walkingduration_pub.publish(walkingduration_msg);
}

void TocabiGui::walkingangvelcb(int value)
{
    double max_angvel = 1;
    double min_angvel = -1;
    double scale = value;

    walkingangvel_msg.data = scale/100*(max_angvel - min_angvel) + min_angvel;
    walkingangvel_pub.publish(walkingangvel_msg);
}

void TocabiGui::kneetargetanglecb(int value)
{
    double max_knee = M_PI/2;
    double min_knee = 0;
    double scale = value;

    kneetargetangle_msg.data = scale/100*(max_knee - min_knee) + min_knee;
    kneetargetangle_pub.publish(kneetargetangle_msg);
}

void TocabiGui::footheightcb(int value)
{
    double max_footz = 0.1;
    double min_footz = 0.005;
    double scale = value;

    footheight_msg.data = scale/100*(max_footz - min_footz) + min_footz;
    footheight_pub.publish(footheight_msg);
}

} // namespace tocabi_gui

PLUGINLIB_EXPORT_CLASS(tocabi_gui::TocabiGui, rqt_gui_cpp::Plugin)
