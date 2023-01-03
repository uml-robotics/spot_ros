#include "spot_camera_nav_ui.hpp"

#include <QFile>
#include <QUiLoader>
#include <QScrollBar>
#include <ros/package.h>
#include <thread>
#include <iostream>

namespace spot_viz{

CameraNavUI::CameraNavUI(QWidget *parent){
    //Initialize UI
    std::string packagePath = ros::package::getPath("spot_viz") + "/resource/spot_camera_nav.ui";
    ROS_INFO("Getting ui file from package path %s", packagePath.c_str());
    QFile file(packagePath.c_str());
    file.open(QIODevice::ReadOnly);

    QUiLoader loader;
    QWidget* ui = loader.load(&file, parent);
    file.close();
    QVBoxLayout* topLayout = new QVBoxLayout();
    this->setLayout(topLayout);
    topLayout->addWidget(ui);
    x = y = 0;

    walkToObjectAction_ = new actionlib::SimpleActionClient<spot_msgs::WalkToObjectAction>("spot/walk_to_object", true);

    sendCommandButton = this->findChild<QPushButton*>("sendCommandButton");
    stopCommandButton = this->findChild<QPushButton*>("stopCommandButton");
    xTitleLabel = this->findChild<QLabel*>("xTitleLabel");
    yTitleLabel = this->findChild<QLabel*>("yTitleLabel");
    xValLabel = this->findChild<QLabel*>("xValLabel");
    yValLabel = this->findChild<QLabel*>("yValLabel");
    feedbackLabel = this->findChild<QLabel*>("feedbackLabel");
    instructionsLabel = this->findChild<QLabel*>("instructionsLabel");
    feedbackTextBrowser = this->findChild<QTextBrowser*>("feedbackTextBrowser");
    cameraSelectionComboBox = this->findChild<QComboBox*>("cameraSelectionComboBox");
    cameraLayout = this->findChild<QVBoxLayout*>("cameraLayout");

    imageLabel = new ImageLabel();
    cameraLayout->insertWidget(1, imageLabel);

    xValLabel->setNum(x);
    yValLabel->setNum(y);

    connect(cameraSelectionComboBox, SIGNAL(currentTextChanged(const QString&)), this, SLOT(switchCamera(const QString&)));
    connect(imageLabel, SIGNAL(clicked(int, int)), this, SLOT(logImageClick(int, int)));
    connect(sendCommandButton, SIGNAL(clicked()), this, SLOT(sendCommand()));
    connect(stopCommandButton, SIGNAL(clicked()), this, SLOT(stopCommand()));
    connect(this, SIGNAL(feedback(const std::string&)), this, SLOT(updateFeedback(const std::string&)));

    //ROS stuff processed outside of UI thread
    spinner_.start();
}

void CameraNavUI::callWalkToObjectAction(actionlib::SimpleActionClient<spot_msgs::WalkToObjectAction> &action, int x, int y, std::string camera_name){
    Q_EMIT feedback("Calling Walk To Object Action at " + std::to_string(x) + ", " + std::to_string(y));
    spot_msgs::WalkToObjectGoal goal;
    goal.x = x;
    goal.y = y;
    goal.camera_name = camera_name;
    action.sendGoal(goal, 
                    boost::bind(&CameraNavUI::actionDoneCB, this, _1, _2),
                    actionlib::SimpleActionClient<spot_msgs::WalkToObjectAction>::SimpleActiveCallback(),
                    boost::bind(&CameraNavUI::actionFeedbackCB, this, _1)
                    );

/*
    //this blocks the main thread so I commented it out
    bool finished_before_timeout = action.waitForResult(ros::Duration(30.0));
    if(finished_before_timeout){
        actionlib::SimpleClientGoalState state = action.getState();
        Q_EMIT feedback("Action finished: " + state.toString());
    }
    else
        Q_EMIT feedback("Action did not finish before timeout.");  
*/
}

void CameraNavUI::actionFeedbackCB(const spot_msgs::WalkToObjectFeedbackConstPtr& action_feedback){
    std::string current_goal = std::to_string(action_feedback->current_goal.position.x) + ", " + 
                                std::to_string(action_feedback->current_goal.position.y) + 
                                ", " + std::to_string(action_feedback->current_goal.position.z
                            );
    Q_EMIT feedback(action_feedback->feedback_name + ". Navigating to " + current_goal + " relative to body.");
}

void CameraNavUI::actionDoneCB(const actionlib::SimpleClientGoalState& state, const spot_msgs::WalkToObjectResultConstPtr& result){
    Q_EMIT feedback("Action finished: " + state.toString());
}

void CameraNavUI::imageCB(const sensor_msgs::Image::ConstPtr& msg){
    QImage::Format encoding;
    if(msg->encoding == "mono8"){
        encoding = QImage::Format_Grayscale8;
    }
    else{
        encoding = QImage::Format_RGB888;
    }
    QImage image(&(msg->data[0]), msg->width, msg->height, msg->step, encoding);
    image = image.transformed(QMatrix().rotate(90.0));
    ROS_INFO("Image received with encoding %s.", msg->encoding.c_str());
    scaleFactor = static_cast<double>(image.width()) / imageLabel->width();
    QPixmap pixmap = QPixmap::fromImage(image).scaledToWidth(imageLabel->width());
    imageLabel->setPixmap(pixmap);
    imageLabel->show();
}

void CameraNavUI::sendCommand(){
    std::string camera = cameraSelectionComboBox->currentText().toStdString();
    if(camera == "Select Camera"){
        Q_EMIT feedback("No camera selected");
        return;
    }
    int comp_x = y;
    int comp_y = imageLabel->width() * scaleFactor - x;
    callWalkToObjectAction(*walkToObjectAction_, comp_x, comp_y, camera + "_fisheye");
}

void CameraNavUI::stopCommand(){
    Q_EMIT feedback("Sending cancel request to WalkToObjectAction");
    walkToObjectAction_->cancelAllGoals();
}

void CameraNavUI::logImageClick(int cx, int cy){
    x = cx * scaleFactor;
    y = cy * scaleFactor;
    xValLabel->setNum(x);
    yValLabel->setNum(y);
}

void CameraNavUI::updateFeedback(const std::string& feedback){
    QScrollBar* sb = feedbackTextBrowser->verticalScrollBar();
    bool end = sb->maximum() - sb->value() <= 10;
    feedbackTextBrowser->append(QString(feedback.c_str()));
    //autoscroll
    if(end){
        sb->setValue(sb->maximum());
    }
}

void CameraNavUI::switchCamera(const QString& text){
    //unsubscribe from current camera topic
    imageSub_.shutdown();
    if(text == "Select Camera"){
        //clear image label
        imageLabel->clear();
        return;
    }
    //subscribe to new camera topic
    std::string topic = "spot/camera/" + text.toStdString() + "/image";
    Q_EMIT feedback("Subscribing to " + topic);
    imageSub_ = nh_.subscribe<sensor_msgs::Image>(topic, 1, &CameraNavUI::imageCB, this);
    ROS_INFO("Subscribed to %s", topic.c_str());
}

void CameraNavUI::save(rviz::Config config) const
{
    rviz::Panel::save(config);
}

// Load all configuration data for this panel from the given Config object.
void CameraNavUI::load(const rviz::Config &config)
{
    rviz::Panel::load(config);
}

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(spot_viz::CameraNavUI, rviz::Panel);