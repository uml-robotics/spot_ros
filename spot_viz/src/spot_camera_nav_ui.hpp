#ifndef SPOT_CAMERA_NAV_UI_H
#define SPOT_CAMERA_NAV_UI_H

#ifndef Q_MOC_RUN
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

# include <rviz/panel.h>
#endif

#include <string>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include <QComboBox>
#include <QLabel>
#include <QVBoxLayout>
#include <QPushButton>
#include <QTextBrowser>
#include <QMouseEvent>

#include <spot_msgs/WalkToObjectAction.h>

namespace spot_viz {

class ImageLabel : public QLabel
{
    Q_OBJECT
    public:
    ImageLabel(QString text="", QWidget *parent=0) : QLabel(text, parent){}
    ~ImageLabel(){}

    Q_SIGNALS:
    void clicked(int x, int y);

    protected:
    void mousePressEvent(QMouseEvent *event) {
        QPointF pos = event->pos();
        Q_EMIT clicked(pos.x(), pos.y());
    }
};

class CameraNavUI : public rviz::Panel 
{
    Q_OBJECT
    public:
    CameraNavUI(QWidget *parent=0);
    virtual void save(rviz::Config config) const;
    virtual void load(const rviz::Config &config);

    Q_SIGNALS:
    void imageReceived(const QImage&);
    void feedback(const std::string&);

    private Q_SLOTS:
    void sendCommand();
    void stopCommand();
    void switchCamera(const QString&);
    void logImageClick(int cx, int cy);
    void updateFeedback(const std::string&);

    private:
    //void setControlButtons();
    void callWalkToObjectAction(actionlib::SimpleActionClient<spot_msgs::WalkToObjectAction> &action, int x, int y, std::string camera_name);
    void actionFeedbackCB(const spot_msgs::WalkToObjectFeedbackConstPtr& action_feedback);
    void actionDoneCB(const actionlib::SimpleClientGoalState& state, const spot_msgs::WalkToObjectResultConstPtr& result);
    void imageCB(const sensor_msgs::Image::ConstPtr&);

    ros::NodeHandle nh_;
    ros::Subscriber imageSub_;
    ros::AsyncSpinner spinner_{2};
    actionlib::SimpleActionClient<spot_msgs::WalkToObjectAction>* walkToObjectAction_;

    QPushButton* sendCommandButton;
    QPushButton* stopCommandButton;
    QLabel* xTitleLabel;
    QLabel* yTitleLabel;
    QLabel* xValLabel;
    QLabel* yValLabel;
    QLabel* feedbackLabel;
    QLabel* instructionsLabel;
    ImageLabel* imageLabel;
    QTextBrowser* feedbackTextBrowser;
    QComboBox* cameraSelectionComboBox;
    QVBoxLayout* cameraLayout;

    int x, y;
    double scaleFactor; // img.width / imageLabel.width
    std::string current_camera;
};

}

#endif