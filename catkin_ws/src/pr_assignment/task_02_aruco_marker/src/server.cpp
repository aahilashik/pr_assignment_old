#include <iostream>
#include <ros/ros.h>

#include <actionlib/server/simple_action_server.h>
#include <task_02_aruco_marker/ArucoDetector2DAction.h>
#include <geometry_msgs/Pose2D.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/aruco.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>

using namespace std;
using namespace cv;

class ArucoLocalization {
    private:
        ros::NodeHandle nh_;
        actionlib::SimpleActionServer<task_02_aruco_marker::ArucoDetector2DAction> as_;
        task_02_aruco_marker::ArucoDetector2DFeedback feedback_;
        task_02_aruco_marker::ArucoDetector2DResult result_;

        Ptr<aruco::DetectorParameters>  detectorParams;
        Ptr<aruco::Dictionary>              dictionary;
        Mat                      camMatrix, distCoeffs;

        Mat image, imageCopy;
        
        int                     roboId;
        double                  scale;
        vector<int>             ids;
        vector<vector<Point2f>> corners, rejected;
        vector<Vec3d>           rvecs, tvecs;


    public:
        ArucoLocalization(string name, int Id, double px2met) : as_(nh_, name, boost::bind(&ArucoLocalization::executeCB, this, _1), false) {
            roboId  = Id;
            scale   = px2met;
            detectorParams = aruco::DetectorParameters::create();
            detectorParams->cornerRefinementMethod = aruco::CORNER_REFINE_SUBPIX;

            dictionary = aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME(cv::aruco::DICT_6X6_250));

            bool readOk = readCameraParameters("/home/ashik/ros_workspace/catkin_ws/src/pr_assignment/task_02_aruco_marker/config/intrinsics.yml", camMatrix, distCoeffs);
            if ( !readOk )
                throw runtime_error("Aruco : Invalid camera file");

            as_.registerPreemptCallback(boost::bind(&ArucoLocalization::preemptCb, this));
            as_.start();
            ROS_INFO("Aruco : Action Server Started Successfully..!");
        }

        ~ArucoLocalization() {  destroyAllWindows();    }

        bool readCameraParameters(string filename, Mat &camMatrix, Mat &distCoeffs) {
            FileStorage fs(filename, FileStorage::READ);
            if(!fs.isOpened())
                return false;
            fs["Camera_Matrix"] >> camMatrix;
            fs["Distortion_Coefficients"] >> distCoeffs;
            return true;
        }

        geometry_msgs::Pose2D get2DPose(vector<Point2f> pts) {
            geometry_msgs::Pose2D pose;

            pose.x      = scale * (pts[0].x + pts[1].x + pts[2].x + pts[3].x)/4;
            pose.y      = scale * (pts[0].y + pts[1].y + pts[2].y + pts[3].y)/4;
            pose.theta  = atan2(pts[1].y-pts[0].y, pts[1].x-pts[0].x);

            return pose;
        }

        void preemptCb(){
            ROS_WARN("Aruco : Preempted");
            if(as_.isActive())
                as_.setPreempted();
        }

        void executeCB(const task_02_aruco_marker::ArucoDetector2DGoal::ConstPtr &goal) {
            bool success = true;
            geometry_msgs::Pose2D pose2D;

            try {
                cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(goal->image, "bgr8");

                image = cv_ptr->image;
                if ( image.empty() ) {
                    ROS_WARN("Aruco : Image is empty..!");
                    success = false;
                }

                aruco::detectMarkers(image, dictionary, corners, ids, detectorParams, rejected);
                for(unsigned int i = 0; i < ids.size(); i++)
                    if (ids[i] == roboId) pose2D = get2DPose(corners[0]);
                    
            } catch (cv_bridge::Exception& e) {
                ROS_WARN("Aruco : Could not convert image to 'BGR8'.");
                success = false;
            }

            if ( success ) {
                result_.pose = pose2D;
                ROS_INFO("Aruco : Succeeded");
                // set the action state to succeeded
                as_.setSucceeded(result_);
            }

            visualize();
        }

        void visualize() {
            // Draw Results
            image.copyTo(imageCopy);
            if(ids.size() > 0) {
                for(unsigned int i = 0; i < ids.size(); i++)
                    if (ids[i] == roboId) aruco::drawDetectedMarkers(imageCopy, corners, ids);
            }

            imshow("Output", imageCopy);
            imwrite("/home/ashik/Downloads/BOutput.jpg", imageCopy);
            waitKey(-1);
            destroyAllWindows();
        }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "aruco_server");

    double    pxToMet = 0.006;
    int       robotID = 62;

    ArucoLocalization localization("localization", robotID, pxToMet);
    ros::spin();

    return 0;
}