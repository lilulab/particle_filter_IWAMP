#include <filter/bootstrapfilter.h>
#include <model/systemmodel.h>
#include <model/measurementmodel.h>
#include "nonlinearSystemPdf.h"
#include "nonlinearMeasurementPdf.h"
#include <iostream>
#include <fstream>
#include <std_msgs/Float32.h>
// Include file with properties
#include "constants.h"
#include "customparticlefilter.h"
#include <ros/ros.h>
#include <string>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <crawler_msgs/VisualHeading.h>
#include <crawler_msgs/JointCmd.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace MatrixWrapper;
using namespace BFL;
using namespace std;

/*
 The necessary SYSTEM MODEL is:
 x_k      = x_{k-1} + v_{k-1} * cos(theta) * delta_t
 y_k      = y_{k-1} + v_{k-1} * sin(theta) * delta_t
*/

class ParticleFilterNode
{
public:
    ros::NodeHandle nh_;
    ros::Subscriber navi_sub;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    ros::Publisher pose_pub;
    ros::Publisher particle_pub;
    ros::Subscriber orientation_sub_;
    NonlinearSystemPdf *sys_pdf;
    SystemModel<ColumnVector> *sys_model;
    NonlinearMeasurementPdf *meas_pdf;
    MeasurementModel<ColumnVector,ColumnVector> *meas_model;
    MCPdf<ColumnVector> *prior_discr;
    CustomParticleFilter *filter;
    float time_old, time_new;
    int odometry_linear, odometry_rotation, odometry_rotation_old;
    
public:
    ParticleFilterNode(): it_(nh_)
    {
    	//Creating ROS publishers and subscribers
        navi_sub = nh_.subscribe("/crawler_msgs/JointCmd", 10, &ParticleFilterNode::InputCb, this);
        image_sub_ = it_.subscribe("/cam_back/image_raw", 10, &ParticleFilterNode::MeasurementCb, this);
        pose_pub = nh_.advertise<geometry_msgs::PoseStamped>("/visual_odometry/state",1);
        particle_pub = nh_.advertise<geometry_msgs::PoseArray>("/particle_cloud",1);
        orientation_sub_ = nh_.subscribe("/crawler/visual_heading",1000, &ParticleFilterNode::orientationCb, this);

        sys_model = NULL;
        meas_model = NULL;
        filter = NULL;

        time_old = 0.0;
        time_new = 0.0;

        odometry_rotation_old = 0.0;

        CreateParticleFilter();
    }

    ~ParticleFilterNode()
    {
        delete sys_model;
        delete meas_model;
        delete filter;
    }

    void CreateParticleFilter()
    {
        /****************************
        * NonLinear system model      *
        ***************************/

        // Create gaussian
        ColumnVector sys_noise_Mu(STATE_SIZE);
        sys_noise_Mu(1) = MU_SYSTEM_NOISE_X;
        sys_noise_Mu(2) = MU_SYSTEM_NOISE_Y;
        sys_noise_Mu(3) = MU_SYSTEM_NOISE_THETA;

        SymmetricMatrix sys_noise_Cov(STATE_SIZE);
        sys_noise_Cov = 0.0;
        sys_noise_Cov(1,1) = SIGMA_SYSTEM_NOISE_X;
        sys_noise_Cov(1,2) = 0.0;
        sys_noise_Cov(1,3) = 0.0;
        sys_noise_Cov(2,1) = 0.0;
        sys_noise_Cov(2,2) = SIGMA_SYSTEM_NOISE_Y;
        sys_noise_Cov(2,3) = 0.0;
        sys_noise_Cov(3,1) = 0.0;
        sys_noise_Cov(3,2) = 0.0;
        sys_noise_Cov(3,3) = SIGMA_SYSTEM_NOISE_THETA;

        Gaussian system_Uncertainty(sys_noise_Mu, sys_noise_Cov);

        // Create the nonlinear system model(motion model)
        sys_pdf = new NonlinearSystemPdf(system_Uncertainty);
        sys_model = new SystemModel<ColumnVector> (sys_pdf);


        /*********************************
        * NonLinear Measurement model   *
        ********************************/

        // Construct the measurement noise (a scalar in this case)
        ColumnVector meas_noise_Mu(MEAS_SIZE);
        meas_noise_Mu(1) = MU_MEAS_NOISE;

        SymmetricMatrix meas_noise_Cov(MEAS_SIZE);
        meas_noise_Cov(1,1) = SIGMA_MEAS_NOISE;

        Gaussian measurement_Uncertainty(meas_noise_Mu, meas_noise_Cov);


        //Create the nonlinear measurement model
        meas_pdf = new NonlinearMeasurementPdf(measurement_Uncertainty);
        //Set the path to fetch templates for measurement model
        meas_pdf->SetEdgeInfoPath("/home/crawler01/catkin_ws/src/particle_filter_IWAMP/edge_info_CAD/");
        meas_model = new MeasurementModel<ColumnVector,ColumnVector>(meas_pdf);


        // ***************************
        // * Linear prior DENSITY     *
        // **************************
        // Continuous Gaussian prior (for Kalman filters)
        ColumnVector prior_Mu(STATE_SIZE);
        prior_Mu(1) = PRIOR_MU_X;
        prior_Mu(2) = PRIOR_MU_Y;
        prior_Mu(3) = PRIOR_MU_THETA;
        SymmetricMatrix prior_Cov(STATE_SIZE);
        prior_Cov(1,1) = PRIOR_COV_X;
        prior_Cov(1,2) = 0.0;
        prior_Cov(1,3) = 0.0;
        prior_Cov(2,1) = 0.0;
        prior_Cov(2,2) = PRIOR_COV_Y;
        prior_Cov(2,3) = 0.0;
        prior_Cov(3,1) = 0.0;
        prior_Cov(3,2) = 0.0;
        prior_Cov(3,3) = PRIOR_COV_THETA;
        Gaussian prior_cont(prior_Mu,prior_Cov);

        // Discrete prior for Particle filter (using the continuous Gaussian prior)
        vector<Sample<ColumnVector> > prior_samples(NUM_SAMPLES);
        prior_discr = new MCPdf<ColumnVector>(NUM_SAMPLES,STATE_SIZE);
        prior_cont.SampleFrom(prior_samples,NUM_SAMPLES,CHOLESKY,NULL);
        prior_discr->ListOfSamplesSet(prior_samples);

        /******************************
        * Construction of the Filter *
        ******************************/
        filter = new CustomParticleFilter (prior_discr, 0.5, NUM_SAMPLES/2.0);
    }

 //    //Used in orientationCb
 //    //@Ensures: radiant to degree
	// int radiantToDegree(double radiant)
	// {
	// 	radiant = radiant * 180.0/M_PI;	
	// 	radiant = radiant + 0.5;
	// 	int degree = (int) radiant;
	// 	return degree;
	// }

	//Used to specify the orientation of templates that measurement model search 
    void orientationCb(const crawler_msgs::VisualHeading& msg)
	{
        odometry_rotation = static_cast<int>(msg.RPY_degree.z);
		//Pass the orientation to measurement model
		meas_pdf->orient = odometry_rotation;
	}

	//Received Images from the robot and compare them with templates 
    void MeasurementCb(const sensor_msgs::ImageConstPtr& msg)
    {    	
        cv_bridge::CvImagePtr cv_ptr;
        cv_bridge::CvImage out_msg;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception&  e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        // Meaningless in our situation
        ColumnVector measurement(1);
        measurement(1) = 0.0;
            
        meas_pdf->NonlinearMeasurementImageInput(cv_ptr->image);  

        // "particle_filter_node.cpp" <=> MeasurementCb : Update (meas_model, measurement);
        // "filter.cpp" <=> Update : UpdateInternal(NULL,u(meaningless),meas_model,measurement,s(meaningless));
        // "particlefilter.cpp" <=> UpdateInternal : UpdateWeightsInternal(sysmodel(NULL),u(meaningless),meas_model,measurement,s(meaningless));
        // "partilcefilter.cpp" <=> UpdateWeightsInternal : meas_model->ProbabilityGet(measurement,x_new(samples's value));
        filter->Update(meas_model, measurement);

        // plot edge map
        Mat overlayEdgemap = meas_pdf->img_similarity->getOverlayEdgeMap();
        // imshow("map", overlayEdgemap);
        // waitKey(3);

        // while (waitKey(100) == -1) {}

        PublishParticles();
        PublishPose();
    }

    void InputCb(crawler_msgs::JointCmd msg)
    {
    	time_new = clock();
    	double dt = (clock() - time_old) / CLOCKS_PER_SEC;

        // Parsing the command message to odometry info
        ColumnVector input(2);

        double linear_vel_scale = 0.01858; // motor model: cmd <=> PWM <=> U, U=0.02n-0.005; pay attention to the sign.
        double linear_vel_intercept = -0.003375;
        double linear_vel = (msg.jointCmdVel[0]-msg.jointCmdVel[1]) * linear_vel_scale + linear_vel_intercept;
        odometry_linear = linear_vel * dt; // left caterpillar command

		input(1) = odometry_linear;
        input(2) = odometry_rotation - odometry_rotation_old;
        
        time_old = time_new;
        odometry_rotation_old = odometry_rotation;
        
        filter->Update(sys_model,input);
    }

    void PublishParticles()
    {
        geometry_msgs::PoseArray particles_msg;
        particles_msg.header.stamp = ros::Time::now();
        particles_msg.header.frame_id = "/map";

        vector<WeightedSample<ColumnVector> >::iterator sample_it;
        vector<WeightedSample<ColumnVector> > samples;

        samples = filter->getNewSamples();

        // ROS_INFO("Particle size = %d\n", samples.size());

        double length = 0.27; // The length between camera and the center of two wheels, 11 inch
        double offset = 0.10; // The offset of the center of camera on x axis, 4 inch

        for(sample_it = samples.begin(); sample_it<samples.end(); sample_it++)
        {
            geometry_msgs::Pose pose;
            ColumnVector sample = (*sample_it).ValueGet();

            // Move the particle cloud to the center of mobile robot
            pose.position.x = sample(1) - cos(sample(3))*offset - sin(sample(3))*length;
            pose.position.y = sample(2) + sin(sample(3))*offset - cos(sample(3))*length;
            pose.orientation.z = sample(3);

            particles_msg.poses.insert(particles_msg.poses.begin(), pose);
        }

        particle_pub.publish(particles_msg);
    }

    void PublishPose()
    {
        Pdf<ColumnVector> * posterior = filter->PostGet();
        ColumnVector pose = posterior->ExpectedValueGet();
        SymmetricMatrix pose_cov = posterior->CovarianceGet();

        geometry_msgs::PoseStamped pose_msg;
        pose_msg.header.stamp = ros::Time::now();
        pose_msg.header.frame_id = "/pose";

        // Move the calculated position to the center of mobile robot
        double length = 0.27; // The length between camera and the center of two wheels, 11 inch
        double offset = 0.10; // The offset of the center of camera on x axis, 4 inch
        pose_msg.pose.position.x = pose(1) - cos(pose(3))*offset - sin(pose(3)) * length;
        pose_msg.pose.position.y = pose(2) + sin(pose(3))*offset - cos(pose(3)) * length;
        pose_msg.pose.position.z = 0.35;
        pose_msg.pose.orientation.z = pose(3);

        pose_pub.publish(pose_msg);
    }

};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "ParticleFilterNode");
    ParticleFilterNode pfNode;
    ros::spin();
    return 0;
}
