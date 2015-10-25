#include "imageSimilarity.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string.h>
#include <stdio.h>
#include <iomanip>
#include <locale>
#include <cmath>
#include <math.h>
#include <ros/ros.h>
#include "constants.h"

namespace BFL
{
    using namespace MatrixWrapper;

	ImageSimilarity::ImageSimilarity(const Mat& src)
	{
        image = src;
        //Roate the image for 180 degree
        // imageRotate(image);   
        //Detect edges         
        findLines(image);
        //Generate DT image
        DTImageGeneration();
	}

    ImageSimilarity::~ImageSimilarity(){}
	

    /**
    * Rotate the image from V-REP 180 degree.
    */ 
    void ImageSimilarity::imageRotate(Mat& src)
    {
        Point center = Point(src.cols/2.0, src.rows/2.0);
        Mat rot_mat = getRotationMatrix2D(center, 180, 1.0);
            
        warpAffine(src, src, rot_mat, src.size());
    }

    /** 
    * Create an edge map of the input image.
    */
    void ImageSimilarity::findLines(const Mat& src)
    {
        Mat image;
        vector<Vec4i> lines;

        //Canny edge detection variables
        int edgeThresh = 1;
        int lowThreshold = 50;
        int const max_lowThreshold = 100;
        int ratio = 2;
        int kernel_size = 3;

        //Setup Hough Transform inputs
        //Hough Transform variables
        double rho = 1;
        double theta = CV_PI/180;
        int pointThreshold = 40; //points needed to form line
        int minLineLength = 30;
        int maxLineGap = 40;
          
        //Make image and stream be the same resolution
        cvtColor(src, image, CV_BGR2GRAY); //convert to grayscale for the edge detector
        Canny( image, image, lowThreshold, lowThreshold*ratio, kernel_size ); 

        //Find lines in the Canny image
        HoughLinesP( image, lines, rho, theta, pointThreshold, minLineLength, maxLineGap); 

        //Lines from input image
        //Edge map
        image_edge = Mat::ones(src.size(),src.depth()) * 255;
        for(size_t i = 0; i < lines.size(); i++)
        {
            line( image_edge, Point(lines[i][0], lines[i][1]), Point(lines[i][2], lines[i][3]), Scalar(0,0,0), 1, 4); 
        }
    }

    /**
    * Create a distance trasform image of the edge map.
    */ 
    void ImageSimilarity::DTImageGeneration()
    {
        Mat image_DT_temp;
        distanceTransform(image_edge,image_DT_temp,CV_DIST_L1,3);
        threshold(image_DT_temp,image_DT_temp,truncate,truncate,THRESH_TRUNC);
        image_DT_temp.convertTo(image_DT,CV_8U);
    }  
 
    /**
    * Calculate the similarity score of the DT_image given the edge information
    * of one edge map.
    */ 
    double ImageSimilarity::AlignmentScore_DT(vector<Vec4i> oneEdgeMap)
    {
        unsigned int distanceSum = 0;
        unsigned int pointCount = 0;
        unsigned char *imageData = (unsigned char*)(image_DT.data);

        for(size_t i = 0; i < oneEdgeMap.size(); i++)
        {
            LineIterator iter(image_DT, Point(oneEdgeMap[i][0],oneEdgeMap[i][1]), \
            Point(oneEdgeMap[i][2], oneEdgeMap[i][3]),4);
            pointCount += iter.count;
            for(int q = 0; q < iter.count; q++)
            {
                distanceSum += imageData[image_DT.step * iter.pos().y+iter.pos().x];
            }
        }
          
        if (pointCount == 0)
            return truncate;
        else
            return (double)distanceSum/pointCount;
    }

    /**
    * Extract the edge count from the txt file which contains edge coordinates.
    */ 
    int ImageSimilarity::ExtractEdgeCount(char *line)
    {
        if (strlen(line) == 0)
            return -1;

        int edge_count;
        char *token;
        char delim[] = " ";

        token = strtok(line, delim);
        if (strcmp(token, "*") != 0)
        {
            cerr << "Incorrect format!" << endl;
            return -1;
        }

        if ((token = strtok(NULL, delim)) == NULL)
        {
            cerr << "Incorrect format!" << endl;
            return -1;
        }

        if ((token = strtok(NULL, delim)) == NULL)
        {
            cerr << "Incorrect format!" << endl;
            return -1;
        }

        edge_count = atoi(token);   
        // cout << "edge count = " << edge_count << endl;

        return edge_count;
    }

    /**
    * Extract edge coordinates from the txt file which contains edge coordinate.
    */
    vector<Vec4i> ImageSimilarity::ExtractEdgeCoordinates(char *line, int edge_count)
    {    
        vector<Vec4i> oneEdgeMap;

        if (strlen(line) == 0)
            return oneEdgeMap;

        char *token;
        char delimLine[] = ";";
        char delimPoint[] = " ";
        Vec4i temp_points;

        int count = 0;
        token = strtok(line, delimPoint);
        temp_points[0] = atoi(token);

        for (count = 1; count < edge_count*4; count++)
        {
            int index = count % 4;
            switch (index)
            {
                case 0:
                    token = strtok(NULL, delimPoint);
                    temp_points[0] = atoi(token);
                break;

                case 1:
                    token = strtok(NULL, delimPoint);
                    temp_points[1] = atoi(token);
                break;

                case 2:
                    token = strtok(NULL, delimPoint);
                    temp_points[2] = atoi(token);
                break;

                case 3:
                    token = strtok(NULL, delimLine);
                    temp_points[3] = atoi(token);
                    oneEdgeMap.push_back(temp_points);
                break;
            }
        }
        return oneEdgeMap;
    }
        
    /**
    * Convert rotation angle (theta) to quaternion which is used when publishing messages to V-REP.
    */ 
    void ImageSimilarity::Theta2Quaternion(geometry_msgs::PoseStamped& state_message, double theta)
    {
        double q1 = 0;
        double q2 = 0;
        double q3 = sin(theta/2);
        double q4 = cos(theta/2);

        state_message.pose.orientation.x = q1; 
        state_message.pose.orientation.y = q2;
        state_message.pose.orientation.z = q3;
        state_message.pose.orientation.w = q4; 
    }

    void ImageSimilarity::SetEdgeInfoPath(string edge_info)
    {
        _edgeInfoPath = edge_info.c_str();
    }

    /**
    * Convert radian to degree.
    */ 
    int ImageSimilarity::radianToDegree(double radian)
    {
        radian = radian * 180.0/M_PI; 
        radian = radian + 0.5;
        int degree = (int) radian;
        return degree;
    }

    /**
    * Calculate and return the best similarity score given previous state.
    * This function will be called in nonlinearMeasurementPdf.cpp
    */
    double ImageSimilarity::TemplatesComparison(ColumnVector prevState) 
    {
        geometry_msgs::PoseStamped state_msg;
        double best_score = truncate;   

        const int MAXLINE = 1024;

        double x = prevState(1)*100.0;
        double y = prevState(2)*100.0;
        // int orientation = (int)(prevState(3)/M_PI*180);

        //convert the radian to an even number
        if ((int)(x+0.5) %2 != 0)
        {
            x = (int)(x+1.5);
            x = static_cast<double>(x/100.0);
        }
        else
        {
            x = (int)(x+0.5);
            x = static_cast<double>(x/100.0);
        }

        if ((int)(y+0.5) %2 != 0)
        {
            y = (int)(y+1.5);
            y = static_cast<double>(y/100.0);
        }
        else
        {
            y = (int)(y+0.5);
            y = static_cast<double>(y/100.0);
        }

        // If the particle is out of boundary, 
        // simply return the largest score to save computation cost
        if (x > X_MAX || x < X_MIN || y > Y_MAX || y < Y_MIN)
            return truncate;

        char filename[100];
        sprintf(filename, "%s%1.3f_%1.3f.txt",_edgeInfoPath.c_str(), x, y);
        // ROS_INFO("(%f,%f)",x,y);

        ifstream inFile (filename);
        char oneline[MAXLINE];
        inFile.getline(oneline, MAXLINE); // initial check

        bool isInfoFlag = true;
        int edge_count = 0;
        int theta = -25;
        while (inFile) 
        {
            if (isInfoFlag)
            {
                // Check current theta and the edge count
                inFile.getline(oneline, MAXLINE);   
                edge_count = ExtractEdgeCount(oneline);

                if (edge_count > 0)
                    isInfoFlag = false;     
            }
            else
            {
                // Get the coordinates if the edge count is not zero.
                char coordinate_line[edge_count*32];
                inFile.getline(coordinate_line, edge_count*32);     
                  
                if (edge_count > 20 && theta == orient)// && theta == orientation) // For test only!!!!!!!!!!!!
                {
                    vector<Vec4i> oneEdgeMap = ExtractEdgeCoordinates(coordinate_line, edge_count);

                    double score_tmp = AlignmentScore_DT(oneEdgeMap);

                    if (score_tmp < best_score)
                        best_score = score_tmp;

                    // ROS_INFO("%f",score_tmp);  
                    break; 
                }                        
                isInfoFlag = true;
                theta++;
            }
        }

        inFile.close();

// printf("(%f,%d)", best_score, theta);

        // if (!isfinite(best_score)) printf("%s\n","The score is infinite!!!\n");
        // ROS_INFO("best score = %f",best_score); 
        return best_score;
    }

    /**
    * Calculate and return the best similarity score given previous state.
    * This function will be called in nonlinearMeasurementPdf.cpp
    */
    Mat ImageSimilarity::getOverlayEdgeMap()
    {
        Mat image_edge_depth, overlay;
        
        cvtColor(image_edge, image_edge_depth, CV_GRAY2BGR);
        addWeighted(image, 1, image_edge_depth, 1, -255, overlay);

        return overlay;
    }

};
