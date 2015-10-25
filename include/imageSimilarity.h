#ifndef __IMAGE_SIMILARITY_
#define __IMAGE_SIMILARITY_

#include "nonlinearMeasurementPdf.h"
#include <wrappers/rng/rng.h> // Wrapper around several rng libraries
#include <geometry_msgs/PoseStamped.h>
#include <opencv2/core/core.hpp>

namespace BFL
{
    using namespace cv;
    
    class ImageSimilarity
    {
    private:
        Mat image, image_edge, image_DT;
        
    public:
        // Consrtuctor
        ImageSimilarity(const Mat& src);
        
        // Destructor
        ~ImageSimilarity();

        int orient;

        void DTImageGeneration();

        double TemplatesComparison(MatrixWrapper::ColumnVector prevState);

        void NonlinearMeasurementImageInput(Mat& image);

        void SetEdgeInfoPath(string edge_info);

        Mat getOverlayEdgeMap();

        
    private:
     
        const double truncate = 40.0;

        string _edgeInfoPath;

        geometry_msgs::PoseStamped previousState;
     
        void imageRotate(Mat& src);

        double AlignmentScore_DT(vector<Vec4i> oneEdgeMap);

        void findLines(const Mat& src);
     
        int ExtractEdgeCount(char *line);

        vector<Vec4i> ExtractEdgeCoordinates(char *line, int edge_count);

        void Theta2Quaternion(geometry_msgs::PoseStamped& state_message, double theta);

        double GetFloatPrecision(double value, double precision);

        int radianToDegree(double radiant);
        
        Vec4i OrderEndPoints(Vec4i new_line);
        
        double PointDistance(int old_line_x,int old_line_y,int new_line_x,int new_line_y);

    };
}

#endif
