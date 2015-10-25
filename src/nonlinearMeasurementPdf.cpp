#include "nonlinearMeasurementPdf.h"
#include <wrappers/rng/rng.h> // Wrapper around several rng libraries
#include <ros/ros.h>

#define MEASMODEL_NUMCONDARGUMENTS_MOBILE 1
#define MEASMODEL_DIMENSION_MOBILE        3

namespace BFL
{
    using namespace MatrixWrapper;
    using namespace cv;

    NonlinearMeasurementPdf::NonlinearMeasurementPdf(const Gaussian& measNoise)
    : ConditionalPdf<ColumnVector,ColumnVector>(MEASMODEL_DIMENSION_MOBILE,MEASMODEL_NUMCONDARGUMENTS_MOBILE)
    {
        _measNoise = measNoise;
    }

    NonlinearMeasurementPdf::~NonlinearMeasurementPdf(){}

    //receive images from ParticleFilterNode::MeasurementCb
    void NonlinearMeasurementPdf::NonlinearMeasurementImageInput(const Mat& src)
    {
        _image = src;

        //construction of template comparison
        img_similarity = new ImageSimilarity(_image);
    }

    //Calculate the probability of each sample 
    Probability NonlinearMeasurementPdf::ProbabilityGet(const ColumnVector& measurement) const
    {

        ColumnVector state = ConditionalArgumentGet(0);

        Probability prb;
        ColumnVector score_calculated(1);

        img_similarity->orient = orient;
        // Compute the similarity score
        img_similarity->SetEdgeInfoPath(edgeInfoPath);   
        score_calculated(1) = (img_similarity->TemplatesComparison(state))/10.0;//10.0;  

        prb = _measNoise.ProbabilityGet(score_calculated);
           
        // std::cout << "score_calculated = " << score_calculated(1) << std::endl;
        // std::cout << "probability = " << prb.getValue() << std::endl;
        return prb;
    }

    void NonlinearMeasurementPdf::SetEdgeInfoPath(string edge_info)
    {
    	//Set the path to fetch templates
        edgeInfoPath = edge_info.c_str();
    }

}//namespace BFL