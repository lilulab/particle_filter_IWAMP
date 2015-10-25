#ifndef __NON_LINEAR_MEAS_MOBILE__
#define __NON_LINEAR_MEAS_MOBILE__

#include <pdf/conditionalpdf.h>
#include <pdf/gaussian.h>
#include <opencv2/core/core.hpp>
#include "imageSimilarity.h"


namespace BFL
{
    using namespace cv;
    using namespace std;

    class ImageSimilarity;

    /// Non Linear Conditional Gaussian
    class NonlinearMeasurementPdf : public ConditionalPdf<MatrixWrapper::ColumnVector, MatrixWrapper::ColumnVector>
    {
    public:
        /// Constructor
        /**
        @param additiveNoise Pdf representing the additive Gaussian uncertainty
        */
        int orient;

        NonlinearMeasurementPdf( const Gaussian& measNoise);

        /// Destructor
        virtual ~NonlinearMeasurementPdf();

        // implement this virtual function for measurement model of a particle filter
        virtual Probability ProbabilityGet(const MatrixWrapper::ColumnVector& measurement) const;

        void NonlinearMeasurementImageInput(const Mat& src);

        void SetEdgeInfoPath(string edge_info);

        ImageSimilarity* img_similarity;

    private:
        Mat _image;

        string edgeInfoPath;

        Gaussian _measNoise;
    };

} // End namespace BFL

#endif //
