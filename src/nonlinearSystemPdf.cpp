#include "nonlinearSystemPdf.h"
#include <wrappers/rng/rng.h> // Wrapper around several rng libraries

#define SYSMODEL_NUMCONDARGUMENTS_MOBILE 2
#define SYSMODEL_DIMENSION_MOBILE        3

namespace BFL
{
    using namespace MatrixWrapper;

    NonlinearSystemPdf::NonlinearSystemPdf(const Gaussian& additiveNoise)
      : ConditionalPdf<ColumnVector,ColumnVector>(SYSMODEL_DIMENSION_MOBILE,SYSMODEL_NUMCONDARGUMENTS_MOBILE)
    {
        _additiveNoise = additiveNoise;
    }

    NonlinearSystemPdf::~NonlinearSystemPdf(){}

    bool NonlinearSystemPdf::SampleFrom (Sample<ColumnVector>& one_sample, int method, void * args) const
    {
        ColumnVector state = ConditionalArgumentGet(0);
        ColumnVector odometry = ConditionalArgumentGet(1);

        ColumnVector centerState(3);
        double length = 0.27; // The length between camera and the center of two wheels, 11 inch
        double offset = 0.10; // The offset of the center of camera on x axis, 4 inch
        centerState(1) = state(1) - cos(state(3))*offset - sin(state(3))*length;
        centerState(2) = state(2) + sin(state(3))*offset - cos(state(3))*length;
        centerState(3) = state(3);

        // system update
        centerState(1) += sin(centerState(3) + odometry(2)/2) * odometry(1);
        centerState(2) += cos(centerState(3) + odometry(2)/2) * odometry(1);
        centerState(3) += odometry(2);

        state(1) = centerState(1) + cos(centerState(3))*offset + sin(centerState(3))*length;
        state(2) = centerState(2) - sin(centerState(3))*offset + cos(centerState(3))*length;
        state(3) = centerState(3);

        // sample from additive noise
        Sample<ColumnVector> noise;
        _additiveNoise.SampleFrom(noise, method, args);

        // store results in one_sample
        one_sample.ValueSet(state + noise.ValueGet());

        return true;
    }

}//namespace BFL

