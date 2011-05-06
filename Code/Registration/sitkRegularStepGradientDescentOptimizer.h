#ifndef __sitkRegularStepGradientDescentOptimizer_h
#define __sitkRegularStepGradientDescentOptimizer_h

#include "sitkMacro.h"
#include "sitkDetail.h"
#include "sitkImage.h"
#include "sitkOptimizer.h"
#include "itkRegularStepGradientDescentOptimizer.h"
#include "sitkMemberFunctionFactory.h"

namespace itk
{
namespace simple
{

  class RegularStepGradientDescentOptimizer : public Optimizer
  {
  public:
    RegularStepGradientDescentOptimizer();
    virtual ~RegularStepGradientDescentOptimizer();
    virtual ::itk::Optimizer::Pointer GetOptimizer();

    RegularStepGradientDescentOptimizer& SetNumberOfIterations ( uint32_t n );
    uint32_t GetNumberOfIterations();
    RegularStepGradientDescentOptimizer& SetMinimumStepLength ( float n );
    float GetMinimumStepLength();
    RegularStepGradientDescentOptimizer& SetMaximumStepLength ( float n );
    float GetMaximumStepLength();

  protected:
    uint32_t m_NumberOfIterations;
    float m_MinimumStepLength;
    float m_MaximumStepLength;
    Optimizer* Clone();
  };
}
}

#endif
