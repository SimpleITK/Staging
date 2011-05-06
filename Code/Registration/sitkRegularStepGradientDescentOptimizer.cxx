#include "sitkRegularStepGradientDescentOptimizer.h"

namespace itk
{
namespace simple
{

  Optimizer* RegularStepGradientDescentOptimizer::Clone()
  {
    return new RegularStepGradientDescentOptimizer ( *this );
  }
  RegularStepGradientDescentOptimizer::RegularStepGradientDescentOptimizer()
  {
  this->m_NumberOfIterations = 4000;
  this->m_MinimumStepLength = 0.000005;
  this->m_MaximumStepLength = 1.0;
  }
  RegularStepGradientDescentOptimizer::~RegularStepGradientDescentOptimizer()
  {
  }

  RegularStepGradientDescentOptimizer& RegularStepGradientDescentOptimizer::SetNumberOfIterations ( uint32_t n )
  {
  this->m_NumberOfIterations = n;
  return *this;
  }
  uint32_t RegularStepGradientDescentOptimizer::GetNumberOfIterations()
  {
  return this->m_NumberOfIterations;
  }
  RegularStepGradientDescentOptimizer& RegularStepGradientDescentOptimizer::SetMinimumStepLength ( float n )
  {
  this->m_MinimumStepLength = n;
  return *this;
  }
  float RegularStepGradientDescentOptimizer::GetMinimumStepLength()
  {
  return this->m_MinimumStepLength;
  }
  RegularStepGradientDescentOptimizer& RegularStepGradientDescentOptimizer::SetMaximumStepLength ( float n )
  {
  this->m_MaximumStepLength = n;
  return *this;
  }
  float RegularStepGradientDescentOptimizer::GetMaximumStepLength()
  {
  return this->m_MaximumStepLength;
  }


  ::itk::Optimizer::Pointer RegularStepGradientDescentOptimizer::GetOptimizer()
  {
    ::itk::RegularStepGradientDescentOptimizer::Pointer optimizer = ::itk::RegularStepGradientDescentOptimizer::New();
    optimizer->SetNumberOfIterations ( this->m_NumberOfIterations );
    optimizer->SetMinimumStepLength ( this->m_MinimumStepLength );
    optimizer->SetMaximumStepLength ( this->m_MaximumStepLength );
    optimizer->SetMinimize(false);
    optimizer->Print ( std::cout );
    return optimizer.GetPointer();
  }


}
}


