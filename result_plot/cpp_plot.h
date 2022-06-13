#ifndef CPP_PLOT
#define CPP_PLOT

#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;

void Show_Result(const std::vector<double>& x, const std::vector<double>& y, const std::vector<double>& z){
  plt::clf();
  // Plot line from given x and y data. Color is selected automatically.
  plt::plot(x, y); 


  // Add graph title
  plt::title("Sample figure");
  // Enable legend.
  plt::legend();
  // Display plot continuously 
  plt::pause(0.1);

}

#endif