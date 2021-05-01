#ifndef UTILS_H
#define UTILS_H

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <vtk-9.0/vtkMath.h>
#include <vtk-9.0/vtkSmartPointer.h>
#include <vtk-9.0/vtkActor.h>
#include <vtk-9.0/vtkCamera.h>
#include <vtk-9.0/vtkFloatArray.h>
#include <vtk-9.0/vtkHedgeHog.h>
#include <vtk-9.0/vtkMath.h>
#include <vtk-9.0/vtkPointData.h>
#include <vtk-9.0/vtkPoints.h>
#include <vtk-9.0/vtkPolyDataMapper.h>
#include <vtk-9.0/vtkProperty.h>
#include <vtk-9.0/vtkRenderWindow.h>
#include <vtk-9.0/vtkRenderWindowInteractor.h>
#include <vtk-9.0/vtkRenderer.h>
#include <vtk-9.0/vtkStructuredGrid.h>

#include "config.h"

class Utils {
  
public:
    static int diplayLightDirections();
};

#endif