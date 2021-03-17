#ifndef CAMERAWIDGET_H
#define CAMERAWIDGET_H

#include <vtkSmartPointer.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkImageData.h>
#include <vtkImageActor.h>
#include <vtkImageImport.h>
#include <vtkRenderWindow.h>
#include <vtkCamera.h>
#include <vtkMatrixToHomogeneousTransform.h>
#include <vtkMatrix4x4.h>

#include <QWidget>
#include <QVTKOpenGLNativeWidget.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>


class CameraWidget : public QVTKOpenGLNativeWidget {
    Q_OBJECT

public:
    CameraWidget(QVTKOpenGLNativeWidget *parent = 0, int width=0, int height=0);
    ~CameraWidget();

public slots:
    void setImage(cv::Mat image);

private:
    /** vtk renderer */
    vtkSmartPointer<vtkRenderer> renderer;
    /** vtk image actor for displaying the current frame */
    vtkSmartPointer<vtkImageActor> imgActor;
    /** vtk image data store for the current frame */
    vtkSmartPointer<vtkImageData> imgData;
    /** vtk image importer for converting a CvMat to vtk image data */
    vtkSmartPointer<vtkImageImport> importer;
    /** setup widget for using vtk drawing camera images */
    void setup(cv::Mat& image);
};

#endif
