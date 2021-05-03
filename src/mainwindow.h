#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMetaType>
#include <QThread>
#include <QMainWindow>
#include <QWidget>
#include <QMenu>
#include <QGroupBox>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QStatusBar>
#include <QRadioButton>
#include <QPushButton>
#include <QCheckBox>
#include <QPushButton>
#include <QSlider>
#include <QDoubleSpinBox>
#include <QtCore/QTimer>
#include <QtCore/QString>
#include <QLabel>
#include <QSizePolicy>
#include <QWidget>
#include <QVTKOpenGLNativeWidget.h>

#include "camera.h"
#include "rs_cam.h"
#include "camerawidget.h"
#include "modelwidget.h"
#include "normalswidget.h"
#include "photometricstereo.h"

class MainWindow : public QMainWindow {
    Q_OBJECT
    
public:
    MainWindow(QWidget *parent = 0);
    ~MainWindow();
    
public slots:
    void setStatusMessage(QString msg);
    void onModelFinished(std::vector<cv::Mat> MatXYZN);
        
private slots:
    void onTestModeChecked(int state);
    void onViewRadioButtonsChecked(bool checked);
    void onToggleSettingsMenu();
    
private:
    void createInterface();
    
    vtkSmartPointer<vtkRenderWindow> renderWindow;
    QVTKOpenGLNativeWidget *centralWidget;
    QGridLayout *gridLayout, *radioButtonsLayout, *paramsLayout;
    QLabel *maxpqLabel, *lambdaLabel, *muLabel, *minIntensLabel, *unsharpNormsLabel;
    QDoubleSpinBox *maxpqSpinBox, *lambdaSpinBox, *muSpinBox;
    QSlider *minIntensSlider, *unsharpNormSlider;
    QGroupBox *paramsGroupBox;
    QPushButton *exportButton, *toggleSettingsButton;
    QRadioButton *normalsRadioButton, *surfaceRadioButton;
    QCheckBox *testModeCheckBox;
    QPushButton *calibrateButton;
    QPushButton *screenshotButton;
    QPushButton *lightButton;
    QPushButton *save_maskButton;
    QThread *camThread;
    
    // Camera *camera;
    RsCam *camera;
    CameraWidget *camWidget;
    ModelWidget *modelWidget;
    NormalsWidget *normalsWidget;
    PhotometricStereo *ps;
};

#endif
