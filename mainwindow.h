#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <iostream>
#include <vector>

#include <QObject>
#include <QThread>
#include <QRunnable>
#include <QTimer>
#include <QMainWindow>
#include <QString>
#include <QProgressDialog>
#include <qtconcurrentrun.h>
#include <QtConcurrent>

#include <boost/thread/mutex.hpp>
#include <boost/thread.hpp>

#include <pcl/io/openni2_grabber.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <vtkRenderWindow.h>

#include "PCDIO.h"
#include "PCLOperations.h"
#include "types.h"
#include "capture.h"

namespace Ui {
    class MainWindow;
}

class MainWindow : public QMainWindow, public QRunnable {
    Q_OBJECT
public:
    typedef pcl::visualization::PCLVisualizer Visualizer;
    typedef boost::shared_ptr<Visualizer> VisualizerPtr;
    static const int STEP_COUNT_PER_ROTATE = 50;
    
    explicit MainWindow(QWidget *parent = 0);
    virtual ~MainWindow();
    void cloudCallBack(const GrayCloudConstPtr& cloud);

	void
		keyboard_callback(const pcl::visualization::KeyboardEvent& event, void*)
	{
		if (event.getKeyCode())
			cout << "the key \'" << event.getKeyCode() << "\' (" << event.getKeyCode() << ") was";
		else
			cout << "the special key \'" << event.getKeySym() << "\' was";
		if (event.keyDown())
			cout << " pressed" << endl;
		else
			cout << " released" << endl;
	}

	void
		mouse_callback(const pcl::visualization::MouseEvent& mouse_event, void*)
	{
		if (mouse_event.getType() == pcl::visualization::MouseEvent::MouseButtonPress && mouse_event.getButton() == pcl::visualization::MouseEvent::LeftButton)
		{
			cout << "left button pressed @ " << mouse_event.getX() << " , " << mouse_event.getY() << endl;
		}
	}

    void meshToTriangle();
    void run();
    void updateClippingBox();

    inline bool isCapturing() { return _capture; }
    inline void capture() { _capture = !_capture; }
    inline int getProgress() { return _progress; }

    //inline float getXMin () const {return (_x_min);}
    //inline float getXMax () const {return (_x_max);}
    //inline float getYMin () const {return (_y_min);}
    //inline float getYMax () const {return (_y_max);}
    //inline float getZMin () const {return (_z_min);}
    //inline float getZMax () const {return (_z_max);}

    inline int getCaptureCount () const {return (_captureCount);}

public slots:
    void setCaptureCount (int count);
    void onCaptureClick();
    void onRefreshClick();
    void onProgressUpdate(int progress, const QString& statusMsg);
    void visualize();
    void showCloudMesh(bool status);

signals:
    void updateKinectPixmap(QPixmap pixmap);

private:
    Ui::MainWindow *ui;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> _visualizer;
    
    GrayCloudConstPtr _latestCloud;
    GrayCloudConstPtr _combinedCloud;
    Mesh _combinedMesh;
    mutable boost::mutex _cloud_mutex;
    mutable boost::mutex _visualizer_mutex;
    std::vector<GrayCloudPtr> _clouds;
    pcl::Grabber* _grabber;
    bool _showMesh;
    QTimer *_vis_timer;
    QTimer *_oneTimeEvent;
    bool running;
    bool _capture;
    int _progress;
    int _captureCount;

	//Capture* _capturer;

    QProgressDialog *_progressDialog;

    inline void setProgress(int progress) { _progress = progress; }
    void progressUpdate(int progress,const char* status);
    int getSleepTime();
    
    void refreshFormElements();
    void SensorStatusOff();
    void SensorStatusOn();
    void SensorStatusDisconnected();

	
private slots:
    void init();
};

#endif // MAINWINDOW_H
