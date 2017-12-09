#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QMessageBox>


#include <pcl/io/ply_io.h>

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow),
    _grabber(new pcl::io::OpenNI2Grabber()),
    _showMesh(true),
    running(false),
    _capture(false),
    _captureCount(5)
{
    
    ui->setupUi(this);

	// qt connections
	connect(ui->captureButton, SIGNAL(clicked()), this, SLOT(onCaptureClick()));

    std::cerr << "Creating Visualizer" << std::endl;
    _visualizer.reset (new pcl::visualization::PCLVisualizer ("", false));
    ui->vtkWidget->SetRenderWindow (_visualizer->getRenderWindow());
    _visualizer->setupInteractor (ui->vtkWidget->GetInteractor(), ui->vtkWidget->GetRenderWindow());
    _visualizer->getInteractorStyle ()->setKeyboardModifier (pcl::visualization::INTERACTOR_KB_MOD_SHIFT);
    ui->vtkWidget->update();
    
   _vis_timer = new QTimer (this);
    connect (_vis_timer, SIGNAL (timeout ()), this, SLOT (visualize()));
    _vis_timer->start (5);//5ms
    
    _oneTimeEvent = new QTimer(this);
    _oneTimeEvent->setInterval(50);
    _oneTimeEvent->setSingleShot(true);
    connect (_oneTimeEvent, SIGNAL (timeout ()), this, SLOT (init()));
    _oneTimeEvent->start();
}

void MainWindow::init() {
    ui->captureCountSpinBox->setValue(getCaptureCount());
    
    _visualizer->addCoordinateSystem(1);
    _visualizer->initCameraParameters();
	_visualizer->setCameraPosition(0,0,-5,0,-1,0);
    //_visualizer->setBackgroundColor(1,1,1);
    _visualizer->addText("SucaBot MedTech", 10, 10, "v1 text", 0);
    ui->vtkWidget->update();

	// setup capture class
	 //_capturer = new Capture;
	//_capturer->SetVisualizer(_visualizer&);

    progressUpdate(5,"Initialized");
    _oneTimeEvent->stop();
    delete _oneTimeEvent;    
}

MainWindow::~MainWindow() {
    if(_grabber->isRunning())
        _grabber->stop();
    delete _grabber;

    if(_vis_timer->isActive())
        _vis_timer->stop();
    delete _vis_timer;
    delete ui;
    //delete scanner;
}

void MainWindow::visualize() {
    //updateClippingBox();
    
    if(_latestCloud) {
       GrayCloudPtr tmp(new GrayCloud),
                      cloud_out(new GrayCloud);
        boost::mutex::scoped_lock lock(_cloud_mutex);
        pcl::copyPointCloud(*_latestCloud,*tmp);

        lock.unlock();
        //PCLOperations::clip(tmp,cloud_out,_x_min,_x_max,_y_min,_y_max,_z_min,_z_max);
        //PCLOperations::translateCloud(cloud_out,-(_x_min+(_x_max-_x_min)/2.0),-(_y_min+(_y_max-_y_min)/2.0),-(_z_min+(_z_max-_z_min)/2.0));
        //PCLOperations::rotateCloud(cloud_out,getCaptureCount(),_rotationCount);
        boost::mutex::scoped_lock vlock(_visualizer_mutex);
        if (!_visualizer->updatePointCloud (cloud_out, "ClippedCloudData")) {
            std::cerr << "addPointCloud()" << std::endl;
            _visualizer->addPointCloud (cloud_out, "ClippedCloudData");
        }
        vlock.unlock();

        ui->vtkWidget->update();
    }
}

void MainWindow::setCaptureCount (int count) {
    if (count > 1) 
        _captureCount = count;
}

void MainWindow::onCaptureClick() 
{
	// register keyboard and mouse
	_visualizer->registerMouseCallback(&MainWindow::mouse_callback, *this);
	_visualizer->registerKeyboardCallback(&MainWindow::keyboard_callback, *this);

	progressUpdate(2, "Connecting to device...");
	boost::function<void(const GrayCloudConstPtr&) > f =
		boost::bind(&MainWindow::cloudCallBack, this, _1);

	// register callback functions
	boost::signals2::connection connection = _grabber->registerCallback(f);

	this->SensorStatusOn();

	this->run();

	//QFuture<void> future = QtConcurrent::run(_capturer->Run());

	
    //refreshFormElements();
    //running = true;
    ////std::cerr << "Capture Count " << getCaptureCount() << std::cerr;
    //_progressDialog = new QProgressDialog("Capturing Started...", "C", 0, getCaptureCount()+1, this);
    //_progressDialog->setWindowModality(Qt::WindowModal);
    //_progressDialog->setCancelButton(NULL);
    //_progressDialog->show();
    //_progressDialog->raise();
    //_progressDialog->activateWindow();
    //run();
}

void MainWindow::onRefreshClick() {
//    updateKinectStatusDisconnected();
    _clouds.clear();
    //_latestCloud.reset(NULL);
    _visualizer->removeAllPointClouds();
    _visualizer->removeAllShapes();
    _vis_timer->stop();
    _showMesh=true;
    running=false;
    _capture=false;
}

void MainWindow::SensorStatusOff() {
    QPixmap sensorPixmap(":/icons/resources/kinect-off.png");
    emit updateKinectPixmap(sensorPixmap);
    ui->sensorStatusIcon->setToolTip("Sensor is off");
}

void MainWindow::SensorStatusOn() {
    QPixmap sensorPixmap(":/icons/resources/kinect-on.png");
    emit updateKinectPixmap(sensorPixmap);
    ui->sensorStatusIcon->setToolTip("Sensor is on");
}


void MainWindow::SensorStatusDisconnected() {
    QPixmap sensorPixmap(":/icons/resources/kinect-disconnected.png");
    emit updateKinectPixmap(sensorPixmap);
    ui->sensorStatusIcon->setToolTip("Sensor is not connected!");
}

void MainWindow::onProgressUpdate(int progress, const QString& statusMsg) {
    ui->progressBar->setFormat(QString("%1 - %2").arg("%p%", statusMsg));
    ui->progressBar->setValue(progress);
}

//void MainWindow::refreshFormElements() {
//    ui->xMinDoubleSpinBox->setEnabled(!running);
//    ui->xMaxDoubleSpinBox->setEnabled(!running);
//    ui->yMinDoubleSpinBox->setEnabled(!running);
//    ui->yMaxDoubleSpinBox->setEnabled(!running);
//    ui->zMinDoubleSpinBox->setEnabled(!running);
//    ui->zMaxDoubleSpinBox->setEnabled(!running);
//}

//void MainWindow::updateClippingBox() {
//        boost::mutex::scoped_lock lock(_visualizer_mutex);
//
//        pcl::ModelCoefficients coeff_cube; 
//        coeff_cube.values.resize(10); 
//        coeff_cube.values[0] = 0; // Translation along the X axis 
//        coeff_cube.values[1] = 0; // Translation along the Y axis 
//        coeff_cube.values[2] = 0; // Translation along the Z axis 
//        coeff_cube.values[3] = 0; 
//        coeff_cube.values[4] = 0; 
//        coeff_cube.values[5] = 0; 
//        coeff_cube.values[6] = 1; 
//        coeff_cube.values[7] = fabs(_x_max-_x_min); 
//        coeff_cube.values[8] = fabs(_y_max-_y_min); 
//        coeff_cube.values[9] = fabs(_z_max-_z_min); 
//
//        _visualizer->removeShape("ClippingBox",0);
//        _visualizer->addCube(coeff_cube,"ClippingBox");
//        _visualizer->setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 3, "ClippingBox");
//
//        PointXYZ p1,p2;
//        p1.z = p2.z = 0;
//        p1.x = p2.x = 0;
//        p1.y =_y_min;
//        p2.y = _y_max;
//        _visualizer->removeShape("MiddleLine",0);
//        _visualizer->addLine(p1,p2,1,0,0,"MiddleLine");
//}

void MainWindow::cloudCallBack(const GrayCloudConstPtr& cloud) 
{
 //   _latestCloud = cloud;

	//// point cloud process

 //   _showMesh = true;

	boost::mutex::scoped_lock lock(_cloud_mutex);
	//_latestCloud = cloud;
	_latestCloud = cloud->makeShared();
	cout << "cloud callback" << endl;
}

int MainWindow::getSleepTime() {
    return 100 * (STEP_COUNT_PER_ROTATE - getCaptureCount() + 1);
}

void MainWindow::progressUpdate(int progress,const char* status) {
    std::cerr << status << std:: endl;    
    onProgressUpdate(progress,status);
}

void MainWindow::run() 
{
	// start grabber
	_grabber->start();

	_capture = true;

	while (_capture)
	{
		cout << "prining out of mainwindow thread..." << endl;
		//_visualizer->spinOnce();

		//GrayCloudConstPtr buffer_cloud;

		//if (_latestCloud)
		//{
		//	cout << "aaa" << endl;
		//	// See if we can get a cloud
		//	if (_cloud_mutex.try_lock())
		//	{
		//		_latestCloud.swap(buffer_cloud);
		//		std::cout << "latest cloud swap success" << std::endl;
		//		_cloud_mutex.unlock();
		//	}

		//	if (buffer_cloud)
		//	{
		//		cout << "buffer cloud exist" << endl;
		//		if (!_visualizer->updatePointCloud(buffer_cloud, "cloud"))
		//		{
		//			cout << "add cloud" << endl;
		//			_visualizer->addPointCloud(buffer_cloud, "cloud");
		//			_visualizer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "cloud");
		//		}
		//		else
		//		{
		//			cout << "update cloud" << endl;
		//		}
		//	}
		//	else
		//	{
		//		cout << "buffer cloud empty" << endl;
		//	}

		//	//_combinedCloud = _latestCloud;
		//	//_latestCloud.reset();
		//}
	}
	_grabber->stop();





//    if(!_grabber->isRunning())
//        _grabber->start();
//    
//    _vis_timer->stop();
//    _progressDialog->setValue(0);
//    progressUpdate(7,"Sensor was started");
//    progressUpdate(8,"Capturing Started");
//    while(_clouds.size() <= getCaptureCount()) {
//        progressUpdate(8,"Getting Cloud");
//        GrayCloudPtr tmp(new GrayCloud),
//                      cloud_out(new GrayCloud);
//        boost::mutex::scoped_lock lock(_cloud_mutex);
//        pcl::copyPointCloud(*_latestCloud,*tmp);
//        progressUpdate(8,"Unlocking");
//        lock.unlock();
//        progressUpdate(8,"Clipping");
//        //PCLOperations::clip(tmp,cloud_out,_x_min,_x_max,_y_min,_y_max,_z_min,_z_max);
//        progressUpdate(8,"Translating");
//        //PCLOperations::translateCloud(cloud_out,-(_x_min+(_x_max-_x_min)/2.0),-(_y_min+(_y_max-_y_min)/2.0),-(_z_min+(_z_max-_z_min)/2.0));
//        progressUpdate(8,"Rotating");
//        std::cerr << "sleep time: " << getSleepTime() << std::endl;
//        //PCLOperations::rotateCloud(cloud_out,getCaptureCount(),_rotationCount++);
//        boost::this_thread::sleep(boost::posix_time::milliseconds(getSleepTime()));
//        //_motorDriver.stepLeft(50/getCaptureCount());
//        boost::this_thread::sleep(boost::posix_time::milliseconds(getSleepTime()));
//        progressUpdate(8,"Cleaning");
//        std::vector<int> indices;
//        pcl::removeNaNFromPointCloud(*cloud_out,*cloud_out, indices);
//        progressUpdate(8,"pushback");
//        _clouds.push_back(cloud_out);
//        _progressDialog->setValue(_clouds.size());
//        progressUpdate(7,"push_back end");
//    }
//    
//    
//    _grabber->stop();
//    _progressDialog->cancel();
//    delete _progressDialog;
//    _progressDialog = new QProgressDialog("Creating Mesh...", "C", 0, _clouds.size()-1, this);
//    _progressDialog->setWindowModality(Qt::WindowModal);
//    _progressDialog->setCancelButton(NULL);
//    _progressDialog->show();
//    _progressDialog->raise();
//    _progressDialog->activateWindow();
//    _progressDialog->setValue(0);
//    progressUpdate(7,"Creating transform");
//    GrayCloudPtr result (_clouds.at(0)), 
//                  source(new GrayCloud), 
//                  target(new GrayCloud);
//    Eigen::Matrix4f GlobalTransform = Eigen::Matrix4f::Identity (), 
//                    pairTransform = Eigen::Matrix4f::Identity ();
//    //ColorCloudPtr result = _clouds[0];
//    for (size_t i = 0; i < _clouds.size (); ++i) {
//        //PCLOperations::voxelGrid(_clouds.at(i));
//        _progressDialog->setValue(i);
//        progressUpdate(7,"done voxel");
//        //7PCLOperations::radiusOutlierRemoval(_clouds.at(i));        
//        progressUpdate(7,"done outlier");
//       
//        //_clouds.at(i) = PCLOperations::polynomialReconstruction(_clouds.at(i));
//       
//        progressUpdate(7,"done polynomial");
//        
////                    ColorCloudPtr temp (new ColorCloud);
////                    Eigen::Matrix4f transform;
////                    PCLOperations::pairAlign (result, _clouds.at(i), temp, transform, true);
////                    result = temp;
//
//
////                    ColorCloudPtr temp (source);
////                    *result += *temp;
//        *result += *_clouds.at(i);
//        std::stringstream ss;
//        ss << "icp-"<<i << ".pcd";
//        if(_clouds.at(i))
//            pcl::io::savePCDFile (ss.str (), *_clouds.at(i), true);
//        
//        //_progressDialog->setValue(4);
//    }
//
//    progressUpdate(70,"Done Capturing");
//    _visualizer->removeAllPointClouds();
//    _visualizer->addPointCloud(result, "FinalCloud");
//    //_visualizer->resetCameraViewpoint ("FinalCloud");
//    pcl::io::savePCDFile<PointXYZ> ("FinalCloud.pcd", *result);
//    _combinedCloud = result;
//    ui->vtkWidget->update();
//    
//    GrayCloudPtr temp(new GrayCloud);
//    pcl::copyPointCloud(*result,*temp);
//    progressUpdate(75,"Generating Mesh");
//    _combinedMesh = PCLOperations::convertToMesh(temp);
//
//    std::cerr << "addPolygonMesh()" << std::endl;
//    _visualizer->addPolygonMesh (_combinedMesh, "ClippedMeshData");
//    //visualizer->resetCameraViewpoint ("ClippedMeshData");
//    progressUpdate(90,"Mesh Generated");
//
//    pcl::io::savePLYFile("FinalMesh.ply",_combinedMesh);
//    _capture = false;
//    
//    progressUpdate(95,"Stopping kinect grabber");
////   _grabber->stop();
//    progressUpdate(100,"Done");
//    ui->vtkWidget->update();
}

void MainWindow::showCloudMesh(bool status) {
    _visualizer->removeAllShapes();
    _visualizer->removeAllPointClouds();
    _showMesh = status;
    if(!status) {
        _visualizer->addPolygonMesh(_combinedMesh, "ClippedMeshData");
    } else {
        _visualizer->addPointCloud(_combinedCloud, "FinalCloud");
    }
}
