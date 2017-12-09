#ifndef CAPTURE_H
#define CAPTURE_H

//#include "mainwindow.h"

#include <QObject>

#include <pcl/visualization/pcl_visualizer.h>

#include "types.h"

class Capture : QObject
{
	Q_OBJECT
public:
	typedef pcl::visualization::PCLVisualizer Visualizer;
	typedef boost::shared_ptr<Visualizer> VisualizerPtr;

	Capture();
	~Capture();

	//void SetVisualizer(boost::shared_ptr<pcl::visualization::PCLVisualizer> visualizer);
	void Run();

public slots :

signals:

private:
	boost::shared_ptr<pcl::visualization::PCLVisualizer> _visualizer;

private slots:

};

#endif