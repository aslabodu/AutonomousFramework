#ifndef PLOT_APP_H
#define PLOT_APP_H

#ifndef Q_MOC_RUN
#include "Node.h"
#include "FloatObject.h"
#include "LocationObject.h"

#include <QApplication>
#include <QMainWindow>
#include <QWidget>
#include <QVBoxLayout>
#include <QStatusBar>
#include <QTimer>

#include <future>
#include <chrono>

#include "main_window.h"
#endif
class QtWorkerObj : public QObject
{
	Q_OBJECT
private:
	MainWindow* window;
public:	
	void drawLine(qreal x0, qreal y0, qreal x1, qreal y1);
	void drawRover(qreal x, qreal y, qreal theta, int Rover_ID);
	void drawObstacle(qreal x, qreal y);
	void setMaxRange(qreal size);
	void setFieldOfView(qreal fov);
	void setBounds(qreal minX, qreal maxX, qreal minY, qreal maxY);
	void Run(int argc, char** argv);	
};


class PlotApp : public Node
{
private:
	LocationObject vehicle_location,vehicle_location2, vehicle_location3, vehicle_location4;	

	LocationObject obstacle; 

	QtWorkerObj worker;
	std::future<void> qt_worker_future;	

	float max_range;
	float field_of_view;
	
protected:
	void Setup(int argc, char** argv);
	void SetNodeName(int argc, char** argv, std::string& nodeName);
	void Init();
	bool Load(const char* filename);
	bool LoadEnv(const char* filename);
	void OnReceiveLocation();
	void OnReceiveLocation2();
	void OnReceiveLocation3();
	void OnReceiveLocation4();
	void OnReceiveObstacle();
	void Process();
};

#endif