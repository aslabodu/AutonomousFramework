
#include <QtGui>
#include <QMessageBox>

#ifndef Q_MOC_RUN
#include <iostream>
#include "main_window.h"
#endif


MainWindow::MainWindow(QWidget* parent):  QMainWindow(parent)
{
	customPlot.addGraph();
	customPlot.graph()->setLineStyle((QCPGraph::LineStyle)0);
	customPlot.graph()->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssSquare, 5));

	max_range = 10.0f;

	arrow_Rover = new QCPItemLine(&customPlot);
	arrow_fov1_Rover = new QCPItemLine(&customPlot);
	arrow_fov2_Rover = new QCPItemLine(&customPlot);

/* 
	arrow_Rover2 = new QCPItemLine(&customPlot);
	arrow_fov1_Rover2 = new QCPItemLine(&customPlot);
	arrow_fov2_Rover2 = new QCPItemLine(&customPlot);

	arrow_Rover3 = new QCPItemLine(&customPlot);
	arrow_fov1_Rover3 = new QCPItemLine(&customPlot);
	arrow_fov2_Rover3 = new QCPItemLine(&customPlot);

	arrow_Rover4 = new QCPItemLine(&customPlot);
	arrow_fov1_Rover4 = new QCPItemLine(&customPlot);
	arrow_fov2_Rover4 = new QCPItemLine(&customPlot);
*/
	verticalLayout.addWidget(&customPlot);  
	centralWidget.setLayout(&verticalLayout);
	this->setCentralWidget(&centralWidget);  
}


MainWindow::~MainWindow()
{
}

void MainWindow::setBounds(qreal _minX, qreal _maxX, qreal _minY, qreal _maxY)
{
  minX = _minX;
  maxX = _maxX;
  minY = _minY;
  maxY = _maxY;

  qreal centerX = (maxX + minX) / 2.0f;
  qreal widthX = (maxX - minX);
  qreal centerY = (maxY + minY) / 2.0f;
  qreal widthY = (maxY - minY);  

  customPlot.xAxis->setRange(centerX, widthX, Qt::AlignCenter);
  customPlot.yAxis->setRange(centerY, widthY, Qt::AlignCenter);
  customPlot.replot();
}

void MainWindow::drawRover(qreal x, qreal y, qreal theta, int RoverID)
{
//	switch(RoverID){
	//	case 1:
			//printf("Test Test MainWindow Setup 1\n");

			arrow_Rover->start->setCoords(x,y);
			arrow_fov1_Rover->start->setCoords(x,y);
			arrow_fov2_Rover->start->setCoords(x,y);

			// arrow->end->setCoords(x + cos(theta*M_PI/180.0f)*max_range*0.7f, y + sin(theta*M_PI/180.0f)*max_range*0.7f);
			arrow_Rover->end->setCoords(x + cos(theta*M_PI/180.0f)*max_range*1.0f, y + sin(theta*M_PI/180.0f)*max_range*1.0f);
			
			arrow_fov1_Rover->end->setCoords(x + cos((theta-field_of_view)*M_PI/180.0f)*max_range*1.0f, y + sin((theta-field_of_view)*M_PI/180.0f)*max_range*1.0f);

			arrow_fov2_Rover->end->setCoords(x + cos((theta+field_of_view)*M_PI/180.0f)*max_range*1.0f, y + sin((theta+field_of_view)*M_PI/180.0f)*max_range*1.0f);

			arrow_Rover->setHead(QCPLineEnding::esSpikeArrow);
			// arrow->setHead(QCPLineEnding::esBar);
			// arrow->setHead(QCPLineEnding::esNone);

			arrow_fov1_Rover->setHead(QCPLineEnding::esNone);

			arrow_fov2_Rover->setHead(QCPLineEnding::esNone);
	/* 		break;
		case 2:
			//printf("Test Test MainWindow Setup 2\n");
			arrow_Rover2->start->setCoords(x,y);
			arrow_fov1_Rover2->start->setCoords(x,y);
			arrow_fov2_Rover2->start->setCoords(x,y);

			// arrow->end->setCoords(x + cos(theta*M_PI/180.0f)*max_range*0.7f, y + sin(theta*M_PI/180.0f)*max_range*0.7f);
			arrow_Rover2->end->setCoords(x + cos(theta*M_PI/180.0f)*max_range*1.0f, y + sin(theta*M_PI/180.0f)*max_range*1.0f);
			
			arrow_fov1_Rover2->end->setCoords(x + cos((theta-field_of_view)*M_PI/180.0f)*max_range*1.0f, y + sin((theta-field_of_view)*M_PI/180.0f)*max_range*1.0f);

			arrow_fov2_Rover2->end->setCoords(x + cos((theta+field_of_view)*M_PI/180.0f)*max_range*1.0f, y + sin((theta+field_of_view)*M_PI/180.0f)*max_range*1.0f);

			arrow_Rover2->setHead(QCPLineEnding::esSpikeArrow);
			// arrow->setHead(QCPLineEnding::esBar);
			// arrow->setHead(QCPLineEnding::esNone);

			arrow_fov1_Rover2->setHead(QCPLineEnding::esNone);

			arrow_fov2_Rover2->setHead(QCPLineEnding::esNone);
			break;
		case 3:
		//printf("Test Test MainWindow Setup 2\n");
			arrow_Rover3->start->setCoords(x,y);
			arrow_fov1_Rover3->start->setCoords(x,y);
			arrow_fov2_Rover3->start->setCoords(x,y);

			// arrow->end->setCoords(x + cos(theta*M_PI/180.0f)*max_range*0.7f, y + sin(theta*M_PI/180.0f)*max_range*0.7f);
			arrow_Rover3->end->setCoords(x + cos(theta*M_PI/180.0f)*max_range*1.0f, y + sin(theta*M_PI/180.0f)*max_range*1.0f);
			
			arrow_fov1_Rover3->end->setCoords(x + cos((theta-field_of_view)*M_PI/180.0f)*max_range*1.0f, y + sin((theta-field_of_view)*M_PI/180.0f)*max_range*1.0f);

			arrow_fov2_Rover3->end->setCoords(x + cos((theta+field_of_view)*M_PI/180.0f)*max_range*1.0f, y + sin((theta+field_of_view)*M_PI/180.0f)*max_range*1.0f);

			arrow_Rover3->setHead(QCPLineEnding::esSpikeArrow);
			// arrow->setHead(QCPLineEnding::esBar);
			// arrow->setHead(QCPLineEnding::esNone);

			arrow_fov1_Rover3->setHead(QCPLineEnding::esNone);

			arrow_fov2_Rover3->setHead(QCPLineEnding::esNone);
			break;
		case 4:
		//printf("Test Test MainWindow Setup 2\n");
			arrow_Rover4->start->setCoords(x,y);
			arrow_fov1_Rover4->start->setCoords(x,y);
			arrow_fov2_Rover4->start->setCoords(x,y);

			// arrow->end->setCoords(x + cos(theta*M_PI/180.0f)*max_range*0.7f, y + sin(theta*M_PI/180.0f)*max_range*0.7f);
			arrow_Rover4->end->setCoords(x + cos(theta*M_PI/180.0f)*max_range*1.0f, y + sin(theta*M_PI/180.0f)*max_range*1.0f);
			
			arrow_fov1_Rover4->end->setCoords(x + cos((theta-field_of_view)*M_PI/180.0f)*max_range*1.0f, y + sin((theta-field_of_view)*M_PI/180.0f)*max_range*1.0f);

			arrow_fov2_Rover4->end->setCoords(x + cos((theta+field_of_view)*M_PI/180.0f)*max_range*1.0f, y + sin((theta+field_of_view)*M_PI/180.0f)*max_range*1.0f);

			arrow_Rover4->setHead(QCPLineEnding::esSpikeArrow);
			// arrow->setHead(QCPLineEnding::esBar);
			// arrow->setHead(QCPLineEnding::esNone);

			arrow_fov1_Rover4->setHead(QCPLineEnding::esNone);

			arrow_fov2_Rover4->setHead(QCPLineEnding::esNone);
			break; 
		}
*/
	// // For wheel visualization
	// customPlot.graph(0)->setData(QVector<double>(),QVector<double>());
	// customPlot.graph(0)->addData(x + cos((theta-90.0f)*M_PI/180.0f)*5.0f, y + sin((theta-90.0f)*M_PI/180.0f)*5.0f);
	// customPlot.graph(0)->addData(x + cos((theta+90.0f)*M_PI/180.0f)*5.0f, y + sin((theta+90.0f)*M_PI/180.0f)*5.0f);

	// // For chasis visualization
	// customPlot.graph(0)->setData(QVector<double>(),QVector<double>());
	// customPlot.graph(0)->addData(x, y);

	customPlot.replot();
}

void MainWindow::drawObstacle(qreal x, qreal y)
{		
	customPlot.addGraph();
	customPlot.graph()->setLineStyle((QCPGraph::LineStyle)0);
	customPlot.graph()->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssCircle, 5.0));
	customPlot.graph()->addData(x,y);
	customPlot.replot();
}

void MainWindow::setMaxRange(qreal size)
{
	max_range = size;
}

void MainWindow::setFieldOfView(qreal fov)
{
	field_of_view = fov;
}

void MainWindow::drawLine(qreal x0, qreal y0, qreal x1, qreal y1)
{
	customPlot.addGraph();
	customPlot.graph()->setLineStyle((QCPGraph::LineStyle)1);
	customPlot.graph()->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssNone, max_range));
	customPlot.graph()->addData(x0,y0);
	customPlot.graph()->addData(x1,y1);
	customPlot.replot();
}

// void MainWindow::drawLine(qreal x0, qreal y0, qreal x1, qreal y1)
// {
// 	customPlot.addGraph();
// 	customPlot.graph()->setLineStyle((QCPGraph::LineStyle)1);
// 	customPlot.graph()->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssCircle, 5));
// 	customPlot.graph()->addData(x0,y0);
// 	customPlot.graph()->addData(x1,y1);

// 	customPlot.replot();
// }

// void MainWindow::clearPlot()
// {
// 	customPlot.clearGraphs();
// 	customPlot.replot();
// }

// void MainWindow::drawPoint(qreal _x, qreal _y)
// {
// 	static QTime time(QTime::currentTime());
// 	double key = time.elapsed() / 1000.0;
// 	static double lastPointKey = 0;

// 	if(customPlot.graphCount() == 0)
// 		customPlot.addGraph();

// 	customPlot.graph()->setLineStyle((QCPGraph::LineStyle)0);
// 	customPlot.graph()->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssCircle, 5));

// //	if(key - lastPointKey > 0.1)
// //    {		

// 		//printf("_x = %f | _y = %f\n", _x, _y);
		
// 		this->customPlot.graph()->addData(_x,_y);
		
// 		customPlot.replot();

// 		lastPointKey = key;
// //	}
// }

void MainWindow::keyPressEvent(QKeyEvent* k)
{
	switch(tolower(k->key()))
	{
		case('r'):		// reset plot
		{
			
			break;
		}
		case('q'):		// quit
		{
			QApplication::exit();
			break;
		}
	}
}



void MainWindow::resizeEvent(QResizeEvent *event)
{
	printf("Resizing\n");
	customPlot.yAxis->setScaleRatio(customPlot.xAxis, 1.0);
	//customPlot.yAxis->setAutoTickStep(false);
    //customPlot.yAxis->setTickStep(chart->xAxis->tickStep());
	customPlot.replot();
}

// -------------------------------------------------------- //


