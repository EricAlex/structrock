/*
 * Software License Agreement (BSD License)
 *
 *  Xin Wang
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author : Xin Wang
 * Email  : ericrussell@zju.edu.cn
 *
 */

#include <vector>
#include <fstream>
#include <string>
#include <sstream>
#include <algorithm>
#include <math.h>
#include <libpq-fe.h>
#include "pcl/common/common_headers.h"
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/boost.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/openni2_grabber.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/extract_indices.h>
#include <vtkRenderWindow.h>
#include <vtkImageViewer2.h>
#include <boost/thread/thread.hpp>
#include <Eigen/src/Core/Matrix.h>
#include <boost/algorithm/string/predicate.hpp>
#include <qfiledialog.h>
#include <qinputdialog.h>
#include <qmessagebox.h>
#include <qevent.h>
#include <qthread.h>
#include <qstringlist.h>
#include "ReadFileWorker.h"
#include "checkstatusThread.h"
#include "resampleWorker.h"
#include "downsampleWorker.h"
#include "knnormalWorker.h"
#include "ranormalWorker.h"
#include "StaticROWorker.h"
#include "RGSWorker.h"
#include "SaveClustersWorker.h"
#include "ReadXYZWorker.h"
#include "ShowProcessWorker.h"
#include "TestWorker.h"
#include "globaldef.h"
#include "dataLibrary.h"
#include "structrock.h"
#include "geo_region_growing.h"
#include "plotwindow.h"
#include "TimingShutdown.h"
#include "KinectV2Viewer.h"

using namespace std;

structrock::structrock(QWidget *parent, Qt::WFlags flags)
	: QMainWindow(parent, flags),
	viewer(new pcl::visualization::PCLVisualizer("3D Viewer")),
    imageViewer(vtkImageViewer2::New()),
	v1(1),
	v2(2)
{
	ui.setupUi(this);
	QPalette pa;
	pa.setColor(QPalette::WindowText,Qt::black);
	ui.label->setPalette(pa);
	ui.qvtkWidget->SetRenderWindow(viewer->getRenderWindow());

	viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
	viewer->setBackgroundColor(0.4, 0.4, 0.4, v1);
	viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
	viewer->setBackgroundColor(0.4, 0.4, 0.4, v2);
	viewer->initCameraParameters();
    viewer->resetCamera();
    ui.qvtkWidget->update();

	openAction = new QAction(tr("&pcd File"), this);
	openAction->setShortcut(QKeySequence::Open);
	connect(openAction, SIGNAL(triggered()), this, SLOT(open()));

	open_clusters = new QAction(tr("&bin File"), this);
	connect(open_clusters, SIGNAL(triggered()), this, SLOT(OpenClusters()));

	open_xyz = new QAction(tr("&xyz File"), this);
	connect(open_xyz, SIGNAL(triggered()), this, SLOT(OpenXYZ()));

	open_workflow = new QAction(tr("&Work Flow"), this);
	connect(open_workflow, SIGNAL(triggered()), this, SLOT(OpenWorkFlow()));

	SaveAsASCII = new QAction(tr("ASCII"), this);
	connect(SaveAsASCII, SIGNAL(triggered()), this, SLOT(saveasascii()));

	SaveAsBinary = new QAction(tr("Binary"), this);
	connect(SaveAsBinary, SIGNAL(triggered()), this, SLOT(saveasbinary()));

	exitAction = new QAction(tr("&Exit"), this);
	exitAction->setShortcut(QKeySequence::Quit);
	connect(exitAction, SIGNAL(triggered()), this, SLOT(exit()));

	Reboot = new QAction(tr("&Reboot"), this);
	connect(Reboot, SIGNAL(triggered()), this, SLOT(slotReboot()));

	newWindow = new QAction(tr("&New Window"), this);
	connect(newWindow, SIGNAL(triggered()), this, SLOT(NewWindow()));

	save_normals = new QAction(tr("&Save Normals"), this);
	connect(save_normals, SIGNAL(triggered()), this, SLOT(SaveNormals()));

	save_clusters = new QAction(tr("&Save Clusters"), this);
	connect(save_clusters, SIGNAL(triggered()), this, SLOT(SaveClusters()));

	QMenu *file = menuBar()->addMenu(tr("&File"));
	QMenu *open = file->addMenu(tr("&Open"));
	open->addAction(openAction);
	open->addAction(open_clusters);
	open->addAction(open_xyz);
	open->addAction(open_workflow);
	QMenu *Save = file->addMenu(tr("&Save"));
	QMenu *SavePCD = Save->addMenu(tr("&Save PCD as"));
	SavePCD->addAction(SaveAsASCII);
	SavePCD->addAction(SaveAsBinary);
	Save->addAction(save_normals);
	Save->addAction(save_clusters);
	file->addAction(Reboot);
	file->addAction(newWindow);
	file->addAction(exitAction);

	DownSampling = new QAction(tr("&Down Sampling"), this);
	connect(DownSampling, SIGNAL(triggered()), this, SLOT(downsampling()));

	ReSampling = new QAction(tr("&Resampling"), this);
	connect(ReSampling, SIGNAL(triggered()), this, SLOT(resampling()));

	normal_k_neighbor = new QAction(tr("&K-Neighbor"), this);
	connect(normal_k_neighbor, SIGNAL(triggered()), this, SLOT(k_neighbor()));

	normal_radius = new QAction(tr("&Radius-threshold"), this);
	connect(normal_radius, SIGNAL(triggered()), this, SLOT(radius()));


	Static_remove_outlier = new QAction(tr("&Statistical"), this);
	connect(Static_remove_outlier, SIGNAL(triggered()), this, SLOT(StaticRemoveOutlier()));

	Conditional_remove_outlier = new QAction(tr("&Conditional(coming)"), this);
	connect(Conditional_remove_outlier, SIGNAL(triggered()), this, SLOT(ConditionalRemoveOutlier()));

	Radius_remove_outlier = new QAction(tr("&Radius(coming)"), this);
	connect(Radius_remove_outlier, SIGNAL(triggered()), this, SLOT(RadiusRemoveOutlier()));

	region_growing_segmentation = new QAction(tr("Region Growing"), this);
	connect(region_growing_segmentation, SIGNAL(triggered()), this, SLOT(RegionGrowingSegmentation()));

	QMenu *edit = menuBar()->addMenu(tr("&Edit"));
	QMenu *filter = edit->addMenu(tr("Filtering"));
	filter->addAction(DownSampling);
	filter->addAction(ReSampling);

	QMenu *normal = edit->addMenu(tr("&Extract Normals"));
	normal->addAction(normal_k_neighbor);
	normal->addAction(normal_radius);

	QMenu *removeoutlier = edit->addMenu(tr("&Remove Outlier"));
	removeoutlier->addAction(Static_remove_outlier);
	removeoutlier->addAction(Conditional_remove_outlier);
	removeoutlier->addAction(Radius_remove_outlier);

	QMenu *segmentation = edit->addMenu(tr("&Segmentation"));
	segmentation->addAction(region_growing_segmentation);
    
    QMenu *kinect = menuBar()->addMenu(tr("&Kinect"));
    connect_Kinect = new QAction(tr("&Connect"), this);
    connect(connect_Kinect, SIGNAL(triggered()), this, SLOT(Connect_Kinect()));
    disconnect_Kinect = new QAction(tr("&Disconnect"), this);
    connect(disconnect_Kinect, SIGNAL(triggered()), this, SLOT(Disconnect_Kinect()));
    kinect->addAction(connect_Kinect);
    kinect->addAction(disconnect_Kinect);

	testing = new QAction(tr("&Test"), this);
	connect(testing, SIGNAL(triggered()), this, SLOT(Testing()));
	about = new QAction(tr("&About"), this);
	connect(about, SIGNAL(triggered()), this, SLOT(About()));
	stereonet = new QAction(tr("&Stereonet"), this);
	connect(stereonet, SIGNAL(triggered()), this, SLOT(Stereonet()));

	QMenu *help = menuBar()->addMenu(tr("&Help"));
	help->addAction(about);
	help->addAction(stereonet);
	help->addAction(testing);
    
    checkstatusThread *csThread(new checkstatusThread);
    connect(csThread, SIGNAL(show(int)), this, SLOT(ShowStatus(int)));
    connect(csThread, SIGNAL(finished()), csThread, SLOT(deleteLater()));
    csThread->start();
}

structrock::~structrock()
{
    
}

void structrock::Connect_Kinect()
{
    boost::shared_ptr<pcl::io::openni2::OpenNI2DeviceManager> deviceManager = pcl::io::openni2::OpenNI2DeviceManager::getInstance ();
    if (deviceManager->getNumOfConnectedDevices () > 0)
    {
        grabber = new pcl::io::OpenNI2Grabber("", pcl::io::OpenNI2Grabber::OpenNI_Default_Mode, pcl::io::OpenNI2Grabber::OpenNI_Default_Mode);
        KinectV2Viewer<pcl::PointXYZRGBA> openni_viewer (grabber, viewer);
        openni_viewer.run();
    }
    else
    {
        Show_Errors(QString("No Kinect Senser Devices Connected!"));
    }
}

void structrock::Disconnect_Kinect()
{
    if (grabber && grabber->isRunning ()) grabber->stop ();
}

void structrock::open()
{
    QString filename = QFileDialog::getOpenFileName(NULL,tr("Open Point Cloud Data"),QDir::currentPath(),tr("Point Cloud Data (*.pcd);;All files (*.*)"));
    
    if(!filename.isNull())
    {
		readfileworker.setWorkFlowMode(false);
        readfileworker.setUnmute();
		QObject::connect(&readfileworker, SIGNAL(ReadFileReady(int)), this, SLOT(ShowPCD(int)));
		connect(&readfileworker, SIGNAL(showErrors(QString)), this, SLOT(Show_Errors(QString)));
		QObject::connect(&readfileworker, SIGNAL(showReadyStatus()), this, SLOT(ShowReady()));

		readfileworker.readFile(filename);
    }
}

void structrock::OpenXYZ()
{
	QString filename = QFileDialog::getOpenFileName(NULL,tr("Open XYZ Data"),QDir::currentPath(),tr("XYZ Data (*.txt);;All files (*.*)"));
    
    if(!filename.isNull())
    {
		readxyzworker.setWorkFlowMode(false);
        readxyzworker.setUnmute();
        connect(&readxyzworker, SIGNAL(ReadXYZReady(int)), this, SLOT(ShowPCD(int)));
		connect(&readxyzworker, SIGNAL(showErrors(QString)), this, SLOT(Show_Errors(QString)));
        connect(&readxyzworker, SIGNAL(showReadyStatus()), this, SLOT(ShowReady()));
        
        readxyzworker.readXYZ(filename);
	}
}

void structrock::OpenWorkFlow()
{
	QString filename = QFileDialog::getOpenFileName(NULL,tr("Open Work Flow File"),QDir::currentPath(),tr("Work Flow (*.txt);;All files (*.*)"));
    
    if(!filename.isNull())
    {
		QByteArray ba = filename.toLocal8Bit();
		std::string* strfilename = new std::string(ba.data());
		std::ifstream file(strfilename->c_str());

		if(file)
		{
			dataLibrary::clearWorkFlow();

			std::stringstream buffer;
			buffer << file.rdbuf();
			file.close();
			char line_split_str = ';';
			std::string each_line;
			while(std::getline(buffer, each_line, line_split_str))
			{
				std::istringstream line_stream(each_line);
				char command_split_str = ',';
				std::vector<std::string> tokens;
				for(std::string each; std::getline(line_stream, each, command_split_str); tokens.push_back(each));

				if(tokens.size()>0)
				{
					tokens[0].erase(std::remove(tokens[0].begin(), tokens[0].end(),'\n'), tokens[0].end());
					tokens[0].erase(std::remove(tokens[0].begin(), tokens[0].end(),' '), tokens[0].end());
					if(boost::starts_with(tokens[0],"#"))
					{

					}
					else
					{
						WorkLine work;
						work.command = tokens[0];
						for(int i=1; i<tokens.size(); i++)
						{
							tokens[i].erase(std::remove(tokens[i].begin(), tokens[i].end(),'\n'), tokens[i].end());
							tokens[i].erase(std::remove(tokens[i].begin(), tokens[i].end(),' '), tokens[i].end());
							work.parameters.push_back(tokens[i]);
						}
						dataLibrary::Workflow.push_back(work);
					}
				}
			}

			if(dataLibrary::Workflow.size()>0)
			{
				dataLibrary::current_workline_index=0;
				dataLibrary::have_called_read_file = false;
				command_parser();
			}
		}

		delete strfilename;
	}
}

void structrock::OpenWorkFlow(QString filename)
{
	if(!filename.isNull())
    {
		QByteArray ba = filename.toLocal8Bit();
		std::string* strfilename = new std::string(ba.data());
		std::ifstream file(strfilename->c_str());

		if(file)
		{
			dataLibrary::clearWorkFlow();

			std::stringstream buffer;
			buffer << file.rdbuf();
			file.close();
			char line_split_str = ';';
			std::string each_line;
			while(std::getline(buffer, each_line, line_split_str))
			{
				std::istringstream line_stream(each_line);
				char command_split_str = ',';
				std::vector<std::string> tokens;
				for(std::string each; std::getline(line_stream, each, command_split_str); tokens.push_back(each));

				if(tokens.size()>0)
				{
					tokens[0].erase(std::remove(tokens[0].begin(), tokens[0].end(),'\n'), tokens[0].end());
					tokens[0].erase(std::remove(tokens[0].begin(), tokens[0].end(),' '), tokens[0].end());
					if(boost::starts_with(tokens[0],"#"))
					{

					}
					else
					{
						WorkLine work;
						work.command = tokens[0];
						for(int i=1; i<tokens.size(); i++)
						{
							tokens[i].erase(std::remove(tokens[i].begin(), tokens[i].end(),'\n'), tokens[i].end());
							tokens[i].erase(std::remove(tokens[i].begin(), tokens[i].end(),' '), tokens[i].end());
							work.parameters.push_back(tokens[i]);
						}
						dataLibrary::Workflow.push_back(work);
					}
				}
			}

			if(dataLibrary::Workflow.size()>0)
			{
				dataLibrary::current_workline_index=0;
				command_parser();
			}
		}

		delete strfilename;
	}
}

void structrock::OpenWorkFlow(std::string commands)
{
	dataLibrary::clearWorkFlow();

	std::istringstream buffer(commands);
	char line_split_str = ';';
	std::string each_line;
	while(std::getline(buffer, each_line, line_split_str))
	{
		std::istringstream line_stream(each_line);
		char command_split_str = ',';
		std::vector<std::string> tokens;
		for(std::string each; std::getline(line_stream, each, command_split_str); tokens.push_back(each));

		if(tokens.size()>0)
		{
			tokens[0].erase(std::remove(tokens[0].begin(), tokens[0].end(),'\n'), tokens[0].end());
			tokens[0].erase(std::remove(tokens[0].begin(), tokens[0].end(),' '), tokens[0].end());
			if(boost::starts_with(tokens[0],"#"))
			{

			}
			else
			{
				WorkLine work;
				work.command = tokens[0];
				for(int i=1; i<tokens.size(); i++)
				{
					tokens[i].erase(std::remove(tokens[i].begin(), tokens[i].end(),'\n'), tokens[i].end());
					tokens[i].erase(std::remove(tokens[i].begin(), tokens[i].end(),' '), tokens[i].end());
					work.parameters.push_back(tokens[i]);
				}
				dataLibrary::Workflow.push_back(work);
			}
		}
	}

	if(dataLibrary::Workflow.size()>0)
	{
		dataLibrary::current_workline_index=0;
		command_parser();
	}
}

void structrock::command_parser()
{
	if(dataLibrary::Workflow.size()!=0)
	{
		if(dataLibrary::current_workline_index<dataLibrary::Workflow.size())
	{
		bool need_move_on(true);

		std::string command_string = dataLibrary::Workflow[dataLibrary::current_workline_index].command;
		std::transform(command_string.begin(), command_string.end(), command_string.begin(), ::tolower);
		if(command_string == "openpcd")
		{
			if(dataLibrary::Workflow[dataLibrary::current_workline_index].parameters.size()>0)
			{
				if(!dataLibrary::have_called_read_file)
				{
					readfileworker.setWorkFlowMode(true);
                    readfileworker.setUnmute();
                    if(dataLibrary::Workflow[dataLibrary::current_workline_index].parameters.size()>1)
                    {
                        if(dataLibrary::Workflow[dataLibrary::current_workline_index].parameters[1] == "mute")
                        {
                            readfileworker.setMute();
                        }
                    }
					QObject::connect(&readfileworker, SIGNAL(ReadFileReady(int)), this, SLOT(ShowPCD(int)));
					QObject::connect(&readfileworker, SIGNAL(showReadyStatus()), this, SLOT(ShowReady()));
					QObject::connect(&readfileworker, SIGNAL(showErrors(QString)), this, SLOT(Show_Errors(QString)));
					QObject::connect(&readfileworker, SIGNAL(GoWorkFlow()), this, SLOT(command_parser()));

					readfileworker.readFile(QString::fromUtf8(dataLibrary::Workflow[dataLibrary::current_workline_index].parameters[0].c_str()));
					dataLibrary::have_called_read_file = true;
				}
				else
				{
					std::string commands = "";
					for(int i=dataLibrary::current_workline_index; i<dataLibrary::Workflow.size(); i++)
					{
						if(dataLibrary::Workflow[i].parameters.size()>0)
						{
							commands += dataLibrary::Workflow[i].command;
							commands += ",";
							for(int j=0; j<dataLibrary::Workflow[i].parameters.size()-1; j++)
							{
								commands += dataLibrary::Workflow[i].parameters[j];
								commands += ",";
							}
							commands += dataLibrary::Workflow[i].parameters[dataLibrary::Workflow[i].parameters.size()-1];
							commands += ";";
						}
						else
						{
							commands += dataLibrary::Workflow[i].command;
							commands += ";";
						}
					}
					#if defined(_WIN32)||defined(_WIN64)
						size_t f = commands.find("\\");
						commands.replace(f, std::string("\\").length(), "\\\\");
					#endif

					QProcess *myProcess = new QProcess;
					QStringList commandsList;
					commandsList << "-c"<<commands.c_str();
					myProcess->setWorkingDirectory(QApplication::applicationDirPath());
					#if !defined(_WIN32)&&(defined(__unix__)||defined(__unix)||(defined(__APPLE__)&&defined(__MACH__)))
						myProcess->start("./structrock",commandsList);
					#elif defined(_WIN32)||defined(_WIN64)
						myProcess->start("structrock",commandsList);
					#endif
				}
			}
			else
			{
				Show_Errors(QString("Openpcd: Location of PCD file not provided."));
			}
		}
		else if(command_string == "openbin")
		{
            
		}
		else if(command_string == "openxyz")
		{
			if(dataLibrary::Workflow[dataLibrary::current_workline_index].parameters.size()>0)
			{
				if(!dataLibrary::have_called_read_file)
				{
					readxyzworker.setWorkFlowMode(true);
                    readxyzworker.setUnmute();
                    if(dataLibrary::Workflow[dataLibrary::current_workline_index].parameters.size()>1)
                    {
                        if(dataLibrary::Workflow[dataLibrary::current_workline_index].parameters[1] == "mute")
                        {
                            readxyzworker.setMute();
                        }
                    }
					connect(&readxyzworker, SIGNAL(ReadXYZReady(int)), this, SLOT(ShowPCD(int)));
					connect(&readxyzworker, SIGNAL(showReadyStatus()), this, SLOT(ShowReady()));
					connect(&readxyzworker, SIGNAL(showErrors(QString)), this, SLOT(Show_Errors(QString)));
					connect(&readxyzworker, SIGNAL(GoWorkFlow()), this, SLOT(command_parser()));

					readxyzworker.readXYZ(QString::fromUtf8(dataLibrary::Workflow[dataLibrary::current_workline_index].parameters[0].c_str()));
					dataLibrary::have_called_read_file = true;
				}
				else
				{
					std::string commands = "";
					for(int i=dataLibrary::current_workline_index; i<dataLibrary::Workflow.size(); i++)
					{
						if(dataLibrary::Workflow[i].parameters.size()>0)
						{
							commands += dataLibrary::Workflow[i].command;
							commands += ",";
							for(int j=0; j<dataLibrary::Workflow[i].parameters.size()-1; j++)
							{
								commands += dataLibrary::Workflow[i].parameters[j];
								commands += ",";
							}
							commands += dataLibrary::Workflow[i].parameters[dataLibrary::Workflow[i].parameters.size()-1];
							commands += ";";
						}
						else
						{
							commands += dataLibrary::Workflow[i].command;
							commands += ";";
						}
					}
					
					#if defined(_WIN32)||defined(_WIN64)
						size_t f = commands.find("\\");
						commands.replace(f, std::string("\\").length(), "\\\\");
					#endif

					QProcess *myProcess = new QProcess;
					QStringList commandsList;
					commandsList << "-c"<<commands.c_str();
					myProcess->setWorkingDirectory(QApplication::applicationDirPath());
					#if !defined(_WIN32)&&(defined(__unix__)||defined(__unix)||(defined(__APPLE__)&&defined(__MACH__)))
						myProcess->start("./structrock",commandsList);
					#elif defined(_WIN32)||defined(_WIN64)
						myProcess->start("structrock",commandsList);
					#endif
				}
			}
			else
			{
				Show_Errors(QString("Openxyz: Location of XYZ file not provided."));
			}
		}
		else if(command_string == "savepcdascii")
		{
			if(dataLibrary::Workflow[dataLibrary::current_workline_index].parameters.size()>0)
			{
				savepcdASCIIworker.setWorkFlowMode(true);
				connect(&savepcdASCIIworker, SIGNAL(showErrors(QString)), this, SLOT(Show_Errors(QString)));
				connect(&savepcdASCIIworker, SIGNAL(showReadyStatus()), this, SLOT(ShowReady()));
				connect(&savepcdASCIIworker, SIGNAL(GoWorkFlow()), this, SLOT(command_parser()));

				savepcdASCIIworker.saveascii(QString::fromUtf8(dataLibrary::Workflow[dataLibrary::current_workline_index].parameters[0].c_str()));
			}
			else
			{
				Show_Errors(QString("Savepcdascii: Save path not provided."));
			}
		}
		else if(command_string == "savepcdbinary")
		{
			if(dataLibrary::Workflow[dataLibrary::current_workline_index].parameters.size()>0)
			{
				savepcdBinaryworker.setWorkFlowMode(true);
				connect(&savepcdBinaryworker, SIGNAL(showErrors(QString)), this, SLOT(Show_Errors(QString)));
				connect(&savepcdBinaryworker, SIGNAL(showReadyStatus()), this, SLOT(ShowReady()));
				connect(&savepcdBinaryworker, SIGNAL(GoWorkFlow()), this, SLOT(command_parser()));

				savepcdBinaryworker.savebinary(QString::fromUtf8(dataLibrary::Workflow[dataLibrary::current_workline_index].parameters[0].c_str()));
			}
			else
			{
				Show_Errors(QString("Savepcdbinary: Save path not provided."));
			}
		}
		else if(command_string == "savenormals")
		{
			if(dataLibrary::Workflow[dataLibrary::current_workline_index].parameters.size()>0)
			{
				savenormalsworker.setWorkFlowMode(true);
				connect(&savenormalsworker, SIGNAL(showErrors(QString)), this, SLOT(Show_Errors(QString)));
				connect(&savenormalsworker, SIGNAL(showReadyStatus()), this, SLOT(ShowReady()));
				connect(&savenormalsworker, SIGNAL(GoWorkFlow()), this, SLOT(command_parser()));

				savenormalsworker.savenormals(QString::fromUtf8(dataLibrary::Workflow[dataLibrary::current_workline_index].parameters[0].c_str()));
			}
			else
			{
				Show_Errors(QString("Savenormals: Save path not provided."));
			}
		}
		else if(command_string == "saveclusters")
		{
			if(dataLibrary::Workflow[dataLibrary::current_workline_index].parameters.size()>0)
			{
				saveclustersworker.setWorkFlowMode(true);
                saveclustersworker.setUnmute();
                if(dataLibrary::Workflow[dataLibrary::current_workline_index].parameters.size()>1)
                {
                    if(dataLibrary::Workflow[dataLibrary::current_workline_index].parameters[1] == "mute")
                    {
                        saveclustersworker.setMute();
                    }
                }
				connect(&saveclustersworker, SIGNAL(SaveClustersReady(QString)), this, SLOT(ShowSavedClusters(QString)));
				connect(&saveclustersworker, SIGNAL(showReadyStatus()), this, SLOT(ShowReady()));
				connect(&saveclustersworker, SIGNAL(showErrors(QString)), this, SLOT(Show_Errors(QString)));
				connect(&saveclustersworker, SIGNAL(GoWorkFlow()), this, SLOT(command_parser()));
            
				saveclustersworker.saveclusters(QString::fromUtf8(dataLibrary::Workflow[dataLibrary::current_workline_index].parameters[0].c_str()));
			}
			else
			{
				Show_Errors(QString("Saveclusters: Save path not provided."));
			}
		}
		else if(command_string == "downsample")
		{
			if(dataLibrary::Workflow[dataLibrary::current_workline_index].parameters.size()>0)
			{
				double leaf;
				std::stringstream ss(dataLibrary::Workflow[dataLibrary::current_workline_index].parameters[0]);
				ss >> leaf;

				downsampleworker.setWorkFlowMode(true);
                downsampleworker.setUnmute();
                if(dataLibrary::Workflow[dataLibrary::current_workline_index].parameters.size()>1)
                {
                    if(dataLibrary::Workflow[dataLibrary::current_workline_index].parameters[1] == "mute")
                    {
                        downsampleworker.setMute();
                    }
                }
				connect(&downsampleworker, SIGNAL(show()), this, SLOT(ShowDownsample()));
				connect(&downsampleworker, SIGNAL(showReadyStatus()), this, SLOT(ShowReady()));
				connect(&downsampleworker, SIGNAL(showErrors(QString)), this, SLOT(Show_Errors(QString)));
				connect(&downsampleworker, SIGNAL(GoWorkFlow()), this, SLOT(command_parser()));
				
				downsampleworker.downsample(leaf);
			}
			else
			{
				Show_Errors(QString("Downsample: Minimum point distance not given."));
			}
		}
		else if(command_string == "resample")
		{
			if(dataLibrary::Workflow[dataLibrary::current_workline_index].parameters.size()>0)
			{
				double radius;
				std::stringstream ss(dataLibrary::Workflow[dataLibrary::current_workline_index].parameters[0]);
				ss >> radius;

				resampleworker.setWorkFlowMode(true);
                resampleworker.setUnmute();
                if(dataLibrary::Workflow[dataLibrary::current_workline_index].parameters.size()>1)
                {
                    if(dataLibrary::Workflow[dataLibrary::current_workline_index].parameters[1] == "mute")
                    {
                        resampleworker.setMute();
                    }
                }
				connect(&resampleworker, SIGNAL(show()), this, SLOT(ShowResample()));
				connect(&resampleworker, SIGNAL(showReadyStatus()), this, SLOT(ShowReady()));
				connect(&resampleworker, SIGNAL(showErrors(QString)), this, SLOT(Show_Errors(QString)));
				connect(&resampleworker, SIGNAL(GoWorkFlow()), this, SLOT(command_parser()));

				resampleworker.resample(radius);
			}
			else
			{
				Show_Errors(QString("Resample: Search radius not given."));
			}
		}
		else if(command_string == "knnormal")
		{
			if(dataLibrary::Workflow[dataLibrary::current_workline_index].parameters.size()>0)
			{
				int k;
				std::stringstream ss(dataLibrary::Workflow[dataLibrary::current_workline_index].parameters[0]);
				ss >> k;

				knnormalworker.setWorkFlowMode(true);
                knnormalworker.setUnmute();
                if(dataLibrary::Workflow[dataLibrary::current_workline_index].parameters.size()>1)
                {
                    if(dataLibrary::Workflow[dataLibrary::current_workline_index].parameters[1] == "mute")
                    {
                        knnormalworker.setMute();
                    }
                }
				connect(&knnormalworker, SIGNAL(show()), this, SLOT(ShowknNormal()));
				connect(&knnormalworker, SIGNAL(showReadyStatus()), this, SLOT(ShowReady()));
				connect(&knnormalworker, SIGNAL(showErrors(QString)), this, SLOT(Show_Errors(QString)));
				connect(&knnormalworker, SIGNAL(GoWorkFlow()), this, SLOT(command_parser()));

				knnormalworker.knnormal(k);
			}
			else
			{
				Show_Errors(QString("Knnormal: Number of neoghbor points not given."));
			}
		}
		else if(command_string == "ranormal")
		{
			if(dataLibrary::Workflow[dataLibrary::current_workline_index].parameters.size()>0)
			{
				double radius;
				std::stringstream ss(dataLibrary::Workflow[dataLibrary::current_workline_index].parameters[0]);
				ss >> radius;

				ranormalworker.setWorkFlowMode(true);
                ranormalworker.setUnmute();
                if(dataLibrary::Workflow[dataLibrary::current_workline_index].parameters.size()>1)
                {
                    if(dataLibrary::Workflow[dataLibrary::current_workline_index].parameters[1] == "mute")
                    {
                        ranormalworker.setMute();
                    }
                }
				connect(&ranormalworker, SIGNAL(show()), this, SLOT(ShowraNormal()));
				connect(&ranormalworker, SIGNAL(showReadyStatus()), this, SLOT(ShowReady()));
				connect(&ranormalworker, SIGNAL(showErrors(QString)), this, SLOT(Show_Errors(QString)));
				connect(&ranormalworker, SIGNAL(GoWorkFlow()), this, SLOT(command_parser()));
				
				ranormalworker.ranormal(radius);
			}
			else
			{
				Show_Errors(QString("Ranormal: Search radius not given."));
			}
		}
		else if(command_string == "rostatic")
		{
			if(dataLibrary::Workflow[dataLibrary::current_workline_index].parameters.size()>0)
			{
				double stdDev;
				std::stringstream ss(dataLibrary::Workflow[dataLibrary::current_workline_index].parameters[0]);
				ss >> stdDev;

				staticroworker.setWorkFlowMode(true);
                staticroworker.setUnmute();
                if(dataLibrary::Workflow[dataLibrary::current_workline_index].parameters.size()>1)
                {
                    if(dataLibrary::Workflow[dataLibrary::current_workline_index].parameters[1] == "mute")
                    {
                        staticroworker.setMute();
                    }
                }
				connect(&staticroworker, SIGNAL(show()), this, SLOT(ShowSRO()));
				connect(&staticroworker, SIGNAL(showReadyStatus()), this, SLOT(ShowReady()));
				connect(&staticroworker, SIGNAL(showErrors(QString)), this, SLOT(Show_Errors(QString)));
				connect(&staticroworker, SIGNAL(GoWorkFlow()), this, SLOT(command_parser()));
            
				staticroworker.rostatic(stdDev);
			}
			else
			{
				Show_Errors(QString("Standard deviation not given."));
			}
		}
		else if(command_string == "rgsegmentation")
		{
			if(dataLibrary::Workflow[dataLibrary::current_workline_index].parameters.size()>5)
			{
				double smoothness;
				std::stringstream ss_smoothness(dataLibrary::Workflow[dataLibrary::current_workline_index].parameters[0]);
				ss_smoothness >> smoothness;
				dataLibrary::RGSparameter.smoothness = smoothness;
				double curvature;
				std::stringstream ss_curvature(dataLibrary::Workflow[dataLibrary::current_workline_index].parameters[1]);
				ss_curvature >> curvature;
				dataLibrary::RGSparameter.curvature = curvature;
				double residual;
				std::stringstream ss_residual(dataLibrary::Workflow[dataLibrary::current_workline_index].parameters[2]);
				ss_residual >> residual;
				dataLibrary::RGSparameter.residual = residual;
				int min_number_of_Points;
				std::stringstream ss_min_number_of_Points(dataLibrary::Workflow[dataLibrary::current_workline_index].parameters[3]);
				ss_min_number_of_Points >> min_number_of_Points;
				dataLibrary::RGSparameter.min_number_of_Points = min_number_of_Points;
				int number_of_neighbors;
				std::stringstream ss_number_of_neighbors(dataLibrary::Workflow[dataLibrary::current_workline_index].parameters[4]);
				ss_number_of_neighbors >> number_of_neighbors;
				dataLibrary::RGSparameter.number_of_neighbors = number_of_neighbors;
				std::string IsSmoothMode_string = dataLibrary::Workflow[dataLibrary::current_workline_index].parameters[5];
				std::transform(IsSmoothMode_string.begin(), IsSmoothMode_string.end(), IsSmoothMode_string.begin(), ::tolower);
				if((IsSmoothMode_string == "true")||(IsSmoothMode_string == "false"))
				{
					if(IsSmoothMode_string == "true")
					{
						dataLibrary::RGSparameter.IsSmoothMode=true;
					}
					else if(IsSmoothMode_string == "false")
					{
						dataLibrary::RGSparameter.IsSmoothMode=false;
					}
					rgsworker.setWorkFlowMode(true);
                    rgsworker.setUnmute();
                    if(dataLibrary::Workflow[dataLibrary::current_workline_index].parameters.size()>6)
                    {
                        if(dataLibrary::Workflow[dataLibrary::current_workline_index].parameters[6] == "mute")
                        {
                            rgsworker.setMute();
                        }
                    }
					connect(&rgsworker, SIGNAL(show()), this, SLOT(ShowRGS()));
					connect(&rgsworker, SIGNAL(showReadyStatus()), this, SLOT(ShowReady()));
					connect(&rgsworker, SIGNAL(showErrors(QString)), this, SLOT(Show_Errors(QString)));
					connect(&rgsworker, SIGNAL(GoWorkFlow()), this, SLOT(command_parser()));

					rgsworker.rgs();
				}
				else
				{
					Show_Errors(QString("Rgsegmentation: IsSmoothMode not set correctly, set with \"true\" or \"false\"."));
				}
			}
			else
			{
				Show_Errors(QString("Rgsegmentation: Not enough parameters given."));
			}
		}
		else if(command_string == "showprocess")
		{
			if(dataLibrary::Workflow[dataLibrary::current_workline_index].parameters.size()>0)
			{
				for(int i=0; i<dataLibrary::Workflow[dataLibrary::current_workline_index].parameters.size(); i++)
				{
					dataLibrary::contents.push_back(dataLibrary::Workflow[dataLibrary::current_workline_index].parameters[i]);
				}

				showprocessworker.setWorkFlowMode(true);
				connect(&showprocessworker, SIGNAL(show()), this, SLOT(Show_Process()));
				connect(&showprocessworker, SIGNAL(showReadyStatus()), this, SLOT(ShowReady()));
				connect(&showprocessworker, SIGNAL(showErrors(QString)), this, SLOT(Show_Errors(QString)));
				connect(&showprocessworker, SIGNAL(GoWorkFlow()), this, SLOT(command_parser()));

				showprocessworker.showProcess();
			}
			else
			{
				Show_Errors(QString("Showprocess: No parameters given."));
			}
		}
		else if(command_string == "quitsession")
		{
			if(dataLibrary::current_workline_index+1<dataLibrary::Workflow.size())
			{
				std::string commands = "";
				for(int i=dataLibrary::current_workline_index+1; i<dataLibrary::Workflow.size(); i++)
				{
					if(dataLibrary::Workflow[i].parameters.size()>0)
					{
						commands += dataLibrary::Workflow[i].command;
						commands += ",";
						for(int j=0; j<dataLibrary::Workflow[i].parameters.size()-1; j++)
						{
							commands += dataLibrary::Workflow[i].parameters[j];
							commands += ",";
						}
						commands += dataLibrary::Workflow[i].parameters[dataLibrary::Workflow[i].parameters.size()-1];
						commands += ";";
					}
					else
					{
						commands += dataLibrary::Workflow[i].command;
						commands += ";";
					}
				}
				
				#if defined(_WIN32)||defined(_WIN64)
					size_t f = commands.find("\\");
					commands.replace(f, std::string("\\").length(), "\\\\");
				#endif

				QProcess *myProcess = new QProcess;
				QStringList commandsList;
				commandsList << "-c"<<commands.c_str();
				myProcess->setWorkingDirectory(QApplication::applicationDirPath());
				#if !defined(_WIN32)&&(defined(__unix__)||defined(__unix)||(defined(__APPLE__)&&defined(__MACH__)))
					myProcess->start("./structrock",commandsList);
				#elif defined(_WIN32)||defined(_WIN64)
					myProcess->start("structrock",commandsList);
				#endif
			}
			TimingShutdown *shutdown(new TimingShutdown);
			connect(shutdown, SIGNAL(shutdown()), this, SLOT(exit()));
			shutdown->start();
		}
		else if(command_string == "showstereonet")
		{
			if(dataLibrary::Workflow[dataLibrary::current_workline_index].parameters.size()>0)
			{
				std::string filename = dataLibrary::Workflow[dataLibrary::current_workline_index].parameters[0];
				dataLibrary::current_workline_index+=1;
				need_move_on = false;
				Stereonet(QString::fromUtf8(filename.c_str()));
			}
			else
			{
				dataLibrary::current_workline_index+=1;
				need_move_on = false;
				Stereonet();
			}
		}
        else if(command_string == "test")
		{
			if(dataLibrary::Workflow[dataLibrary::current_workline_index].parameters.size()>0)
			{
				testworker.setWorkFlowMode(true);
                testworker.setSplitMode(false);
                if(dataLibrary::Workflow[dataLibrary::current_workline_index].parameters.size()>1)
                {
                    if(dataLibrary::Workflow[dataLibrary::current_workline_index].parameters[1] == "split")
                    {
                        testworker.setSplitMode(true);
                    }
                }
				connect(&testworker, SIGNAL(showErrors(QString)), this, SLOT(Show_Errors(QString)));
				connect(&testworker, SIGNAL(showReadyStatus()), this, SLOT(ShowReady()));
				connect(&testworker, SIGNAL(GoWorkFlow()), this, SLOT(command_parser()));
                
				testworker.testing(QString::fromUtf8(dataLibrary::Workflow[dataLibrary::current_workline_index].parameters[0].c_str()));
			}
			else
			{
				Show_Errors(QString("Test: Path not provided."));
			}
		}
		else
		{
			std::stringstream ss;
			ss<<dataLibrary::current_workline_index+1;
			std::string errors = "Error at line "+ss.str()+", "+dataLibrary::Workflow[dataLibrary::current_workline_index].command+": No such command.";
			Show_Errors(QString::fromUtf8(errors.c_str()));
		}
		if(need_move_on)
		{
			dataLibrary::current_workline_index+=1;
		}
	}
	}
}

void structrock::Show_Errors(const QString &errors)
{
	QMessageBox::warning(NULL, "warning", errors, QMessageBox::Ok);
}

void structrock::ShowPCD(int i)
{
	viewer->removeAllPointClouds(v1);
	viewer->removeAllShapes(v1);
	viewer->removeAllPointClouds(v2);
	viewer->removeAllShapes(v2);

	if(i==CLOUDXYZ)
	{
		viewer->addPointCloud(dataLibrary::cloudxyz, dataLibrary::cloudID, v1);
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.7, 0.0, dataLibrary::cloudID, v1);
        
        viewer->resetCameraViewpoint (dataLibrary::cloudID);
        viewer->setCameraPosition (0,0,0,		// Position
                                   0,0,-1,		// Viewpoint
                                   0,1,0);	    // Down
        viewer->resetCamera();
        
		ui.qvtkWidget->update();
	}
	else if(i==CLOUDXYZRGB)
	{
		viewer->addPointCloud(dataLibrary::cloudxyzrgb, dataLibrary::cloudID, v1);
        
        viewer->resetCameraViewpoint (dataLibrary::cloudID);
        viewer->setCameraPosition (0,0,0,		// Position
                                   0,0,-1,		// Viewpoint
                                   0,1,0);	    // Down
        viewer->resetCamera();
        
		ui.qvtkWidget->update();
	}
}

void structrock::saveasascii()
{
	QString filename = QFileDialog::getSaveFileName(this,tr("Save PCD data as ASCII"),QDir::currentPath(),tr("Point Cloud Data (*.pcd)"));
	if(!filename.isNull())
	{
		savepcdASCIIworker.setWorkFlowMode(false);
		connect(&savepcdASCIIworker, SIGNAL(showErrors(QString)), this, SLOT(Show_Errors(QString)));
		connect(&savepcdASCIIworker, SIGNAL(showReadyStatus()), this, SLOT(ShowReady()));

		savepcdASCIIworker.saveascii(filename);
	}
}

void structrock::saveasbinary()
{
	QString filename = QFileDialog::getSaveFileName(this,tr("Save PCD data as Binary"),QDir::currentPath(),tr("Point Cloud Data (*.pcd)"));
	if(!filename.isNull())
	{
		savepcdBinaryworker.setWorkFlowMode(false);
		connect(&savepcdBinaryworker, SIGNAL(showErrors(QString)), this, SLOT(Show_Errors(QString)));
		connect(&savepcdBinaryworker, SIGNAL(showReadyStatus()), this, SLOT(ShowReady()));

		savepcdBinaryworker.savebinary(filename);
	}
}

void structrock::exit()
{
    if (grabber && grabber->isRunning ()) grabber->stop ();
    
	QApplication::closeAllWindows();
	qApp->exit();
}

void structrock::resampling()
{
    if(dataLibrary::haveBaseData())
    {
        bool ok = false;
        double radius = QInputDialog::getDouble(NULL, "Search Radius", "Please Set The Search Radius", 0.01, 0, 100, 3, &ok);
        if(ok)
        {
			resampleworker.setWorkFlowMode(false);
            resampleworker.setUnmute();
            connect(&resampleworker, SIGNAL(show()), this, SLOT(ShowResample()));
			connect(&resampleworker, SIGNAL(showErrors(QString)), this, SLOT(Show_Errors(QString)));
            connect(&resampleworker, SIGNAL(showReadyStatus()), this, SLOT(ShowReady()));
            
            resampleworker.resample(radius);
        }
    }
    else
    {
		Show_Errors(QString("You Do Not Have Any Point Cloud Data in Memery!"));
    }
}

void structrock::ShowResample()
{
	viewer->removeAllPointClouds(v1);
	viewer->addPointCloud(dataLibrary::cloudxyz, dataLibrary::cloudID, v1);
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.7, 0.0, dataLibrary::cloudID, v1);

	viewer->removeAllPointClouds(v2);
	viewer->addPointCloud(dataLibrary::mls_points, "resampled", v2);
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.7, 0.0, "resampled", v2);
	ui.qvtkWidget->update();

	dataLibrary::temp_cloud->clear();
	*dataLibrary::temp_cloud = *dataLibrary::mls_points;
	dataLibrary::mls_points->clear();
}

void structrock::downsampling()
{
    if(dataLibrary::haveBaseData())
    {
        bool ok = false;
        double leaf = QInputDialog::getDouble(NULL, "Minimum Point Distance", "Please Set The Minimum Point Distance (Meter)", 0.01, 0, 100, 3, &ok);
        if(ok)
        {
			downsampleworker.setWorkFlowMode(false);
            downsampleworker.setUnmute();
            connect(&downsampleworker, SIGNAL(show()), this, SLOT(ShowDownsample()));
			connect(&downsampleworker, SIGNAL(showErrors(QString)), this, SLOT(Show_Errors(QString)));
            connect(&downsampleworker, SIGNAL(showReadyStatus()), this, SLOT(ShowReady()));
            
            downsampleworker.downsample(leaf);
        }
    }
    else
    {
		Show_Errors(QString("You Do Not Have Any Point Cloud Data in Memery!"));
    }
}

void structrock::ShowDownsample()
{
	viewer->removeAllPointClouds(v1);
	viewer->addPointCloud(dataLibrary::cloudxyz, dataLibrary::cloudID, v1);
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.7, 0.0, dataLibrary::cloudID, v1);
	
	viewer->removeAllPointClouds(v2);
	viewer->addPointCloud(dataLibrary::downsampledxyz, "downsampled", v2);
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.7, 0.0, "downsampled", v2);
	ui.qvtkWidget->update();

	dataLibrary::temp_cloud->clear();
	*dataLibrary::temp_cloud = *dataLibrary::downsampledxyz;
	dataLibrary::downsampledxyz->clear();
}

void structrock::k_neighbor()
{
    if(dataLibrary::haveBaseData())
    {
        bool ok = false;
        int k = QInputDialog::getInt(NULL, "Number of Neighbor Points", "Please Set The Number of Neighbor Points", 20, 0, 200, 1, &ok);
        if(ok)
        {
            knnormalworker.setWorkFlowMode(false);
            knnormalworker.setUnmute();
			connect(&knnormalworker, SIGNAL(show()), this, SLOT(ShowknNormal()));
			connect(&knnormalworker, SIGNAL(showErrors(QString)), this, SLOT(Show_Errors(QString)));
			connect(&knnormalworker, SIGNAL(showReadyStatus()), this, SLOT(ShowReady()));

			knnormalworker.knnormal(k);
        }
    }
    else
    {
		Show_Errors(QString("You Do Not Have Any Point Cloud Data in Memery!"));
    }
}

void structrock::ShowknNormal()
{
	viewer->removeAllPointClouds(v1);
	viewer->addPointCloud(dataLibrary::cloudxyz, dataLibrary::cloudID, v1);
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.7, 0.0, dataLibrary::cloudID, v1);

	viewer->removeAllPointClouds(v2);
	viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(dataLibrary::cloudxyz, dataLibrary::normal, 50, 0.02, "normals", v2);
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.0, 1.0, "normals", v2);
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 2, "normals", v2);
	ui.qvtkWidget->update();

	pcl::concatenateFields(*dataLibrary::cloudxyz, *dataLibrary::normal, *dataLibrary::pointnormals);
}

void structrock::radius()
{
    if(dataLibrary::haveBaseData())
    {
        bool ok = false;
        double radius = QInputDialog::getDouble(NULL, "Search Radius", "Please Set The Search Radius", 0.01, 0, 100, 3, &ok);
        if(ok)
        {
            ranormalworker.setWorkFlowMode(false);
            ranormalworker.setUnmute();
			connect(&ranormalworker, SIGNAL(show()), this, SLOT(ShowraNormal()));
			connect(&ranormalworker, SIGNAL(showErrors(QString)), this, SLOT(Show_Errors(QString)));
			connect(&ranormalworker, SIGNAL(showReadyStatus()), this, SLOT(ShowReady()));
			
			ranormalworker.ranormal(radius);
        }
    }
    else
    {
		Show_Errors(QString("You Do Not Have Any Point Cloud Data in Memery!"));
    }
}

void structrock::ShowraNormal()
{
	viewer->removeAllPointClouds(v1);
	viewer->addPointCloud(dataLibrary::cloudxyz, dataLibrary::cloudID, v1);
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.7, 0.0, dataLibrary::cloudID, v1);

	viewer->removeAllPointClouds(v2);
	viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(dataLibrary::cloudxyz, dataLibrary::normal, 50, 0.02, "normals", v2);
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.0, 1.0, "normals", v2);
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 2, "normals", v2);
	ui.qvtkWidget->update();

	pcl::concatenateFields(*dataLibrary::cloudxyz, *dataLibrary::normal, *dataLibrary::pointnormals);
}

void structrock::saveScreen()
{
	QString filename = QFileDialog::getSaveFileName(this,tr("Save Screen Shot as png"),tr(""),tr("(*.png)"));
	QByteArray ba = filename.toLocal8Bit();
	string* strfilename = new string(ba.data());
	if(!filename.isNull())
	{
		viewer->saveScreenshot(*strfilename);
	}

	delete strfilename;
}

void structrock::StaticRemoveOutlier()
{
    if(dataLibrary::haveBaseData())
    {
        bool ok = false;
        double stdDev = QInputDialog::getDouble(NULL, "Standard Deviation", "Please Set The Standard Deviation", 1, 0, 100, 2, &ok);
        if(ok)
        {
            staticroworker.setWorkFlowMode(false);
            staticroworker.setUnmute();
			connect(&staticroworker, SIGNAL(show()), this, SLOT(ShowSRO()));
			connect(&staticroworker, SIGNAL(showErrors(QString)), this, SLOT(Show_Errors(QString)));
			connect(&staticroworker, SIGNAL(showReadyStatus()), this, SLOT(ShowReady()));
            
			staticroworker.rostatic(stdDev);
        }
    }
    else
    {
		Show_Errors(QString("You Do Not Have Any Point Cloud Data in Memery!"));
    }
}

void structrock::ShowSRO()
{
	viewer->removeAllPointClouds(v1);
	viewer->addPointCloud(dataLibrary::cloudxyz, dataLibrary::cloudID, v1);
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.7, 0.0, dataLibrary::cloudID, v1);


	viewer->removeAllPointClouds(v2);
	viewer->addPointCloud(dataLibrary::outlier_removed_outlier, "outlier", v2);
	//viewer->resetCameraViewpoint("outlier");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.7, 0.0, 0.0, "outlier", v2);

	viewer->addPointCloud(dataLibrary::outlier_removed_inlier, "inlier", v2);
	//viewer->resetCameraViewpoint("inlier");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.7, 0.0, "inlier", v2);

	ui.qvtkWidget->update();

	dataLibrary::temp_cloud->clear();
	*dataLibrary::temp_cloud = *dataLibrary::outlier_removed_inlier;
	dataLibrary::outlier_removed_inlier->clear();
	dataLibrary::outlier_removed_outlier->clear();
}

void structrock::ConditionalRemoveOutlier()
{

}

void structrock::RadiusRemoveOutlier()
{

}

void structrock::SaveNormals()
{
	QString filename = QFileDialog::getSaveFileName(this,tr("Save normals data as Binary"),QDir::currentPath(),tr("Point Cloud Data (*.pcd)"));
	if(!filename.isNull())
	{
		savenormalsworker.setWorkFlowMode(false);
		connect(&savenormalsworker, SIGNAL(showErrors(QString)), this, SLOT(Show_Errors(QString)));
		connect(&savenormalsworker, SIGNAL(showReadyStatus()), this, SLOT(ShowReady()));

		savenormalsworker.savenormals(filename);
	}
}

void structrock::RegionGrowingSegmentation()
{
    if(dataLibrary::haveBaseData())
    {
        if(dataLibrary::pointnormals->empty())
        {
			Show_Errors(QString("You Haven't Extracted Any Normals Yet! Please Extract Normals First."));
        }
        else
        {
            MultiInputDialog multi_input;
            
            if(multi_input.exec())
            {
                dataLibrary::RGSparameter.curvature = multi_input.getCurvatureThreshold();
                dataLibrary::RGSparameter.smoothness = multi_input.getSmoothnessThreshold();
                dataLibrary::RGSparameter.residual = multi_input.getResidualThreshold();
                dataLibrary::RGSparameter.number_of_neighbors = multi_input.getNumberOfNeighbors();
                dataLibrary::RGSparameter.min_number_of_Points = multi_input.getMinNumberOfPoints();
                dataLibrary::RGSparameter.IsSmoothMode = multi_input.IsSmoothMode();
                
                rgsworker.setWorkFlowMode(false);
                rgsworker.setUnmute();
				connect(&rgsworker, SIGNAL(show()), this, SLOT(ShowRGS()));
				connect(&rgsworker, SIGNAL(showErrors(QString)), this, SLOT(Show_Errors(QString)));
				connect(&rgsworker, SIGNAL(showReadyStatus()), this, SLOT(ShowReady()));

				rgsworker.rgs();
            }
        }
    }
    else
    {
		Show_Errors(QString("You Do Not Have Any Point Cloud Data in Memery!"));
    }
}

void structrock::ShowRGS()
{
	if(!dataLibrary::cloudxyzrgb->empty())
	{
		viewer->removeAllPointClouds(v1);
		viewer->addPointCloud(dataLibrary::cloudxyzrgb, dataLibrary::cloudID, v1);
	}
	else
	{
		viewer->removeAllPointClouds(v1);
		viewer->addPointCloud(dataLibrary::cloudxyz, dataLibrary::cloudID, v1);
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.7, 0.0, dataLibrary::cloudID, v1);
	}

	viewer->removeAllPointClouds(v2);
	viewer->addPointCloud(dataLibrary::cloudxyzrgb_clusters, "region_growing_segmentation", v2);

	ui.qvtkWidget->update();
}

void structrock::DrawLine(const Eigen::Vector3f begin, const Eigen::Vector3f end, float r, float g, float b, std::string id, int viewpot)
{
	pcl::PointXYZ point_begin, point_end;
	point_begin.x=begin(0);
	point_begin.y=begin(1);
	point_begin.z=begin(2);
	point_end.x=end(0);
	point_end.y=end(1);
	point_end.z=end(2);

	viewer->addLine<pcl::PointXYZ>(point_begin, point_end, r,g,b,id, viewpot);
}

void structrock::DrawLine(const Eigen::Vector3f begin, const Eigen::Vector3f end, float r, float g, float b, double lineWidth, std::string id, int viewpot)
{
	pcl::PointXYZ point_begin, point_end;
	point_begin.x=begin(0);
	point_begin.y=begin(1);
	point_begin.z=begin(2);
	point_end.x=end(0);
	point_end.y=end(1);
	point_end.z=end(2);

	viewer->addLine<pcl::PointXYZ>(point_begin, point_end, r,g,b,id, viewpot);
	viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, lineWidth, id, viewpot);
}

void structrock::SaveClusters()
{
	if(dataLibrary::clusters.size() == 0)
	{
		Show_Errors(QString("You Haven't Performed Any Segmentation Yet!"));
	}
	else
	{
        QString filename = QFileDialog::getSaveFileName(this,tr("Save Clusters"),QDir::currentPath(),tr("(*.bin)"));

        if(!filename.isNull())
        {   
			saveclustersworker.setWorkFlowMode(false);
            saveclustersworker.setUnmute();
            connect(&saveclustersworker, SIGNAL(SaveClustersReady(QString)), this, SLOT(ShowSavedClusters(QString)));
			connect(&saveclustersworker, SIGNAL(showErrors(QString)), this, SLOT(Show_Errors(QString)));
            connect(&saveclustersworker, SIGNAL(showReadyStatus()), this, SLOT(ShowReady()));
            
            saveclustersworker.saveclusters(filename);
        }
	}
}

void structrock::ShowSavedClusters(QString filename)
{
	viewer->removeAllPointClouds(v2);
	viewer->removeAllShapes(v2);
	viewer->addPolygon<pcl::PointXYZ>(dataLibrary::cloud_hull_all, 1, 1, 1, "outcrop_polygon", v2);

    for(int i=0; i<dataLibrary::Lines.size(); i++)
    {
		Eigen::Vector3f begin, end;
        begin(0) = dataLibrary::Lines[i].begin.x;
        begin(1) = dataLibrary::Lines[i].begin.y;
        begin(2) = dataLibrary::Lines[i].begin.z;
        end(0) = dataLibrary::Lines[i].end.x;
        end(1) = dataLibrary::Lines[i].end.y;
        end(2) = dataLibrary::Lines[i].end.z;
        DrawLine(begin, end, dataLibrary::Lines[i].r, dataLibrary::Lines[i].g, dataLibrary::Lines[i].b, 3, dataLibrary::Lines[i].ID, v2);
    }

	ui.qvtkWidget->update();
    
    Show_SaveTraceMap(filename);
}

void structrock::Show_SaveTraceMap(QString filename)
{
	PlotWindow plotWin;
    
    Eigen::Vector3f V_x = dataLibrary::cloud_hull_all->at(1).getVector3fMap() - dataLibrary::cloud_hull_all->at(0).getVector3fMap();
    Eigen::Vector3f V_y = dataLibrary::plane_normal_all.cross(V_x);
	float max_value, min_value;
	int max_i, min_i;
	max_i=min_i=0;
	for(int i=0; i<dataLibrary::cloud_hull_all->size(); i++)
	{
		Eigen::Vector3f Q;
		Q(0)=dataLibrary::cloud_hull_all->at(i).x;
		Q(1)=dataLibrary::cloud_hull_all->at(i).y;
		Q(2)=dataLibrary::cloud_hull_all->at(i).z;
		float value = (Q-dataLibrary::cloud_hull_all->at(0).getVector3fMap()).dot(V_x)/std::sqrt(V_x.dot(V_x));
		if(i==0)
		{
			max_value=min_value=value;
		}
		else
		{
			if(value>=max_value)
			{
				max_value=value;
				max_i=i;
			}
			else if(value<=min_value)
			{
				min_value=value;
				min_i=i;
			}
		}
	}
	Eigen::Vector3f x_dis = dataLibrary::cloud_hull_all->at(max_i).getVector3fMap()-dataLibrary::cloud_hull_all->at(min_i).getVector3fMap();
	float x_length = std::sqrt(x_dis.dot(x_dis));

    std::vector<Eigen::Vector2f> convex_hull_all_2d;
    dataLibrary::projection322(V_x, V_y, dataLibrary::cloud_hull_all, convex_hull_all_2d);
    QVector<double> x, y;
    for(int i=0; i<convex_hull_all_2d.size(); i++)
    {
        x.append(convex_hull_all_2d[i](0));
        y.append(convex_hull_all_2d[i](1));
    }

	x.append(x[0]);
	y.append(y[0]);

	QCPCurve *newCurve = new QCPCurve(plotWin.ui.plotWidget->xAxis, plotWin.ui.plotWidget->yAxis);
	plotWin.ui.plotWidget->addPlottable(newCurve);
    
    newCurve->setData(x, y);
    // set axes ranges, so we see all data:
	QVector<double>::iterator x_max_it = std::max_element(x.begin(), x.end());
    QVector<double>::iterator x_min_it = std::min_element(x.begin(), x.end());
    QVector<double>::iterator y_max_it = std::max_element(y.begin(), y.end());
    QVector<double>::iterator y_min_it = std::min_element(y.begin(), y.end());
	double xrange = *x_max_it-*x_min_it;
	double yrange = *y_max_it-*y_min_it;
	double mid_x = (*x_max_it+*x_min_it)/2;
	double mid_y = (*y_max_it+*y_min_it)/2;
	double bias;
	if(xrange>yrange)
	{
		bias = 0.54*xrange;
	}
	else
	{
		bias = 0.54*yrange;
	}
	plotWin.ui.plotWidget->xAxis->setRange(mid_x-bias, mid_x+bias);
	plotWin.ui.plotWidget->yAxis->setRange(mid_y-bias, mid_y+bias);
	plotWin.ui.plotWidget->xAxis->setVisible(false);
	plotWin.ui.plotWidget->yAxis->setVisible(false);
	plotWin.ui.plotWidget->xAxis->grid()->setVisible(false);
	plotWin.ui.plotWidget->yAxis->grid()->setVisible(false);
	plotWin.ui.plotWidget->axisRect()->setAutoMargins(QCP::msNone);
	plotWin.ui.plotWidget->axisRect()->setMargins(QMargins(0,0,0,0));
    plotWin.ui.plotWidget->replot();

	QCPCurve *scale = new QCPCurve(plotWin.ui.plotWidget->xAxis, plotWin.ui.plotWidget->yAxis);
	plotWin.ui.plotWidget->addPlottable(scale);
	QVector<double> scale_x, scale_y;
	scale_x.append(*x_min_it);
	scale_y.append(*y_max_it+(*y_max_it-*y_min_it)*0.02);
	scale_x.append(*x_min_it);
	scale_y.append(*y_max_it+(*y_max_it-*y_min_it)*0.01);
	scale_x.append(*x_max_it);
	scale_y.append(*y_max_it+(*y_max_it-*y_min_it)*0.01);
	scale_x.append(*x_max_it);
	scale_y.append(*y_max_it+(*y_max_it-*y_min_it)*0.02);
	scale->setData(scale_x, scale_y);
	scale->setPen(QPen(Qt::blue));
	plotWin.ui.plotWidget->replot();

	QCPItemText *text = new QCPItemText(plotWin.ui.plotWidget);
	plotWin.ui.plotWidget->addItem(text);
	text->position->setCoords((*x_max_it+*x_min_it)/2, *y_max_it+(*y_max_it-*y_min_it)*0.025);
	std::stringstream ss;
	ss<<x_length<<" m";
	text->setText(QString::fromUtf8(ss.str().c_str()));
	plotWin.ui.plotWidget->replot();

	Eigen::Vector3f point_in_begin, point_in_end;
	Eigen::Vector2f point_out_begin, point_out_end;
    for(int i=0; i<dataLibrary::Lines.size(); i++)
    {
        if(dataLibrary::Lines[i].ID.find("Line_in") != std::string::npos)
        {
			point_in_begin(0)=dataLibrary::Lines[i].begin.x;
			point_in_begin(1)=dataLibrary::Lines[i].begin.y;
			point_in_begin(2)=dataLibrary::Lines[i].begin.z;
			point_in_end(0)=dataLibrary::Lines[i].end.x;
			point_in_end(1)=dataLibrary::Lines[i].end.y;
			point_in_end(2)=dataLibrary::Lines[i].end.z;

			dataLibrary::projection322(V_x, V_y, point_in_begin, point_out_begin);
			dataLibrary::projection322(V_x, V_y, point_in_end, point_out_end);

			QCPCurve *newCurve = new QCPCurve(plotWin.ui.plotWidget->xAxis, plotWin.ui.plotWidget->yAxis);
			plotWin.ui.plotWidget->addPlottable(newCurve);
			QVector<double> x, y;
			x.append(point_out_begin(0));
			x.append(point_out_end(0));
			y.append(point_out_begin(1));
			y.append(point_out_end(1));
			newCurve->setData(x, y);
			newCurve->setPen(QPen(Qt::black));
			plotWin.ui.plotWidget->replot();
		}
	}
	filename.chop(4);
	filename+=".pdf";
	plotWin.ui.plotWidget->savePdf(filename,true, 1000, 1000);
}

void structrock::Show_Process()
{
	viewer->removeAllPointClouds(v2);
	viewer->removeAllShapes(v1);
	viewer->removeAllShapes(v2);

	if(dataLibrary::checkContents(dataLibrary::contents, "point_cloud"))
	{
		if(!dataLibrary::cloudxyzrgb->empty())
		{
			viewer->addPointCloud(dataLibrary::cloudxyzrgb, dataLibrary::cloudID+"v2", v2);
		}
		else
		{
			viewer->addPointCloud(dataLibrary::cloudxyz, dataLibrary::cloudID+"v2", v2);
		}
	}

	if(dataLibrary::checkContents(dataLibrary::contents, "suppositional_plane"))
	{
		viewer->addPolygon<pcl::PointXYZ>(dataLibrary::cloud_hull_all, 0, 1, 1, "outcrop_polygon_v1", v1);
		viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE, "outcrop_polygon_v1", v1);
		viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, "outcrop_polygon_v1", v1);

		viewer->addPolygon<pcl::PointXYZ>(dataLibrary::cloud_hull_all, 0, 1, 1, "outcrop_polygon_v2", v2);
		viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE, "outcrop_polygon_v2", v2);
		viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, "outcrop_polygon_v2", v2);
	}

	if(dataLibrary::checkContents(dataLibrary::contents, "fracture_faces"))
	{
		bool show_rem = false;
		if(dataLibrary::checkContents(dataLibrary::contents, "rgs_remanent"))
		{
			show_rem = true;
		}
		bool show_fracture_traces = false;
		bool show_extension_line = false;
		if(dataLibrary::checkContents(dataLibrary::contents, "suppositional_plane")&&dataLibrary::checkContents(dataLibrary::contents, "fracture_traces"))
		{
			show_fracture_traces = true;
		}
		if(dataLibrary::checkContents(dataLibrary::contents, "suppositional_plane")&&dataLibrary::checkContents(dataLibrary::contents, "extension_line"))
		{
			show_extension_line = true;
		}

		for(int cluster_index = 0; cluster_index < dataLibrary::clusters.size(); cluster_index++)
		{
			stringstream ss;
			ss << cluster_index;
			float r, g, b;
			pcl::PointXYZRGB point = dataLibrary::cloudxyzrgb_clusters->at(dataLibrary::clusters[cluster_index].indices[0]);
			r=point.r/255.0;
			g=point.g/255.0;
			b=point.b/255.0;
			viewer->addPolygon<pcl::PointXYZ>(dataLibrary::fracture_faces_hull[cluster_index], r, g, b, "polygon_"+ss.str(), v2);
			viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE, "polygon_"+ss.str(), v2);

			if(show_fracture_traces)
			{
				if(show_extension_line)
				{
					Eigen::Vector3f begin_max, end_max;
					begin_max(0) = dataLibrary::Lines_max[cluster_index].begin.x;
					begin_max(1) = dataLibrary::Lines_max[cluster_index].begin.y;
					begin_max(2) = dataLibrary::Lines_max[cluster_index].begin.z;
					end_max(0) = dataLibrary::Lines_max[cluster_index].end.x;
					end_max(1) = dataLibrary::Lines_max[cluster_index].end.y;
					end_max(2) = dataLibrary::Lines_max[cluster_index].end.z;
					DrawLine(begin_max, end_max, dataLibrary::Lines_max[cluster_index].r, dataLibrary::Lines_max[cluster_index].g, dataLibrary::Lines_max[cluster_index].b, dataLibrary::Lines_max[cluster_index].ID, v2);

					Eigen::Vector3f begin_min, end_min;
					begin_min(0) = dataLibrary::Lines_min[cluster_index].begin.x;
					begin_min(1) = dataLibrary::Lines_min[cluster_index].begin.y;
					begin_min(2) = dataLibrary::Lines_min[cluster_index].begin.z;
					end_min(0) = dataLibrary::Lines_min[cluster_index].end.x;
					end_min(1) = dataLibrary::Lines_min[cluster_index].end.y;
					end_min(2) = dataLibrary::Lines_min[cluster_index].end.z;
					DrawLine(begin_min, end_min, dataLibrary::Lines_min[cluster_index].r, dataLibrary::Lines_min[cluster_index].g, dataLibrary::Lines_min[cluster_index].b, dataLibrary::Lines_min[cluster_index].ID, v2);
				}

				Eigen::Vector3f begin, end;
				begin(0) = dataLibrary::Lines[cluster_index].begin.x;
				begin(1) = dataLibrary::Lines[cluster_index].begin.y;
				begin(2) = dataLibrary::Lines[cluster_index].begin.z;
				end(0) = dataLibrary::Lines[cluster_index].end.x;
				end(1) = dataLibrary::Lines[cluster_index].end.y;
				end(2) = dataLibrary::Lines[cluster_index].end.z;
				DrawLine(begin, end, dataLibrary::Lines[cluster_index].r, dataLibrary::Lines[cluster_index].g, dataLibrary::Lines[cluster_index].b, 3, dataLibrary::Lines[cluster_index].ID, v2);
			}
		}

		if(show_rem)
		{
			viewer->addPointCloud(dataLibrary::segmentation_rem, dataLibrary::cloudID+"v2", v2);
		}
	}

	ui.qvtkWidget->update();
}

void structrock::OpenClusters()
{
	QString filename = QFileDialog::getOpenFileName(this,tr("Open Point Cloud Clusters Data"),QDir::currentPath(),tr("(*.bin)"));
	QByteArray ba = filename.toLocal8Bit();
	string* strfilename = new string(ba.data());
    stringstream ss;

	if(!filename.isNull())
	{
		if(!dataLibrary::cluster_patches.empty())
			dataLibrary::cluster_patches.clear();
		if(!dataLibrary::dips.empty())
			dataLibrary::dips.clear();
		if(!dataLibrary::dip_directions.empty())
			dataLibrary::dip_directions.clear();
		if(!dataLibrary::areas.empty())
			dataLibrary::areas.clear();
		if(!dataLibrary::patchIDs.empty())
			dataLibrary::patchIDs.clear();
		if(!dataLibrary::selectedPatches.empty())
			dataLibrary::selectedPatches.clear();

		ifstream fbinaryin(strfilename->c_str(), ios::in|ios::binary);
		if(fbinaryin.is_open())
		{
			viewer->removeAllPointClouds(v2);

			int num_of_clusters, num_of_points;
			float x, y, z, rgb, dip, dip_direction, area;
			fbinaryin.read(reinterpret_cast<char*>(&num_of_clusters), sizeof(num_of_clusters));
			Eigen::Vector4f centroid;
			float centor_x, centor_y, centor_z;
			for(int i=0; i<num_of_clusters; i++)
			{
				pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_patch (new pcl::PointCloud<pcl::PointXYZRGB>);
				fbinaryin.read(reinterpret_cast<char*>(&num_of_points), sizeof(num_of_points));
				fbinaryin.read(reinterpret_cast<char*>(&rgb), sizeof(rgb));
				for(int j=0; j<num_of_points; j++)
				{
					fbinaryin.read(reinterpret_cast<char*>(&x), sizeof(x));
					fbinaryin.read(reinterpret_cast<char*>(&y), sizeof(y));
					fbinaryin.read(reinterpret_cast<char*>(&z), sizeof(z));
					pcl::PointXYZRGB point;
					point.x = x;
					point.y = y;
					point.z = z;
					point.rgb = rgb;
					cloud_patch->push_back(point);
				}
				fbinaryin.read(reinterpret_cast<char*>(&dip), sizeof(dip));
				fbinaryin.read(reinterpret_cast<char*>(&dip_direction), sizeof(dip_direction));
				fbinaryin.read(reinterpret_cast<char*>(&area), sizeof(area));
				dataLibrary::cluster_patches.push_back(cloud_patch);
				dataLibrary::dips.push_back(dip);
				dataLibrary::dip_directions.push_back(dip_direction);
				dataLibrary::areas.push_back(area);

                ss << i;
				string patchID = *strfilename += ss.str();
				dataLibrary::patchIDs.push_back(patchID);
				viewer->addPointCloud(cloud_patch, patchID, v2);
				pcl::compute3DCentroid(*cloud_patch, centroid);
				centor_x += centroid[0];
				centor_y += centroid[1];
				centor_z += centroid[2];
			}
			centor_x /= num_of_clusters;
			centor_y /= num_of_clusters;
			centor_z /= num_of_clusters;
			viewer->setCameraPosition(centor_x,centor_y,centor_z,centor_x,centor_y,centor_z);
			viewer->resetCamera();
			ui.qvtkWidget->update();
			fbinaryin.close();
		}
		else
		{
			Show_Errors(QString("Can Not Open the File!"));
		}
	}

	delete strfilename;
}

void structrock::MoveForwardPatch()
{
	if(dataLibrary::patchIDs.size()!=0)
	{
		if(dataLibrary::currentPatch+1 == dataLibrary::patchIDs.size())
		{
			dataLibrary::currentPatch = 0;
		}
		else
		{
			dataLibrary::currentPatch +=1;
		}
		updatePatch();
	}
	else
	{
		Show_Errors(QString("You Haven't Performed Any Segmentation Yet!"));
	}
}

void structrock::MoveBackPatch()
{
	if(dataLibrary::patchIDs.size()!=0)
	{
		if(dataLibrary::currentPatch == 0)
		{
			dataLibrary::currentPatch = dataLibrary::patchIDs.size() - 1;
		}
		else
		{
			dataLibrary::currentPatch -= 1;
		}
		updatePatch();
	}
	else
	{
		Show_Errors(QString("You Haven't Performed Any Segmentation Yet!"));
	}
}

void structrock::updatePatch()
{
	viewer->removeAllPointClouds(v1);
	viewer->removeAllShapes(v1);
	viewer->removeAllPointClouds(v2);
	viewer->removeAllShapes(v2);
	viewer->addPointCloud(dataLibrary::cluster_patches[dataLibrary::currentPatch], dataLibrary::patchIDs[dataLibrary::currentPatch], v1);
	Eigen::Vector4f centroid;
	pcl::compute3DCentroid(*dataLibrary::cluster_patches[dataLibrary::currentPatch], centroid);
	viewer->setCameraPosition(centroid[0],centroid[1],centroid[2],centroid[0],centroid[1],centroid[2]);
	viewer->resetCamera();
	std::ostringstream o_dip, o_dip_direction, o_area;
	o_dip<<dataLibrary::dips[dataLibrary::currentPatch];
	o_dip_direction<<dataLibrary::dip_directions[dataLibrary::currentPatch];
	o_area<<dataLibrary::areas[dataLibrary::currentPatch];
	//viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_FONT_SIZE, 20.0, cloudID, v1);
	viewer->addText("dip: "+o_dip.str()+" dip direction: "+o_dip_direction.str()+" area: "+o_area.str(), 10, 10, dataLibrary::patchIDs[dataLibrary::currentPatch]+" text", v1);
	ui.qvtkWidget->update();
}

void structrock::CheckPatchesIfSelected(float x, float y, float z)
{
	for(int i=0; i<dataLibrary::patchIDs.size(); i++)
	{
		Eigen::Vector4f min_pt, max_pt;
		pcl::getMinMax3D(*dataLibrary::cluster_patches[i], min_pt, max_pt);
		if(min_pt[0]<x&&x<max_pt[0]&&min_pt[1]<y&&y<max_pt[1]&&min_pt[2]<z&&z<max_pt[2])
		{
			bool haveSelected = false;
			if(dataLibrary::selectedPatches.size()!=0)
			{
				for(int k=0; k<dataLibrary::selectedPatches.size(); k++)
				{
					if(dataLibrary::selectedPatches[k] == i)
					{
						haveSelected = true;
					}
				}
			}
			if(!haveSelected)
			{
				for(int j=0; j<dataLibrary::cluster_patches[i]->points.size(); j++)
				{
					if(dataLibrary::cluster_patches[i]->points[j].x==x&&dataLibrary::cluster_patches[i]->points[j].y==y&&dataLibrary::cluster_patches[i]->points[j].z==z)
					{
						dataLibrary::selectedPatches.push_back(i);
						viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 1.0, 1.0, dataLibrary::patchIDs[i], v2);
					}
				}
			}
		}
	}
}

void structrock::ShowSelectedPatches()
{
	if(dataLibrary::selectedPatches.size()!=0)
	{
		viewer->removeAllPointClouds(v1);
		viewer->removeAllShapes(v1);
		viewer->removeAllPointClouds(v2);
		viewer->removeAllShapes(v2);
		if(!dataLibrary::cloudxyzrgb->empty())
		{
			viewer->addPointCloud(dataLibrary::cloudxyzrgb, "rgbcloud", v1);
		}
		Eigen::Vector4f centroid;
		float centor_x, centor_y, centor_z;
		for(int i=0; i<dataLibrary::selectedPatches.size(); i++)
		{
			viewer->addPointCloud(dataLibrary::cluster_patches[dataLibrary::selectedPatches[i]], dataLibrary::patchIDs[dataLibrary::selectedPatches[i]], v1);
			pcl::compute3DCentroid(*dataLibrary::cluster_patches[dataLibrary::selectedPatches[i]], centroid);
			centor_x += centroid[0];
			centor_y += centroid[1];
			centor_z += centroid[2];
		}
		centor_x /= dataLibrary::selectedPatches.size();
		centor_y /= dataLibrary::selectedPatches.size();
		centor_z /= dataLibrary::selectedPatches.size();
		viewer->setCameraPosition(centor_x,centor_y,centor_z,0,0,0);
		viewer->resetCamera();
		ui.qvtkWidget->update();
	}
}

void structrock::About()
{
	QMessageBox message(this);
	message.setText(QString::fromLocal8Bit("StructRock v 1.0\nCopyright 2014 Xin Wang\nEmail: ericrussell@zju.edu.cn"));
	message.setIconPixmap(QPixmap("icon.png"));
	message.exec();
}

void drawStereonetFram(PlotWindow *plotWin)
{
	QCPCurve *newCurve = new QCPCurve(plotWin->ui.plotWidget->xAxis, plotWin->ui.plotWidget->yAxis);
	plotWin->ui.plotWidget->addPlottable(newCurve);
	QVector<double> x, y;
	for(float i=0.0; i<TWOPI; i+=0.01)
	{
		x.append(std::cos(i));
		y.append(std::sin(i));
	}
    newCurve->setData(x, y);
	newCurve->setPen(QPen(Qt::black));

	QString label[4]={"N","E","S","W"};
	int label_index=0;
	for(float i=TWOPI/4; i>-TWOPI*3/4; i-=TWOPI/4)
	{
		QCPCurve *newCurve = new QCPCurve(plotWin->ui.plotWidget->xAxis, plotWin->ui.plotWidget->yAxis);
		plotWin->ui.plotWidget->addPlottable(newCurve);
		QVector<double> x, y;
		x.append(std::cos(i));
		y.append(std::sin(i));
		x.append(1.03*std::cos(i));
		y.append(1.03*std::sin(i));
		newCurve->setData(x, y);
		newCurve->setPen(QPen(Qt::black));
		plotWin->ui.plotWidget->replot();

		QCPItemText *text = new QCPItemText(plotWin->ui.plotWidget);
		plotWin->ui.plotWidget->addItem(text);
		text->position->setCoords(1.05*std::cos(i), 1.05*std::sin(i));
		text->setText(label[label_index]);
		plotWin->ui.plotWidget->replot();
		label_index++;
	}

	for(float i=0.0; i<TWOPI; i+=TWOPI/36)
	{
		QCPCurve *newCurve = new QCPCurve(plotWin->ui.plotWidget->xAxis, plotWin->ui.plotWidget->yAxis);
		plotWin->ui.plotWidget->addPlottable(newCurve);
		QVector<double> x, y;
		x.append(std::cos(i));
		y.append(std::sin(i));
		x.append(1.02*std::cos(i));
		y.append(1.02*std::sin(i));
		newCurve->setData(x, y);
		newCurve->setPen(QPen(Qt::black));
		plotWin->ui.plotWidget->replot();
	}

	float mid_x(0);
	float mid_y(0);
	float bias(1.1);
	plotWin->ui.plotWidget->xAxis->setRange(mid_x-bias, mid_x+bias);
	plotWin->ui.plotWidget->yAxis->setRange(mid_y-bias, mid_y+bias);
	plotWin->ui.plotWidget->xAxis->setVisible(false);
	plotWin->ui.plotWidget->yAxis->setVisible(false);
	plotWin->ui.plotWidget->xAxis->grid()->setVisible(false);
	plotWin->ui.plotWidget->yAxis->grid()->setVisible(false);
	plotWin->ui.plotWidget->axisRect()->setAutoMargins(QCP::msNone);
	plotWin->ui.plotWidget->axisRect()->setMargins(QMargins(0,0,0,0));
    plotWin->ui.plotWidget->replot();
}

void drawWulffNet(PlotWindow *plotWin)
{
	for(float i=TWOPI/36; i<TWOPI/4; i+=TWOPI/36)
	{
		QCPCurve *newArc = new QCPCurve(plotWin->ui.plotWidget->xAxis, plotWin->ui.plotWidget->yAxis);
		plotWin->ui.plotWidget->addPlottable(newArc);
		float dip_dir(TWOPI/4);
		float dip(i);
		QVector<double> x_arc, y_arc;
		std::vector<Eigen::Vector3f> arc;
		dataLibrary::dd2arc(dip_dir, dip, arc);
		for(int i=0; i<arc.size(); i++)
		{
			Eigen::Vector2f projected;
			dataLibrary::stereonet_project(arc[i],projected);
			x_arc.append(projected(0));
			y_arc.append(projected(1));
		}
		newArc->setData(x_arc, y_arc);
		newArc->setPen(QPen(Qt::black));
		plotWin->ui.plotWidget->replot();
	}

	for(float i=TWOPI/36; i<TWOPI/4-TWOPI/36; i+=TWOPI/36)
	{
		QCPCurve *newArc = new QCPCurve(plotWin->ui.plotWidget->xAxis, plotWin->ui.plotWidget->yAxis);
		plotWin->ui.plotWidget->addPlottable(newArc);
		float dip_dir(-TWOPI/4);
		float dip(i);
		QVector<double> x_arc, y_arc;
		std::vector<Eigen::Vector3f> arc;
		dataLibrary::dd2arc(dip_dir, dip, arc);
		for(int i=0; i<arc.size(); i++)
		{
			Eigen::Vector2f projected;
			dataLibrary::stereonet_project(arc[i],projected);
			x_arc.append(projected(0));
			y_arc.append(projected(1));
		}
		newArc->setData(x_arc, y_arc);
		newArc->setPen(QPen(Qt::black));
		plotWin->ui.plotWidget->replot();
	}

	for(float L=-TWOPI/4+TWOPI/36; L<TWOPI/4; L+=TWOPI/36)
	{
		float y = std::sin(L);
		QCPCurve *newArc = new QCPCurve(plotWin->ui.plotWidget->xAxis, plotWin->ui.plotWidget->yAxis);
		plotWin->ui.plotWidget->addPlottable(newArc);
		QVector<double> x_arc, y_arc;
		float r = std::sqrt(1-y*y);
		for(float i=-TWOPI/2; i<0; i+=TWOPI/360)
		{
			Eigen::Vector2f projected;
			Eigen::Vector3f point;
			point(0)=r*std::cos(i);
			point(1)=y;
			point(2)=r*std::sin(i);
			dataLibrary::stereonet_project(point, projected);
			x_arc.append(projected(0));
			y_arc.append(projected(1));
		}
		newArc->setData(x_arc, y_arc);
		newArc->setPen(QPen(Qt::black));
		plotWin->ui.plotWidget->replot();
	}
}

void drawSchmidtNet(PlotWindow *plotWin)
{
	for(float i=TWOPI/36; i<TWOPI/4; i+=TWOPI/36)
	{
		QCPCurve *newArc = new QCPCurve(plotWin->ui.plotWidget->xAxis, plotWin->ui.plotWidget->yAxis);
		plotWin->ui.plotWidget->addPlottable(newArc);
		float dip_dir(TWOPI/4);
		float dip(i);
		QVector<double> x_arc, y_arc;
		std::vector<Eigen::Vector3f> arc;
		dataLibrary::dd2arc(dip_dir, dip, arc);
		for(int i=0; i<arc.size(); i++)
		{
			Eigen::Vector2f projected;
			dataLibrary::eqArea_project(arc[i],projected);
			x_arc.append(projected(0));
			y_arc.append(projected(1));
		}
		newArc->setData(x_arc, y_arc);
		newArc->setPen(QPen(Qt::black));
		plotWin->ui.plotWidget->replot();
	}

	for(float i=TWOPI/36; i<TWOPI/4-TWOPI/36; i+=TWOPI/36)
	{
		QCPCurve *newArc = new QCPCurve(plotWin->ui.plotWidget->xAxis, plotWin->ui.plotWidget->yAxis);
		plotWin->ui.plotWidget->addPlottable(newArc);
		float dip_dir(-TWOPI/4);
		float dip(i);
		QVector<double> x_arc, y_arc;
		std::vector<Eigen::Vector3f> arc;
		dataLibrary::dd2arc(dip_dir, dip, arc);
		for(int i=0; i<arc.size(); i++)
		{
			Eigen::Vector2f projected;
			dataLibrary::eqArea_project(arc[i],projected);
			x_arc.append(projected(0));
			y_arc.append(projected(1));
		}
		newArc->setData(x_arc, y_arc);
		newArc->setPen(QPen(Qt::black));
		plotWin->ui.plotWidget->replot();
	}

	for(float L=-TWOPI/4+TWOPI/36; L<TWOPI/4; L+=TWOPI/36)
	{
		float y = std::sin(L);
		QCPCurve *newArc = new QCPCurve(plotWin->ui.plotWidget->xAxis, plotWin->ui.plotWidget->yAxis);
		plotWin->ui.plotWidget->addPlottable(newArc);
		QVector<double> x_arc, y_arc;
		float r = std::sqrt(1-y*y);
		for(float i=-TWOPI/2; i<0; i+=TWOPI/360)
		{
			Eigen::Vector2f projected;
			Eigen::Vector3f point;
			point(0)=r*std::cos(i);
			point(1)=y;
			point(2)=r*std::sin(i);
			dataLibrary::eqArea_project(point, projected);
			x_arc.append(projected(0));
			y_arc.append(projected(1));
		}
		newArc->setData(x_arc, y_arc);
		newArc->setPen(QPen(Qt::black));
		plotWin->ui.plotWidget->replot();
	}
}

void structrock::Stereonet()
{
	if(dataLibrary::out_dips.size()>0&&dataLibrary::out_dip_directions.size()>0&&(dataLibrary::out_dips.size()==dataLibrary::out_dip_directions.size()))
	{
		PlotWindow *plotWin = new PlotWindow;

		drawStereonetFram(plotWin);

		drawSchmidtNet(plotWin);

		plotWin->ui.plotWidget->addGraph(plotWin->ui.plotWidget->xAxis, plotWin->ui.plotWidget->yAxis);
		plotWin->ui.plotWidget->graph(0)->setLineStyle(QCPGraph::lsNone);
		plotWin->ui.plotWidget->graph(0)->setScatterStyle(QCPScatterStyle::ssDisc);
		QVector<double> x, y;
		for(int i=0; i<dataLibrary::out_dips.size(); i++)
		{
			Eigen::Vector3f pole;
			Eigen::Vector2f projected;
			dataLibrary::dd2pole(dataLibrary::out_dip_directions[i], dataLibrary::out_dips[i], pole);
			dataLibrary::eqArea_project(pole, projected);
			x.append(projected(0));
			y.append(projected(1));
		}
		plotWin->ui.plotWidget->graph(0)->setData(x, y);
		plotWin->ui.plotWidget->replot();

		plotWin->setModal(false);
		plotWin->show();

		command_parser();
	}
	else
	{
		Show_Errors("You haven't saved any clusters yet.");
	}
}

void structrock::Stereonet(QString filename)
{
	if(dataLibrary::out_dips.size()>0&&dataLibrary::out_dip_directions.size()>0&&(dataLibrary::out_dips.size()==dataLibrary::out_dip_directions.size()))
	{
		PlotWindow *plotWin = new PlotWindow;

		drawStereonetFram(plotWin);

		drawSchmidtNet(plotWin);

		plotWin->ui.plotWidget->addGraph(plotWin->ui.plotWidget->xAxis, plotWin->ui.plotWidget->yAxis);
		plotWin->ui.plotWidget->graph(0)->setLineStyle(QCPGraph::lsNone);
		plotWin->ui.plotWidget->graph(0)->setScatterStyle(QCPScatterStyle::ssDisc);
		QVector<double> x, y;
		for(int i=0; i<dataLibrary::out_dips.size(); i++)
		{
			Eigen::Vector3f pole;
			Eigen::Vector2f projected;
			dataLibrary::dd2pole(dataLibrary::out_dip_directions[i], dataLibrary::out_dips[i], pole);
			dataLibrary::eqArea_project(pole, projected);
			x.append(projected(0));
			y.append(projected(1));
		}
		plotWin->ui.plotWidget->graph(0)->setData(x, y);
		plotWin->ui.plotWidget->replot();

		plotWin->setModal(false);
		plotWin->show();

		if(!plotWin->ui.plotWidget->savePdf(filename,true, 1000, 1000))
		{
			Show_Errors("Save Stereonet as pdf failed.");
		}

		command_parser();
	}
	else
	{
		Show_Errors("You haven't saved any clusters yet.");
	}
}

void structrock::Testing()
{
    // Function: transform downloaded original ASCII text point cloud file to XYZ file.
    /*QString filename = QFileDialog::getOpenFileName(NULL,tr("Open Data File"),QDir::currentPath(),tr("Data (*.txt);;All files (*.*)"));
	if(!filename.isNull())
	{
		testworker.setWorkFlowMode(false);
        connect(&testworker, SIGNAL(showErrors(QString)), this, SLOT(Show_Errors(QString)));
        connect(&testworker, SIGNAL(showReadyStatus()), this, SLOT(ShowReady()));
        
        testworker.testing(filename);
	}*/
    
    //Function: connect to postgreSQL database.
    bool isOK;
    QString conn_string = QInputDialog::getText(NULL, "Input Dialog", "Connection Strings:", QLineEdit::Normal, "keyword = value", &isOK);
    if(isOK)
    {
        const char *conninfo;
        PGconn     *conn;
        PGresult   *res;
        int         nFields;
        int         i, j;
        
        conninfo = conn_string.toStdString().c_str();

        /* Make a connection to the database */
        conn = PQconnectdb(conninfo);

        /* Check to see that the backend connection was successfully made */
        if (PQstatus(conn) != CONNECTION_OK)
        {
            std::string Error_message("Connection to database failed: ");
            std::ostringstream oss;
            oss << PQerrorMessage(conn);
            Error_message += oss.str();
            Show_Errors(QString::fromUtf8(Error_message.c_str()));
        }
        else
        {
            /*
            * Our test case here involves using a cursor, for which we must be inside
            * a transaction block.  We could do the whole thing with a single
            * PQexec() of "select * from pg_database", but that's too trivial to make
            * a good example.
            */

            /* Start a transaction block */
            res = PQexec(conn, "BEGIN");
            if (PQresultStatus(res) != PGRES_COMMAND_OK)
            {
                std::string Error_message("BEGIN command failed: ");
                std::ostringstream oss;
                oss << PQerrorMessage(conn);
                Error_message += oss.str();
                Show_Errors(QString::fromUtf8(Error_message.c_str()));
                PQclear(res);
            }

            /* end the transaction */
            res = PQexec(conn, "END");
            PQclear(res);

            /* close the connection to the database and cleanup */
            PQfinish(conn);
        }
    }
}

void structrock::TestResult(int i)
{
	
}

void structrock::ShowReady()
{
	QPalette pa;
	pa.setColor(QPalette::WindowText,Qt::black);
	ui.label->setText("Ready");
	ui.label->setPalette(pa);
}

void structrock::ShowStatus(int i)
{
	QPalette pa;
	pa.setColor(QPalette::WindowText,Qt::blue);
	string head(""), tail("");
	switch(i)
	{
	case 0:
		{
			tail = "";
			break;
		}
	case 1:
		{
			tail = ".";
			break;
		}
	case 2:
		{
			tail = "..";
			break;
		}
	case 3:
		{
			tail = "...";
			break;
		}
	default:{break;}
	}
	switch(dataLibrary::Status)
	{
	case STATUS_OPENPCD:
		{
			head="Busy	opening point cloud data";
			head+=tail;
			ui.label->setText(QString::fromStdString(head));
			ui.label->setPalette(pa);
			break;
		}
	case STATUS_RESAMPLE:
		{
			head="Busy	resampling";
			head+=tail;
			ui.label->setText(QString::fromStdString(head));
			ui.label->setPalette(pa);
			break;
		}
	case STATUS_DOWNSAMPLE:
		{
			head="Busy	downsampling";
			head+=tail;
			ui.label->setText(QString::fromStdString(head));
			ui.label->setPalette(pa);
			break;
		}
	case STATUS_KNNORMAL:
		{
			head="Busy	extracting normals via k-neighbors";
			head+=tail;
			ui.label->setText(QString::fromStdString(head));
			ui.label->setPalette(pa);
			break;
		}
	case STATUS_RANORMAL:
		{
			head="Busy	extracting normals via radius";
			head+=tail;
			ui.label->setText(QString::fromStdString(head));
			ui.label->setPalette(pa);
			break;
		}
	case STATUS_STATICRO:
		{
			head="Busy	removing outlier via static";
			head+=tail;
			ui.label->setText(QString::fromStdString(head));
			ui.label->setPalette(pa);
			break;
		}
	case STATUS_RGS:
		{
			head="Busy	region growing segmentation";
			head+=tail;
			ui.label->setText(QString::fromStdString(head));
			ui.label->setPalette(pa);
			break;
		}
	case STATUS_SAVECLUSTERS:
		{
			head="Busy	saving clusters";
			head+=tail;
			ui.label->setText(QString::fromStdString(head));
			ui.label->setPalette(pa);
			break;
		}
	case STATUS_OPENXYZ:
		{
			head="Busy	opening xyz format data";
			head+=tail;
			ui.label->setText(QString::fromStdString(head));
			ui.label->setPalette(pa);
			break;
		}
	case STATUS_SAVEASCII:
		{
			head="Busy	saving current point cloud as pcd ASCII format";
			head+=tail;
			ui.label->setText(QString::fromStdString(head));
			ui.label->setPalette(pa);
			break;
		}
	case STATUS_SAVEBINARY:
		{
			head="Busy	saving current point cloud as pcd binary format";
			head+=tail;
			ui.label->setText(QString::fromStdString(head));
			ui.label->setPalette(pa);
			break;
		}
	case STATUS_SAVENORMALS:
		{
			head="Busy	saving point cloud normals";
			head+=tail;
			ui.label->setText(QString::fromStdString(head));
			ui.label->setPalette(pa);
			break;
		}
	case STATUS_SHOWPROCESS:
		{
			head="Busy	preparing to show process";
			head+=tail;
			ui.label->setText(QString::fromStdString(head));
			ui.label->setPalette(pa);
			break;
		}
    case STATUS_TESTING:
		{
			head="Busy	testing";
			head+=tail;
			ui.label->setText(QString::fromStdString(head));
			ui.label->setPalette(pa);
			break;
		}
	default:{break;}
	}
}
