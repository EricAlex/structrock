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

//#include <unistd.h>
#include <vector>
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <algorithm>
#include <time.h>
#include <math.h>
//#include <libpq-fe.h>
#include <pcl/common/common_headers.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
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
#include "ShowSFeatureWorker.h"
#include "TestWorker.h"
#include "globaldef.h"
#include "dataLibrary.h"
#include "structrock.h"
#include "geo_region_growing.h"
#include "plotwindow.h"
#include "TimingShutdown.h"

using namespace std;

structrock::structrock(QWidget *parent, Qt::WindowFlags flags)
	: QMainWindow(parent, flags),
	viewer(new pcl::visualization::PCLVisualizer("3D Viewer")),
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

void structrock::NewWindow_current_command()
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
void structrock::NewWindow_next_command()
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

void structrock::Reboot_with_commands(std::string commands)
{
	QProcess *myProcess = new QProcess;
	QStringList commandsList;
	commandsList << "-c"<<commands.c_str();
	myProcess->setWorkingDirectory(QApplication::applicationDirPath());
	#if !defined(_WIN32)&&(defined(__unix__)||defined(__unix)||(defined(__APPLE__)&&defined(__MACH__)))
		myProcess->start("./structrock",commandsList);
	#elif defined(_WIN32)||defined(_WIN64)
		myProcess->start("structrock",commandsList);
	#endif

	TimingShutdown *shutdown(new TimingShutdown);
	connect(shutdown, SIGNAL(shutdown()), this, SLOT(exit()));
	shutdown->start();
}

void structrock::open()
{
    QString filename = QFileDialog::getOpenFileName(NULL,tr("Open Point Cloud Data"),QDir::currentPath(),tr("Point Cloud Data (*.pcd);;All files (*.*)"));
    
    if(!filename.isNull())
    {
		readfileworker.setFileName(filename);
		readfileworker.setWorkFlowMode(false);
		readfileworker.setUnmute();
		readfileworker.setWriteLog();
		QObject::connect(&readfileworker, SIGNAL(ReadFileReady(int)), this, SLOT(ShowPCD(int)));
		connect(&readfileworker, SIGNAL(showErrors(QString)), this, SLOT(Show_Errors(QString)));
		QObject::connect(&readfileworker, SIGNAL(showReadyStatus()), this, SLOT(ShowReady()));

		readfileworker.readFile();
    }
}

void structrock::OpenXYZ()
{
	QString filename = QFileDialog::getOpenFileName(NULL,tr("Open XYZ Data"),QDir::currentPath(),tr("XYZ Data (*.txt);;All files (*.*)"));
    
    if(!filename.isNull())
    {
		readxyzworker.setFileName(filename);
		readxyzworker.setWorkFlowMode(false);
		readxyzworker.setUnmute();
		readxyzworker.setWriteLog();
        connect(&readxyzworker, SIGNAL(ReadXYZReady(int)), this, SLOT(ShowPCD(int)));
		connect(&readxyzworker, SIGNAL(showErrors(QString)), this, SLOT(Show_Errors(QString)));
        connect(&readxyzworker, SIGNAL(showReadyStatus()), this, SLOT(ShowReady()));
        
        readxyzworker.readXYZ();
	}
}

void structrock::OpenWorkFlow()
{
	QString filename = QFileDialog::getOpenFileName(NULL,tr("Open Work Flow File"),QDir::currentPath(),tr("All files (*.*)"));
    
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

				for(int i=0; i<tokens.size(); i++)
				{
					tokens[i].erase(std::remove(tokens[i].begin(), tokens[i].end(),'\n'), tokens[i].end());
					tokens[i].erase(std::remove(tokens[i].begin(), tokens[i].end(),' '), tokens[i].end());
					tokens[i].erase(std::remove(tokens[i].begin(), tokens[i].end(),'\r'), tokens[i].end());
					tokens[i].erase(std::remove(tokens[i].begin(), tokens[i].end(),'\t'), tokens[i].end());
					if(tokens[i].empty())
						tokens.erase(tokens.begin()+i);
				}

				if(tokens.size()>0)
				{
					if(!boost::starts_with(tokens[0],"#"))
					{
						WorkLine work;
						work.command = tokens[0];
						for(int i=1; i<tokens.size(); i++)
						{
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

				for(int i=0; i<tokens.size(); i++)
				{
					tokens[i].erase(std::remove(tokens[i].begin(), tokens[i].end(),'\n'), tokens[i].end());
					tokens[i].erase(std::remove(tokens[i].begin(), tokens[i].end(),' '), tokens[i].end());
					tokens[i].erase(std::remove(tokens[i].begin(), tokens[i].end(),'\r'), tokens[i].end());
					tokens[i].erase(std::remove(tokens[i].begin(), tokens[i].end(),'\t'), tokens[i].end());
					if(tokens[i].empty())
						tokens.erase(tokens.begin()+i);
				}

				if(tokens.size()>0)
				{
					if(!boost::starts_with(tokens[0],"#"))
					{
						WorkLine work;
						work.command = tokens[0];
						for(int i=1; i<tokens.size(); i++)
						{
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

		for(int i=0; i<tokens.size(); i++)
		{
			tokens[i].erase(std::remove(tokens[i].begin(), tokens[i].end(),'\n'), tokens[i].end());
			tokens[i].erase(std::remove(tokens[i].begin(), tokens[i].end(),' '), tokens[i].end());
			tokens[i].erase(std::remove(tokens[i].begin(), tokens[i].end(),'\r'), tokens[i].end());
			tokens[i].erase(std::remove(tokens[i].begin(), tokens[i].end(),'\t'), tokens[i].end());
			if(tokens[i].empty())
				tokens.erase(tokens.begin()+i);
		}

		if(tokens.size()>0)
		{
			if(!boost::starts_with(tokens[0],"#"))
			{
				WorkLine work;
				work.command = tokens[0];
				for(int i=1; i<tokens.size(); i++)
				{
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
		QString error_msg("");
		if(command_string == "delete")
		{
			if(dataLibrary::Workflow[dataLibrary::current_workline_index].parameters.size()>0)
			{
				std::string file_path = dataLibrary::Workflow[dataLibrary::current_workline_index].parameters[0];
				#if defined(_WIN32)||defined(_WIN64)
					size_t f = file_path.find("\\");
					file_path.replace(f, std::string("\\").length(), "\\\\");
				#endif
				#if !defined(_WIN32)&&(defined(__unix__)||defined(__unix)||(defined(__APPLE__)&&defined(__MACH__)))
					unlink(file_path.c_str());
				#elif defined(_WIN32)||defined(_WIN64)
					DeleteFile(file_path.c_str());
				#endif

				dataLibrary::current_workline_index+=1;
				command_parser();
			}
			else
			{
				Show_Errors(QString("delete: Path not provided."));
			}
		}
		else if(command_string == "downsample")
		{
			if(downsampleworker.is_para_satisfying(error_msg))
			{
				downsampleworker.setWorkFlowMode(true);
				downsampleworker.prepare();
				connect(&downsampleworker, SIGNAL(show()), this, SLOT(ShowDownsample()));
				connect(&downsampleworker, SIGNAL(showReadyStatus()), this, SLOT(ShowReady()));
				connect(&downsampleworker, SIGNAL(showErrors(QString)), this, SLOT(Show_Errors(QString)));
				connect(&downsampleworker, SIGNAL(GoWorkFlow()), this, SLOT(command_parser()));
				
				downsampleworker.downsample();
			}
			else
			{
				Show_Errors(error_msg);
			}
		}
		else if(command_string == "foreach")
		{
			if(dataLibrary::Workflow[dataLibrary::current_workline_index].parameters.size()>1)
			{
				std::string loop_var = dataLibrary::Workflow[dataLibrary::current_workline_index].parameters[0];
				std::string key_word = "$"+loop_var+"$";
				std::istringstream line_stream(dataLibrary::Workflow[dataLibrary::current_workline_index].parameters[1]);
				char command_split_str = ':';
				std::vector<std::string> tokens;
				for(std::string each; std::getline(line_stream, each, command_split_str); tokens.push_back(each));

				for(int i=0; i<tokens.size(); i++)
				{
					tokens[i].erase(std::remove(tokens[i].begin(), tokens[i].end(),'\n'), tokens[i].end());
					tokens[i].erase(std::remove(tokens[i].begin(), tokens[i].end(),'\r'), tokens[i].end());
					tokens[i].erase(std::remove(tokens[i].begin(), tokens[i].end(),'\t'), tokens[i].end());
					if(tokens[i].empty())
						tokens.erase(tokens.begin()+i);
				}

				if(tokens.size()>0&&(dataLibrary::current_workline_index+1<dataLibrary::Workflow.size()))
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

					std::string commands_all = "";
					for(int i=0; i<tokens.size(); i++)
					{
						std::string temp_commands = commands;
						std::string::size_type n = 0;
						while( (n = temp_commands.find(key_word, n) ) != std::string::npos )
						{
							temp_commands.replace( n, key_word.size(), tokens[i]);
							n += tokens[i].size();
						}

						commands_all += temp_commands;
					}

					#if defined(_WIN32)||defined(_WIN64)
						size_t f = commands_all.find("\\");
						commands_all.replace(f, std::string("\\").length(), "\\\\");
					#endif

					this->Reboot_with_commands(commands_all);
				}
				else
				{
					Show_Errors(QString("foreach: None Loop Element provided and/or No Operations specified."));
				}
			}
			else
			{
				Show_Errors(QString("foreach: Loop Variable and/or Loop Array not provided."));
			}
		}
		else if(command_string == "ftriangulation")
		{
			if(triangulationworker.is_para_satisfying(error_msg))
			{
				triangulationworker.setWorkFlowMode(true);
				triangulationworker.prepare();
				connect(&triangulationworker, SIGNAL(show()), this, SLOT(ShowTriangulation()));
				connect(&triangulationworker, SIGNAL(showReadyStatus()), this, SLOT(ShowReady()));
				connect(&triangulationworker, SIGNAL(showErrors(QString)), this, SLOT(Show_Errors(QString)));
				connect(&triangulationworker, SIGNAL(GoWorkFlow()), this, SLOT(command_parser()));

				triangulationworker.triangulation();
			}
			else
			{
				Show_Errors(error_msg);
			}
		}
		else if(command_string == "knnormal")
		{
			if(knnormalworker.is_para_satisfying(error_msg))
			{
				knnormalworker.setWorkFlowMode(true);
				knnormalworker.prepare();
				connect(&knnormalworker, SIGNAL(show(bool)), this, SLOT(ShowNormal(bool)));
				connect(&knnormalworker, SIGNAL(showReadyStatus()), this, SLOT(ShowReady()));
				connect(&knnormalworker, SIGNAL(showErrors(QString)), this, SLOT(Show_Errors(QString)));
				connect(&knnormalworker, SIGNAL(GoWorkFlow()), this, SLOT(command_parser()));

				knnormalworker.knnormal();
			}
			else
			{
				Show_Errors(error_msg);
			}
		}
		else if(command_string == "lagrangetensor")
		{
			if(lagrangeTensorWorker.is_para_satisfying(error_msg))
			{
				lagrangeTensorWorker.setWorkFlowMode(true);
				lagrangeTensorWorker.prepare();
				connect(&lagrangeTensorWorker, SIGNAL(showReadyStatus()), this, SLOT(ShowReady()));
				connect(&lagrangeTensorWorker, SIGNAL(showErrors(QString)), this, SLOT(Show_Errors(QString)));
				connect(&lagrangeTensorWorker, SIGNAL(GoWorkFlow()), this, SLOT(command_parser()));

				lagrangeTensorWorker.LagrangeTensor();
			}
			else
			{
				Show_Errors(error_msg);
			}
		}
		else if(command_string == "movefile")
		{
			if(dataLibrary::Workflow[dataLibrary::current_workline_index].parameters.size()>1)
			{
				std::string existing_file_path = dataLibrary::Workflow[dataLibrary::current_workline_index].parameters[0];
				std::string new_file_path = dataLibrary::Workflow[dataLibrary::current_workline_index].parameters[1];
				#if defined(_WIN32)||defined(_WIN64)
					size_t f = existing_file_path.find("\\");
					existing_file_path.replace(f, std::string("\\").length(), "\\\\");
					size_t t = new_file_path.find("\\");
					new_file_path.replace(t, std::string("\\").length(), "\\\\");
				#endif
				#if !defined(_WIN32)&&(defined(__unix__)||defined(__unix)||(defined(__APPLE__)&&defined(__MACH__)))
					rename(existing_file_path.c_str(), new_file_path.c_str());
				#elif defined(_WIN32)||defined(_WIN64)
					MoveFile(existing_file_path.c_str(), new_file_path.c_str());
				#endif

				dataLibrary::current_workline_index+=1;
				command_parser();
			}
			else
			{
				Show_Errors(QString("movefile: Existing and/or New File Path not provided."));
			}
		}
		else if(command_string == "multistation")
		{
			if(multiStationworker.is_para_satisfying(error_msg))
			{
				if(!dataLibrary::have_called_read_file)
				{
					multiStationworker.setWorkFlowMode(true);
					multiStationworker.prepare();
					connect(&multiStationworker, SIGNAL(show(int)), this, SLOT(ShowPCD(int)));
					connect(&multiStationworker, SIGNAL(showReadyStatus()), this, SLOT(ShowReady()));
					connect(&multiStationworker, SIGNAL(showErrors(QString)), this, SLOT(Show_Errors(QString)));
					connect(&multiStationworker, SIGNAL(GoWorkFlow()), this, SLOT(command_parser()));

					multiStationworker.multiStation();
					dataLibrary::have_called_read_file = true;
				}
				else
				{
					this->NewWindow_current_command();
				}
			}
			else
			{
				Show_Errors(error_msg);
			}
		}
		else if(command_string == "openclusters")
		{
			if(openclustersworker.is_para_satisfying(error_msg))
			{
				openclustersworker.setWorkFlowMode(true);
				openclustersworker.prepare();
				connect(&openclustersworker, SIGNAL(show()), this, SLOT(ShowClusters()));
				connect(&openclustersworker, SIGNAL(showReadyStatus()), this, SLOT(ShowReady()));
				connect(&openclustersworker, SIGNAL(showErrors(QString)), this, SLOT(Show_Errors(QString)));
				connect(&openclustersworker, SIGNAL(GoWorkFlow()), this, SLOT(command_parser()));

				openclustersworker.openclusters();
			}
			else
			{
				Show_Errors(error_msg);
			}
		}
		else if(command_string == "openftriangulation")
		{
			if(readPolygonMeshworker.is_para_satisfying(error_msg))
			{
				readPolygonMeshworker.setWorkFlowMode(true);
				readPolygonMeshworker.prepare();
				connect(&readPolygonMeshworker, SIGNAL(show()), this, SLOT(ShowTriangulation()));
				connect(&readPolygonMeshworker, SIGNAL(showErrors(QString)), this, SLOT(Show_Errors(QString)));
				connect(&readPolygonMeshworker, SIGNAL(showReadyStatus()), this, SLOT(ShowReady()));
				connect(&readPolygonMeshworker, SIGNAL(GoWorkFlow()), this, SLOT(command_parser()));

				readPolygonMeshworker.readpolygonmesh();
			}
			else
			{
				Show_Errors(error_msg);
			}
		}
		else if(command_string == "openpcd")
		{
			if(readfileworker.is_para_satisfying(error_msg))
			{
				if(!dataLibrary::have_called_read_file)
				{
					readfileworker.setWorkFlowMode(true);
					readfileworker.prepare();
					QObject::connect(&readfileworker, SIGNAL(ReadFileReady(int)), this, SLOT(ShowPCD(int)));
					QObject::connect(&readfileworker, SIGNAL(showReadyStatus()), this, SLOT(ShowReady()));
					QObject::connect(&readfileworker, SIGNAL(showErrors(QString)), this, SLOT(Show_Errors(QString)));
					QObject::connect(&readfileworker, SIGNAL(GoWorkFlow()), this, SLOT(command_parser()));

					readfileworker.readFile();
					dataLibrary::have_called_read_file = true;
				}
				else
				{
					this->NewWindow_current_command();
				}
			}
			else
			{
				Show_Errors(error_msg);
			}
		}
		else if(command_string == "openxyz")
		{
			if(readxyzworker.is_para_satisfying(error_msg))
			{
				if(!dataLibrary::have_called_read_file)
				{
					readxyzworker.setWorkFlowMode(true);
					readxyzworker.prepare();
					connect(&readxyzworker, SIGNAL(ReadXYZReady(int)), this, SLOT(ShowPCD(int)));
					connect(&readxyzworker, SIGNAL(showReadyStatus()), this, SLOT(ShowReady()));
					connect(&readxyzworker, SIGNAL(showErrors(QString)), this, SLOT(Show_Errors(QString)));
					connect(&readxyzworker, SIGNAL(GoWorkFlow()), this, SLOT(command_parser()));

					readxyzworker.readXYZ();
					dataLibrary::have_called_read_file = true;
				}
				else
				{
					this->NewWindow_current_command();
				}
			}
			else
			{
				Show_Errors(error_msg);
			}
		}
		else if(command_string == "quitsession")
		{
			if(dataLibrary::current_workline_index+1<dataLibrary::Workflow.size())
			{
				this->NewWindow_next_command();
			}
			TimingShutdown *shutdown(new TimingShutdown);
			connect(shutdown, SIGNAL(shutdown()), this, SLOT(exit()));
			shutdown->start();
		}
		else if(command_string == "ranormal")
		{
			if(ranormalworker.is_para_satisfying(error_msg))
			{
				ranormalworker.setWorkFlowMode(true);
				ranormalworker.prepare();
				connect(&ranormalworker, SIGNAL(show(bool)), this, SLOT(ShowNormal(bool)));
				connect(&ranormalworker, SIGNAL(showReadyStatus()), this, SLOT(ShowReady()));
				connect(&ranormalworker, SIGNAL(showErrors(QString)), this, SLOT(Show_Errors(QString)));
				connect(&ranormalworker, SIGNAL(GoWorkFlow()), this, SLOT(command_parser()));
				
				ranormalworker.ranormal();
			}
			else
			{
				Show_Errors(error_msg);
			}
		}
		else if(command_string == "readnshowshearfeatures")
		{
			if(readnShowClassesworker.is_para_satisfying(error_msg))
			{
				readnShowClassesworker.setWorkFlowMode(true);
				readnShowClassesworker.prepare();
				connect(&readnShowClassesworker, SIGNAL(show()), this, SLOT(ShowFractureShearFeatures()));
				connect(&readnShowClassesworker, SIGNAL(showErrors(QString)), this, SLOT(Show_Errors(QString)));
				connect(&readnShowClassesworker, SIGNAL(showReadyStatus()), this, SLOT(ShowReady()));
				connect(&readnShowClassesworker, SIGNAL(GoWorkFlow()), this, SLOT(command_parser()));

				readnShowClassesworker.readnshowfeatures();
			}
			else
			{
				Show_Errors(error_msg);
			}
		}
		else if(command_string == "resample")
		{
			if(resampleworker.is_para_satisfying(error_msg))
			{
				resampleworker.setWorkFlowMode(true);
				resampleworker.prepare();
				connect(&resampleworker, SIGNAL(show()), this, SLOT(ShowResample()));
				connect(&resampleworker, SIGNAL(showReadyStatus()), this, SLOT(ShowReady()));
				connect(&resampleworker, SIGNAL(showErrors(QString)), this, SLOT(Show_Errors(QString)));
				connect(&resampleworker, SIGNAL(GoWorkFlow()), this, SLOT(command_parser()));
            
				resampleworker.resample();
			}
			else
			{
				Show_Errors(error_msg);
			}
		}
		else if(command_string == "rgsegmentation")
		{
			if(rgsworker.is_para_satisfying(error_msg))
    		{
				rgsworker.setWorkFlowMode(true);
				rgsworker.prepare();
				connect(&rgsworker, SIGNAL(show()), this, SLOT(ShowRGS()));
				connect(&rgsworker, SIGNAL(showReadyStatus()), this, SLOT(ShowReady()));
				connect(&rgsworker, SIGNAL(showErrors(QString)), this, SLOT(Show_Errors(QString)));
				connect(&rgsworker, SIGNAL(GoWorkFlow()), this, SLOT(command_parser()));

				rgsworker.rgs();
			}
    		else
    		{
				Show_Errors(error_msg);
    		}
		}
		else if(command_string == "rostatic")
		{
			if(staticroworker.is_para_satisfying(error_msg))
			{
				staticroworker.setWorkFlowMode(true);
				staticroworker.prepare();
				connect(&staticroworker, SIGNAL(show()), this, SLOT(ShowSRO()));
				connect(&staticroworker, SIGNAL(showReadyStatus()), this, SLOT(ShowReady()));
				connect(&staticroworker, SIGNAL(showErrors(QString)), this, SLOT(Show_Errors(QString)));
				connect(&staticroworker, SIGNAL(GoWorkFlow()), this, SLOT(command_parser()));
            
				staticroworker.rostatic();
			}
			else
			{
				Show_Errors(error_msg);
			}
		}
		else if(command_string == "saveclusters")
		{
			if(saveclustersworker.is_para_satisfying(error_msg))
			{
				saveclustersworker.setWorkFlowMode(true);
				saveclustersworker.prepare();
				connect(&saveclustersworker, SIGNAL(showReadyStatus()), this, SLOT(ShowReady()));
				connect(&saveclustersworker, SIGNAL(showErrors(QString)), this, SLOT(Show_Errors(QString)));
				connect(&saveclustersworker, SIGNAL(GoWorkFlow()), this, SLOT(command_parser()));

				saveclustersworker.saveclusters();
			}
			else
			{
				Show_Errors(error_msg);
			}
		}
		else if(command_string == "saveftriangulation")
		{
			if(savePolygonMeshworker.is_para_satisfying(error_msg))
			{
				savePolygonMeshworker.setWorkFlowMode(true);
				savePolygonMeshworker.prepare();
				connect(&savePolygonMeshworker, SIGNAL(showErrors(QString)), this, SLOT(Show_Errors(QString)));
				connect(&savePolygonMeshworker, SIGNAL(showReadyStatus()), this, SLOT(ShowReady()));
				connect(&savePolygonMeshworker, SIGNAL(GoWorkFlow()), this, SLOT(command_parser()));

				savePolygonMeshworker.savepolygonmesh();
			}
			else
			{
				Show_Errors(error_msg);
			}
		}
		else if(command_string == "savemesh")
		{
			if(saveMeshworker.is_para_satisfying(error_msg))
			{
				saveMeshworker.setWorkFlowMode(true);
				saveMeshworker.prepare();
				connect(&saveMeshworker, SIGNAL(showErrors(QString)), this, SLOT(Show_Errors(QString)));
				connect(&saveMeshworker, SIGNAL(showReadyStatus()), this, SLOT(ShowReady()));
				connect(&saveMeshworker, SIGNAL(GoWorkFlow()), this, SLOT(command_parser()));

				saveMeshworker.savemesh();
			}
			else
			{
				Show_Errors(error_msg);
			}
		}
		else if(command_string == "savenormals")
		{
			if(savenormalsworker.is_para_satisfying(error_msg))
			{
				savenormalsworker.setWorkFlowMode(true);
				savenormalsworker.prepare();
				connect(&savenormalsworker, SIGNAL(showErrors(QString)), this, SLOT(Show_Errors(QString)));
				connect(&savenormalsworker, SIGNAL(showReadyStatus()), this, SLOT(ShowReady()));
				connect(&savenormalsworker, SIGNAL(GoWorkFlow()), this, SLOT(command_parser()));

				savenormalsworker.savenormals();
			}
			else
			{
				Show_Errors(error_msg);
			}
		}
		else if(command_string == "savepcdascii")
		{
			if(savepcdASCIIworker.is_para_satisfying(error_msg))
			{
				savepcdASCIIworker.setWorkFlowMode(true);
				savepcdASCIIworker.prepare();
				connect(&savepcdASCIIworker, SIGNAL(showErrors(QString)), this, SLOT(Show_Errors(QString)));
				connect(&savepcdASCIIworker, SIGNAL(showReadyStatus()), this, SLOT(ShowReady()));
				connect(&savepcdASCIIworker, SIGNAL(GoWorkFlow()), this, SLOT(command_parser()));

				savepcdASCIIworker.saveascii();
			}
			else
			{
				Show_Errors(error_msg);
			}
		}
		else if(command_string == "savepcdbinary")
		{
			if(savepcdBinaryworker.is_para_satisfying(error_msg))
			{
				savepcdBinaryworker.setWorkFlowMode(true);
				savepcdBinaryworker.prepare();
				connect(&savepcdBinaryworker, SIGNAL(showErrors(QString)), this, SLOT(Show_Errors(QString)));
				connect(&savepcdBinaryworker, SIGNAL(showReadyStatus()), this, SLOT(ShowReady()));
				connect(&savepcdBinaryworker, SIGNAL(GoWorkFlow()), this, SLOT(command_parser()));

				savepcdBinaryworker.savebinary();
			}
			else
			{
				Show_Errors(error_msg);
			}
		}
		else if(command_string == "savetracemap")
		{
			if(saveTraceMapWorker.is_para_satisfying(error_msg))
			{
				saveTraceMapWorker.setWorkFlowMode(true);
				saveTraceMapWorker.prepare();
				connect(&saveTraceMapWorker, SIGNAL(SaveTraceMapReady(QString)), this, SLOT(Show_SaveTraceMap(QString)));
				connect(&saveTraceMapWorker, SIGNAL(ShowTraceMap()), this, SLOT(ShowSavedClusters()));
				connect(&saveTraceMapWorker, SIGNAL(showErrors(QString)), this, SLOT(Show_Errors(QString)));
				connect(&saveTraceMapWorker, SIGNAL(showReadyStatus()), this, SLOT(ShowReady()));
				connect(&saveTraceMapWorker, SIGNAL(GoWorkFlow()), this, SLOT(command_parser()));

				saveTraceMapWorker.savetracemap();
			}
			else
			{
				Show_Errors(error_msg);
			}
		}
		else if(command_string == "shearpara")
		{
			if(shearparaworker.is_para_satisfying(error_msg))
			{
				shearparaworker.setWorkFlowMode(true);
				shearparaworker.prepare();
				connect(&shearparaworker, SIGNAL(show()), this, SLOT(ShowShearPara()));
				connect(&shearparaworker, SIGNAL(showReadyStatus()), this, SLOT(ShowReady()));
				connect(&shearparaworker, SIGNAL(prepare_2_s_f()), this, SLOT(Prepare_2_s_f()));
				connect(&shearparaworker, SIGNAL(show_f_save_screen(const QString &)), this, SLOT(Show_f_n_SaveScreen(const QString &)));
				connect(&shearparaworker, SIGNAL(showErrors(QString)), this, SLOT(Show_Errors(QString)));
				connect(&shearparaworker, SIGNAL(GoWorkFlow()), this, SLOT(command_parser()));
				
				shearparaworker.shearpara();
			}
			else
			{
				Show_Errors(error_msg);
			}
		}
		else if(command_string == "showprocess")
		{
			if(showprocessworker.is_para_satisfying(error_msg))
			{
				showprocessworker.setWorkFlowMode(true);
				showprocessworker.prepare();
				connect(&showprocessworker, SIGNAL(show(QStringList)), this, SLOT(Show_Process(QStringList)));
				connect(&showprocessworker, SIGNAL(showReadyStatus()), this, SLOT(ShowReady()));
				connect(&showprocessworker, SIGNAL(showErrors(QString)), this, SLOT(Show_Errors(QString)));
				connect(&showprocessworker, SIGNAL(GoWorkFlow()), this, SLOT(command_parser()));

				showprocessworker.showProcess();
			}
			else
			{
				Show_Errors(error_msg);
			}
		}
		else if(command_string == "showsfeature")
		{
			if(showsfeatureworker.is_para_satisfying(error_msg))
			{
				showsfeatureworker.setWorkFlowMode(true);
				showsfeatureworker.prepare();
				connect(&showsfeatureworker, SIGNAL(show()), this, SLOT(Show_SFeature()));
				connect(&showsfeatureworker, SIGNAL(showReadyStatus()), this, SLOT(ShowReady()));
				connect(&showsfeatureworker, SIGNAL(showErrors(QString)), this, SLOT(Show_Errors(QString)));
				connect(&showsfeatureworker, SIGNAL(GoWorkFlow()), this, SLOT(command_parser()));

				showsfeatureworker.showSFeature();
			}
			else
			{
				Show_Errors(error_msg);
			}
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
			if(testworker.is_para_satisfying(error_msg))
			{
				testworker.setWorkFlowMode(true);
                testworker.prepare();
				connect(&testworker, SIGNAL(ReadFileReady(int)), this, SLOT(ShowPCD(int)));
				connect(&testworker, SIGNAL(showErrors(QString)), this, SLOT(Show_Errors(QString)));
				connect(&testworker, SIGNAL(showReadyStatus()), this, SLOT(ShowReady()));
				connect(&testworker, SIGNAL(GoWorkFlow()), this, SLOT(command_parser()));
                
				testworker.testing();
			}
			else
			{
				Show_Errors(error_msg);
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

void structrock::Prepare_2_s_f()
{
	if(!dataLibrary::cloudxyzrgb->empty())
	{
		viewer->removeAllPointClouds(v1);
		viewer->removeAllShapes(v1);
		viewer->addPointCloud(dataLibrary::cloudxyzrgb, dataLibrary::cloudID, v1);
		ui.qvtkWidget->update();
	}
	else if(!dataLibrary::cloudxyz->empty())
	{
		viewer->removeAllPointClouds(v1);
		viewer->removeAllShapes(v1);
		viewer->addPointCloud(dataLibrary::cloudxyz, dataLibrary::cloudID, v1);
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.7, 0.0, dataLibrary::cloudID, v1);
		ui.qvtkWidget->update();
	}
	else
	{
		Show_Errors(QString("shearpara: Please Read Point Cloud Data First (to show each fracture)!"));
	}
}

void structrock::Show_f_n_SaveScreen(const QString &filename)
{
	QByteArray ba = filename.toLocal8Bit();
    std::string* strfilename = new string(ba.data());
	viewer->addPointCloud(dataLibrary::fracture_patches.back(), *strfilename, v1);
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, *strfilename, v1);
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, *strfilename, v1);
	ui.qvtkWidget->update();
	viewer->saveScreenshot(*strfilename);
	// important to save twice here
	viewer->saveScreenshot(*strfilename);
	viewer->removePointCloud(*strfilename,v1);
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
        // Position, Viewpoint, Down
        viewer->setCameraPosition (0,0,0,0,0,-1,0,1,0);
        viewer->resetCamera();
		ui.qvtkWidget->update();
	}
	else if(i==CLOUDXYZRGB)
	{
		viewer->addPointCloud(dataLibrary::cloudxyzrgb, dataLibrary::cloudID, v1);
        // to show the reference coordinate system
		// Eigen::Vector3f centroid = dataLibrary::compute3DCentroid(*dataLibrary::cloudxyz);
		// viewer->addCoordinateSystem(0.3,centroid(0),centroid(1)-0.5,centroid(2)-1,dataLibrary::cloudID + "_reference",v1);

        viewer->resetCameraViewpoint (dataLibrary::cloudID);
        // Position, Viewpoint, Down
        viewer->setCameraPosition (0,0,0,0,0,-1,0,0,1);
        viewer->resetCamera();
        
		ui.qvtkWidget->update();
	}
	else if(i==CLOUDXYZI)
	{
		pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> intensity_distribution(dataLibrary::cloudxyzi, "intensity"); 
		viewer->addPointCloud<pcl::PointXYZI> (dataLibrary::cloudxyzi, intensity_distribution, dataLibrary::cloudID, v1);

        viewer->resetCameraViewpoint (dataLibrary::cloudID);
        // Position, Viewpoint, Down
        viewer->setCameraPosition (0,0,0,0,0,-1,0,1,0);
        viewer->resetCamera();
        
		ui.qvtkWidget->update();
	}
}

void structrock::saveasascii()
{
	QString filename = QFileDialog::getSaveFileName(this,tr("Save PCD data as ASCII"),QDir::currentPath(),tr("Point Cloud Data (*.pcd)"));
	if(!filename.isNull())
	{
		savepcdASCIIworker.setFileName(filename);
		savepcdASCIIworker.setWorkFlowMode(false);
		savepcdASCIIworker.setSaveRGBMode(false);
		connect(&savepcdASCIIworker, SIGNAL(showErrors(QString)), this, SLOT(Show_Errors(QString)));
		connect(&savepcdASCIIworker, SIGNAL(showReadyStatus()), this, SLOT(ShowReady()));

		savepcdASCIIworker.saveascii();
	}
}

void structrock::saveasbinary()
{
	QString filename = QFileDialog::getSaveFileName(this,tr("Save PCD data as Binary"),QDir::currentPath(),tr("Point Cloud Data (*.pcd)"));
	if(!filename.isNull())
	{
		savepcdBinaryworker.setFileName(filename);
		savepcdBinaryworker.setWorkFlowMode(false);
		savepcdBinaryworker.setSaveRGBMode(false);
		connect(&savepcdBinaryworker, SIGNAL(showErrors(QString)), this, SLOT(Show_Errors(QString)));
		connect(&savepcdBinaryworker, SIGNAL(showReadyStatus()), this, SLOT(ShowReady()));

		savepcdBinaryworker.savebinary();
	}
}

void structrock::exit()
{
	cout << '\a';
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
			resampleworker.setRadius(radius);
			resampleworker.setWorkFlowMode(false);
			resampleworker.setUnmute();
			resampleworker.setWriteLog();
            connect(&resampleworker, SIGNAL(show()), this, SLOT(ShowResample()));
			connect(&resampleworker, SIGNAL(showErrors(QString)), this, SLOT(Show_Errors(QString)));
            connect(&resampleworker, SIGNAL(showReadyStatus()), this, SLOT(ShowReady()));
            
            resampleworker.resample();
        }
    }
    else
    {
		Show_Errors(QString("resample: You Do Not Have Any Point Cloud Data in Memery!"));
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
}

void structrock::downsampling()
{
    if(dataLibrary::haveBaseData())
    {
        bool ok = false;
        double leaf = QInputDialog::getDouble(NULL, "Minimum Point Distance", "Please Set The Minimum Point Distance (Meter)", 0.01, 0, 100, 3, &ok);
        if(ok)
        {
			downsampleworker.setLeaf(leaf);
			downsampleworker.setWorkFlowMode(false);
			downsampleworker.setUnmute();
			downsampleworker.setWriteLog();
            connect(&downsampleworker, SIGNAL(show()), this, SLOT(ShowDownsample()));
			connect(&downsampleworker, SIGNAL(showErrors(QString)), this, SLOT(Show_Errors(QString)));
            connect(&downsampleworker, SIGNAL(showReadyStatus()), this, SLOT(ShowReady()));
            
            downsampleworker.downsample();
        }
    }
    else
    {
		Show_Errors(QString("downsample: You Do Not Have Any Point Cloud Data in Memery!"));
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
}

void structrock::k_neighbor()
{
    if(dataLibrary::haveBaseData())
    {
        bool ok = false;
        int k = QInputDialog::getInt(NULL, "Number of Neighbor Points", "Please Set The Number of Neighbor Points", 20, 0, 200, 1, &ok);
        if(ok)
        {
			knnormalworker.setK(k);
            knnormalworker.setWorkFlowMode(false);
			knnormalworker.setShowCurvature(false);
			knnormalworker.setUnmute();
			knnormalworker.setWriteLog();
			connect(&knnormalworker, SIGNAL(show(bool)), this, SLOT(ShowNormal(bool)));
			connect(&knnormalworker, SIGNAL(showErrors(QString)), this, SLOT(Show_Errors(QString)));
			connect(&knnormalworker, SIGNAL(showReadyStatus()), this, SLOT(ShowReady()));

			knnormalworker.knnormal();
        }
    }
    else
    {
		Show_Errors(QString("knnormal: You Do Not Have Any Point Cloud Data in Memery!"));
    }
}

void structrock::ShowNormal(bool showCurvature)
{
	if(!showCurvature)
	{
		viewer->removeAllPointClouds(v1);
		viewer->addPointCloud(dataLibrary::cloudxyz, dataLibrary::cloudID, v1);
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.7, 0.0, dataLibrary::cloudID, v1);
	}
	else
	{
		viewer->removeAllPointClouds(v1);
		pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointNormal> handler_k(dataLibrary::pointnormals, "curvature");
		viewer->addPointCloud(dataLibrary::pointnormals, handler_k, "curvature", v1);
	}

	viewer->removeAllPointClouds(v2);
	viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(dataLibrary::cloudxyz, dataLibrary::normal, 50, 0.02, "normals", v2);
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.0, 1.0, "normals", v2);
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 2, "normals", v2);
	ui.qvtkWidget->update();
}

void structrock::radius()
{
    if(dataLibrary::haveBaseData())
    {
        bool ok = false;
        double radius = QInputDialog::getDouble(NULL, "Search Radius", "Please Set The Search Radius", 0.01, 0, 100, 3, &ok);
        if(ok)
        {
			ranormalworker.setRadius(radius);
            ranormalworker.setWorkFlowMode(false);
			ranormalworker.setShowCurvature(false);
			ranormalworker.setUnmute();
			ranormalworker.setWriteLog();
			connect(&ranormalworker, SIGNAL(show(bool)), this, SLOT(ShowNormal(bool)));
			connect(&ranormalworker, SIGNAL(showErrors(QString)), this, SLOT(Show_Errors(QString)));
			connect(&ranormalworker, SIGNAL(showReadyStatus()), this, SLOT(ShowReady()));
			
			ranormalworker.ranormal();
        }
    }
    else
    {
		Show_Errors(QString("ranormal: You Do Not Have Any Point Cloud Data in Memery!"));
    }
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
			staticroworker.setStdDev(stdDev);
            staticroworker.setWorkFlowMode(false);
			staticroworker.setUnmute();
			staticroworker.setWriteLog();
			connect(&staticroworker, SIGNAL(show()), this, SLOT(ShowSRO()));
			connect(&staticroworker, SIGNAL(showErrors(QString)), this, SLOT(Show_Errors(QString)));
			connect(&staticroworker, SIGNAL(showReadyStatus()), this, SLOT(ShowReady()));
            
			staticroworker.rostatic();
        }
    }
    else
    {
		Show_Errors(QString("rostatic: You Do Not Have Any Point Cloud Data in Memery!"));
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
		savenormalsworker.setFileName(filename);
		savenormalsworker.setWorkFlowMode(false);
		connect(&savenormalsworker, SIGNAL(showErrors(QString)), this, SLOT(Show_Errors(QString)));
		connect(&savenormalsworker, SIGNAL(showReadyStatus()), this, SLOT(ShowReady()));

		savenormalsworker.savenormals();
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
				RGSpara temp_para;
                temp_para.curvature = multi_input.getCurvatureThreshold();
                temp_para.smoothness = multi_input.getSmoothnessThreshold();
                temp_para.residual = multi_input.getResidualThreshold();
                temp_para.number_of_neighbors = multi_input.getNumberOfNeighbors();
                temp_para.min_number_of_Points = multi_input.getMinNumberOfPoints();
                temp_para.IsSmoothMode = multi_input.IsSmoothMode();
                rgsworker.setRGSpara(temp_para);
                rgsworker.setWorkFlowMode(false);
				rgsworker.setUnmute();
				rgsworker.setWriteLog();
				connect(&rgsworker, SIGNAL(show()), this, SLOT(ShowRGS()));
				connect(&rgsworker, SIGNAL(showErrors(QString)), this, SLOT(Show_Errors(QString)));
				connect(&rgsworker, SIGNAL(showReadyStatus()), this, SLOT(ShowReady()));

				rgsworker.rgs();
            }
        }
    }
    else
    {
		Show_Errors(QString("rgsegmentation: You Do Not Have Any Point Cloud Data in Memery!"));
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

void structrock::ShowTriangulation()
{
	if(dataLibrary::Fracture_Triangles.size()>0)
	{
		viewer->removeAllPointClouds(v2);
		viewer->removeAllShapes(v2);
		for(int i=0; i<dataLibrary::Fracture_Triangles.size(); i++)
		{
			stringstream ss;
			ss << i;
			string patchID = "Mesh_" + ss.str();
			viewer->addPolygonMesh(*dataLibrary::Fracture_Triangles[i], patchID, v2);
		}
		// to add reference coordinate system on the fracture surface for shear parameters
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromPCLPointCloud2(dataLibrary::Fracture_Triangles[0]->cloud, *cloud_ptr);
		Eigen::Vector3f Cloud_center = dataLibrary::compute3DCentroid(*cloud_ptr);
		
		/*float nx, ny, nz;
		float curvature;
		Eigen::Matrix3f convariance_matrix;
		Eigen::Vector4f xyz_centroid;
		xyz_centroid << Cloud_center(0), Cloud_center(1), Cloud_center(2), 1;
		pcl::computeCovarianceMatrix(*cloud_ptr, xyz_centroid, convariance_matrix);
		pcl::solvePlaneParameters(convariance_matrix, nx, ny, nz, curvature);
		Eigen::Vector3f plane_normal;
        plane_normal << nx, ny, nz;*/

		float nx, ny, nz;
        Eigen::Vector4f plane_normal_param = dataLibrary::fitPlaneManually(*cloud_ptr);
        nx = plane_normal_param(0);
        ny = plane_normal_param(1);
        nz = plane_normal_param(2);
        Eigen::Vector3f plane_normal;
        plane_normal << nx, ny, nz;

		std::vector<Eigen::Vector3f> sdirections;
        Eigen::Vector3f original;
        if((nx==0)&&(ny==0))
        {
            original(0)=1.0;
            original(1)=0.0;
            original(2)=0.0;
        }
        else
        {
            original(0)=-plane_normal(1);
            original(1)=plane_normal(0);
            original(2)=0.0;
        }
	    for(float j=-TWOPI/2; j<-0.001; j+=TWOPI/72)
	    {
		    float x = plane_normal(0);
		    float y = plane_normal(1);
		    float z = plane_normal(2);
		    float c = std::cos(j);
		    float s = std::sqrt(1-c*c);
		    float C = 1-c;
		    Eigen::Matrix3f rmat;
		    rmat<<x*x*C+c,x*y*C-z*s,x*z*C+y*s,y*x*C+z*s,y*y*C+c,y*z*C-x*s,z*x*C-y*s,z*y*C+x*s,z*z*C+c;
            Eigen::Vector3f rotated=rmat*original;
            rotated.normalize();
		    sdirections.push_back(rotated);
        }
        int halfsize = sdirections.size();
        for(int j=0; j<halfsize; j++)
        {
            Eigen::Vector3f temp_new;
            temp_new(0) = -sdirections[j](0);
            temp_new(1) = -sdirections[j](1);
            temp_new(2) = -sdirections[j](2);
            sdirections.push_back(temp_new);
        }
		float scale = 0.25;
		plane_normal.normalize();
		Cloud_center = Cloud_center + 0.02*plane_normal;
		plane_normal *= scale;
		DrawLine(Cloud_center,Cloud_center + plane_normal,0,0,1,3,"Normal_dir",v2);
		DrawLine(Cloud_center,Cloud_center + scale * sdirections[0],1,0,0,3,"ZeroD_dir",v2);
		DrawLine(Cloud_center,Cloud_center + scale * sdirections[18],0,1,0,3,"NinetyD_dir",v2);
		/*for(int j=0; j<sdirections.size(); j++){
			stringstream ss;
			ss << j;
			string ShearDirID = "Shear_Dir_" + ss.str();
			float color = (float)j/sdirections.size();
			DrawLine(Cloud_center,Cloud_center + scale * sdirections[j],color,1.0-color,color,3,ShearDirID,v2);
		}*/

		ui.qvtkWidget->update();
	}
}

void structrock::ShowShearPara()
{}

void structrock::ShowFractureShearFeatures()
{
	if(!dataLibrary::cloudxyzrgb->empty()){
		viewer->removeAllPointClouds(v2);
		viewer->removeAllShapes(v2);
		viewer->removeAllPointClouds(v1);
		viewer->removeAllShapes(v1);
		viewer->addPointCloud(dataLibrary::cloudxyzrgb, dataLibrary::cloudID+"2", v2);
		viewer->addPointCloud(dataLibrary::cloudxyzrgb, dataLibrary::cloudID+"1", v1);
		viewer->addText(dataLibrary::info_str, 10, 10, 26, 1.0, 1.0, 1.0, "v1 info text", v1);
		ui.qvtkWidget->update();
	}
	else if(!dataLibrary::cloudxyz->empty()){
		viewer->removeAllPointClouds(v2);
		viewer->removeAllShapes(v2);
		viewer->removeAllPointClouds(v1);
		viewer->removeAllShapes(v1);
		viewer->addPointCloud(dataLibrary::cloudxyz, dataLibrary::cloudID+"2", v2);
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.7, 0.0, dataLibrary::cloudID+"2", v2);
		viewer->addPointCloud(dataLibrary::cloudxyz, dataLibrary::cloudID+"1", v1);
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.7, 0.0, dataLibrary::cloudID+"1", v1);
		viewer->addText(dataLibrary::info_str, 10, 10, 26, 1.0, 1.0, 1.0, "v1 info text", v1);
		ui.qvtkWidget->update();
	}
	for(int i=0; i<dataLibrary::fractures_with_feature.size(); i++){
		std::ostringstream strs;
		strs << i;
		viewer->addPointCloud(dataLibrary::fractures_with_feature[i], strs.str(), v1);
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, strs.str(), v1);
	}
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
		Show_Errors(QString("saveclusters: You Haven't Performed Any Segmentation Yet!"));
	}
	else
	{
        QString filename = QFileDialog::getSaveFileName(this,tr("Save Clusters"),QDir::currentPath(),tr("(*.bin)"));

        if(!filename.isNull())
        {
			saveclustersworker.setFileName(filename);
			saveclustersworker.setWorkFlowMode(false);
			saveclustersworker.setUnmute();
			saveclustersworker.setWriteLog();
            connect(&saveclustersworker, SIGNAL(SaveClustersReady()), this, SLOT(ShowSavedClusters()));
			connect(&saveclustersworker, SIGNAL(showErrors(QString)), this, SLOT(Show_Errors(QString)));
            connect(&saveclustersworker, SIGNAL(showReadyStatus()), this, SLOT(ShowReady()));
            
            saveclustersworker.saveclusters();
        }
	}
}

void structrock::ShowSavedClusters()
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

void structrock::Show_Process(QStringList Qcontents)
{
	std::vector<std::string> contents;
	for(int i=0; i<Qcontents.size(); i++)
	{
		contents.push_back(Qcontents[i].toStdString());
	}
	viewer->removeAllPointClouds(v2);
	viewer->removeAllShapes(v1);
	viewer->removeAllShapes(v2);

	if(dataLibrary::checkContents(contents, "point_cloud"))
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

	if(dataLibrary::checkContents(contents, "suppositional_plane"))
	{
		viewer->addPolygon<pcl::PointXYZ>(dataLibrary::cloud_hull_all, 0, 1, 1, "outcrop_polygon_v1", v1);
		viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE, "outcrop_polygon_v1", v1);
		viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, "outcrop_polygon_v1", v1);

		viewer->addPolygon<pcl::PointXYZ>(dataLibrary::cloud_hull_all, 0, 1, 1, "outcrop_polygon_v2", v2);
		viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE, "outcrop_polygon_v2", v2);
		viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, "outcrop_polygon_v2", v2);
	}

	if(dataLibrary::checkContents(contents, "fracture_faces"))
	{
		bool show_rem = false;
		if(dataLibrary::checkContents(contents, "rgs_remanent"))
		{
			show_rem = true;
		}
		bool show_fracture_traces = false;
		bool show_extension_line = false;
		if(dataLibrary::checkContents(contents, "suppositional_plane")&&dataLibrary::checkContents(contents, "fracture_traces"))
		{
			show_fracture_traces = true;
		}
		if(dataLibrary::checkContents(contents, "suppositional_plane")&&dataLibrary::checkContents(contents, "extension_line"))
		{
			show_extension_line = true;
		}

		for(int cluster_index = 0; cluster_index < dataLibrary::fracture_faces_hull.size(); cluster_index++)
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
		}

		if(show_fracture_traces)
		{
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
			if(show_extension_line)
			{
				if((dataLibrary::checkContents(contents, QString::number(FMAP_RECTANGULAR).toStdString()))||(dataLibrary::checkContents(contents, QString::number(FMAP_CIRCULAR).toStdString())))
				{
					for(int i=0; i<dataLibrary::fracture_faces_expanded.size(); i++)
					{
						stringstream ss;
						ss << i;
						viewer->addPolygon<pcl::PointXYZ>(dataLibrary::fracture_faces_expanded[i], 1.0, 1.0, 1.0, "polygon_expanded_"+ss.str(), v2);
						viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE, "polygon_expanded_"+ss.str(), v2);
						viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, "polygon_expanded_"+ss.str(), v2);
					}

					for(int i=0; i<dataLibrary::fracture_faces_circle_original.size(); i++)
					{
						stringstream ss;
						ss << i;
						viewer->addPolygon<pcl::PointXYZ>(dataLibrary::fracture_faces_circle_original[i], 1.0, 1.0, 1.0, "polygon_circle_original_"+ss.str(), v2);
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
						viewer->addPolygon<pcl::PointXYZ>(dataLibrary::fracture_faces_hull_up[cluster_index], r, g, b, "polygon_replot_up_"+ss.str(), v2);
						viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE, "polygon_replot_up_"+ss.str(), v2);
						viewer->addPolygon<pcl::PointXYZ>(dataLibrary::fracture_faces_hull_down[cluster_index], r, g, b, "polygon_replot_down_"+ss.str(), v2);
						viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE, "polygon_replot_down_"+ss.str(), v2);
					}
				}
			}
		}

		if(show_rem)
		{
			viewer->addPointCloud(dataLibrary::segmentation_rem, dataLibrary::cloudID+"v2", v2);
		}
	}

	ui.qvtkWidget->update();
}

void structrock::Show_SFeature()
{
	if(!dataLibrary::cloudxyzrgb->empty())
	{
		viewer->removeAllPointClouds(v1);
		viewer->addPointCloud(dataLibrary::cloudxyzrgb, dataLibrary::cloudID, v1);
		viewer->removeAllPointClouds(v2);
		viewer->addPointCloud(dataLibrary::cloudxyzrgb, dataLibrary::cloudID+"v2", v2);
	}
	else
	{
		viewer->removeAllPointClouds(v1);
		viewer->addPointCloud(dataLibrary::cloudxyz, dataLibrary::cloudID, v1);
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.7, 0.0, dataLibrary::cloudID, v1);
		viewer->removeAllPointClouds(v2);
		viewer->addPointCloud(dataLibrary::cloudxyz, dataLibrary::cloudID+"v2", v2);
		viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.7, 0.0, dataLibrary::cloudID+"v2", v2);
	}

	viewer->removeAllShapes(v2);
	viewer->addPointCloud(dataLibrary::cloudxyzrgb_features, "feature", v2);
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "feature", v2);

	ui.qvtkWidget->update();
}

void structrock::OpenClusters()
{
	QString filename = QFileDialog::getOpenFileName(this,tr("Open Point Cloud Clusters Data"),QDir::currentPath(),tr("(*.bin)"));
	
	if(!filename.isNull())
    {
		openclustersworker.setFileName(filename);
		openclustersworker.setWorkFlowMode(false);
		openclustersworker.setUnmute();
		openclustersworker.setWriteLog();
        connect(&openclustersworker, SIGNAL(show()), this, SLOT(ShowClusters()));
		connect(&openclustersworker, SIGNAL(showErrors(QString)), this, SLOT(Show_Errors(QString)));
        connect(&openclustersworker, SIGNAL(showReadyStatus()), this, SLOT(ShowReady()));
        
        openclustersworker.openclusters();
	}
}

void structrock::ShowClusters()
{
	viewer->removeAllPointClouds(v2);
	for(int i=0; i<dataLibrary::cluster_patches.size(); i++)
	{
		viewer->addPointCloud(dataLibrary::cluster_patches[i], dataLibrary::patchIDs[i], v2);
	}
	viewer->setCameraPosition(dataLibrary::cloud_centor.x,dataLibrary::cloud_centor.y,dataLibrary::cloud_centor.z,dataLibrary::cloud_centor.x,dataLibrary::cloud_centor.y,dataLibrary::cloud_centor.z);
	viewer->resetCamera();
	ui.qvtkWidget->update();
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
    /*bool isOK;
    QString conn_string = QInputDialog::getText(NULL, "Input Dialog", "Connection Strings:", QLineEdit::Normal, "keyword = value", &isOK);
    if(isOK)
    {
        const char *conninfo;
        PGconn     *conn;
        PGresult   *res;
        int         nFields;
        int         i, j;
        
        conninfo = conn_string.toStdString().c_str();

        // Make a connection to the database
        conn = PQconnectdb(conninfo);

        // Check to see that the backend connection was successfully made
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
            //
            // Our test case here involves using a cursor, for which we must be inside
            // a transaction block.  We could do the whole thing with a single
            // PQexec() of "select * from pg_database", but that's too trivial to make
            // a good example.
            //

            // Start a transaction block
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

			// end the transaction
            res = PQexec(conn, "END");
            PQclear(res);

			// close the connection to the database and cleanup
            PQfinish(conn);
        }
    }*/
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
			head="Busy	removing outlier via statistic";
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
	case STATUS_SHOWSFEATURE:
		{
			head="Busy	preparing to show surface features";
			head+=tail;
			ui.label->setText(QString::fromStdString(head));
			ui.label->setPalette(pa);
			break;
		}
	case STATUS_TRIANGULATION:
		{
			head="Busy	performing fracture surface triangulation";
			head+=tail;
			ui.label->setText(QString::fromStdString(head));
			ui.label->setPalette(pa);
			break;
		}
	case STATUS_OPENCLUSTERS:
		{
			head="Busy	opening fracture point cluster data";
			head+=tail;
			ui.label->setText(QString::fromStdString(head));
			ui.label->setPalette(pa);
			break;
		}
	case STATUS_SHEARPARA:
		{
			head="Busy	estimating fracture shear parameters";
			head+=tail;
			ui.label->setText(QString::fromStdString(head));
			ui.label->setPalette(pa);
			break;
		}
	case STATUS_SAVEPOLYGONMESH:
		{
			head="Busy	saving triangulated polymeshes";
			head+=tail;
			ui.label->setText(QString::fromStdString(head));
			ui.label->setPalette(pa);
			break;
		}
	case STATUS_OPENPOLYGONMESH:
		{
			head="Busy	opening triangulated polymeshes";
			head+=tail;
			ui.label->setText(QString::fromStdString(head));
			ui.label->setPalette(pa);
			break;
		}
	case STATUS_READNSHOWCLASSES:
		{
			head="Busy	opening fracture types data";
			head+=tail;
			ui.label->setText(QString::fromStdString(head));
			ui.label->setPalette(pa);
			break;
		}
	case STATUS_MULTISTATION:
		{
			head="Busy	processing multistation point cloud data";
			head+=tail;
			ui.label->setText(QString::fromStdString(head));
			ui.label->setPalette(pa);
			break;
		}
	case STATUS_SAVEMESH:
		{
			head="Busy	saving meshes of fracture traces map";
			head+=tail;
			ui.label->setText(QString::fromStdString(head));
			ui.label->setPalette(pa);
			break;
		}
	case STATUS_LAGRANGETENSOR:
		{
			head="Busy	calculating Lagrange strain tensor for each fracture";
			head+=tail;
			ui.label->setText(QString::fromStdString(head));
			ui.label->setPalette(pa);
			break;
		}
	case STATUS_SAVETRACEMAP:
		{
			head="Busy	saving trace map";
			head+=tail;
			ui.label->setText(QString::fromStdString(head));
			ui.label->setPalette(pa);
			break;
		}
	default:{break;}
	}
}
