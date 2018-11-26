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

#ifndef STRUCTROCK_H
#define STRUCTROCK_H

#include <QtGui/QMainWindow>
#include <QtGui/QApplication>
#include <QDesktopWidget>
#include <qprocess.h>
#include <qstringlist.h>
#include <qfileinfo.h>
#include <QStringList>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include "ReadFileWorker.h"
#include "checkstatusThread.h"
#include "resampleWorker.h"
#include "../build/ui_structrock.h"
#include "MultiInputDialog.h"
#include "globaldef.h"
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
#include "SavePcdASCIIWorker.h"
#include "SavePcdBinaryWorker.h"
#include "SaveNormalsWorker.h"
#include "ShowProcessWorker.h"
#include "ShowSFeatureWorker.h"
#include "triangulationWorker.h"
#include "openClustersWorker.h"
#include "ShearParaWorker.h"
#include "SavePolygonMeshWorker.h"
#include "ReadPolygonMeshWorker.h"
#include "ReadnShowClassesWorker.h"
#include "MultiStationWorker.h"
#include "SaveMeshWorker.h"
#include "TestWorker.h"
#include "TimingShutdown.h"

class structrock : public QMainWindow
{
	Q_OBJECT

public:
	structrock(QWidget *parent = 0, Qt::WFlags flags = 0);
	~structrock();
	void OpenWorkFlow(QString filename);
	void OpenWorkFlow(std::string commands);

private slots:
	void open();
	void exit();
	void saveasascii();
	void saveasbinary();
	void downsampling();
	void ShowDownsample();
	void resampling();
	void ShowResample();
	void k_neighbor();
	void ShowNormal(bool showCurvature);
	void radius();
	void StaticRemoveOutlier();
	void ShowSRO();
	void ConditionalRemoveOutlier();
	void RadiusRemoveOutlier();
	void SaveNormals();
	void SaveClusters();
    void ShowSavedClusters(QString filename);
	void Show_SaveTraceMap(QString filename);
	void Prepare_2_s_f();
	void Show_f_n_SaveScreen(const QString &filename);
	void RegionGrowingSegmentation();
	void ShowRGS();
	void ShowTriangulation();
	void ShowShearPara();
	void ShowFractureShearFeatures();
	void OpenClusters();
	void ShowClusters();
	void OpenXYZ();
	void OpenWorkFlow();
	void Testing();
	void About();
	void Stereonet();
	void Stereonet(QString filename);
	void TestResult(int i);
	void ShowPCD(int i);
	void ShowReady();
	void ShowStatus(int i);
	void command_parser();
	void Show_Errors(const QString &errors);
	void Show_Process(QStringList Qcontents);
	void Show_SFeature();
	void slotReboot()
	{
		QProcess *myProcess = new QProcess;
		myProcess->setWorkingDirectory(QApplication::applicationDirPath());
		#if !defined(_WIN32)&&(defined(__unix__)||defined(__unix)||(defined(__APPLE__)&&defined(__MACH__)))
			myProcess->start("./structrock");
		#elif defined(_WIN32)||defined(_WIN64)
			myProcess->start("structrock");
		#endif
		
		TimingShutdown *shutdown(new TimingShutdown);
		connect(shutdown, SIGNAL(shutdown()), this, SLOT(exit()));
		shutdown->start();
	}
	void NewWindow()
	{
		QProcess *myProcess = new QProcess;
		myProcess->setWorkingDirectory(QApplication::applicationDirPath());
		#if !defined(_WIN32)&&(defined(__unix__)||defined(__unix)||(defined(__APPLE__)&&defined(__MACH__)))
			myProcess->start("./structrock");
		#elif defined(_WIN32)||defined(_WIN64)
			myProcess->start("structrock");
		#endif
	}

private:
	Ui::structrock_ui ui;
	QAction *openAction;
	QAction *exitAction;
	QAction *SaveAsASCII;
	QAction *SaveAsBinary;
	QAction *DownSampling;
	QAction *normal_k_neighbor;
	QAction *normal_radius;
	QAction *ReSampling;
	QAction *Static_remove_outlier;
	QAction *Conditional_remove_outlier;
	QAction *Radius_remove_outlier;
	QAction *save_normals;
	QAction *save_clusters;
	QAction *region_growing_segmentation;
	QAction *open_clusters;
	QAction *open_xyz;
	QAction *open_workflow;
	QAction *testing;
	QAction *about;
	QAction *stereonet;
	QAction *Reboot;
	QAction *newWindow;

public:
	pcl::visualization::PCLVisualizer *viewer;
	int v1;
	int v2;

private:
	void NewWindow_current_command();
	void NewWindow_next_command();
	void Reboot_with_commands(std::string commands);
	void updatePatch();
	void DrawLine(const Eigen::Vector3f begin, const Eigen::Vector3f end, float r, float g, float b, std::string id, int viewpot);
	void DrawLine(const Eigen::Vector3f begin, const Eigen::Vector3f end, float r, float g, float b, double lineWidth, std::string id, int viewpot);
    
private:
	ReadFileWorker readfileworker;
    resampleWorker resampleworker;
    downsampleWorker downsampleworker;
    knnormalWorker knnormalworker;
    ranormalWorker ranormalworker;
    StaticROWorker staticroworker;
    RGSWorker rgsworker;
    SaveClustersWorker saveclustersworker;
	ReadXYZWorker readxyzworker;
	SavePcdASCIIWorker savepcdASCIIworker;
	SavePcdBinaryWorker savepcdBinaryworker;
	SaveNormalsWorker savenormalsworker;
	ShowProcessWorker showprocessworker;
	ShowSFeatureWorker showsfeatureworker;
	triangulationWorker triangulationworker;
	openClustersWorker openclustersworker;
	ShearParaWorker shearparaworker;
	SavePolygonMeshWorker savePolygonMeshworker;
	ReadPolygonMeshWorker readPolygonMeshworker;
	ReadnShowClassesWorker readnShowClassesworker;
	MultiStationWorker multiStationworker;
	SaveMeshWorker saveMeshworker;
	TestWorker testworker;

public:
	void MoveForwardPatch();
	void MoveBackPatch();
	void saveScreen();
	void CheckPatchesIfSelected(float x, float y, float z);
	void ShowSelectedPatches();
};

#endif // STRUCTROCK_H
