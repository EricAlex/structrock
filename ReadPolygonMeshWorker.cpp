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

#include <string>
#include <sstream>
#include <fstream>
#include <pcl/io/io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/io/vtk_lib_io.h>
#include "ReadPolygonMeshWorker.h"
#include "globaldef.h"
#include "dataLibrary.h"

bool ReadPolygonMeshWorker::is_para_satisfying(QString message)
{
	if(dataLibrary::Fracture_Triangles.size()>0)
	{
		message = QString("openftriangulation: Triangulation Data Already Loaded!");
		return false;
	}
	else
	{
		this->setParaSize(1);
		if(dataLibrary::Workflow[dataLibrary::current_workline_index].parameters.size()>0)
		{
			this->setFileName(QString::fromUtf8(dataLibrary::Workflow[dataLibrary::current_workline_index].parameters[0].c_str()));
			this->setParaIndex(this->getParaSize());
			return true;
		}
		else
		{
			message = QString("openftriangulation: Save Path Not Provided.");
			return false;
		}
	}
}

void ReadPolygonMeshWorker::prepare()
{
	this->setUnmute();
	this->setWriteLog();
	this->check_mute_nolog();
}

void ReadPolygonMeshWorker::doWork()
{
	bool is_success(false);

	QByteArray ba = this->getFileName().toLocal8Bit();
	std::string* strfilename = new std::string(ba.data());

	dataLibrary::Status = STATUS_OPENPOLYGONMESH;

	this->timer_start();

	//begin of processing
	int fracture_count;
	std::string fracture_count_file = *strfilename + "_count.txt";
	ifstream fracture_count_in(fracture_count_file.c_str());
	fracture_count_in>>fracture_count;
	fracture_count_in.close();
	for(int i=0; i<fracture_count; i++)
	{
		pcl::PolygonMesh::Ptr temp_mesh (new pcl::PolygonMesh);
		std::ostringstream strs;
		strs << i;
		std::string vtkfilename = *strfilename + "_" + strs.str() +"_.ply";
		pcl::io::loadPolygonFile(vtkfilename, *temp_mesh);
		dataLibrary::Fracture_Triangles.push_back(temp_mesh);
	}
	is_success = true;
	//end of processing

	this->timer_stop();

	if(this->getWriteLogMode()&&is_success)
    {
        std::string log_text = "\tOpening Fractures Triangulation Data costs: ";
        std::ostringstream strs;
        strs << this->getTimer_sec();
        log_text += (strs.str() +" seconds.");
        dataLibrary::write_text_to_log_file(log_text);
    }

    if(!this->getMuteMode()&&is_success)
    {
        emit show();
    }

	dataLibrary::Status = STATUS_READY;
	emit showReadyStatus();
	delete strfilename;
	if(this->getWorkFlowMode()&&is_success)
	{
		this->Sleep(1000);
		emit GoWorkFlow();
	}
}