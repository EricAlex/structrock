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

#include <time.h>
#include <string>
#include <sstream>
#include <qinputdialog.h>
#include <qmessagebox.h>
#include "RGSWorker.h"
#include "dataLibrary.h"
#include "globaldef.h"
#include "MultiInputDialog.h"
#include "geo_region_growing.h"

bool RGSWorker::is_para_satisfying(QString message)
{
	if(dataLibrary::haveBaseData())
    {
		if(!dataLibrary::pointnormals->empty())
		{
			this->setParaSize(6);
			if(dataLibrary::Workflow[dataLibrary::current_workline_index].parameters.size()>5)
			{
				double smoothness, curvature, residual;
				std::stringstream ss_smoothness(dataLibrary::Workflow[dataLibrary::current_workline_index].parameters[0]);
				ss_smoothness >> smoothness;
				std::stringstream ss_curvature(dataLibrary::Workflow[dataLibrary::current_workline_index].parameters[1]);
				ss_curvature >> curvature;
				std::stringstream ss_residual(dataLibrary::Workflow[dataLibrary::current_workline_index].parameters[2]);
				ss_residual >> residual;
				int min_number_of_Points, number_of_neighbors;
				std::stringstream ss_min_number_of_Points(dataLibrary::Workflow[dataLibrary::current_workline_index].parameters[3]);
				ss_min_number_of_Points >> min_number_of_Points;
				std::stringstream ss_number_of_neighbors(dataLibrary::Workflow[dataLibrary::current_workline_index].parameters[4]);
				ss_number_of_neighbors >> number_of_neighbors;
				std::string IsSmoothMode_string = dataLibrary::Workflow[dataLibrary::current_workline_index].parameters[5];
				std::transform(IsSmoothMode_string.begin(), IsSmoothMode_string.end(), IsSmoothMode_string.begin(), ::tolower);
				if((IsSmoothMode_string == "true")||(IsSmoothMode_string == "false"))
				{
					RGSpara temp_para;
					if(IsSmoothMode_string == "true")
					{
						temp_para.IsSmoothMode=true;
					}
					else if(IsSmoothMode_string == "false")
					{
						temp_para.IsSmoothMode=false;
					}
					temp_para.curvature = curvature;
					temp_para.min_number_of_Points = min_number_of_Points;
					temp_para.number_of_neighbors = number_of_neighbors;
					temp_para.residual = residual;
					temp_para.smoothness = smoothness;
					this->setRGSpara(temp_para);

					this->setParaIndex(this->getParaSize());
					return true;
				}
				else
				{
					message = QString("rgsegmentation: IsSmoothMode Not Set Correctly, Please Set It With \"true\" or \"false\".");
					return false;
				}
			}
			else
			{
				message = QString("rgsegmentation: No (or not enough) Parameter Given.\n(smoothness, curvature, residual, min_number_of_Points, number_of_neighbors and IsSmoothMode)");
				return false;
			}
		}
		else
		{
			message = QString("rgsegmentation: Please Estimate Normals First.");
			return false;
		}
	}
	else
	{
		message = QString("rgsegmentation: You Do Not Have Any Point Cloud Data in Memery!");
		return false;
	}
}

void RGSWorker::prepare()
{
	this->setUnmute();
	this->setWriteLog();
	this->check_mute_nolog();
}

void RGSWorker::doWork()
{
	bool is_success(false);

    dataLibrary::checkupflow();

    dataLibrary::Status = STATUS_RGS;

    dataLibrary::start = clock();

	//begin of processing
    double curvature = this->getRGSpara().curvature;
    double smoothness = this->getRGSpara().smoothness;
    double residual = this->getRGSpara().residual;
    int number_of_neighbors = this->getRGSpara().number_of_neighbors;
    int min_number_of_Points = this->getRGSpara().min_number_of_Points;

	pcl::search::Search<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);

    GeoRegionGrowing<pcl::PointXYZ, pcl::PointNormal> reg;
    reg.setMinClusterSize(min_number_of_Points);
    reg.setSearchMethod(tree);
    reg.setNumberOfNeighbours(number_of_neighbors);
    reg.setInputCloud(dataLibrary::cloudxyz);
    reg.setInputNormals(dataLibrary::pointnormals);
    reg.setSmoothnessThreshold(smoothness/180.0*M_PI);
    reg.setCurvatureThreshold(curvature);
    if(this->getRGSpara().IsSmoothMode)
    {
        reg.setSmoothModeFlag(true);
    }
    else
    {
        reg.setSmoothModeFlag(false);
        reg.setResidualThreshold(residual/180.0*M_PI);
    }

    reg.extract(dataLibrary::clusters);

    dataLibrary::cloudxyzrgb_clusters = reg.getColoredCloud();

	is_success = true;
	//end of processing

    dataLibrary::finish = clock();

    if(this->getWriteLogMpde()&&is_success)
    {
        std::string log_text = "\tRegion Growing Segmentation costs: ";
        std::ostringstream strs;
        strs << (double)(dataLibrary::finish-dataLibrary::start)/CLOCKS_PER_SEC;
        log_text += (strs.str() +" seconds.");
        dataLibrary::write_text_to_log_file(log_text);
    }

    if(!this->getMuteMode()&&is_success)
    {
        emit show();
    }

    dataLibrary::Status = STATUS_READY;
    emit showReadyStatus();
	if(this->getWorkFlowMode()&&is_success)
	{
		this->Sleep(1000);
		emit GoWorkFlow();
	}
}
