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
#include <pcl/filters/voxel_grid.h>
#include "globaldef.h"
#include "downsampleWorker.h"
#include "dataLibrary.h"

bool downsampleWorker::is_para_satisfying(QString &message)
{
	if(dataLibrary::haveBaseData())
    {
		this->setParaSize(1);
		if(dataLibrary::Workflow[dataLibrary::current_workline_index].parameters.size()>=this->getParaSize())
		{
			double leaf;
			std::stringstream ss(dataLibrary::Workflow[dataLibrary::current_workline_index].parameters[0]);
			ss >> leaf;
			this->setLeaf(leaf);

			this->setParaIndex(this->getParaSize());
			return true;
		}
		else
		{
			message = QString("downsample: Minimum Point Distance Not Given.");
			return false;
		}
	}
	else
	{
		message = QString("downsample: You Do Not Have Any Point Cloud Data in Memery!");
		return false;
	}
}
void downsampleWorker::prepare()
{
	this->setUnmute();
	this->setWriteLog();
	this->check_mute_nolog();
}

void downsampleWorker::doWork()
{
	bool is_success(false);

    dataLibrary::checkupflow();

    dataLibrary::Status = STATUS_DOWNSAMPLE;

    this->timer_start();

	//begin of processing
    // Create the filtering object
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud (dataLibrary::cloudxyz);
    sor.setLeafSize (this->getLeaf(), this->getLeaf(), this->getLeaf());
	if(!dataLibrary::downsampledxyz->empty())
	{
		dataLibrary::downsampledxyz->clear();
	}
    sor.filter (*dataLibrary::downsampledxyz);

	dataLibrary::temp_cloud->clear();
	*dataLibrary::temp_cloud = *dataLibrary::downsampledxyz;

	is_success = true;
	//end of processing

    this->timer_stop();

    if(this->getWriteLogMode()&&is_success)
    {
        std::string log_text = "\tDownsampling costs: ";
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
	if(this->getWorkFlowMode()&&is_success)
	{
		this->Sleep(1000);
		emit GoWorkFlow();
	}
}