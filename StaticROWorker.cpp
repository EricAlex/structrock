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
#include <pcl/filters/statistical_outlier_removal.h>
#include "StaticROWorker.h"
#include "dataLibrary.h"
#include "globaldef.h"

bool StaticROWorker::is_para_satisfying(QString message)
{
	if(dataLibrary::haveBaseData())
    {
		this->setParaSize(1);
		if(dataLibrary::Workflow[dataLibrary::current_workline_index].parameters.size()>0)
		{
			double stdDev;
			std::stringstream ss(dataLibrary::Workflow[dataLibrary::current_workline_index].parameters[0]);
			ss >> stdDev;
			this->setStdDev(stdDev);

			this->setParaIndex(this->getParaSize());
			return true;
		}
		else
		{
			message = QString("rostatic: Standard Deviation Not Given.");
			return false;
		}
	}
	else
	{
		message = QString("rostatic: You Do Not Have Any Point Cloud Data in Memery!");
		return false;
	}
}

void StaticROWorker::prepare()
{
	this->setUnmute();
	this->setWriteLog();
	this->check_mute_nolog();
}

void StaticROWorker::doWork()
{
	bool is_success(false);

    dataLibrary::checkupflow();

    dataLibrary::Status = STATUS_STATICRO;

    this->timer_start();

	//begin of processing
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(dataLibrary::cloudxyz);
    sor.setMeanK(50);
    sor.setStddevMulThresh(this->getStdDev());

	if(!dataLibrary::outlier_removed_outlier->empty())
	{
		dataLibrary::outlier_removed_outlier->clear();
	}
    sor.setNegative(true);
    sor.filter(*dataLibrary::outlier_removed_outlier);

	if(!dataLibrary::outlier_removed_inlier->empty())
	{
		dataLibrary::outlier_removed_inlier->clear();
	}
    sor.setNegative(false);
    sor.filter(*dataLibrary::outlier_removed_inlier);

	dataLibrary::temp_cloud->clear();
	*dataLibrary::temp_cloud = *dataLibrary::outlier_removed_inlier;

	is_success = true;
	//end of processing

    this->timer_stop();

    if(this->getWriteLogMpde()&&is_success)
    {
        std::string log_text = "\tStatistical Outlier Removing costs: ";
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