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
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include "ReadFileWorker.h"
#include "globaldef.h"
#include "dataLibrary.h"

bool ReadFileWorker::is_para_satisfying(QString message)
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
		message = QString("openpcd: Location of PCD File Not Provided.");
		return false;
	}
}

void ReadFileWorker::prepare()
{
	this->setUnmute();
	this->setWriteLog();
	this->check_mute_nolog();
}

void ReadFileWorker::doWork()
{
	bool is_success(false);

    QByteArray ba = this->getFileName().toLocal8Bit();
    std::string* strfilename = new std::string(ba.data());

	sensor_msgs::PointCloud2::Ptr cloud_blob(new sensor_msgs::PointCloud2);

	dataLibrary::Status = STATUS_OPENPCD;

	dataLibrary::start = clock();

	//begin of processing
	if(!pcl::io::loadPCDFile (*strfilename, *cloud_blob))
	{
		dataLibrary::clearall();

		pcl::fromROSMsg (*cloud_blob, *dataLibrary::cloudxyz);

		if(pcl::getFieldIndex (*cloud_blob, "rgb")<0)
		{
			dataLibrary::cloudID = *strfilename;

			if(!this->getMuteMode())
            {
                emit ReadFileReady(CLOUDXYZ);
            }
			is_success = true;
		}
		else
		{
			pcl::fromROSMsg (*cloud_blob, *dataLibrary::cloudxyzrgb);
			dataLibrary::cloudID = *strfilename;
        
			if(!this->getMuteMode())
            {
                emit ReadFileReady(CLOUDXYZRGB);
            }
			is_success = true;
		}
	}
	else
	{
		emit showErrors("Error opening pcd file.");
	}
	//end of processing

	dataLibrary::finish = clock();

    if(this->getWriteLogMpde()&&is_success)
    {
		std::string string_filename = this->getFileName().toUtf8().constData();
        std::string log_text = string_filename + "\n\tReading PCD file costs: ";
        std::ostringstream strs;
        strs << (double)(dataLibrary::finish-dataLibrary::start)/CLOCKS_PER_SEC;
        log_text += (strs.str() +" seconds.");
        dataLibrary::write_text_to_log_file(log_text);
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