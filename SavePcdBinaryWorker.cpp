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

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include "SavePcdBinaryWorker.h"
#include "globaldef.h"
#include "dataLibrary.h"

bool SavePcdBinaryWorker::is_para_satisfying(QString message)
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
		message = QString("savepcdbinary: Path Not Provided.");
		return false;
	}
}

void SavePcdBinaryWorker::prepare()
{
	this->setSaveRGBMode(false);
	if(dataLibrary::Workflow[dataLibrary::current_workline_index].parameters.size()>this->getParaIndex())
	{
		if(dataLibrary::Workflow[dataLibrary::current_workline_index].parameters[this->getParaIndex()] == "rgb")
		{
			this->setSaveRGBMode(true);
			this->setParaIndex(this->getParaIndex()+1);
		}
	}
}

void SavePcdBinaryWorker::doWork()
{
	bool is_success(false);

	dataLibrary::checkupflow();

	QByteArray ba = this->getFileName().toLocal8Bit();
	std::string* strfilename = new std::string(ba.data());

	dataLibrary::Status = STATUS_SAVEBINARY;

	//begin of processing
	if(dataLibrary::cloudxyz->empty()&&dataLibrary::cloudxyzrgb->empty())
	{
		emit showErrors(QString("You Haven't Opened Any Dataset Yet!"));
	}
	else if((!this->getSaveRGBMode())&&(!dataLibrary::cloudxyz->empty()))
	{
		if(!pcl::io::savePCDFileBinary(*strfilename, *dataLibrary::cloudxyz))
		{
			is_success = true;
		}
		else
		{
			emit showErrors("Saving pcd as binary failed.");
		}
	}
	else if(!dataLibrary::cloudxyzrgb->empty())
	{
		if(!pcl::io::savePCDFileBinary(*strfilename, *dataLibrary::cloudxyzrgb))
		{
			is_success = true;
		}
		else
		{
			emit showErrors("Saving pcd as binary failed.");
		}
	}
	else
	{
		emit showErrors("You Want to Save RGB Data as Binary, But You Haven't Opened Any RGB Dataset Yet!");
	}
	//end of processing

	dataLibrary::Status = STATUS_READY;
	emit showReadyStatus();
	delete strfilename;
	if(this->getWorkFlowMode()&&is_success)
	{
		this->Sleep(1000);
		emit GoWorkFlow();
	}
}