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
#include "SavePcdASCIIWorker.h"
#include "globaldef.h"
#include "dataLibrary.h"

void SavePcdASCIIWorker::doWork(const QString &filename)
{
	bool is_success(false);

	dataLibrary::checkupflow();

	QByteArray ba = filename.toLocal8Bit();
	std::string* strfilename = new std::string(ba.data());

	dataLibrary::Status = STATUS_SAVEASCII;

	//begin of processing
	if(dataLibrary::cloudxyz->empty()&&dataLibrary::cloudxyzrgb->empty())
	{
		emit showErrors(QString("You Haven't Opened Any Dataset Yet!"));
	}
	else if(!dataLibrary::cloudxyz->empty())
	{
		if(!pcl::io::savePCDFileASCII(*strfilename, *dataLibrary::cloudxyz))
		{
			is_success = true;
		}
		else
		{
			emit showErrors("Saving pcd as ASCII failed.");
		}
	}
	else if(!dataLibrary::cloudxyzrgb->empty())
	{
		if(!pcl::io::savePCDFileASCII(*strfilename, *dataLibrary::cloudxyzrgb))
		{
			is_success = true;
		}
		else
		{
			emit showErrors("Saving pcd as ASCII failed.");
		}
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