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
#include <pcl/filters/voxel_grid.h>
#include <qinputdialog.h>
#include <qmessagebox.h>
#include "globaldef.h"
#include "downsampleWorker.h"
#include "dataLibrary.h"

void downsampleWorker::doWork(const double &leaf)
{
	bool is_success(false);

    dataLibrary::checkupflow();

    dataLibrary::Status = STATUS_DOWNSAMPLE;

    dataLibrary::start = clock();

    // Create the filtering object
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud (dataLibrary::cloudxyz);
    sor.setLeafSize (leaf, leaf, leaf);
    sor.filter (*dataLibrary::downsampledxyz);

    dataLibrary::finish = clock();

    if(this->getWriteLogMpde())
    {
        std::string log_text = "\tDownsampling costs: ";
        std::ostringstream strs;
        strs << (double)(dataLibrary::finish-dataLibrary::start)/CLOCKS_PER_SEC;
        log_text += (strs.str() +" seconds.");
        dataLibrary::write_text_to_log_file(log_text);
    }

    if(!this->getMuteMode())
    {
        emit show();
    }
	is_success = true;
    dataLibrary::Status = STATUS_READY;
    emit showReadyStatus();
	if(this->getWorkFlowMode()&&is_success)
	{
		this->Sleep(1000);
		emit GoWorkFlow();
	}
}