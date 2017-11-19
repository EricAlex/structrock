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
#include <pcl/filters/voxel_grid.h>
#include "globaldef.h"
#include "MultiStationWorker.h"
#include "dataLibrary.h"

void MultiStationWorker::doWork(const double &leaf)
{
	bool is_success(false);

    dataLibrary::Status = STATUS_MULTISTATION;

    dataLibrary::start = clock();

	//begin of processing
	bool is_reading_success(true);
	for(int i=0; i<dataLibrary::multiStationFilePath.size(); i++)
	{
		sensor_msgs::PointCloud2::Ptr cloud_blob(new sensor_msgs::PointCloud2);
		if(!pcl::io::loadPCDFile (dataLibrary::multiStationFilePath[i], *cloud_blob))
		{
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_cloudxyzrgb(new pcl::PointCloud<pcl::PointXYZRGB>);
			pcl::fromROSMsg (*cloud_blob, *temp_cloudxyzrgb);
			dataLibrary::multiStationPointClouds.push_back(temp_cloudxyzrgb);
		}
		else
		{
			std::string error_msg = "Error opening pcd file: " + dataLibrary::multiStationFilePath[i];
			emit showErrors(QString::fromUtf8(error_msg.c_str()));
			is_reading_success = false;
			break;
		}
	}
	if(is_reading_success)
	{
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_cloudxyzrgb_all(new pcl::PointCloud<pcl::PointXYZRGB>);
		*temp_cloudxyzrgb_all = *dataLibrary::multiStationPointClouds[0];
		for(int i=1; i<dataLibrary::multiStationPointClouds.size(); i++)
		{
			*temp_cloudxyzrgb_all += *dataLibrary::multiStationPointClouds[i];
		}
		// Create the filtering object
		pcl::VoxelGrid<pcl::PointXYZRGB> sor;
		sor.setInputCloud (temp_cloudxyzrgb_all);
		sor.setLeafSize (leaf, leaf, leaf);
		if(!dataLibrary::cloudxyzrgb->empty())
		{
			dataLibrary::cloudxyzrgb->clear();
		}
		sor.filter (*dataLibrary::cloudxyzrgb);

		if(!dataLibrary::cloudxyz->empty())
		{
			dataLibrary::cloudxyz->clear();
		}
		pcl::copyPointCloud(*dataLibrary::cloudxyzrgb, *dataLibrary::cloudxyz);

		is_success = true;
	}
	//end of processing

    dataLibrary::finish = clock();

    if(this->getWriteLogMpde()&&is_success)
    {
        std::string log_text = "\tMultiStation costs: ";
        std::ostringstream strs;
        strs << (double)(dataLibrary::finish-dataLibrary::start)/CLOCKS_PER_SEC;
        log_text += (strs.str() +" seconds.");
        dataLibrary::write_text_to_log_file(log_text);
    }

    if(!this->getMuteMode()&&is_success)
    {
        emit show(CLOUDXYZRGB);
    }

    dataLibrary::Status = STATUS_READY;
    emit showReadyStatus();
	if(this->getWorkFlowMode()&&is_success)
	{
		this->Sleep(1000);
		emit GoWorkFlow();
	}
}