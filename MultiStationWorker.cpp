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
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include "globaldef.h"
#include "MultiStationWorker.h"
#include "dataLibrary.h"

bool MultiStationWorker::is_para_satisfying(QString message)
{
	this->setParaSize(3);
	if(dataLibrary::Workflow[dataLibrary::current_workline_index].parameters.size()>2)
	{
		std::istringstream line_stream(dataLibrary::Workflow[dataLibrary::current_workline_index].parameters[0]);
		char command_split_str = '|';
		std::vector<std::string> tokens;
		for(std::string each; std::getline(line_stream, each, command_split_str); tokens.push_back(each));

		for(int i=0; i<tokens.size(); i++)
		{
			tokens[i].erase(std::remove(tokens[i].begin(), tokens[i].end(),'\n'), tokens[i].end());
			tokens[i].erase(std::remove(tokens[i].begin(), tokens[i].end(),'\r'), tokens[i].end());
			tokens[i].erase(std::remove(tokens[i].begin(), tokens[i].end(),'\t'), tokens[i].end());
			if(tokens[i].empty())
				tokens.erase(tokens.begin()+i);
		}
		if(tokens.size()>0)
		{
			for(int i=0; i<tokens.size(); i++)
			{
				this->multiStationFilePath.push_back(tokens[i]);
			}
			double stdDev, leaf;
			std::stringstream ss_stdDev(dataLibrary::Workflow[dataLibrary::current_workline_index].parameters[1]);
			ss_stdDev >> stdDev;
			this->setStdDev(stdDev);
			std::stringstream ss_leaf(dataLibrary::Workflow[dataLibrary::current_workline_index].parameters[2]);
			ss_leaf >> leaf;
			this->setLeaf(leaf);

			this->setParaIndex(this->getParaSize());
			return true;
		}
		else
		{
			message = QString("multistation: None Valid MultiStation Point Cloud File Path Provided.");
			return false;
		}
	}
	else
	{
		message = QString("multistation: MultiStation Point Cloud File Paths and/or Remove Outliers 'stdDev' and/or Downsample 'Leaf' Not Provided.");
		return false;
	}
}
void MultiStationWorker::prepare()
{
	this->setUnmute();
	this->setWriteLog();
	this->check_mute_nolog();
}

void MultiStationWorker::doWork()
{
	bool is_success(false);

    dataLibrary::Status = STATUS_MULTISTATION;

    dataLibrary::start = clock();

	//begin of processing
	bool is_reading_success(true);
	for(int i=0; i<this->multiStationFilePath.size(); i++)
	{
		sensor_msgs::PointCloud2::Ptr cloud_blob(new sensor_msgs::PointCloud2);
		if(!pcl::io::loadPCDFile (this->multiStationFilePath[i], *cloud_blob))
		{
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_cloudxyzrgb(new pcl::PointCloud<pcl::PointXYZRGB>);
			pcl::fromROSMsg (*cloud_blob, *temp_cloudxyzrgb);
			this->multiStationPointClouds.push_back(temp_cloudxyzrgb);
		}
		else
		{
			std::string error_msg = "Error opening pcd file: " + this->multiStationFilePath[i];
			emit showErrors(QString::fromUtf8(error_msg.c_str()));
			is_reading_success = false;
			break;
		}
	}
	if(is_reading_success)
	{
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr mid_rgb_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
		pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
		sor.setInputCloud(this->multiStationPointClouds[0]);
		sor.setMeanK(50);
		sor.setStddevMulThresh(this->getStdDev());
		sor.setNegative(false);
		sor.filter(*mid_rgb_cloud);
		if(!dataLibrary::cloudxyzrgb->empty())
		{
			dataLibrary::cloudxyzrgb->clear();
		}
		if(this->getLeaf() == 0.0)
		{
			*dataLibrary::cloudxyzrgb = *mid_rgb_cloud;
		}
		else
		{
			pcl::VoxelGrid<pcl::PointXYZRGB> vg;
			vg.setInputCloud (mid_rgb_cloud);
			vg.setLeafSize (this->getLeaf(), this->getLeaf(), this->getLeaf());
			vg.filter (*dataLibrary::cloudxyzrgb);
		}
		for(int i=1; i<this->multiStationPointClouds.size(); i++)
		{
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr mid_rgb_cloud_i(new pcl::PointCloud<pcl::PointXYZRGB>);
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr dsed_rgb_cloud_i(new pcl::PointCloud<pcl::PointXYZRGB>);
			pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor_i;
			sor_i.setInputCloud(this->multiStationPointClouds[i]);
			sor_i.setMeanK(50);
			sor_i.setStddevMulThresh(this->getStdDev());
			sor_i.setNegative(false);
			sor_i.filter(*mid_rgb_cloud_i);
			if(this->getLeaf() == 0.0)
			{
				*dataLibrary::cloudxyzrgb += *mid_rgb_cloud_i;
			}
			else
			{
				pcl::VoxelGrid<pcl::PointXYZRGB> vg_i;
				vg_i.setInputCloud (mid_rgb_cloud_i);
				vg_i.setLeafSize (this->getLeaf(), this->getLeaf(), this->getLeaf());
				vg_i.filter (*dsed_rgb_cloud_i);
				*dataLibrary::cloudxyzrgb += *dsed_rgb_cloud_i;
			}
		}

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
        std::string log_text = "MultiStation costs: ";
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