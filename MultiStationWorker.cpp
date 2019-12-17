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
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>
#include "geo_normal_3d.h"
#include "geo_normal_3d_omp.h"
#include "globaldef.h"
#include "MultiStationWorker.h"
#include "dataLibrary.h"

bool MultiStationWorker::is_para_satisfying(QString &message)
{
	this->setParaSize(6);
	if(dataLibrary::Workflow[dataLibrary::current_workline_index].parameters.size()>=this->getParaSize())
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
			double pre_align_ds_leaf, pre_align_StdDev, max_correspondence_distance, euclidean_fitness_epsilon;
			int pre_align_normals_k;
			std::stringstream ss_pre_align_ds_leaf(dataLibrary::Workflow[dataLibrary::current_workline_index].parameters[1]);
			ss_pre_align_ds_leaf >> pre_align_ds_leaf;
			this->setPreAlignDSLeaf(pre_align_ds_leaf);
			std::stringstream ss_pre_align_StdDev(dataLibrary::Workflow[dataLibrary::current_workline_index].parameters[2]);
			ss_pre_align_StdDev >> pre_align_StdDev;
			this->setPreAlignStdDev(pre_align_StdDev);
			std::stringstream ss_pre_align_normals_k(dataLibrary::Workflow[dataLibrary::current_workline_index].parameters[3]);
			ss_pre_align_normals_k >> pre_align_normals_k;
			this->setPreAlignNormalsK(pre_align_normals_k);
			std::stringstream ss_max_correspondence_distance(dataLibrary::Workflow[dataLibrary::current_workline_index].parameters[4]);
			ss_max_correspondence_distance >> max_correspondence_distance;
			this->setMaxCorrDistance(max_correspondence_distance);
			std::stringstream ss_euclidean_fitness_epsilon(dataLibrary::Workflow[dataLibrary::current_workline_index].parameters[5]);
			ss_euclidean_fitness_epsilon >> euclidean_fitness_epsilon;
			this->setEFEpsilon(euclidean_fitness_epsilon);

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
		message = QString("multistation: MultiStation Point Cloud File Paths Not Provided and/or Not Enough Parameters Given.");
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

    this->timer_start();

	//begin of processing
	bool is_reading_success(true);
	for(int i=0; i<this->multiStationFilePath.size(); i++)
	{
		pcl::PCLPointCloud2::Ptr cloud_blob(new pcl::PCLPointCloud2);
		if(!pcl::io::loadPCDFile (this->multiStationFilePath[i], *cloud_blob))
		{
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_cloudxyzrgb(new pcl::PointCloud<pcl::PointXYZRGB>);
			pcl::fromPCLPointCloud2 (*cloud_blob, *temp_cloudxyzrgb);
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
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr dsed_rgb_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr ro_rgb_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
		pcl::PointCloud<pcl::Normal>::Ptr target_normals (new pcl::PointCloud<pcl::Normal>);
		pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr target_pointnormal (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
		if(!dataLibrary::cloudxyzrgb->empty())
		{
			dataLibrary::cloudxyzrgb->clear();
		}
		*dataLibrary::cloudxyzrgb = *this->multiStationPointClouds[0];
		pcl::VoxelGrid<pcl::PointXYZRGB> vg;
		vg.setInputCloud (this->multiStationPointClouds[0]);
		vg.setLeafSize (this->getPreAlignDSLeaf(), this->getPreAlignDSLeaf(), this->getPreAlignDSLeaf());
		vg.filter (*dsed_rgb_cloud);
		pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
    	sor.setInputCloud(dsed_rgb_cloud);
    	sor.setMeanK(50);
    	sor.setStddevMulThresh(this->getPreAlignStdDev());
		sor.setNegative(false);
    	sor.filter(*ro_rgb_cloud);
		GeoNormalEstimationOMP<pcl::PointXYZRGB, pcl::Normal> ne_target;
    	ne_target.setInputCloud(ro_rgb_cloud);
    	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree_target (new pcl::search::KdTree<pcl::PointXYZRGB>());
    	ne_target.setSearchMethod(tree_target);
    	ne_target.setKSearch(this->getPreAlignNormalsK());
    	ne_target.compute(*target_normals);
		pcl::concatenateFields(*ro_rgb_cloud, *target_normals, *target_pointnormal);
		for(int i=1; i<this->multiStationPointClouds.size(); i++)
		{
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr dsed_rgb_cloud_i(new pcl::PointCloud<pcl::PointXYZRGB>);
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr ro_rgb_cloud_i(new pcl::PointCloud<pcl::PointXYZRGB>);
			pcl::PointCloud<pcl::Normal>::Ptr src_normals (new pcl::PointCloud<pcl::Normal>);
			pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr src_pointnormal (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
			pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr transformed_pointnormal (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_rgb_cloud_i(new pcl::PointCloud<pcl::PointXYZRGB>);
			pcl::VoxelGrid<pcl::PointXYZRGB> vg_i;
			vg_i.setInputCloud (this->multiStationPointClouds[i]);
			vg_i.setLeafSize (this->getPreAlignDSLeaf(), this->getPreAlignDSLeaf(), this->getPreAlignDSLeaf());
			vg_i.filter (*dsed_rgb_cloud_i);
			pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor_i;
    		sor_i.setInputCloud(dsed_rgb_cloud_i);
    		sor_i.setMeanK(50);
    		sor_i.setStddevMulThresh(this->getPreAlignStdDev());
			sor_i.setNegative(false);
    		sor_i.filter(*ro_rgb_cloud_i);
			GeoNormalEstimationOMP<pcl::PointXYZRGB, pcl::Normal> ne_i;
    		ne_i.setInputCloud(ro_rgb_cloud_i);
    		pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree_i (new pcl::search::KdTree<pcl::PointXYZRGB>());
    		ne_i.setSearchMethod(tree_i);
    		ne_i.setKSearch(this->getPreAlignNormalsK());
    		ne_i.compute(*src_normals);
			pcl::concatenateFields(*ro_rgb_cloud_i, *src_normals, *src_pointnormal);
			pcl::IterativeClosestPointWithNormals<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal> icp;
			icp.setInputCloud(src_pointnormal);
			icp.setInputTarget(target_pointnormal);
			icp.setMaxCorrespondenceDistance(this->getMaxCorrDistance());
			icp.setEuclideanFitnessEpsilon(this->getEFEpsilon());
			icp.align (*transformed_pointnormal);
			//pcl::copyPointCloud(*transformed_pointnormal, *transformed_rgb_cloud_i);
			pcl::transformPointCloud (*this->multiStationPointClouds[i], *transformed_rgb_cloud_i, icp.getFinalTransformation());
			*dataLibrary::cloudxyzrgb += *transformed_rgb_cloud_i;
		}

		if(!dataLibrary::cloudxyz->empty())
		{
			dataLibrary::cloudxyz->clear();
		}
		pcl::copyPointCloud(*dataLibrary::cloudxyzrgb, *dataLibrary::cloudxyz);

		is_success = true;
	}
	//end of processing

    this->timer_stop();

    if(this->getWriteLogMode()&&is_success)
    {
        std::string log_text = "MultiStation costs: ";
        std::ostringstream strs;
        strs << this->getTimer_sec();
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