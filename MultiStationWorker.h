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

#pragma once
#include <vector>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include "Worker.h"

class MultiStationWorker :
	public Worker
{
	Q_OBJECT

private:
	std::vector<std::string> multiStationFilePath;
	std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> multiStationPointClouds;
	double _pre_align_ds_leaf;
	double _pre_align_StdDev;
	int _pre_align_normals_k;
	double _max_correspondence_distance;
	double _euclidean_fitness_epsilon;

public:
	void multiStation(){
		QMetaObject::invokeMethod(this, "doWork");
	}
	void setPreAlignDSLeaf(double pre_align_ds_leaf){
		_pre_align_ds_leaf = pre_align_ds_leaf;
	}
	double getPreAlignDSLeaf(){
		return _pre_align_ds_leaf;
	}
	void setPreAlignStdDev(double pre_align_StdDev){
		_pre_align_StdDev = pre_align_StdDev;
	}
	double getPreAlignStdDev(){
		return _pre_align_StdDev;
	}
	void setPreAlignNormalsK(int pre_align_normals_k){
		_pre_align_normals_k = pre_align_normals_k;
	}
	int getPreAlignNormalsK(){
		return _pre_align_normals_k;
	}
	void setMaxCorrDistance(double max_correspondence_distance){
		_max_correspondence_distance = max_correspondence_distance;
	}
	double getMaxCorrDistance(){
		return _max_correspondence_distance;
	}
	void setEFEpsilon(double euclidean_fitness_epsilon){
		_euclidean_fitness_epsilon = euclidean_fitness_epsilon;
	}
	double getEFEpsilon(){
		return _euclidean_fitness_epsilon;
	}
	virtual bool is_para_satisfying(QString &message);
	virtual void prepare();

private slots:
    void doWork();

signals:
	void show(int i);
};