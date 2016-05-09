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

#include <qinputdialog.h>
#include <qmessagebox.h>
#include "RGSWorker.h"
#include "dataLibrary.h"
#include "globaldef.h"
#include "MultiInputDialog.h"
#include "geo_region_growing.h"

void RGSWorker::doWork()
{
	bool is_success(false);

    dataLibrary::checkupflow();

    dataLibrary::Status = STATUS_RGS;

    double curvature = dataLibrary::RGSparameter.curvature;
    double smoothness = dataLibrary::RGSparameter.smoothness;
    double residual = dataLibrary::RGSparameter.residual;
    int number_of_neighbors = dataLibrary::RGSparameter.number_of_neighbors;
    int min_number_of_Points = dataLibrary::RGSparameter.min_number_of_Points;

	pcl::search::Search<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);

    GeoRegionGrowing<pcl::PointXYZ, pcl::PointNormal> reg;
    reg.setMinClusterSize(min_number_of_Points);
    reg.setSearchMethod(tree);
    reg.setNumberOfNeighbours(number_of_neighbors);
    reg.setInputCloud(dataLibrary::cloudxyz);
    reg.setInputNormals(dataLibrary::pointnormals);
    reg.setSmoothnessThreshold(smoothness/180.0*M_PI);
    reg.setCurvatureThreshold(curvature);
    if(dataLibrary::RGSparameter.IsSmoothMode)
    {
        reg.setSmoothModeFlag(true);
    }
    else
    {
        reg.setSmoothModeFlag(false);
        reg.setResidualThreshold(residual/180.0*M_PI);
    }

    reg.extract(dataLibrary::clusters);

    dataLibrary::cloudxyzrgb_clusters = reg.getColoredCloud();

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
