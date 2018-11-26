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
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/PolygonMesh.h>
#include <pcl/surface/gp3.h>
#include "globaldef.h"
#include "dataLibrary.h"
#include "triangulationWorker.h"

bool triangulationWorker::is_para_satisfying(QString message)
{
	if((dataLibrary::clusters.size()>0)||(dataLibrary::cluster_patches.size()>0))
    {
		this->setParaSize(8);
		if(dataLibrary::Workflow[dataLibrary::current_workline_index].parameters.size()>7)
		{
			int knNeighbors, maxNearestNeighbors;
			double searchRadius, Mu, maxSurfaceAngle, minAngle, maxAngle;
			std::stringstream ss_knNeighbors(dataLibrary::Workflow[dataLibrary::current_workline_index].parameters[0]);
			ss_knNeighbors >> knNeighbors;
			std::stringstream ss_searchRadius(dataLibrary::Workflow[dataLibrary::current_workline_index].parameters[1]);
			ss_searchRadius >> searchRadius;
			std::stringstream ss_Mu(dataLibrary::Workflow[dataLibrary::current_workline_index].parameters[2]);
			ss_Mu >> Mu;
			std::stringstream ss_maxNearestNeighbors(dataLibrary::Workflow[dataLibrary::current_workline_index].parameters[3]);
			ss_maxNearestNeighbors >> maxNearestNeighbors;
			std::stringstream ss_maxSurfaceAngle(dataLibrary::Workflow[dataLibrary::current_workline_index].parameters[4]);
			ss_maxSurfaceAngle >> maxSurfaceAngle;
			std::stringstream ss_minAngle(dataLibrary::Workflow[dataLibrary::current_workline_index].parameters[5]);
			ss_minAngle >> minAngle;
			std::stringstream ss_maxAngle(dataLibrary::Workflow[dataLibrary::current_workline_index].parameters[6]);
			ss_maxAngle >> maxAngle;
			std::string normalConsistancy_string = dataLibrary::Workflow[dataLibrary::current_workline_index].parameters[7];
			std::transform(normalConsistancy_string.begin(), normalConsistancy_string.end(), normalConsistancy_string.begin(), ::tolower);
			if((normalConsistancy_string == "true")||(normalConsistancy_string == "false"))
			{
				TriangulationPara temp_para;
				if(normalConsistancy_string == "true")
				{
					temp_para.normalConsistancy=true;
				}
				else if(normalConsistancy_string == "false")
				{
					temp_para.normalConsistancy=false;
				}
				temp_para.knNeighbors = knNeighbors;
				temp_para.maxAngle = maxAngle;
				temp_para.maxNearestNeighbors = maxNearestNeighbors;
				temp_para.maxSurfaceAngle = maxSurfaceAngle;
				temp_para.minAngle = minAngle;
				temp_para.Mu = Mu;
				temp_para.searchRadius = searchRadius;
				this->setTriPara(temp_para);

				this->setParaIndex(this->getParaSize());
				return true;
			}
			else
			{
				message = QString("ftriangulation: normalConsistancy Not Set Correctly, Please Set It With \"true\" or \"false\".");
				return false;
			}
		}
		else
		{
			message = QString("ftriangulation: No (or not enough) Parameter Given.");
			return false;
		}
	}
	else
	{
		message = QString("ftriangulation: Fracture Data Was Not Opened or Extracted.");
		return false;
	}
}

void triangulationWorker::prepare()
{
	this->setUnmute();
	this->setWriteLog();
	this->check_mute_nolog();
}

void triangulationWorker::doWork()
{
	bool is_success(false);

    dataLibrary::Status = STATUS_TRIANGULATION;

    this->timer_start();

	//begin of processing
    if(dataLibrary::cluster_patches.size()>0)
    {
        for(int i=0; i<dataLibrary::cluster_patches.size(); i++)
        {
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
            pcl::copyPointCloud(*dataLibrary::cluster_patches[i], *cloud);
            // Normal estimation
            pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
            pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
            pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
            tree->setInputCloud (cloud);
            ne.setInputCloud (cloud);
            ne.setSearchMethod (tree);
            ne.setKSearch (this->getTriPara().knNeighbors);
            ne.compute (*normals);
            // normals should not contain the point normals + surface curvatures

            // Concatenate the XYZ and normal fields
            pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
            pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);
            // cloud_with_normals = cloud + normals

            // Create search tree
            pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
            tree2->setInputCloud (cloud_with_normals);

           // Initialize objects
            pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
            pcl::PolygonMesh::Ptr triangles (new pcl::PolygonMesh);

            // Set the maximum distance between connected points (maximum edge length)
            gp3.setSearchRadius (this->getTriPara().searchRadius);

            // Set typical values for the parameters
            gp3.setMu (this->getTriPara().Mu);
            gp3.setMaximumNearestNeighbors (this->getTriPara().maxNearestNeighbors);
            gp3.setMaximumSurfaceAngle(this->getTriPara().maxSurfaceAngle); // 45 degrees
            gp3.setMinimumAngle(this->getTriPara().minAngle); // 10 degrees
            gp3.setMaximumAngle(this->getTriPara().maxAngle); // 120 degrees
            gp3.setNormalConsistency(this->getTriPara().normalConsistancy);

            // Get result
            gp3.setInputCloud (cloud_with_normals);
            gp3.setSearchMethod (tree2);
            gp3.reconstruct (*triangles);
            dataLibrary::Fracture_Triangles.push_back(triangles);
        }
        is_success = true;
    }
    else if(dataLibrary::clusters.size()>0)
    {
        for(int cluster_index = 0; cluster_index < dataLibrary::clusters.size(); cluster_index++)
        {
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
            for(int j = 0; j < dataLibrary::clusters[cluster_index].indices.size(); j++)
            {
                cloud->push_back(dataLibrary::cloudxyz->at(dataLibrary::clusters[cluster_index].indices[j]));
            }
            // Normal estimation
            pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
            pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
            pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
            tree->setInputCloud (cloud);
            ne.setInputCloud (cloud);
            ne.setSearchMethod (tree);
            ne.setKSearch (this->getTriPara().knNeighbors);
            ne.compute (*normals);
            // normals should not contain the point normals + surface curvatures

            // Concatenate the XYZ and normal fields
            pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
            pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);
            // cloud_with_normals = cloud + normals

            // Create search tree
            pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
            tree2->setInputCloud (cloud_with_normals);

           // Initialize objects
            pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
            pcl::PolygonMesh::Ptr triangles (new pcl::PolygonMesh);

            // Set the maximum distance between connected points (maximum edge length)
            gp3.setSearchRadius (this->getTriPara().searchRadius);

            // Set typical values for the parameters
            gp3.setMu (this->getTriPara().Mu);
            gp3.setMaximumNearestNeighbors (this->getTriPara().maxNearestNeighbors);
            gp3.setMaximumSurfaceAngle(this->getTriPara().maxSurfaceAngle); // 45 degrees
            gp3.setMinimumAngle(this->getTriPara().minAngle); // 10 degrees
            gp3.setMaximumAngle(this->getTriPara().maxAngle); // 120 degrees
            gp3.setNormalConsistency(this->getTriPara().normalConsistancy);

            // Get result
            gp3.setInputCloud (cloud_with_normals);
            gp3.setSearchMethod (tree2);
            gp3.reconstruct (*triangles);
            dataLibrary::Fracture_Triangles.push_back(triangles);
        }
        is_success = true;
    }
	//end of processing

    this->timer_stop();

    if(this->getWriteLogMode()&&is_success)
    {
        std::string log_text = "\tFractures Triangulation costs: ";
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