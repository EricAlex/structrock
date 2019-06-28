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

#include <QStringList>
#include <vector>
#include <string>
#include <math.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/extract_indices.h>
#include <Eigen/src/Core/Matrix.h>
#include "ShowProcessWorker.h"
#include "globaldef.h"
#include "dataLibrary.h"

bool ShowProcessWorker::is_para_satisfying(QString &message)
{
	if(dataLibrary::clusters.size() == 0)
	{
		message = QString("showprocess: You Haven't Performed Any Segmentation Yet!");
		return false;
	}
	else
	{
		this->setParaSize(0);
		if(dataLibrary::Workflow[dataLibrary::current_workline_index].parameters.size()>=this->getParaSize())
		{
			this->setParaIndex(this->getParaSize());
			this->setDefaltFMAP_Mode();
			bool need_expand_ratio(false);
			if((dataLibrary::Workflow[dataLibrary::current_workline_index].parameters.size()>this->getParaIndex())&&(dataLibrary::Workflow[dataLibrary::current_workline_index].parameters[this->getParaIndex()] == "circular"))
			{
				this->setFMAP_Mode(FMAP_CIRCULAR);
				need_expand_ratio = true;
				this->setParaIndex(this->getParaIndex()+1);
			}
			else if((dataLibrary::Workflow[dataLibrary::current_workline_index].parameters.size()>this->getParaIndex())&&(dataLibrary::Workflow[dataLibrary::current_workline_index].parameters[this->getParaIndex()] == "rectangular"))
			{
				this->setFMAP_Mode(FMAP_RECTANGULAR);
				need_expand_ratio = true;
				this->setParaIndex(this->getParaIndex()+1);
			}
			if(need_expand_ratio)
			{
				if((dataLibrary::Workflow[dataLibrary::current_workline_index].parameters.size()>this->getParaIndex())&&(dataLibrary::isOnlyDouble(dataLibrary::Workflow[dataLibrary::current_workline_index].parameters[this->getParaIndex()].c_str())))
				{
					double ratio;
					std::stringstream ss(dataLibrary::Workflow[dataLibrary::current_workline_index].parameters[this->getParaIndex()]);
					ss >> ratio;
					this->setExpandRatio(ratio);
					this->setParaIndex(this->getParaIndex()+1);
				}
				else
				{
					message = QString("showprocess: The Expand Ratio (double) Is Needed For the 'circular' and 'rectangular' Options.");
					return false;
				}
			}
			if(dataLibrary::Workflow[dataLibrary::current_workline_index].parameters.size()>this->getParaIndex())
			{
				for(; this->getParaIndex()<dataLibrary::Workflow[dataLibrary::current_workline_index].parameters.size(); this->setParaIndex(this->getParaIndex()+1))
				{
					this->contents.push_back(dataLibrary::Workflow[dataLibrary::current_workline_index].parameters[this->getParaIndex()]);
				}
				return true;
			}
			else
			{
				message = QString("showprocess: No Name of Process Given.");
				return false;
			}
		}
		else
		{
			message = QString("showprocess: No Parameter Given.");
			return false;
		}
	}
}

void ShowProcessWorker::prepare()
{
	this->setUnmute();
	this->setWriteLog();
}

void ShowProcessWorker::doWork()
{
	bool is_success(false);

	dataLibrary::Status = STATUS_SHOWPROCESS;

	this->timer_start();

	//begin of processing
	//Clear data if needed
	if(!dataLibrary::cloud_hull_all->empty())
		dataLibrary::cloud_hull_all->clear();
	if(dataLibrary::fracture_faces_hull.size()!=0)
		std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>().swap(dataLibrary::fracture_faces_hull);
	if(dataLibrary::Lines.size()!=0)
		std::vector<Line>().swap(dataLibrary::Lines);
	if(dataLibrary::Lines_max.size()!=0)
		std::vector<Line>().swap(dataLibrary::Lines_max);
	if(dataLibrary::Lines_min.size()!=0)
		std::vector<Line>().swap(dataLibrary::Lines_min);
	if(!dataLibrary::segmentation_rem->empty())
		dataLibrary::segmentation_rem->clear();

	//compute centor point and normal
	float nx_all, ny_all, nz_all;
	float curvature_all;
	Eigen::Matrix3f convariance_matrix_all;
	Eigen::Vector4f xyz_centroid_all, plane_parameters_all;
	pcl::compute3DCentroid(*dataLibrary::cloudxyz, xyz_centroid_all);
	pcl::computeCovarianceMatrix(*dataLibrary::cloudxyz, xyz_centroid_all, convariance_matrix_all);
	pcl::solvePlaneParameters(convariance_matrix_all, nx_all, ny_all, nz_all, curvature_all);
	Eigen::Vector3f centroid_all;
	dataLibrary::plane_normal_all(0)=nx_all;
	dataLibrary::plane_normal_all(1)=ny_all;
	dataLibrary::plane_normal_all(2)=nz_all;
	centroid_all(0)=xyz_centroid_all(0);
	centroid_all(1)=xyz_centroid_all(1);
	centroid_all(2)=xyz_centroid_all(2);

	if(dataLibrary::checkContents(this->contents, "suppositional_plane"))
	{
		//project all points
		pcl::ModelCoefficients::Ptr coefficients_all (new pcl::ModelCoefficients());
		coefficients_all->values.resize(4);
		coefficients_all->values[0] = nx_all;
		coefficients_all->values[1] = ny_all;
		coefficients_all->values[2] = nz_all;
		coefficients_all->values[3] = - (nx_all*xyz_centroid_all[0] + ny_all*xyz_centroid_all[1] + nz_all*xyz_centroid_all[2]);

		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected_all (new pcl::PointCloud<pcl::PointXYZ>);
		pcl::ProjectInliers<pcl::PointXYZ> proj_all;
		proj_all.setModelType(pcl::SACMODEL_PLANE);
		proj_all.setInputCloud(dataLibrary::cloudxyz);
		proj_all.setModelCoefficients(coefficients_all);
		proj_all.filter(*cloud_projected_all);

		//compute convex hull
		pcl::ConvexHull<pcl::PointXYZ> chull_all;
		chull_all.setInputCloud(cloud_projected_all);
		chull_all.setDimension(2);
		chull_all.reconstruct(*dataLibrary::cloud_hull_all);
	}

	if(dataLibrary::checkContents(this->contents, "fracture_faces"))
	{
		bool show_rem = false;
		pcl::PointIndices::Ptr indices_all(new pcl::PointIndices);
		indices_all->header=dataLibrary::clusters[0].header;
		if(dataLibrary::checkContents(this->contents, "rgs_remanent"))
		{
			show_rem = true;
		}
		bool show_fracture_traces = false;
		bool show_extension_line = false;
		if(dataLibrary::checkContents(this->contents, "suppositional_plane")&&dataLibrary::checkContents(this->contents, "fracture_traces"))
		{
			show_fracture_traces = true;
		}
		if(dataLibrary::checkContents(this->contents, "suppositional_plane")&&dataLibrary::checkContents(this->contents, "extension_line"))
		{
			show_extension_line = true;
		}

		for(int cluster_index = 0; cluster_index < dataLibrary::clusters.size(); cluster_index++)
		{
			//Add colored patch polygons
			pcl::PointCloud<pcl::PointXYZ>::Ptr plane_cloud (new pcl::PointCloud<pcl::PointXYZ>);
			for(int j = 0; j < dataLibrary::clusters[cluster_index].indices.size(); j++)
			{
				plane_cloud->push_back(dataLibrary::cloudxyz->at(dataLibrary::clusters[cluster_index].indices[j]));
			}

			//prepare for projecting data onto plane
			float nx, ny, nz;
			float curvature;
			Eigen::Matrix3f convariance_matrix;
			Eigen::Vector4f xyz_centroid, plane_parameters;
			pcl::compute3DCentroid(*plane_cloud, xyz_centroid);
			pcl::computeCovarianceMatrix(*plane_cloud, xyz_centroid, convariance_matrix);
			pcl::solvePlaneParameters(convariance_matrix, nx, ny, nz, curvature);
			Eigen::Vector3f centroid;
			centroid(0)=xyz_centroid(0);
			centroid(1)=xyz_centroid(1);
			centroid(2)=xyz_centroid(2);
			Eigen::Vector3f normal;
			normal(0) = nx;
			normal(1) = ny;
			normal(2) = nz;

			Eigen::Vector3f V1 = normal.cross(dataLibrary::plane_normal_all);
			V1 = V1/std::sqrt(V1.dot(V1));
			Eigen::Vector3f V2 = normal.cross(V1);
			V2 = V2/std::sqrt(V2.dot(V2));
			pcl::PointCloud<pcl::PointXY>::Ptr cloud_projected_2d (new pcl::PointCloud<pcl::PointXY>);
			dataLibrary::projection322(centroid, V1, V2, plane_cloud, cloud_projected_2d);
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected_3d (new pcl::PointCloud<pcl::PointXYZ>);
			dataLibrary::projection223(centroid, V1, V2, cloud_projected_2d, cloud_projected_3d);

			//generate a convex hull
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull (new pcl::PointCloud<pcl::PointXYZ>);
			pcl::ConvexHull<pcl::PointXYZ> chull;
			chull.setInputCloud(cloud_projected_3d);
			chull.setDimension(2);
			chull.reconstruct(*cloud_hull);
			dataLibrary::fracture_faces_hull.push_back(cloud_hull);

			if(show_rem)
			{
				indices_all->indices.insert(indices_all->indices.end(), dataLibrary::clusters[cluster_index].indices.begin(), dataLibrary::clusters[cluster_index].indices.end());
			}

			if(show_fracture_traces)
			{
				float length;
				bool flag;
				if(this->getFMAP_Mode() == FMAP_LOWER_BOUND)
					flag = dataLibrary::LowerBound(dataLibrary::plane_normal_all, centroid_all, dataLibrary::cloud_hull_all, normal, centroid, cloud_hull, cluster_index, length, false, show_extension_line);
				else if(this->getFMAP_Mode() == FMAP_RECTANGULAR)
					flag = dataLibrary::Rectangular(dataLibrary::plane_normal_all, centroid_all, dataLibrary::cloud_hull_all, normal, centroid, cloud_hull, cluster_index, length, this->getExpandRatio(), false, show_extension_line);
				else if(this->getFMAP_Mode() == FMAP_CIRCULAR)
					flag = dataLibrary::Circular(dataLibrary::plane_normal_all, centroid_all, dataLibrary::cloud_hull_all, normal, centroid, cloud_hull, cluster_index, length, this->getExpandRatio(), false, show_extension_line);
			}
		}

		if(show_rem)
		{
			pcl::ExtractIndices<pcl::PointXYZ> eifilter (true);
			eifilter.setInputCloud(dataLibrary::cloudxyz);
			eifilter.setIndices(indices_all);
			eifilter.setNegative(true);
			eifilter.filter(*dataLibrary::segmentation_rem);
		}
	}

	is_success = true;
	//end of processing

	this->timer_stop();

	if(this->getWriteLogMode()&&is_success)
    {
        std::string log_text_head = "\tShowProcess (";
		std::string log_text_body = "";
		for(int i=0; i<this->contents.size(); i++)
		{
			log_text_body += (this->contents[i]+" ");
		}
		std::string log_text_tail = ") Costs: ";
		std::string log_text = log_text_head + log_text_body + log_text_tail;
        std::ostringstream strs;
        strs << this->getTimer_sec();
        log_text += (strs.str() +" seconds.");
        dataLibrary::write_text_to_log_file(log_text);
    }

	if(!this->getMuteMode()&&is_success)
    {
		QStringList Qcontents;
		for(int i=0; i<this->contents.size(); i++)
		{
			Qcontents.append(QString::fromStdString(this->contents[i]));
		}
		Qcontents.append(QString::number(this->getFMAP_Mode()));
        emit show(Qcontents);
    }

	dataLibrary::Status = STATUS_READY;
	emit showReadyStatus();
	if(this->getWorkFlowMode()&&is_success)
	{
		this->Sleep(1000);
		emit GoWorkFlow();
	}
}