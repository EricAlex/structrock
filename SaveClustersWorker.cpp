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

#include <sstream>
#include <vector>
#include <fstream>
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
#include <Eigen/src/Core/Matrix.h>
#include "SaveClustersWorker.h"
#include "globaldef.h"
#include "dataLibrary.h"

using namespace std;

bool SaveClustersWorker::is_para_satisfying(QString &message)
{
	if(dataLibrary::clusters.size() == 0)
	{
		message = QString("saveclusters: You Haven't Performed Any Segmentation Yet!");
		return false;
	}
	else
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
			message = QString("saveclusters: Path Not Provided.");
			return false;
		}
	}
}

void SaveClustersWorker::prepare()
{
	this->setUnmute();
	this->setWriteLog();
	this->check_mute_nolog();
}

void SaveClustersWorker::doWork()
{
	bool is_success(false);

    QByteArray ba = this->getFileName().toLocal8Bit();
    string* strfilename = new string(ba.data());
    
    dataLibrary::Status = STATUS_SAVECLUSTERS;

    this->timer_start();
    
	//begin of processing
    string dip_dipdir_file = strfilename->substr(0, strfilename->size()-4) += "_dip_dipdir.txt";
    string dipdir_dip_file = strfilename->substr(0, strfilename->size()-4) += "_dipdir_dip.txt";
	string area_file = strfilename->substr(0, strfilename->size()-4) += "_area.txt";
	string roughness_file = strfilename->substr(0, strfilename->size()-4) += "_roughness.txt";
    ofstream dip_dipdir_out(dip_dipdir_file.c_str());
    ofstream dipdir_dip_out(dipdir_dip_file.c_str());
	ofstream area_out(area_file.c_str());
	ofstream roughness_out(roughness_file.c_str());
    
    int num_of_clusters = dataLibrary::clusters.size();
    ofstream fbinaryout(strfilename->c_str(), std::ios::out|std::ios::binary|std::ios::app);
    fbinaryout.write(reinterpret_cast<const char*>(&num_of_clusters), sizeof(num_of_clusters));
    for(int cluster_index = 0; cluster_index < dataLibrary::clusters.size(); cluster_index++)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr plane_cloud (new pcl::PointCloud<pcl::PointXYZ>);
        int num_of_points = dataLibrary::clusters[cluster_index].indices.size();
        fbinaryout.write(reinterpret_cast<const char*>(&num_of_points), sizeof(num_of_points));
        float rgb = dataLibrary::cloudxyzrgb_clusters->at(dataLibrary::clusters[cluster_index].indices[0]).rgb;
        fbinaryout.write(reinterpret_cast<const char*>(&rgb), sizeof(rgb));
        float x, y, z;
        for(int j = 0; j < dataLibrary::clusters[cluster_index].indices.size(); j++)
        {
            plane_cloud->push_back(dataLibrary::cloudxyz->at(dataLibrary::clusters[cluster_index].indices[j]));
            x = dataLibrary::cloudxyz->at(dataLibrary::clusters[cluster_index].indices[j]).x;
            y = dataLibrary::cloudxyz->at(dataLibrary::clusters[cluster_index].indices[j]).y;
            z = dataLibrary::cloudxyz->at(dataLibrary::clusters[cluster_index].indices[j]).z;
            fbinaryout.write(reinterpret_cast<const char*>(&x), sizeof(x));
            fbinaryout.write(reinterpret_cast<const char*>(&y), sizeof(y));
            fbinaryout.write(reinterpret_cast<const char*>(&z), sizeof(z));
        }
        
        //prepare for projecting data onto plane
        Eigen::Vector3f centroid = dataLibrary::compute3DCentroid(*plane_cloud);
        float nx, ny, nz;
        Eigen::Vector4f plane_normal_param_patch = dataLibrary::fitPlaneManually(*plane_cloud);
        nx = plane_normal_param_patch(0);
        ny = plane_normal_param_patch(1);
        nz = plane_normal_param_patch(2);
        Eigen::Vector3f normal;
        normal << nx, ny, nz;

		Eigen::Vector3f any_vector;
        any_vector << 0.0, 0.0, 1.0;
        Eigen::Vector3f V1 = normal.cross(any_vector);
		V1 = V1/std::sqrt(V1.dot(V1));
		Eigen::Vector3f V2 = normal.cross(V1);
		V2 = V2/std::sqrt(V2.dot(V2));
		pcl::PointCloud<pcl::PointXY>::Ptr cloud_projected_2d (new pcl::PointCloud<pcl::PointXY>);
		dataLibrary::projection322(centroid, V1, V2, plane_cloud, cloud_projected_2d);
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected_3d (new pcl::PointCloud<pcl::PointXYZ>);
		dataLibrary::projection223(centroid, V1, V2, cloud_projected_2d, cloud_projected_3d);
        
        //generate a concave or convex
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::ConvexHull<pcl::PointXYZ> chull;
        chull.setInputCloud(cloud_projected_3d);
		chull.setDimension(2);
        chull.reconstruct(*cloud_hull);
        
        //calculate polygon area
        float area = 0.0f;
        int num_points = cloud_hull->size();
        int j = 0;
        Eigen::Vector3f va, vb, res;
        res(0) = res(1) = res(2) = 0.0f;
        for(int i = 0; i < num_points; i++)
        {
            j = (i+1) % num_points;
            va = cloud_hull->at(i).getVector3fMap();
            vb = cloud_hull->at(j).getVector3fMap();
            res += va.cross(vb);
        }
        area = fabs(res.dot(normal) * 0.5);
        
        float dip_direction, dip;
        
        if(nz < 0.0)
        {
            nx = -nx;
            ny = -ny;
            nz = -nz;
        }
        
        //Dip Direction
        if(nx == 0.0)
        {
            if((ny > 0.0)||(ny == 0.0))
                dip_direction = 0.0;
            else
                dip_direction = 180.0;
        }
        else if(nx > 0.0)
        {
            dip_direction = 90.0 - atan(ny/nx)*180/M_PI;
        }
        else
        {
            dip_direction = 270.0 - atan(ny/nx)*180/M_PI;
        }
        //dip
        if((nx*nx + ny*ny) == 0.0)
        {
            dip = 0.0;
        }
        else
        {
            dip = 90.0 - atan(fabs(nz)/sqrt((nx*nx + ny*ny)))*180/M_PI;
        }

		//calculate fracture surface roughness
		float fracture_total_distance=0.0;
		for(int j = 0; j < plane_cloud->size(); j++)
		{
			Eigen::Vector3f Q;
			Q(0)=plane_cloud->at(j).x;
			Q(1)=plane_cloud->at(j).y;
			Q(2)=plane_cloud->at(j).z;
			fracture_total_distance+=std::abs((Q-centroid).dot(normal)/std::sqrt((normal.dot(normal))));
		}
		float fracture_roughness=fracture_total_distance/plane_cloud->size();

		//saved for further analysis
		dataLibrary::areas.push_back(area);
		dataLibrary::roughnesses.push_back(fracture_roughness);

        dataLibrary::out_dips.push_back(dip);
		dataLibrary::out_dip_directions.push_back(dip_direction);
		dataLibrary::selectedPatches.push_back(cluster_index);

		dip_dipdir_out<<dip<<"\t"<<dip_direction<<"\n";
		dipdir_dip_out<<dip_direction<<"\t"<<dip<<"\n";
		area_out<<area<<"\n";
		roughness_out<<fracture_roughness<<"\n";
        fbinaryout.write(reinterpret_cast<const char*>(&dip), sizeof(dip));
        fbinaryout.write(reinterpret_cast<const char*>(&dip_direction), sizeof(dip_direction));
        fbinaryout.write(reinterpret_cast<const char*>(&area), sizeof(area));
    }
    dip_dipdir_out<<flush;
    dip_dipdir_out.close();
    dipdir_dip_out<<flush;
    dipdir_dip_out.close();
	area_out<<flush;
    area_out.close();
	roughness_out<<flush;
    roughness_out.close();
    fbinaryout.close();

	is_success = true;
	//end of processing

    this->timer_stop();

    if(this->getWriteLogMode()&&is_success)
    {
        std::string log_text = "\tSaving Clusters costs: ";
        std::ostringstream strs;
        strs << this->getTimer_sec();
        log_text += (strs.str() +" seconds.");
        dataLibrary::write_text_to_log_file(log_text);
    }

    dataLibrary::Status = STATUS_READY;
    emit showReadyStatus();
	delete strfilename;
	if(this->getWorkFlowMode()&&is_success)
	{
		this->Sleep(1000);
		emit GoWorkFlow();
	}
}