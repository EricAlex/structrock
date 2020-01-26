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
#include "SaveTraceMapWorker.h"
#include "omp.h"
#include "globaldef.h"
#include "dataLibrary.h"

using namespace std;

bool SaveTraceMapWorker::is_para_satisfying(QString &message)
{
    if((dataLibrary::clusters.size() == 0)&&(dataLibrary::cluster_patches.size() == 0))
	{
        message = QString("savetracemap: You Haven't Performed Any Segmentation or Opened Any Clusters Files Yet!");
		return false;
	}
    else
    {
        if(dataLibrary::clusters.size() != 0){
            this->setClustersFromFilesFlag(false);
        }
        if(dataLibrary::cluster_patches.size() != 0){
            this->setClustersFromFilesFlag(true);
        }
        this->setParaSize(1);
		if(dataLibrary::Workflow[dataLibrary::current_workline_index].parameters.size()>0)
		{
			this->setFileName(QString::fromUtf8(dataLibrary::Workflow[dataLibrary::current_workline_index].parameters[0].c_str()));
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
					message = QString("savetracemap: The Expand Ratio (double) Is Needed For the 'circular' and 'rectangular' Options.");
					return false;
				}
			}
			return true;
		}
		else
		{
			message = QString("savetracemap: Path Not Provided.");
			return false;
		}
    }
}

void SaveTraceMapWorker::prepare()
{
	this->setTrimTraceEdgesMode(true);
	if((dataLibrary::Workflow[dataLibrary::current_workline_index].parameters.size()>this->getParaIndex())&&(dataLibrary::Workflow[dataLibrary::current_workline_index].parameters[this->getParaIndex()] == "notrimedges"))
	{
		this->setTrimTraceEdgesMode(false);
		this->setParaIndex(this->getParaIndex()+1);
	}
	this->setUnmute();
	this->setWriteLog();
	this->check_mute_nolog();
}

void SaveTraceMapWorker::doWork()
{
	bool is_success(false);

    QByteArray ba = this->getFileName().toLocal8Bit();
    string* strfilename = new string(ba.data());
    
    dataLibrary::Status = STATUS_SAVETRACEMAP;

    this->timer_start();
    
	//begin of processing
    pcl::PointCloud<pcl::PointXYZ>::Ptr clusters_cloud_all (new pcl::PointCloud<pcl::PointXYZ>);
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clusters_cloud_vec;
    if(this->IsClustersFromFiles()){
        for(int cluster_index = 0; cluster_index < dataLibrary::cluster_patches.size(); cluster_index++)
        {
            pcl::PointCloud<pcl::PointXYZ>::Ptr temp_xyz_cloud (new pcl::PointCloud<pcl::PointXYZ>);
            pcl::copyPointCloud(*dataLibrary::cluster_patches[cluster_index], *temp_xyz_cloud);
            *clusters_cloud_all += *temp_xyz_cloud;
            clusters_cloud_vec.push_back(temp_xyz_cloud);
        }
    }
    else{
        for(int cluster_index = 0; cluster_index < dataLibrary::clusters.size(); cluster_index++)
        {
            pcl::PointCloud<pcl::PointXYZ>::Ptr temp_xyz_cloud (new pcl::PointCloud<pcl::PointXYZ>);
            for(int j = 0; j < dataLibrary::clusters[cluster_index].indices.size(); j++){
                temp_xyz_cloud->push_back(dataLibrary::cloudxyz->at(dataLibrary::clusters[cluster_index].indices[j]));
            }
            *clusters_cloud_all += *temp_xyz_cloud;
            clusters_cloud_vec.push_back(temp_xyz_cloud);
        }
    }

    //compute centor point and normal
    Eigen::Vector3f centroid_all = dataLibrary::compute3DCentroid(*clusters_cloud_all);
    float nx_all, ny_all, nz_all;
    Eigen::Vector4f plane_normal_param = dataLibrary::fitPlaneManually(*clusters_cloud_all);
    nx_all = plane_normal_param(0);
    ny_all = plane_normal_param(1);
    nz_all = plane_normal_param(2);
    dataLibrary::plane_normal_all << nx_all, ny_all, nz_all;
    
    //project all points
    pcl::ModelCoefficients::Ptr coefficients_all (new pcl::ModelCoefficients());
    coefficients_all->values.resize(4);
    coefficients_all->values[0] = nx_all;
    coefficients_all->values[1] = ny_all;
    coefficients_all->values[2] = nz_all;
    coefficients_all->values[3] = - (nx_all*centroid_all[0] + ny_all*centroid_all[1] + nz_all*centroid_all[2]);
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected_all (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ProjectInliers<pcl::PointXYZ> proj_all;
    proj_all.setModelType(pcl::SACMODEL_PLANE);
    proj_all.setInputCloud(clusters_cloud_all);
    proj_all.setModelCoefficients(coefficients_all);
    proj_all.filter(*cloud_projected_all);
    
    //compute convex hull
    pcl::ConvexHull<pcl::PointXYZ> chull_all;
    chull_all.setInputCloud(cloud_projected_all);
	chull_all.setDimension(2);
    chull_all.reconstruct(*dataLibrary::cloud_hull_all);
    
    //compute area
    float area_all = 0.0f;
    int num_points_all = dataLibrary::cloud_hull_all->size();
    int j = 0;
    Eigen::Vector3f va_all, vb_all, res_all;
    res_all(0) = res_all(1) = res_all(2) = 0.0f;
    for(int i = 0; i < num_points_all; i++)
    {
        j = (i+1) % num_points_all;
        va_all = dataLibrary::cloud_hull_all->at(i).getVector3fMap();
        vb_all = dataLibrary::cloud_hull_all->at(j).getVector3fMap();
        res_all += va_all.cross(vb_all);
    }
    area_all = fabs(res_all.dot(dataLibrary::plane_normal_all) * 0.5);
    
    std::vector<bool> Flags;
    std::vector<float> Dip_Directions, Dips, Areas, Lengths, Fracture_Roughnesses, Displacements, Dip2Planes;
    std::vector<int> Num_Points;
    Flags.resize(clusters_cloud_vec.size(), false);
    Dip_Directions.resize(clusters_cloud_vec.size(), 0.0);
    Dips.resize(clusters_cloud_vec.size(), 0.0);
    Areas.resize(clusters_cloud_vec.size(), 0.0);
    Lengths.resize(clusters_cloud_vec.size(), 0.0);
    Fracture_Roughnesses.resize(clusters_cloud_vec.size(), 0.0);
    Displacements.resize(clusters_cloud_vec.size(), 0.0);
    Dip2Planes.resize(clusters_cloud_vec.size(), 0.0);
    Num_Points.resize(clusters_cloud_vec.size(), 0);
    std::vector<Eigen::Vector3f> Centroids, Normals;
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> Cloud_Projected_3ds, Cloud_Hulls;
    for(int cluster_index = 0; cluster_index < clusters_cloud_vec.size(); cluster_index++)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr plane_cloud = clusters_cloud_vec[cluster_index];
        Num_Points[cluster_index] = plane_cloud->size();
        //prepare for projecting data onto plane
        Eigen::Vector3f centroid = dataLibrary::compute3DCentroid(*plane_cloud);
        Centroids.push_back(centroid);
        float nx, ny, nz;
        Eigen::Vector4f plane_normal_param_patch = dataLibrary::fitPlaneManually(*plane_cloud);
        nx = plane_normal_param_patch(0);
        ny = plane_normal_param_patch(1);
        nz = plane_normal_param_patch(2);
        Eigen::Vector3f normal;
        normal << nx, ny, nz;
        Normals.push_back(normal);

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
        Cloud_Projected_3ds.push_back(cloud_projected_3d);
        
        //generate a concave or convex
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::ConvexHull<pcl::PointXYZ> chull;
        chull.setInputCloud(cloud_projected_3d);
		chull.setDimension(2);
        chull.reconstruct(*cloud_hull);
        Cloud_Hulls.push_back(cloud_hull);
    }
    // #pragma omp parallel for
    for(int cluster_index = 0; cluster_index < clusters_cloud_vec.size(); cluster_index++)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr plane_cloud = clusters_cloud_vec[cluster_index];
        // calculate polygon area
        float area = 0.0f;
        int num_points = Cloud_Hulls[cluster_index]->size();
        int j = 0;
        Eigen::Vector3f va, vb, res;
        res(0) = res(1) = res(2) = 0.0f;
        for(int i = 0; i < num_points; i++)
        {
            j = (i+1) % num_points;
            va = Cloud_Hulls[cluster_index]->at(i).getVector3fMap();
            vb = Cloud_Hulls[cluster_index]->at(j).getVector3fMap();
            res += va.cross(vb);
        }
        area = fabs(res.dot(Normals[cluster_index]) * 0.5);
        Areas[cluster_index] = area;
        
        float dip_direction, dip, nx, ny, nz;
        nx = Normals[cluster_index](0);
        ny = Normals[cluster_index](1);
        nz = Normals[cluster_index](2);
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
        Dip_Directions[cluster_index] = dip_direction;
        Dips[cluster_index] = dip;

		// calculate fracture surface roughness
		float fracture_total_distance=0.0;
		for(int j = 0; j < plane_cloud->size(); j++)
		{
			Eigen::Vector3f Q;
			Q(0)=plane_cloud->at(j).x;
			Q(1)=plane_cloud->at(j).y;
			Q(2)=plane_cloud->at(j).z;
			fracture_total_distance+=std::abs((Q-Centroids[cluster_index]).dot(Normals[cluster_index])/std::sqrt((Normals[cluster_index].dot(Normals[cluster_index]))));
		}
		float fracture_roughness=fracture_total_distance/plane_cloud->size();
        Fracture_Roughnesses[cluster_index] = fracture_roughness;

		float length;
        bool flag;
		if(this->getFMAP_Mode() == FMAP_LOWER_BOUND)
			flag = dataLibrary::LowerBound(dataLibrary::plane_normal_all, centroid_all, dataLibrary::cloud_hull_all, Normals[cluster_index], Centroids[cluster_index], Cloud_Hulls[cluster_index], cluster_index, length, this->getTrimTraceEdgesMode(), false);
		else if(this->getFMAP_Mode() == FMAP_RECTANGULAR)
			flag = dataLibrary::Rectangular(dataLibrary::plane_normal_all, centroid_all, dataLibrary::cloud_hull_all, Normals[cluster_index], Centroids[cluster_index], Cloud_Hulls[cluster_index], cluster_index, length, this->getExpandRatio(), this->getTrimTraceEdgesMode(), false);
		else if(this->getFMAP_Mode() == FMAP_CIRCULAR)
			flag = dataLibrary::Circular(dataLibrary::plane_normal_all, centroid_all, dataLibrary::cloud_hull_all, Normals[cluster_index], Centroids[cluster_index], Cloud_Hulls[cluster_index], cluster_index, length, this->getExpandRatio(), this->getTrimTraceEdgesMode(), false);
        
        Flags[cluster_index] = flag;
        Lengths[cluster_index] = length;

        //calculate displacement
		Eigen::Vector3f line_direction = Normals[cluster_index].cross(dataLibrary::plane_normal_all.cross(Normals[cluster_index]));
		if((line_direction(0) == 0)&&(line_direction(1) == 0)&&(line_direction(2) == 0))
		{
			Displacements[cluster_index] = 0.0;
            Dip2Planes[cluster_index] = 0.0;
		}
		else
		{
			if(flag)
			{
				float max_value, min_value;
				int max_index, min_index;
				Eigen::Vector3f point;
				point(0) = Cloud_Projected_3ds[cluster_index]->at(0).x;
				point(1) = Cloud_Projected_3ds[cluster_index]->at(0).y;
				point(2) = Cloud_Projected_3ds[cluster_index]->at(0).z;
				float mod_line_direction = std::sqrt(line_direction.dot(line_direction));
				max_value = min_value = line_direction.dot(point - Centroids[cluster_index])/mod_line_direction;
				max_index = min_index = 0;
				for(int i=1; i<Cloud_Projected_3ds[cluster_index]->size(); i++)
				{
					Eigen::Vector3f point;
					point(0) = Cloud_Projected_3ds[cluster_index]->at(i).x;
					point(1) = Cloud_Projected_3ds[cluster_index]->at(i).y;
					point(2) = Cloud_Projected_3ds[cluster_index]->at(i).z;
					
					float value = line_direction.dot(point - Centroids[cluster_index])/mod_line_direction;
					
					if(max_value<value)
					{
						max_value = value;
						max_index = i;
					}
					if(min_value>value)
					{
						min_value = value;
						min_index = i;
					}
				}
				float alpha = std::acos(std::abs(dataLibrary::plane_normal_all.dot(Normals[cluster_index])/(std::sqrt(dataLibrary::plane_normal_all.dot(dataLibrary::plane_normal_all))*std::sqrt(Normals[cluster_index].dot(Normals[cluster_index])))));
				Dip2Planes[cluster_index] = alpha*360.0/TWOPI;
				float tangent_alpha = std::tan(alpha);
				float height_max = std::abs(dataLibrary::plane_normal_all.dot(Cloud_Projected_3ds[cluster_index]->at(max_index).getVector3fMap()-centroid_all)/std::sqrt(dataLibrary::plane_normal_all.dot(dataLibrary::plane_normal_all)));
				float height_min = std::abs(dataLibrary::plane_normal_all.dot(Cloud_Projected_3ds[cluster_index]->at(min_index).getVector3fMap()-centroid_all)/std::sqrt(dataLibrary::plane_normal_all.dot(dataLibrary::plane_normal_all)));
				float displacement_max = height_max/tangent_alpha;
				float displacement_min = height_min/tangent_alpha;
				Displacements[cluster_index] = (displacement_max+displacement_min)/2.0;
			}
            else{
                Displacements[cluster_index] = 0.0;
                Dip2Planes[cluster_index] = 0.0;
            }
		}
    }

    //initial total length of fracture traces
    float total_length=0.0;
	//initial over estimate length of fracture traces
	float error_length=0.0;
	//initial total displacement
	float total_displacement=0.0;
	//initial mean dip2plane angle
	float total_dip2plane=0.0;
	int inside_num=0;
    string area_file = strfilename->substr(0, strfilename->size()-4) += "_area.txt";
	string roughness_file = strfilename->substr(0, strfilename->size()-4) += "_roughness.txt";
    string textfilename = strfilename->substr(0, strfilename->size()-4) += "_table.txt";
	ofstream area_out(area_file.c_str());
	ofstream roughness_out(roughness_file.c_str());
    ofstream fout(textfilename.c_str());
    fout<<"Flag"<<"\t"<<"Number"<<"\t"<<"Points"<<"\t"<<"Direc"<<"\t"<<"Dip"<<"\t"<<"Area"<<"\t"<<"Length"<<"\t"<<"Roughness"<<"\n";
    for(int cluster_index = 0; cluster_index < clusters_cloud_vec.size(); cluster_index++){
        fout<<Flags[cluster_index]<<"\t"<<cluster_index+1<<"\t"<<Num_Points[cluster_index]<<"\t"<<Dip_Directions[cluster_index]<<"\t"<<Dips[cluster_index]<<"\t"<<Areas[cluster_index]<<"\t"<<Lengths[cluster_index]<<"\t"<<Fracture_Roughnesses[cluster_index]<<"\n";
        if(Flags[cluster_index]){
            total_length += Lengths[cluster_index];
            area_out<<Areas[cluster_index]<<"\n";
			roughness_out<<Fracture_Roughnesses[cluster_index]<<"\n";
            inside_num += 1;
        }
        else{
            error_length += Lengths[cluster_index];
        }
        total_displacement += Displacements[cluster_index];
        total_dip2plane += Dip2Planes[cluster_index];
    }
	fout<<flush;
    fout.close();
	area_out<<flush;
    area_out.close();
	roughness_out<<flush;
    roughness_out.close();
    string fracture_intensity = strfilename->substr(0, strfilename->size()-4) += "_fracture_intensity.txt";
    ofstream fracture_intensity_out(fracture_intensity.c_str());
	fracture_intensity_out<<"Total area:"<<"\t"<<area_all<<"\n";
	fracture_intensity_out<<"Percentage of displacement errors:"<<"\t"<<total_displacement/total_length*100<<" \%\n";
	fracture_intensity_out<<"Percentage of over estimated errors:"<<"\t"<<error_length/total_length*100<<" \%\n";
	fracture_intensity_out<<"Mean dip to plane angle:"<<"\t"<<total_dip2plane/inside_num<<"\n";
    fracture_intensity_out<<"Fracture Density:"<<"\t"<<total_length/area_all;
    fracture_intensity_out<<flush;
    fracture_intensity_out.close();
    
    //save outcrop convex hull and fracture traces, both 3d and 2d
	Eigen::Vector3f V_x = dataLibrary::cloud_hull_all->at(1).getVector3fMap() - dataLibrary::cloud_hull_all->at(0).getVector3fMap();
    Eigen::Vector3f V_y = dataLibrary::plane_normal_all.cross(V_x);
    std::vector<Eigen::Vector2f> convex_hull_all_2d;
    dataLibrary::projection322(V_x, V_y, dataLibrary::cloud_hull_all, convex_hull_all_2d);

    string hull_traces = strfilename->substr(0, strfilename->size()-4) += "_convex_hull&fracture_traces.txt";
    ofstream hull_traces_out(hull_traces.c_str());
    hull_traces_out<<"hull\t"<<dataLibrary::cloud_hull_all->points.size()<<"\t"<<dataLibrary::plane_normal_all(0)<<"\t"<<dataLibrary::plane_normal_all(1)<<"\t"<<dataLibrary::plane_normal_all(2)<<"\n";
    for(int i=0; i<dataLibrary::cloud_hull_all->points.size(); i++)
    {
        hull_traces_out<<dataLibrary::cloud_hull_all->points[i].x<<"\t"<<dataLibrary::cloud_hull_all->points[i].y<<"\t"<<dataLibrary::cloud_hull_all->points[i].z<<"\t"<<convex_hull_all_2d[i](0)<<"\t"<<convex_hull_all_2d[i](1)<<"\n";
    }
    hull_traces_out<<"traces\n";
	Eigen::Vector3f point_in_begin, point_in_end;
	Eigen::Vector2f point_out_begin, point_out_end;
    for(int i=0; i<dataLibrary::Lines.size(); i++)
    {
		point_in_begin(0)=dataLibrary::Lines[i].begin.x;
		point_in_begin(1)=dataLibrary::Lines[i].begin.y;
		point_in_begin(2)=dataLibrary::Lines[i].begin.z;
		point_in_end(0)=dataLibrary::Lines[i].end.x;
		point_in_end(1)=dataLibrary::Lines[i].end.y;
		point_in_end(2)=dataLibrary::Lines[i].end.z;

		dataLibrary::projection322(V_x, V_y, point_in_begin, point_out_begin);
		dataLibrary::projection322(V_x, V_y, point_in_end, point_out_end);
		
		hull_traces_out<<dataLibrary::Lines[i].begin.x<<"\t"<<dataLibrary::Lines[i].begin.y<<"\t"<<dataLibrary::Lines[i].begin.z<<"\t"<<dataLibrary::Lines[i].end.x<<"\t"<<dataLibrary::Lines[i].end.y<<"\t"<<dataLibrary::Lines[i].end.z<<"\t"<<point_out_begin(0)<<"\t"<<point_out_begin(1)<<"\t"<<point_out_end(0)<<"\t"<<point_out_end(1)<<"\n";
    }
    hull_traces_out<<flush;
    hull_traces_out.close();

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
    
    if(is_success)
    {
        emit SaveTraceMapReady(this->getFileName());
        if(!this->getMuteMode()){
            emit ShowTraceMap();
        }
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