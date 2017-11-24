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
#include <pcl/surface/concave_hull.h>
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

bool SaveClustersWorker::is_para_satisfying(QString message)
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
					message = QString("saveclusters: The Expand Ratio (double) Is Needed For the 'circular' and 'rectangular' Options.");
					return false;
				}
			}
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

void SaveClustersWorker::doWork()
{
	bool is_success(false);

    QByteArray ba = this->getFileName().toLocal8Bit();
    string* strfilename = new string(ba.data());
    
    dataLibrary::Status = STATUS_SAVECLUSTERS;

    this->timer_start();
    
	//begin of processing
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

	//calculate total surface roughness of outcrop
	float total_distance=0.0;
	for(int i=0; i<dataLibrary::cloudxyz->size(); i++)
	{
		Eigen::Vector3f Q;
		Q(0)=dataLibrary::cloudxyz->at(i).x;
		Q(1)=dataLibrary::cloudxyz->at(i).y;
		Q(2)=dataLibrary::cloudxyz->at(i).z;
		total_distance+=std::abs((Q-centroid_all).dot(dataLibrary::plane_normal_all)/std::sqrt((dataLibrary::plane_normal_all.dot(dataLibrary::plane_normal_all))));
	}
	float roughness=total_distance/dataLibrary::cloudxyz->size();
    
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
    chull_all.reconstruct(*dataLibrary::cloud_hull_all);

	/*//compute concave hull
    pcl::ConcaveHull<pcl::PointXYZ> chull_all;
    chull_all.setInputCloud(cloud_projected_all);
	chull_all.setAlpha(0.1);
    chull_all.reconstruct(*dataLibrary::cloud_hull_all);*/
    
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
    
    //initial total length of fracture traces
    float total_length=0.0;
	//initial over estimate length of fracture traces
	float error_length=0.0;
	//initial total displacement
	float total_displacement=0.0;
	//initial mean dip2plane angle
	float total_dip2plane=0.0;
	int inside_num=0;
    
    string textfilename = strfilename->substr(0, strfilename->size()-4) += "_table.txt";
    string dip_dipdir_file = strfilename->substr(0, strfilename->size()-4) += "_dip_dipdir.txt";
    string dipdir_dip_file = strfilename->substr(0, strfilename->size()-4) += "_dipdir_dip.txt";
	string area_file = strfilename->substr(0, strfilename->size()-4) += "_area.txt";
	string roughness_file = strfilename->substr(0, strfilename->size()-4) += "_roughness.txt";
    string fracture_intensity = strfilename->substr(0, strfilename->size()-4) += "_fracture_intensity.txt";
    ofstream fout(textfilename.c_str());
    ofstream dip_dipdir_out(dip_dipdir_file.c_str());
    ofstream dipdir_dip_out(dipdir_dip_file.c_str());
	ofstream area_out(area_file.c_str());
	ofstream roughness_out(roughness_file.c_str());
    ofstream fracture_intensity_out(fracture_intensity.c_str());
    fout<<"Flag"<<"\t"<<"Number"<<"\t"<<"Points"<<"\t"<<"Direc"<<"\t"<<"Dip"<<"\t"<<"Area"<<"\t"<<"Length"<<"\t"<<"Roughness"<<"\n";
    
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
        
        //project data onto plane
        //set plane parameter
        pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients());
        coefficients->values.resize(4);
        coefficients->values[0] = nx;
        coefficients->values[1] = ny;
        coefficients->values[2] = nz;
        coefficients->values[3] = - (nx*xyz_centroid[0] + ny*xyz_centroid[1] + nz*xyz_centroid[2]);
        //projecting
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_projected (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::ProjectInliers<pcl::PointXYZ> proj;
        proj.setModelType(pcl::SACMODEL_PLANE);
        proj.setInputCloud(plane_cloud);
        proj.setModelCoefficients(coefficients);
        proj.filter(*cloud_projected);
        
        //generate a concave or convex
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::ConvexHull<pcl::PointXYZ> chull;
        chull.setInputCloud(cloud_projected);
        chull.reconstruct(*cloud_hull);
        
        //calculate polygon area
        Eigen::Vector3f normal;
        normal(0) = nx;
        normal(1) = ny;
        normal(2) = nz;
        
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

		float length;
        bool flag;
		if(this->getFMAP_Mode() == FMAP_LOWER_BOUND)
			flag = dataLibrary::LowerBound(dataLibrary::plane_normal_all, centroid_all, dataLibrary::cloud_hull_all, normal, centroid, cloud_hull, cluster_index, length, this->getTrimTraceEdgesMode(), false);
		else if(this->getFMAP_Mode() == FMAP_RECTANGULAR)
			flag = dataLibrary::Rectangular(dataLibrary::plane_normal_all, centroid_all, dataLibrary::cloud_hull_all, normal, centroid, cloud_projected, cluster_index, length, this->getTrimTraceEdgesMode(), false);
        fout<<flag<<"\t"<<cluster_index+1<<"\t"<<dataLibrary::clusters[cluster_index].indices.size()<<"\t"<<dip_direction<<"\t"<<dip<<"\t"<<area<<"\t"<<length<<"\t"<<fracture_roughness<<"\n";

		//calculate displacement
		Eigen::Vector3f line_direction = normal.cross(dataLibrary::plane_normal_all.cross(normal));
		if((line_direction(0) == 0)&&(line_direction(1) == 0)&&(line_direction(2) == 0))
		{
			total_displacement+=0.0;
		}
		else
		{
			if(flag)
			{
				float max_value, min_value;
				int max_index, min_index;
				Eigen::Vector3f point;
				point(0) = cloud_projected->at(0).x;
				point(1) = cloud_projected->at(0).y;
				point(2) = cloud_projected->at(0).z;
				float mod_line_direction = std::sqrt(line_direction.dot(line_direction));
				max_value = min_value = line_direction.dot(point - centroid)/mod_line_direction;
				max_index = min_index = 0;
				for(int i=1; i<cloud_projected->size(); i++)
				{
					Eigen::Vector3f point;
					point(0) = cloud_projected->at(i).x;
					point(1) = cloud_projected->at(i).y;
					point(2) = cloud_projected->at(i).z;
					
					float value = line_direction.dot(point - centroid)/mod_line_direction;
					
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
				float alpha = std::acos(std::abs(dataLibrary::plane_normal_all.dot(normal)/(std::sqrt(dataLibrary::plane_normal_all.dot(dataLibrary::plane_normal_all))*std::sqrt(normal.dot(normal)))));
				total_dip2plane+=alpha*360.0/TWOPI;
				inside_num+=1;
				float tangent_alpha = std::tan(alpha);
				float height_max = std::abs(dataLibrary::plane_normal_all.dot(cloud_projected->at(max_index).getVector3fMap()-centroid_all)/std::sqrt(dataLibrary::plane_normal_all.dot(dataLibrary::plane_normal_all)));
				float height_min = std::abs(dataLibrary::plane_normal_all.dot(cloud_projected->at(min_index).getVector3fMap()-centroid_all)/std::sqrt(dataLibrary::plane_normal_all.dot(dataLibrary::plane_normal_all)));
				float displacement_max = height_max/tangent_alpha;
				float displacement_min = height_min/tangent_alpha;
				total_displacement += (displacement_max+displacement_min)/2.0;
			}
		}
        
        if(flag)
        {
            total_length += length;
            dip_dipdir_out<<dip<<"\t"<<dip_direction<<"\n";
            dipdir_dip_out<<dip_direction<<"\t"<<dip<<"\n";
			area_out<<area<<"\n";
			roughness_out<<fracture_roughness<<"\n";

			dataLibrary::out_dips.push_back(dip);
			dataLibrary::out_dip_directions.push_back(dip_direction);

			dataLibrary::selectedPatches.push_back(cluster_index);
        }
		else
		{
			error_length += length;
		}
        fbinaryout.write(reinterpret_cast<const char*>(&dip), sizeof(dip));
        fbinaryout.write(reinterpret_cast<const char*>(&dip_direction), sizeof(dip_direction));
        fbinaryout.write(reinterpret_cast<const char*>(&area), sizeof(area));
    }
	fracture_intensity_out<<"Outcrop surface roughness:"<<"\t"<<roughness<<"\n";
	fracture_intensity_out<<"Total area:"<<"\t"<<area_all<<"\n";
	fracture_intensity_out<<"Percentage of displacement errors:"<<"\t"<<total_displacement/total_length*100<<" \%\n";
	fracture_intensity_out<<"Percentage of over estimated errors:"<<"\t"<<error_length/total_length*100<<" \%\n";
	fracture_intensity_out<<"Mean dip to plane angle:"<<"\t"<<total_dip2plane/inside_num<<"\n";
    fracture_intensity_out<<"Fracture Density:"<<"\t"<<total_length/area_all;
    fout<<flush;
    fout.close();
    dip_dipdir_out<<flush;
    dip_dipdir_out.close();
    dipdir_dip_out<<flush;
    dipdir_dip_out.close();
	area_out<<flush;
    area_out.close();
	roughness_out<<flush;
    roughness_out.close();
    fracture_intensity_out<<flush;
    fracture_intensity_out.close();
    fbinaryout.close();
    
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

    if(this->getWriteLogMpde()&&is_success)
    {
        std::string log_text = "\tSaving Clusters costs: ";
        std::ostringstream strs;
        strs << this->getTimer_sec();
        log_text += (strs.str() +" seconds.");
        dataLibrary::write_text_to_log_file(log_text);
    }
    
    if(!this->getMuteMode()&&is_success)
    {
        emit SaveClustersReady(this->getFileName());
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