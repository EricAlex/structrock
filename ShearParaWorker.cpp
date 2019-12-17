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
#include <algorithm>
#include <cmath>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
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
#include "omp.h"
#include "globaldef.h"
#include "dataLibrary.h"
#include "ShearParaWorker.h"

using namespace std;

bool ShearParaWorker::is_para_satisfying(QString &message)
{
	if(dataLibrary::Fracture_Triangles.size() == 0)
	{
		message = QString("shearpara: You Haven't Performed Fracture Triangulation Yet!");
		return false;
	}
	else
	{
		this->setParaSize(1);
		if(dataLibrary::Workflow[dataLibrary::current_workline_index].parameters.size()>=this->getParaSize())
		{
			this->setFileName(QString::fromUtf8(dataLibrary::Workflow[dataLibrary::current_workline_index].parameters[0].c_str()));
			this->setParaIndex(this->getParaSize());
			return true;
		}
		else
		{
			message = QString("shearpara: No Saving Path Given.");
			return false;
		}
	}
}

void ShearParaWorker::prepare()
{
    this->setSaveScreenMode(false);
    if((dataLibrary::Workflow[dataLibrary::current_workline_index].parameters.size()>this->getParaIndex())&&(dataLibrary::Workflow[dataLibrary::current_workline_index].parameters[this->getParaIndex()] == "savescreen"))
	{
		this->setSaveScreenMode(true);
		this->setParaIndex(this->getParaIndex()+1);
	}
	this->setUnmute();
	this->setWriteLog();
	this->check_mute_nolog();
}

Eigen::Vector3f PolygonNormal(const pcl::PointCloud<pcl::PointXYZ> &polygon)
{
    Eigen::Vector3f vec01(polygon.at(0).x-polygon.at(1).x,polygon.at(0).y-polygon.at(1).y,polygon.at(0).z-polygon.at(1).z); 
    Eigen::Vector3f vec21(polygon.at(2).x-polygon.at(1).x,polygon.at(2).y-polygon.at(1).y,polygon.at(2).z-polygon.at(1).z); 
    Eigen::Vector3f normal = vec01.cross(vec21);
    normal.normalize();
    return normal;
}

float PolygonArea(const pcl::PointCloud<pcl::PointXYZ> &polygon, Eigen::Vector3f normal)
{
    int num_points = polygon.size();
    int j = 0;
    Eigen::Vector3f va, vb, res;
    res(0) = res(1) = res(2) = 0.0f;
    for(int i = 0; i < num_points; i++)
    {
        j = (i+1) % num_points;
        va = polygon.at(i).getVector3fMap();
        vb = polygon.at(j).getVector3fMap();
        res += va.cross(vb);
    }
    return fabs(res.dot(normal) * 0.5);
}

float ApparentDipAngle(const pcl::PointCloud<pcl::PointXYZ> &polygon, Eigen::Vector3f shearnorm, Eigen::Vector3f sheardir)
{
    Eigen::Vector3f polynormal = PolygonNormal(polygon);
    float theta = acos(fabs(shearnorm.dot(polynormal))/(shearnorm.norm()*polynormal.norm()));
    Eigen::Vector3f proj_norm = polynormal - (polynormal.dot(shearnorm)/shearnorm.norm())*shearnorm;
    float det = shearnorm.dot(sheardir.cross(proj_norm));
    float alpha = atan2(det, sheardir.dot(proj_norm));
    return atan(-tan(theta)*cos(alpha));
}

void ShearParaWorker::doWork()
{
	bool is_success(false);

    QByteArray ba = this->getFileName().toLocal8Bit();
    string* strfilename = new string(ba.data());

    dataLibrary::Status = STATUS_SHEARPARA;

    this->timer_start();

	//begin of processing
    if(this->getSaveScreenMode())
    {
        emit prepare_2_s_f();

        for(int i=0; i<dataLibrary::Fracture_Triangles.size(); i++)
        {
            std::ostringstream strs;
            strs << i;
            string screen_png = *strfilename + "_" + strs.str() +"_.png";
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::fromPCLPointCloud2(dataLibrary::Fracture_Triangles[i]->cloud, *cloud_ptr);
            dataLibrary::fracture_patches.push_back(cloud_ptr);
            emit show_f_save_screen(QString::fromUtf8(screen_png.c_str()));
            this->Sleep(1000);
        }
    }

    vector<float> dip_arr, dip_dir_arr;
    vector<int> is_large_enough_arr;
    dip_arr.resize(dataLibrary::Fracture_Triangles.size(), 0.0);
    dip_dir_arr.resize(dataLibrary::Fracture_Triangles.size(), 0.0);
    is_large_enough_arr.resize(dataLibrary::Fracture_Triangles.size(), 0);
    for(int i=0; i<dataLibrary::Fracture_Triangles.size(); i++){
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromPCLPointCloud2(dataLibrary::Fracture_Triangles[i]->cloud, *cloud_ptr);
        // determine the shear plane, P

        /*float nx, ny, nz, curvature;
        Eigen::Matrix3f convariance_matrix;
        Eigen::Vector4f xyz_centroid;
        pcl::compute3DCentroid(*cloud_ptr, xyz_centroid);
        pcl::computeCovarianceMatrix(*cloud_ptr, xyz_centroid, convariance_matrix);
        pcl::solvePlaneParameters(convariance_matrix, nx, ny, nz, curvature);
        Eigen::Vector3f plane_normal;
        plane_normal(0)=nx/sqrt(nx*nx+ny*ny+nz*nz);
        plane_normal(1)=ny/sqrt(nx*nx+ny*ny+nz*nz);
        plane_normal(2)=nz/sqrt(nx*nx+ny*ny+nz*nz);*/

        float nx, ny, nz;
        Eigen::Vector4f plane_normal_param = dataLibrary::fitPlaneManually(*cloud_ptr);
        nx = plane_normal_param(0);
        ny = plane_normal_param(1);
        nz = plane_normal_param(2);
        Eigen::Vector3f plane_normal;
        plane_normal << nx, ny, nz;

        is_large_enough_arr[i] = cloud_ptr->points.size();

        float dip_direction, dip;
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
        dip_arr[i] = dip;
        dip_dir_arr[i] = dip_direction;

        // shear directions parallel to P
        std::vector<Eigen::Vector3f> sdirections;
        Eigen::Vector3f original;
        if((nx==0)&&(ny==0))
        {
            original(0)=1.0;
            original(1)=0.0;
            original(2)=0.0;
        }
        else
        {
            original(0)=-plane_normal(1);
            original(1)=plane_normal(0);
            original(2)=0.0;
        }
	    for(float j=-TWOPI/2; j<-0.001; j+=TWOPI/72)
	    {
		    float x = plane_normal(0);
		    float y = plane_normal(1);
		    float z = plane_normal(2);
		    float c = std::cos(j);
		    float s = std::sqrt(1-c*c);
		    float C = 1-c;
		    Eigen::Matrix3f rmat;
		    rmat<<x*x*C+c,x*y*C-z*s,x*z*C+y*s,y*x*C+z*s,y*y*C+c,y*z*C-x*s,z*x*C-y*s,z*y*C+x*s,z*z*C+c;
            Eigen::Vector3f rotated=rmat*original;
            rotated.normalize();
		    sdirections.push_back(rotated);
        }
        int halfsize = sdirections.size();
        for(int j=0; j<halfsize; j++)
        {
            Eigen::Vector3f temp_new;
            temp_new(0) = -sdirections[j](0);
            temp_new(1) = -sdirections[j](1);
            temp_new(2) = -sdirections[j](2);
            sdirections.push_back(temp_new);
        }
        vector<float> SP_arr, reci_big_c_arr, theta_max_arr;
        SP_arr.resize(sdirections.size(), 0.0);
        reci_big_c_arr.resize(sdirections.size(), 0.0);
        theta_max_arr.resize(sdirections.size(), 0.0);
        #pragma omp parallel for
        for(int j=0; j<sdirections.size(); j++){
            float A_0, theta_max;
            A_0 = 0.0;
            std::vector< std::pair<float, float> > ADAs_As;
            std::vector<float> theta_cr, A_theta_cr, Big_Y, Big_X;
            for(int k=0; k<dataLibrary::Fracture_Triangles[i]->polygons.size(); k++)
            {
                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_polygon(new pcl::PointCloud<pcl::PointXYZ>);
                for(int r=0; r<dataLibrary::Fracture_Triangles[i]->polygons[k].vertices.size(); r++)
                {
                    cloud_polygon->push_back(cloud_ptr->at(dataLibrary::Fracture_Triangles[i]->polygons[k].vertices[r]));
                }
                float apparentDA = ApparentDipAngle(*cloud_polygon, plane_normal, sdirections[j]);
                if(apparentDA>0)
                {
                    Eigen::Vector3f trinormal = PolygonNormal(*cloud_polygon);
                    float temp_area = PolygonArea(*cloud_polygon, trinormal);
                    ADAs_As.push_back(make_pair(apparentDA, temp_area));
                    A_0 += temp_area;
                }
            }
            std::sort(ADAs_As.begin(), ADAs_As.end());
            theta_max = ADAs_As.back().first;
            float temp_cr[18] = {0.0, 1.2, 2.5, 3.9, 5.32, 6.82, 8.41, 10.0, 12.5, 15.0, 17.5, 20.0, 23.0, 26.3, 30.0, 34.0, 40.0, 50.0};
            for(int k=0; k<18; k++)
            {
                if((temp_cr[k]/360*TWOPI)<theta_max)
                    theta_cr.push_back(temp_cr[k]/360*TWOPI);
            }
            float sum_X = 0.0;
            float sum_Y = 0.0;
            for(int k=0; k<theta_cr.size(); k++)
            {
                float temp_X = log((theta_max-theta_cr[k])/theta_max);
                Big_X.push_back(temp_X);
                sum_X += temp_X;
                float temp_A_theta_cr = 0.0;
                int r=ADAs_As.size()-1;
                while((r>=0)&&(ADAs_As[r].first>=theta_cr[k]))
                {
                    temp_A_theta_cr += ADAs_As[r].second;
                    r--;
                }
                A_theta_cr.push_back(temp_A_theta_cr);
                float temp_Y = log(temp_A_theta_cr);
                Big_Y.push_back(temp_Y);
                sum_Y += temp_Y;
            }
            float mean_X = sum_X/Big_X.size();
            float mean_Y = sum_Y/Big_Y.size();
            float S_xy = 0.0;
            float S_x = 0.0;
            for(int k=0; k<Big_X.size(); k++)
            {
                S_x += (Big_X[k] - mean_X)*(Big_X[k] - mean_X);
                S_xy += (Big_X[k] - mean_X)*(Big_Y[k] - mean_Y);
            }
            float SP = (theta_max*S_x/S_xy)*360/TWOPI;
            float reci_big_c = S_x/S_xy;
            SP_arr[j] = SP;
            reci_big_c_arr[j] = reci_big_c;
            theta_max_arr[j] = theta_max;
        }
        std::ostringstream strs;
        strs << i;
        string textfilename = *strfilename + "_" + strs.str() +"_.txt";
        string reci_big_c_filename = *strfilename + "_" + strs.str() +"_reciprocal_C_.txt";
        string theta_max_filename = *strfilename + "_" + strs.str() +"_theta_max_.txt";
        ofstream fout(textfilename.c_str());
        ofstream reci_big_c_fout(reci_big_c_filename.c_str());
        ofstream theta_max_fout(theta_max_filename.c_str());
        for(int j=0; j<sdirections.size(); j++){
            fout<<SP_arr[j]*sdirections[j](0)<<"\t"<<SP_arr[j]*sdirections[j](1)<<"\t"<<SP_arr[j]*sdirections[j](2)<<"\t"<<SP_arr[j]<<"\n";
            reci_big_c_fout<<reci_big_c_arr[j]*sdirections[j](0)<<"\t"<<reci_big_c_arr[j]*sdirections[j](1)<<"\t"<<reci_big_c_arr[j]*sdirections[j](2)<<"\t"<<reci_big_c_arr[j]<<"\n";
            theta_max_fout<<theta_max_arr[j]*sdirections[j](0)<<"\t"<<theta_max_arr[j]*sdirections[j](1)<<"\t"<<theta_max_arr[j]*sdirections[j](2)<<"\t"<<theta_max_arr[j]<<"\n";
        }
        fout<<flush;
        fout.close();
        reci_big_c_fout<<flush;
        reci_big_c_fout.close();
        theta_max_fout<<flush;
        theta_max_fout.close();
    }
    string dip_dipdir_file = *strfilename + "_dip_dipdir.txt";
    ofstream dip_dipdir_out(dip_dipdir_file.c_str());
    string is_large_enough = *strfilename + "_is_large_enough.txt";
    ofstream is_large_enough_out(is_large_enough.c_str());
    for(int i=0; i<dataLibrary::Fracture_Triangles.size(); i++){
        dip_dipdir_out<<dip_arr[i]<<'\t'<<dip_dir_arr[i]<<'\n';
        is_large_enough_out<<is_large_enough_arr[i]<<'\n';
    }
	dip_dipdir_out<<flush;
    dip_dipdir_out.close();
    is_large_enough_out<<flush;
    is_large_enough_out.close();
    is_success = true;
	//end of processing

    this->timer_stop();

    if(this->getWriteLogMode()&&is_success)
    {
        std::string log_text = "\tFracture Shear Parameter Estimation costs: ";
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