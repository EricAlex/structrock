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
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include "SaveMeshWorker.h"
#include "gmsh_io.h"
#include "globaldef.h"
#include "dataLibrary.h"

using namespace std;

bool SaveMeshWorker::is_para_satisfying(QString &message)
{
	if((dataLibrary::cloud_hull_all->size() == 0)||(dataLibrary::Lines.size() == 0))
	{
		message = QString("savemesh: You Haven't Saved Any Clusters Yet! Please Save Clusters First!");
		return false;
	}
	else
	{
		this->setParaSize(2);
		if(dataLibrary::Workflow[dataLibrary::current_workline_index].parameters.size()>1)
		{
			this->setFileName(QString::fromUtf8(dataLibrary::Workflow[dataLibrary::current_workline_index].parameters[0].c_str()));
			if(dataLibrary::Workflow[dataLibrary::current_workline_index].parameters[1] == "linear")
            {
                this->setApertureLengthModel(A_L_MODEL_LINEAR);
                this->setParaIndex(this->getParaSize());
                if((dataLibrary::Workflow[dataLibrary::current_workline_index].parameters.size()>this->getParaIndex())&&(dataLibrary::isOnlyDouble(dataLibrary::Workflow[dataLibrary::current_workline_index].parameters[this->getParaIndex()].c_str())))
				{
					double coefficient;
					std::stringstream ss(dataLibrary::Workflow[dataLibrary::current_workline_index].parameters[this->getParaIndex()]);
					ss >> coefficient;
					this->setProportionalityCoefficient(coefficient);
					this->setParaIndex(this->getParaIndex()+1);
				}
				else
				{
					message = QString("savemesh: The Proportionality Coefficient (double) Is Needed For the 'linear' Model.");
					return false;
				}
            }
            else if(dataLibrary::Workflow[dataLibrary::current_workline_index].parameters[1] == "sqrt")
            {
                this->setApertureLengthModel(A_L_MODEL_SQRT);
                this->setParaIndex(this->getParaSize());
                if((dataLibrary::Workflow[dataLibrary::current_workline_index].parameters.size()>this->getParaIndex())&&(dataLibrary::isOnlyDouble(dataLibrary::Workflow[dataLibrary::current_workline_index].parameters[this->getParaIndex()].c_str())))
				{
					double coefficient;
					std::stringstream ss(dataLibrary::Workflow[dataLibrary::current_workline_index].parameters[this->getParaIndex()]);
					ss >> coefficient;
					this->setProportionalityCoefficient(coefficient);
					this->setParaIndex(this->getParaIndex()+1);
				}
				else
				{
					message = QString("savemesh: The Proportionality Coefficient (double) Is Needed For the 'sqrt' Model.");
					return false;
				}
            }
            else
            {
                message = QString("savemesh: Please Provide the Right Aperture Length Model!");
			    return false;
            }
			return true;
		}
		else
		{
			message = QString("savemesh: Path and/or Aperture Length Model Not Provided.");
			return false;
		}
	}
}

void SaveMeshWorker::prepare()
{
	this->setUnmute();
	this->setWriteLog();
	this->check_mute_nolog();
}

void SaveMeshWorker::doWork()
{
	bool is_success(false);

    QByteArray ba = this->getFileName().toLocal8Bit();
    string* strfilename = new string(ba.data());
    
    dataLibrary::Status = STATUS_SAVEMESH;

    this->timer_start();
    
	//begin of processing
	string inside_nodes_filename = strfilename->substr(0, strfilename->size()-4) += "_inside_nodes.txt";
	ofstream inside_nodes_out(inside_nodes_filename.c_str());
	string boundary_nodes_filename = strfilename->substr(0, strfilename->size()-4) += "_boundary_nodes.txt";
	ofstream boundary_nodes_out(boundary_nodes_filename.c_str());
	std::vector<Eigen::Vector3f> nodes, edges;
	Eigen::Vector3f vertical_3d;
	vertical_3d << 0.0, 0.0, 1.0;
	Eigen::Vector3f V_x = dataLibrary::plane_normal_all.cross(vertical_3d);
	Eigen::Vector3f V_y = dataLibrary::plane_normal_all.cross(V_x);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull_all_expanded (new pcl::PointCloud<pcl::PointXYZ>);
	Eigen::Vector4f xyz_centroid_all;
    pcl::compute3DCentroid(*dataLibrary::cloud_hull_all, xyz_centroid_all);
	Eigen::Vector3f hull_all_centroid;
	hull_all_centroid << xyz_centroid_all(0), xyz_centroid_all(1), xyz_centroid_all(2);
	double ratio = 0.01;
	for(int i=0; i<dataLibrary::cloud_hull_all->size(); i++)
	{
		Eigen::Vector3f vertice = dataLibrary::cloud_hull_all->at(i).getVector3fMap();
		Eigen::Vector3f vertice_new = (1+ratio)*vertice - ratio*hull_all_centroid;
		pcl::PointXYZ point;
        point.x = vertice_new(0);
        point.y = vertice_new(1);
        point.z = vertice_new(2);
        cloud_hull_all_expanded->push_back(point);
	}
	std::vector<Eigen::Vector2f> convex_hull_all_2d;
    dataLibrary::projection322(V_x, V_y, cloud_hull_all_expanded, convex_hull_all_2d);
	for(int i=0; i<convex_hull_all_2d.size(); i++)
	{
		Eigen::Vector3f temp_node, temp_edge;
		temp_node << convex_hull_all_2d[i](0), convex_hull_all_2d[i](1), 0;
		nodes.push_back(temp_node);
		boundary_nodes_out << convex_hull_all_2d[i](0) << "\t" << convex_hull_all_2d[i](1) << "\n";
		temp_edge << i, (i+1)%convex_hull_all_2d.size(), 0;
		edges.push_back(temp_edge);
	}
    for(int i=0; i<dataLibrary::Lines.size(); i++)
    {
        if(dataLibrary::Lines[i].ID.find("Line_in") != std::string::npos)
        {
			std::vector<Eigen::Vector3f> inside_nodes;
			Eigen::Vector3f point_in_begin, point_in_end;
			Eigen::Vector2f point_out;
			point_in_begin << dataLibrary::Lines[i].begin.x, dataLibrary::Lines[i].begin.y, dataLibrary::Lines[i].begin.z;
			point_in_end << dataLibrary::Lines[i].end.x, dataLibrary::Lines[i].end.y, dataLibrary::Lines[i].end.z;
			float length = std::sqrt((point_in_begin-point_in_end).dot(point_in_begin-point_in_end));
			Eigen::Vector3f aperture_dir = dataLibrary::plane_normal_all.cross(point_in_begin-point_in_end);
			aperture_dir = aperture_dir/std::sqrt(aperture_dir.dot(aperture_dir));
			double aperture = 0.0;
			if(this->getApertureLengthModel() == A_L_MODEL_LINEAR)
			{
				aperture = this->getProportionalityCoefficient()*length;
			}
			else if(this->getApertureLengthModel() == A_L_MODEL_SQRT)
			{
				aperture = this->getProportionalityCoefficient()*std::sqrt(length);
			}
			int index_begin = nodes.size();
			int trip_begin_index;
			Eigen::Vector3f loose_end, trip_begin;
			bool has_trip_begin(false);
			Eigen::Vector3f out_begin_1, out_end_1;
			Eigen::Vector3f in_begin = point_in_begin + aperture/2*aperture_dir;
			Eigen::Vector3f in_end = point_in_end + aperture/2*aperture_dir;
			if(dataLibrary::edge_inside_part(dataLibrary::plane_normal_all, dataLibrary::cloud_hull_all, in_begin, in_end, out_begin_1, out_end_1))
			{
				dataLibrary::projection322(V_x, V_y, out_begin_1, point_out);
				Eigen::Vector3f temp_node_begin;
				temp_node_begin << point_out(0), point_out(1), 0;
				nodes.push_back(temp_node_begin);
				inside_nodes.push_back(temp_node_begin);
				dataLibrary::projection322(V_x, V_y, out_end_1, point_out);
				Eigen::Vector3f temp_node_end;
				temp_node_end << point_out(0), point_out(1), 0;
				nodes.push_back(temp_node_end);
				inside_nodes.push_back(temp_node_end);
				has_trip_begin = true;
				trip_begin = out_begin_1;
				trip_begin_index = index_begin;
				loose_end = out_end_1;
				
				Eigen::Vector3f temp_edge;
				temp_edge << index_begin, index_begin + 1, 0;
				edges.push_back(temp_edge);
				index_begin++;
			}
			Eigen::Vector3f out_begin_2, out_end_2;
			in_begin = point_in_end + aperture/2*aperture_dir;
			in_end = point_in_end - aperture/2*aperture_dir;
			if(dataLibrary::edge_inside_part(dataLibrary::plane_normal_all, dataLibrary::cloud_hull_all, in_begin, in_end, out_begin_2, out_end_2))
			{
				if(!has_trip_begin)
				{
					dataLibrary::projection322(V_x, V_y, out_begin_2, point_out);
					Eigen::Vector3f temp_node_begin;
					temp_node_begin << point_out(0), point_out(1), 0;
					nodes.push_back(temp_node_begin);
					inside_nodes.push_back(temp_node_begin);
					dataLibrary::projection322(V_x, V_y, out_end_2, point_out);
					Eigen::Vector3f temp_node_end;
					temp_node_end << point_out(0), point_out(1), 0;
					nodes.push_back(temp_node_end);
					inside_nodes.push_back(temp_node_end);
					has_trip_begin = true;
					trip_begin = out_begin_2;
					trip_begin_index = index_begin;
					loose_end = out_end_2;
				
					Eigen::Vector3f temp_edge;
					temp_edge << index_begin, index_begin + 1, 0;
					edges.push_back(temp_edge);
					index_begin++;
				}
				else
				{
					if(out_begin_2 != loose_end)
					{
						dataLibrary::projection322(V_x, V_y, out_begin_2, point_out);
						Eigen::Vector3f temp_node;
						temp_node << point_out(0), point_out(1), 0;
						nodes.push_back(temp_node);
						inside_nodes.push_back(temp_node);
				
						Eigen::Vector3f temp_edge;
						temp_edge << index_begin, index_begin + 1, 0;
						edges.push_back(temp_edge);
						index_begin++;
					}
					dataLibrary::projection322(V_x, V_y, out_end_2, point_out);
					Eigen::Vector3f temp_node;
					temp_node << point_out(0), point_out(1), 0;
					nodes.push_back(temp_node);
					inside_nodes.push_back(temp_node);

					Eigen::Vector3f temp_edge;
					temp_edge << index_begin, index_begin + 1, 0;
					edges.push_back(temp_edge);
					index_begin++;

					loose_end = out_end_2;
				}
			}
			Eigen::Vector3f out_begin_3, out_end_3;
			in_begin = point_in_end - aperture/2*aperture_dir;
			in_end = point_in_begin - aperture/2*aperture_dir;
			if(dataLibrary::edge_inside_part(dataLibrary::plane_normal_all, dataLibrary::cloud_hull_all, in_begin, in_end, out_begin_3, out_end_3))
			{
				if(out_begin_3 != loose_end)
				{
					dataLibrary::projection322(V_x, V_y, out_begin_3, point_out);
					Eigen::Vector3f temp_node;
					temp_node << point_out(0), point_out(1), 0;
					nodes.push_back(temp_node);
					inside_nodes.push_back(temp_node);
				
					Eigen::Vector3f temp_edge;
					temp_edge << index_begin, index_begin + 1, 0;
					edges.push_back(temp_edge);
					index_begin++;
				}
				dataLibrary::projection322(V_x, V_y, out_end_3, point_out);
				Eigen::Vector3f temp_node;
				temp_node << point_out(0), point_out(1), 0;
				nodes.push_back(temp_node);
				inside_nodes.push_back(temp_node);

				Eigen::Vector3f temp_edge;
				temp_edge << index_begin, index_begin + 1, 0;
				edges.push_back(temp_edge);
				index_begin++;

				loose_end = out_end_3;
			}
			Eigen::Vector3f out_begin_4, out_end_4;
			in_begin = point_in_begin - aperture/2*aperture_dir;
			in_end = point_in_begin + aperture/2*aperture_dir;
			if(dataLibrary::edge_inside_part(dataLibrary::plane_normal_all, dataLibrary::cloud_hull_all, in_begin, in_end, out_begin_4, out_end_4))
			{
				if(out_begin_4 != loose_end)
				{
					dataLibrary::projection322(V_x, V_y, out_begin_4, point_out);
					Eigen::Vector3f temp_node;
					temp_node << point_out(0), point_out(1), 0;
					nodes.push_back(temp_node);
					inside_nodes.push_back(temp_node);
				
					Eigen::Vector3f temp_edge;
					temp_edge << index_begin, index_begin + 1, 0;
					edges.push_back(temp_edge);
					index_begin++;
				}
				if(out_end_4 != trip_begin)
				{
					dataLibrary::projection322(V_x, V_y, out_end_4, point_out);
					Eigen::Vector3f temp_node;
					temp_node << point_out(0), point_out(1), 0;
					nodes.push_back(temp_node);
					inside_nodes.push_back(temp_node);
				
					Eigen::Vector3f temp_edge;
					temp_edge << index_begin, index_begin + 1, 0;
					edges.push_back(temp_edge);
					index_begin++;
				}
				Eigen::Vector3f temp_edge;
				temp_edge << index_begin, trip_begin_index, 0;
				edges.push_back(temp_edge);
			}

			// write inside nodes here
			inside_nodes_out << inside_nodes.size() << "\n";
			for(int i=0; i<inside_nodes.size(); i++)
			{
				inside_nodes_out << inside_nodes[i](0) << "\t" << inside_nodes[i](1) << "\n";
			}
		}
	}
	inside_nodes_out<<flush;
	inside_nodes_out.close();
	boundary_nodes_out<<flush;
	boundary_nodes_out.close();
	//write nodes and edges to msh file
	ofstream fout(strfilename->c_str());
	fout << "mshid=1" << "\n" << "ndims=2" << "\n";
	fout << "point=" << nodes.size() << "\n";
	for(int i=0; i<nodes.size(); i++)
	{
		fout << nodes[i](0) << ";" << nodes[i](1) << ";" << nodes[i](2) << "\n";
	}
	fout << "edge2=" << edges.size() << "\n";
	for(int i=0; i<edges.size(); i++)
	{
		fout << edges[i](0) << ";" << edges[i](1) << ";" << edges[i](2) << "\n";
	}
	fout<<flush;
    fout.close();
	
    /*int *element_node;
    int element_num;
    int element_order;
    int m;
    int node_num;
    double *node_x;
    //
    //  Get sizes.
    //
    gmsh_io::gmsh_mesh2d_node_size_example ( node_num, m );
    
    gmsh_io::gmsh_mesh2d_element_size_example ( element_num, element_order );
    //
    //  Get the data.
    //
    node_x = gmsh_io::gmsh_mesh2d_node_data_example ( node_num, m );
    
    element_node = gmsh_io::gmsh_mesh2d_element_data_example ( element_num, element_order );
    //
    //  Write the GMSH file.
    //
    gmsh_io::gmsh_mesh2d_write ( *strfilename, m, node_num, node_x, element_order, element_num, element_node );
    
    //
    //  Clean up.
    //
    delete [] element_node;
    delete [] node_x;*/

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