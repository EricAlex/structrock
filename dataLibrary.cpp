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

#include "dataLibrary.h"
#include "Miniball.hpp"
#include "globaldef.h"

using namespace std;

pcl::PointCloud<pcl::PointXYZ>::Ptr dataLibrary::cloudxyz(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr dataLibrary::cloudxyzrgb(new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZI>::Ptr dataLibrary::cloudxyzi(new pcl::PointCloud<pcl::PointXYZI>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr dataLibrary::cloudxyzrgb_clusters(new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr dataLibrary::cloudxyzrgb_features(new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::Normal>::Ptr dataLibrary::normal (new pcl::PointCloud<pcl::Normal>);
pcl::PointCloud<pcl::PointNormal>::Ptr dataLibrary::pointnormals(new pcl::PointCloud<pcl::PointNormal>);
pcl::PointCloud<pcl::PointXYZ>::Ptr dataLibrary::mls_points(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr dataLibrary::downsampledxyz (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr dataLibrary::outlier_removed_inlier(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr dataLibrary::outlier_removed_outlier(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr dataLibrary::temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr dataLibrary::segmentation_rem(new pcl::PointCloud<pcl::PointXYZ>);
std::vector<pcl::PointIndices> dataLibrary::clusters;
std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> dataLibrary::cluster_patches;
std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> dataLibrary::fracture_faces_hull;
std::vector<float> dataLibrary::dips;
std::vector<float> dataLibrary::dip_directions;
std::vector<float> dataLibrary::out_dips;
std::vector<float> dataLibrary::out_dip_directions;
std::vector<float> dataLibrary::areas;
std::vector<float> dataLibrary::roughnesses;
std::vector<std::string> dataLibrary::patchIDs;
int dataLibrary::currentPatch(0);
std::vector<int> dataLibrary::selectedPatches;
std::string dataLibrary::cloudID("");
int dataLibrary::Status(STATUS_READY);
std::vector<pcl::PolygonMesh::Ptr> dataLibrary::Fracture_Triangles;
pcl::PointCloud<pcl::PointXYZ>::Ptr dataLibrary::cloud_hull_all (new pcl::PointCloud<pcl::PointXYZ>);
std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> dataLibrary::fracture_patches;
std::vector<int> dataLibrary::fracture_classes;
std::vector<Vector3f> dataLibrary::fracture_classes_rgb;
Eigen::Vector3f dataLibrary::plane_normal_all;
std::vector<Line> dataLibrary::Lines;
std::vector<Line> dataLibrary::Lines_max;
std::vector<Line> dataLibrary::Lines_min;
std::vector<WorkLine> dataLibrary::Workflow;
int dataLibrary::current_workline_index = 0;
bool dataLibrary::have_called_read_file = false;
Vector3f dataLibrary::cloud_centor;

bool dataLibrary::isOnlyDouble(const char* str)
{
	char* endptr = 0;
	strtod(str, &endptr);

	if(*endptr != '\0' || endptr == str)
		return false;
	return true;
}
void dataLibrary::checkupflow()
{
	if(!temp_cloud->empty())
	{
		cloudxyz->clear();
		*cloudxyz = *temp_cloud;
		temp_cloud->clear();
	}
}
bool dataLibrary::haveBaseData()
{
	return !cloudxyz->empty();
}
void dataLibrary::clearall()
{
	if(!cloudxyz->empty())
		cloudxyz->clear();
	if(!cloudxyzrgb->empty())
		cloudxyzrgb->clear();
	if(!cloudxyzrgb_clusters->empty())
		cloudxyzrgb_clusters->clear();
	if(!normal->empty())
		normal->clear();
	if(!pointnormals->empty())
		pointnormals->clear();
	if(!mls_points->empty())
		mls_points->clear();
	if(!downsampledxyz->empty())
		downsampledxyz->clear();
	if(!outlier_removed_inlier->empty())
		outlier_removed_inlier->clear();
	if(!outlier_removed_outlier->empty())
		outlier_removed_outlier->clear();
	if(!temp_cloud->empty())
		temp_cloud->clear();
	if(!segmentation_rem->empty())
		segmentation_rem->clear();
	std::vector<pcl::PointIndices>().swap(clusters);
	std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>().swap(cluster_patches);
	std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>().swap(fracture_faces_hull);
	std::vector<float>().swap(dips);
	std::vector<float>().swap(dip_directions);
	std::vector<float>().swap(out_dips);
	std::vector<float>().swap(out_dip_directions);
	std::vector<float>().swap(areas);
	std::vector<std::string>().swap(patchIDs);
	currentPatch = 0;
	std::vector<int>().swap(selectedPatches);
    if(!cloud_hull_all->empty())
        cloud_hull_all->clear();
	std::vector<Line>().swap(Lines);
	std::vector<Line>().swap(Lines_max);
	std::vector<Line>().swap(Lines_min);
}

void dataLibrary::clearWorkFlow()
{
	std::vector<WorkLine>().swap(Workflow);
	current_workline_index=0;
}

bool dataLibrary::projection323(const Eigen::Vector3f &V1, const Eigen::Vector3f &V2, const Eigen::Vector3f &point_in, Eigen::Vector3f &point_out)
{
	Eigen::Vector3f V3 = V1.cross(V2);
    point_out(0)=point_in.dot(V1)/std::sqrt(V1.dot(V1));
    point_out(1)=point_in.dot(V2)/std::sqrt(V2.dot(V2));
    point_out(2)=point_in.dot(V3)/std::sqrt(V3.dot(V3));
    return true;
}

bool dataLibrary::projection322(const Eigen::Vector3f &V1, const Eigen::Vector3f &V2, const Eigen::Vector3f &point_in, Eigen::Vector2f &point_out)
{
    point_out(0)=point_in.dot(V1)/std::sqrt(V1.dot(V1));
    point_out(1)=point_in.dot(V2)/std::sqrt(V2.dot(V2));
    return true;
}

bool dataLibrary::projection322(const Eigen::Vector3f &V1, const Eigen::Vector3f &V2, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, std::vector<Eigen::Vector2f> &Points_out)
{
    Eigen::Vector3f point3d;
    for(int i=0; i<cloud_in->size(); i++)
    {
        point3d(0)=cloud_in->at(i).x;
        point3d(1)=cloud_in->at(i).y;
        point3d(2)=cloud_in->at(i).z;
        Eigen::Vector2f point2d;
        if(projection322(V1, V2, point3d, point2d))
        {
            Points_out.push_back(point2d);
        }
    }
    return true;
}

bool dataLibrary::dd2pole(float &dip_dir, float &dip, Eigen::Vector3f &pole)
{
	pole(0)=std::sin(TWOPI/2-dip)*std::cos(TWOPI*3/4-dip_dir);
	pole(1)=std::sin(TWOPI/2-dip)*std::sin(TWOPI*3/4-dip_dir);
	pole(2)=std::cos(TWOPI/2-dip);
	return true;
}

bool dataLibrary::dd2arc(float &dip_dir, float &dip, std::vector<Eigen::Vector3f> &arc)
{
	Eigen::Vector3f original;
	original(0)=std::cos(TWOPI/2-dip_dir);
	original(1)=std::sin(TWOPI/2-dip_dir);
	original(2)=0.0;
	Eigen::Vector3f pole;
	dd2pole(dip_dir, dip, pole);
	for(float i=-TWOPI/2; i<0.0; i+=TWOPI/360)
	{
		float x = pole(0);
		float y = pole(1);
		float z = pole(2);
		float c = std::cos(i);
		float s = std::sqrt(1-c*c);
		float C = 1-c;
		Eigen::Matrix3f rmat;
		rmat<<x*x*C+c,x*y*C-z*s,x*z*C+y*s,y*x*C+z*s,y*y*C+c,y*z*C-x*s,z*x*C-y*s,z*y*C+x*s,z*z*C+c;
		Eigen::Vector3f rotated=rmat*original;
		arc.push_back(rotated);
	}
	return true;
}

bool dataLibrary::stereonet_project(Eigen::Vector3f &on_sphere, Eigen::Vector2f &projected)
{
	projected(0)=on_sphere(0)/(1-on_sphere(2));
	projected(1)=on_sphere(1)/(1-on_sphere(2));
	return true;
}

bool dataLibrary::eqArea_project(Eigen::Vector3f &on_sphere, Eigen::Vector2f &projected)
{
	float length = std::sqrt(1-std::abs(on_sphere(2)));
	float gain = std::sqrt((length*length)/(on_sphere(0)*on_sphere(0)+on_sphere(1)*on_sphere(1)));
	projected(0)=gain*on_sphere(0);
	projected(1)=gain*on_sphere(1);

	return true;
}

bool dataLibrary::PlaneWithLineIntersection(const Eigen::Vector3f &plane_normal, const Eigen::Vector3f &plane_centor, const Eigen::Vector3f &line_point, const Eigen::Vector3f &line_direction, Eigen::Vector3f &point)
{
    if(plane_normal.dot(line_direction) == 0)
    {
        return false;
    }
    else
    {
        float t = plane_normal.dot(plane_centor - line_point)/plane_normal.dot(line_direction);
        point = line_direction*t + line_point;
        return true;
    }
}

bool dataLibrary::isInPolygon(const std::vector<Eigen::Vector2f> &convex_hull, const Eigen::Vector2f &test_point)
{
    int i, j;
    bool c = false;
    for (i = 0, j = convex_hull.size()-1; i < convex_hull.size(); j = i++) {
        if ( ((convex_hull[i](1)>test_point(1)) != (convex_hull[j](1)>test_point(1))) &&
            (test_point(0) < (convex_hull[j](0)-convex_hull[i](0)) * (test_point(1)-convex_hull[i](1)) / (convex_hull[j](1)-convex_hull[i](1)) + convex_hull[i](0)) )
            c = !c;
    }
    return c;
}

float dataLibrary::crossMultiply(const Eigen::Vector2f &u, const Eigen::Vector2f &v)
{
    return (u(0)*v(1) - u(1)*v(0));
}

bool dataLibrary::isSegmentCrossPolygon(const Eigen::Vector2f &point_a, const Eigen::Vector2f &point_b, const std::vector<Eigen::Vector2f> &convex_hull)
{
    Eigen::Vector2f r = point_b - point_a;
    int j=0;
    Eigen::Vector2f va, vb, s;
    float t, u, deno, nume_t, nume_u;
    int num_points = convex_hull.size();
    for(int i = 0; i < num_points; i++)
    {
        j = (i+1) % num_points;
        va = convex_hull[i];
        vb = convex_hull[j];
        s = vb - va;
        deno = crossMultiply(r, s);
        nume_t = crossMultiply((va - point_a),s);
        nume_u = crossMultiply((va - point_a), r);
        t = nume_t/deno;
        u = nume_u/deno;
        if((std::abs(deno)<=EPSILON)&&(std::abs(nume_u)<=EPSILON))
        {
            if((((va-point_a).dot(r)>=0.0)&&((va-point_a).dot(r)<=r.dot(r)))||(((point_a-va).dot(s)>=0.0)&&((point_a-va).dot(s)<=s.dot(s))))
            {
                return true;
            }
        }
        else if((std::abs(deno)>EPSILON)&&((t>=0.0)&&(t<=1.0))&&((u>=0.0)&&(u<=1.0)))
        {
            return true;
        }
    }
    return false;
}

void dataLibrary::assign_left_with_right(Vector3f &left, const Eigen::Vector3f &right)
{
	left.x = right(0);
	left.y = right(1);
	left.z = right(2);
}

void dataLibrary::assign_left_with_right(Eigen::Vector3f &left, const float* right)
{
	for(int i=0; i<3; i++)
	{
		left(i) = right[i];
	}
}

bool dataLibrary::no_trim_edges(const Eigen::Vector3f &V, const Eigen::Vector3f &xyz_centroid, pcl::PointCloud<pcl::PointXYZ>::Ptr convex_hull, const Eigen::Vector3f &max_intersection, const Eigen::Vector3f &min_intersection, int patchNum, float &length)
{
	stringstream ss;
	ss << patchNum;

	Eigen::Vector3f V_x = convex_hull->at(1).getVector3fMap() - convex_hull->at(0).getVector3fMap();
	Eigen::Vector3f V_y = V.cross(V_x);
	std::vector<Eigen::Vector2f> convex_hull_2d;
	
	float temp_length = std::sqrt((max_intersection-min_intersection).dot(max_intersection-min_intersection));
	
	dataLibrary::projection322(V_x, V_y, convex_hull, convex_hull_2d);
	Eigen::Vector2f test_point_2d_max, test_point_2d_min;
	dataLibrary::projection322(V_x, V_y, max_intersection, test_point_2d_max);
	dataLibrary::projection322(V_x, V_y, min_intersection, test_point_2d_min);

	if((isInPolygon(convex_hull_2d, test_point_2d_max))||(isInPolygon(convex_hull_2d, test_point_2d_min)))
	{
		Line NewLine;
		dataLibrary::assign_left_with_right(NewLine.begin, max_intersection);
		dataLibrary::assign_left_with_right(NewLine.end, min_intersection);
		NewLine.r = 0;
		NewLine.g = 1;
		NewLine.b = 0;
		NewLine.ID = "Line_in"+ss.str();
		dataLibrary::Lines.push_back(NewLine);

		length=temp_length;
		return true;
	}
	else if(isSegmentCrossPolygon(test_point_2d_max, test_point_2d_min, convex_hull_2d))
	{
		Line NewLine;
		dataLibrary::assign_left_with_right(NewLine.begin, max_intersection);
		dataLibrary::assign_left_with_right(NewLine.end, min_intersection);
		NewLine.r = 0;
		NewLine.g = 1;
		NewLine.b = 0;
		NewLine.ID = "Line_in"+ss.str();
		dataLibrary::Lines.push_back(NewLine);
		
		length=temp_length;
		return true;
	}
	else
	{
		Line NewLine;
		dataLibrary::assign_left_with_right(NewLine.begin, max_intersection);
		dataLibrary::assign_left_with_right(NewLine.end, min_intersection);
		NewLine.r = 1;
		NewLine.g = 0;
		NewLine.b = 0;
		NewLine.ID = "Line_out"+ss.str();
		dataLibrary::Lines.push_back(NewLine);
		
		length=temp_length;
		return false;
	}
}

bool dataLibrary::trim_edges(const Eigen::Vector3f &V, const Eigen::Vector3f &xyz_centroid, pcl::PointCloud<pcl::PointXYZ>::Ptr convex_hull, const Eigen::Vector3f &max_intersection, const Eigen::Vector3f &min_intersection, int patchNum, float &length)
{
	stringstream ss;
	ss << patchNum;

	Eigen::Vector3f V_x = convex_hull->at(1).getVector3fMap() - convex_hull->at(0).getVector3fMap();
	Eigen::Vector3f V_y = V.cross(V_x);
	std::vector<Eigen::Vector2f> convex_hull_2d;

	dataLibrary::projection322(V_x, V_y, convex_hull, convex_hull_2d);
	Eigen::Vector2f test_point_2d_max, test_point_2d_min;
	dataLibrary::projection322(V_x, V_y, max_intersection, test_point_2d_max);
	dataLibrary::projection322(V_x, V_y, min_intersection, test_point_2d_min);
	if((isInPolygon(convex_hull_2d, test_point_2d_max))&&(isInPolygon(convex_hull_2d, test_point_2d_min)))
	{// All end points are in.
		Line NewLine;
		dataLibrary::assign_left_with_right(NewLine.begin, max_intersection);
		dataLibrary::assign_left_with_right(NewLine.end, min_intersection);
		NewLine.r = 0;
		NewLine.g = 1;
		NewLine.b = 0;
		NewLine.ID = "Line_in"+ss.str();
		dataLibrary::Lines.push_back(NewLine);
		
		length=std::sqrt((max_intersection-min_intersection).dot(max_intersection-min_intersection));
		return true;
	}
	else if((isInPolygon(convex_hull_2d, test_point_2d_max))&&!(isInPolygon(convex_hull_2d, test_point_2d_min)))
	{// One end point is in, the other one is out.
		int hull_size = convex_hull->size();
		Eigen::Vector3f P_in = max_intersection;
		Eigen::Vector3f P_out = min_intersection;
		Eigen::Vector3f P_edge;
		for(int i=0; i<hull_size; i++)
		{
			Eigen::Vector3f N_i = convex_hull->at(i).getVector3fMap();
			Eigen::Vector3f P_in_P_out = P_out - P_in;
			Eigen::Vector3f P_in_N_i = N_i - P_in;
			if(std::acos(P_in_P_out.dot(P_in_N_i)/(std::sqrt(P_in_P_out.dot(P_in_P_out))*std::sqrt(P_in_N_i.dot(P_in_N_i))))<=EPSILON)
			{
				P_edge = N_i;
				break;
			}
			Eigen::Vector3f N_j = convex_hull->at((i+1)%hull_size).getVector3fMap();
			Eigen::Vector3f P_in_N_j = N_j - P_in;
			Eigen::Vector3f N_i_N_j = N_j - N_i;
			if(P_in_N_i.cross(P_in_P_out).dot(P_in_P_out.cross(P_in_N_j))>0)
			{
				float angle_a = std::acos(P_in_N_i.dot(P_in_P_out)/(std::sqrt(P_in_N_i.dot(P_in_N_i))*std::sqrt(P_in_P_out.dot(P_in_P_out))));
				float angle_b = std::acos(P_in_P_out.dot(P_in_N_j)/(std::sqrt(P_in_P_out.dot(P_in_P_out))*std::sqrt(P_in_N_j.dot(P_in_N_j))));
				if((angle_a+angle_b)<(TWOPI/2))
				{
					float x = P_in_P_out.cross(P_in_N_i)(0)/N_i_N_j.cross(P_in_P_out)(0);
					P_edge = P_in + (P_in_N_i + x*N_i_N_j);
					break;
				}
			}
		}
		Line NewLine;
		dataLibrary::assign_left_with_right(NewLine.begin, P_in);
		dataLibrary::assign_left_with_right(NewLine.end, P_edge);
		NewLine.r = 0;
		NewLine.g = 1;
		NewLine.b = 0;
		NewLine.ID = "Line_in"+ss.str();
		dataLibrary::Lines.push_back(NewLine);
		
		length=std::sqrt((P_edge - P_in).dot(P_edge - P_in));
		return true;
	}
	else if(!(isInPolygon(convex_hull_2d, test_point_2d_max))&&(isInPolygon(convex_hull_2d, test_point_2d_min)))
	{// One end point is in, the other one is out.
		int hull_size = convex_hull->size();
		Eigen::Vector3f P_in = min_intersection;
		Eigen::Vector3f P_out = max_intersection;
		Eigen::Vector3f P_edge;
		for(int i=0; i<hull_size; i++)
		{
			Eigen::Vector3f N_i = convex_hull->at(i).getVector3fMap();
			Eigen::Vector3f P_in_P_out = P_out - P_in;
			Eigen::Vector3f P_in_N_i = N_i - P_in;
			if(std::acos(P_in_P_out.dot(P_in_N_i)/(std::sqrt(P_in_P_out.dot(P_in_P_out))*std::sqrt(P_in_N_i.dot(P_in_N_i))))<=EPSILON)
			{
				P_edge = N_i;
				break;
			}
			Eigen::Vector3f N_j = convex_hull->at((i+1)%hull_size).getVector3fMap();
			Eigen::Vector3f P_in_N_j = N_j - P_in;
			Eigen::Vector3f N_i_N_j = N_j - N_i;
			if(P_in_N_i.cross(P_in_P_out).dot(P_in_P_out.cross(P_in_N_j))>0)
			{
				float angle_a = std::acos(P_in_N_i.dot(P_in_P_out)/(std::sqrt(P_in_N_i.dot(P_in_N_i))*std::sqrt(P_in_P_out.dot(P_in_P_out))));
				float angle_b = std::acos(P_in_P_out.dot(P_in_N_j)/(std::sqrt(P_in_P_out.dot(P_in_P_out))*std::sqrt(P_in_N_j.dot(P_in_N_j))));
				if((angle_a+angle_b)<(TWOPI/2))
				{
					float x = P_in_P_out.cross(P_in_N_i)(0)/N_i_N_j.cross(P_in_P_out)(0);
					P_edge = P_in + (P_in_N_i + x*N_i_N_j);
					break;
				}
			}
		}
		Line NewLine;
		dataLibrary::assign_left_with_right(NewLine.begin, P_in);
		dataLibrary::assign_left_with_right(NewLine.end, P_edge);
		NewLine.r = 0;
		NewLine.g = 1;
		NewLine.b = 0;
		NewLine.ID = "Line_in"+ss.str();
		dataLibrary::Lines.push_back(NewLine);

		length=std::sqrt((P_edge - P_in).dot(P_edge - P_in));
		return true;
	}
	else if(isSegmentCrossPolygon(test_point_2d_max, test_point_2d_min, convex_hull_2d))
	{// All end points are out, but still, the segment is crossing the polygon.
		int hull_size = convex_hull->size();
		Eigen::Vector3f P_in = min_intersection;
		Eigen::Vector3f P_out = max_intersection;
		Eigen::Vector3f P_in_P_out = P_out - P_in;
		Eigen::Vector3f P_edge_in, P_edge_out;
		for(int i=0; i<hull_size; i++)
		{
			Eigen::Vector3f N_i = convex_hull->at(i).getVector3fMap();
			Eigen::Vector3f P_in_N_i = N_i - P_in;
			Eigen::Vector3f N_j = convex_hull->at((i+1)%hull_size).getVector3fMap();
			Eigen::Vector3f P_in_N_j = N_j - P_in;
			Eigen::Vector3f N_i_N_j = N_j - N_i;
			if(P_in_N_i.cross(P_in_P_out).dot(P_in_P_out.cross(P_in_N_j))>0)
			{
				float x = P_in_P_out.cross(P_in_N_i)(0)/N_i_N_j.cross(P_in_P_out)(0);
				P_edge_in = P_in + (P_in_N_i + x*N_i_N_j);
				for(int j=i+1; j<hull_size; j++)
				{
					Eigen::Vector3f N_k = convex_hull->at(j).getVector3fMap();
					Eigen::Vector3f P_in_N_k = N_k - P_in;
					Eigen::Vector3f N_l = convex_hull->at((j+1)%hull_size).getVector3fMap();
					Eigen::Vector3f P_in_N_l = N_l - P_in;
					Eigen::Vector3f N_k_N_l = N_l - N_k;
					if(P_in_N_k.cross(P_in_P_out).dot(P_in_P_out.cross(P_in_N_l))>0)
					{
						float x = P_in_P_out.cross(P_in_N_k)(0)/N_k_N_l.cross(P_in_P_out)(0);
						P_edge_out = P_in + (P_in_N_k + x*N_k_N_l);
						break;
					}
				}
				break;
			}
		}
		Line NewLine;
		dataLibrary::assign_left_with_right(NewLine.begin, P_edge_in);
		dataLibrary::assign_left_with_right(NewLine.end, P_edge_out);
		NewLine.r = 0;
		NewLine.g = 1;
		NewLine.b = 0;
		NewLine.ID = "Line_in"+ss.str();
		dataLibrary::Lines.push_back(NewLine);
		
		length=std::sqrt((P_edge_out - P_edge_in).dot(P_edge_out - P_edge_in));
		return true;
	}
	else
	{
		Line NewLine;
		dataLibrary::assign_left_with_right(NewLine.begin, max_intersection);
		dataLibrary::assign_left_with_right(NewLine.end, min_intersection);
		NewLine.r = 1;
		NewLine.g = 0;
		NewLine.b = 0;
		NewLine.ID = "Line_out"+ss.str();
		dataLibrary::Lines.push_back(NewLine);
		
		length=std::sqrt((max_intersection-min_intersection).dot(max_intersection-min_intersection));
		return false;
	}
}

bool dataLibrary::Circular(const Eigen::Vector3f &V, const Eigen::Vector3f &xyz_centroid, pcl::PointCloud<pcl::PointXYZ>::Ptr convex_hull, const Eigen::Vector3f &V_i, const Eigen::Vector3f &xyz_centroid_i, pcl::PointCloud<pcl::PointXYZ>::Ptr convex_hull_i, int patchNum, float &length, const float &expand_ratio, bool is_triming_edges, bool needExLine)
{
	typedef float mytype;			// coordinate type
	int d = 3;						// dimension
	int n = convex_hull_i->size();	// number of points
	mytype** ap = new mytype*[n];
	for (int i=0; i<n; ++i) {
		mytype* p = new mytype[d];
		for (int j=0; j<d; ++j) {
			p[j] = convex_hull_i->at(i).getVector3fMap()(j);
		}
		ap[i]=p;
	}
	// define the types of iterators through the points and their coordinates
	typedef mytype* const* PointIterator;
	typedef const mytype* CoordIterator;
	// create an instance of Miniball
	typedef Miniball::Miniball <Miniball::CoordAccessor<PointIterator, CoordIterator> > MB;
	MB mb (d, ap, ap+n);
	const mytype* center = mb.center();
	Eigen::Vector3f C_0, W, U_axis, V_axis;
	dataLibrary::assign_left_with_right(C_0, center);
	mytype radius = std::sqrt(mb.squared_radius());
	U_axis = (2*radius*expand_ratio+radius)*(V.cross(V_i)/(std::sqrt(V.cross(V_i).dot(V.cross(V_i)))));
	V_axis = U_axis.cross(V_i);
	float a = V_axis.dot(V);
	float b = U_axis.dot(V);
	float c = (xyz_centroid - C_0).dot(V);
	float right = c/std::sqrt(a*a+b*b);
	float beta = std::acos(a/std::sqrt(a*a+b*b));
	if(std::abs(right)<1)
	{
		float t_p_beta = std::asin(right);
		float temp_ans1, temp_ans2, ans1, ans2;
		if(t_p_beta>=0)
		{
			temp_ans1 = t_p_beta;
			temp_ans2 = TWOPI/2 - temp_ans1;
		}
		else
		{
			temp_ans1 = TWOPI + t_p_beta;
			temp_ans2 = 3*TWOPI/2 - temp_ans1;
		}
		ans1 = temp_ans1 - beta;
		ans2 = temp_ans2 - beta;
		Eigen::Vector3f max_intersection = C_0 + std::cos(ans1)*U_axis + std::sin(ans1)*V_axis;
		Eigen::Vector3f min_intersection = C_0 + std::cos(ans2)*U_axis + std::sin(ans2)*V_axis;
		if(is_triming_edges)
		{
			return dataLibrary::trim_edges(V, xyz_centroid, convex_hull, max_intersection, min_intersection, patchNum, length);
		}
		else
		{
			return dataLibrary::no_trim_edges(V, xyz_centroid, convex_hull, max_intersection, min_intersection, patchNum, length);
		}
	}
	else
	{
		length = 0.0;
		return false;
	}
}

bool dataLibrary::Rectangular(const Eigen::Vector3f &V, const Eigen::Vector3f &xyz_centroid, pcl::PointCloud<pcl::PointXYZ>::Ptr convex_hull, const Eigen::Vector3f &V_i, const Eigen::Vector3f &xyz_centroid_i, pcl::PointCloud<pcl::PointXYZ>::Ptr convex_hull_i, int patchNum, float &length, const float &expand_ratio, bool is_triming_edges, bool needExLine)
{
	Eigen::Vector3f on_plane_direction = V.cross(V_i);
	on_plane_direction = on_plane_direction/std::sqrt(on_plane_direction.dot(on_plane_direction));
    stringstream ss;
	ss << patchNum;
    
	// Check if the fracture patch and the suppositional plane are parallel
    if((on_plane_direction(0) == 0)&&(on_plane_direction(1) == 0)&&(on_plane_direction(2) == 0))
    {
        length=0.0;
        return false;
    }
    else
    {
        float max_value, min_value;
        int max_index, min_index;
        Eigen::Vector3f point = convex_hull_i->at(0).getVector3fMap();
        max_value = min_value = on_plane_direction.dot(point - xyz_centroid_i);
        max_index = min_index = 0;
        for(int i=1; i<convex_hull_i->size(); i++)
        {
            Eigen::Vector3f point = convex_hull_i->at(i).getVector3fMap();
            
            float value = on_plane_direction.dot(point - xyz_centroid_i);
            
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
        Eigen::Vector3f line_direction = V_i.cross(on_plane_direction);
		float D = on_plane_direction.dot(convex_hull_i->at(max_index).getVector3fMap() - convex_hull_i->at(min_index).getVector3fMap());
		float E = D*expand_ratio;
        
        Eigen::Vector3f max_intersection, min_intersection;
        if(PlaneWithLineIntersection(V, xyz_centroid, convex_hull_i->at(max_index).getVector3fMap(), line_direction, max_intersection))
        {
            if(PlaneWithLineIntersection(V, xyz_centroid, convex_hull_i->at(min_index).getVector3fMap(), line_direction, min_intersection))
            {
				Eigen::Vector3f real_max_intersection;
				Eigen::Vector3f max_direction = max_intersection - convex_hull_i->at(max_index).getVector3fMap();
				max_direction = max_direction/std::sqrt(max_direction.dot(max_direction));
				Eigen::Vector3f N_i = convex_hull_i->at(max_index).getVector3fMap();
				Eigen::Vector3f N_j = convex_hull_i->at(max_index).getVector3fMap() + E*max_direction;
				double side_i = V.dot(N_i - xyz_centroid);
				double side_j = V.dot(N_j - xyz_centroid);
				if((side_i*side_j<=0.0f)&&((side_i*side_i+side_j*side_j)>0.0f))
				{
					dataLibrary::PlaneWithLineIntersection(V, xyz_centroid, N_i, N_j-N_i, real_max_intersection);
				}
				else
				{
					Eigen::Vector3f max_p = convex_hull_i->at(max_index).getVector3fMap();
					Eigen::Vector3f max_p_p = convex_hull_i->at((max_index+1)%convex_hull_i->size()).getVector3fMap();
					if(max_direction.cross(max_p_p - max_p).dot((max_p_p - max_p).cross(convex_hull_i->at(min_index).getVector3fMap() - convex_hull_i->at(max_index).getVector3fMap()))>0)
					{
						for(int i=max_index; i!=min_index; i=(i+1)%convex_hull_i->size())
						{
							N_i = convex_hull_i->at(i).getVector3fMap() + E*max_direction;
							N_j = convex_hull_i->at((i+1)%convex_hull_i->size()).getVector3fMap() + E*max_direction;
							double side_i = V.dot(N_i - xyz_centroid);
							double side_j = V.dot(N_j - xyz_centroid);
							if((side_i*side_j<=0.0f)&&((side_i*side_i+side_j*side_j)>0.0f))
							{
								dataLibrary::PlaneWithLineIntersection(V, xyz_centroid, N_i, N_j-N_i, real_max_intersection);
								break;
							}
						}
					}
					else
					{
						for(int i=max_index; i!=min_index; i = (i==0 ? (convex_hull_i->size()-1) : (i - 1)))
						{
							N_i = convex_hull_i->at(i).getVector3fMap() + E*max_direction;
							N_j = convex_hull_i->at(i==0 ? (convex_hull_i->size()-1) : (i - 1)).getVector3fMap() + E*max_direction;
							double side_i = V.dot(N_i - xyz_centroid);
							double side_j = V.dot(N_j - xyz_centroid);
							if((side_i*side_j<=0.0f)&&((side_i*side_i+side_j*side_j)>0.0f))
							{
								dataLibrary::PlaneWithLineIntersection(V, xyz_centroid, N_i, N_j-N_i, real_max_intersection);
								break;
							}
						}
					}
				}

				Eigen::Vector3f real_min_intersection;
				Eigen::Vector3f min_direction = min_intersection - convex_hull_i->at(min_index).getVector3fMap();
				min_direction = min_direction/std::sqrt(min_direction.dot(min_direction));
				N_i = convex_hull_i->at(min_index).getVector3fMap();
				N_j = convex_hull_i->at(min_index).getVector3fMap() + E*min_direction;
				side_i = V.dot(N_i - xyz_centroid);
				side_j = V.dot(N_j - xyz_centroid);
				if((side_i*side_j<=0.0f)&&((side_i*side_i+side_j*side_j)>0.0f))
				{
					dataLibrary::PlaneWithLineIntersection(V, xyz_centroid, N_i, N_j-N_i, real_min_intersection);
				}
				else
				{
					Eigen::Vector3f min_p = convex_hull_i->at(min_index).getVector3fMap();
					Eigen::Vector3f min_p_p = convex_hull_i->at((min_index+1)%convex_hull_i->size()).getVector3fMap();
					if(min_direction.cross(min_p_p - min_p).dot((min_p_p - min_p).cross(convex_hull_i->at(max_index).getVector3fMap() - convex_hull_i->at(min_index).getVector3fMap()))>0)
					{
						for(int i=min_index; i!=max_index; i=(i+1)%convex_hull_i->size())
						{
							N_i = convex_hull_i->at(i).getVector3fMap() + E*min_direction;
							N_j = convex_hull_i->at((i+1)%convex_hull_i->size()).getVector3fMap() + E*min_direction;
							double side_i = V.dot(N_i - xyz_centroid);
							double side_j = V.dot(N_j - xyz_centroid);
							if((side_i*side_j<=0.0f)&&((side_i*side_i+side_j*side_j)>0.0f))
							{
								dataLibrary::PlaneWithLineIntersection(V, xyz_centroid, N_i, N_j-N_i, real_min_intersection);
								break;
							}
						}
					}
					else
					{
						for(int i=min_index; i!=max_index; i = (i==0 ? (convex_hull_i->size()-1) : (i - 1)))
						{
							N_i = convex_hull_i->at(i).getVector3fMap() + E*min_direction;
							N_j = convex_hull_i->at(i==0 ? (convex_hull_i->size()-1) : (i - 1)).getVector3fMap() + E*min_direction;
							double side_i = V.dot(N_i - xyz_centroid);
							double side_j = V.dot(N_j - xyz_centroid);
							if((side_i*side_j<=0.0f)&&((side_i*side_i+side_j*side_j)>0.0f))
							{
								dataLibrary::PlaneWithLineIntersection(V, xyz_centroid, N_i, N_j-N_i, real_min_intersection);
								break;
							}
						}
					}
				}
				if(needExLine)
				{
					
				}
				if(is_triming_edges)
					return dataLibrary::trim_edges(V, xyz_centroid, convex_hull, real_max_intersection, real_min_intersection, patchNum, length);
				else
					return dataLibrary::no_trim_edges(V, xyz_centroid, convex_hull, real_max_intersection, real_min_intersection, patchNum, length);
            }
            else
            {
                length=0.0;
				return false;
            }
        }
        else
        {
            length=0.0;
			return false;
        }
    }
}

bool dataLibrary::LowerBound(const Eigen::Vector3f &V, const Eigen::Vector3f &xyz_centroid, pcl::PointCloud<pcl::PointXYZ>::Ptr convex_hull, const Eigen::Vector3f &V_i, const Eigen::Vector3f &xyz_centroid_i, pcl::PointCloud<pcl::PointXYZ>::Ptr convex_hull_i, int patchNum, float &length, bool is_triming_edges, bool needExLine)
{
	if(needExLine)
	{
		dataLibrary::fracture_faces_hull.push_back(convex_hull_i);
	}
	int hull_size_i = convex_hull_i->size();
	std::vector<Eigen::Vector3f> intersections;
	for(int i=0; i<hull_size_i; i++)
	{
		Eigen::Vector3f N_i = convex_hull_i->at(i).getVector3fMap();
		Eigen::Vector3f N_j = convex_hull_i->at((i+1)%hull_size_i).getVector3fMap();
		double side_i = V.dot(N_i - xyz_centroid);
		double side_j = V.dot(N_j - xyz_centroid);
		if((side_i*side_j<=0.0f)&&((side_i*side_i+side_j*side_j)>0.0f))
		{
			Eigen::Vector3f temp_intersec;
			if(dataLibrary::PlaneWithLineIntersection(V, xyz_centroid, N_i, N_j-N_i, temp_intersec))
			{
				intersections.push_back(temp_intersec);
			}
		}
	}
	if(intersections.size()>=2)
	{
		if(is_triming_edges)
			return dataLibrary::trim_edges(V, xyz_centroid, convex_hull, intersections[0], intersections[1], patchNum, length);
		else
			return dataLibrary::no_trim_edges(V, xyz_centroid, convex_hull, intersections[0], intersections[1], patchNum, length);
	}
	else
	{
		length = 0.0;
		return false;
	}
}

bool dataLibrary::checkContents(std::vector<std::string> contents, std::string query)
{
	for(int i=0; i<contents.size(); i++)
	{
		if(contents[i].compare(query)==0)
		{
			return true;
		}
	}
	return false;
}

void dataLibrary::write_text_to_log_file( const std::string &text )
{
    std::ofstream log_file( "log_file.txt", std::ios_base::out | std::ios_base::app );
    log_file << text << std::endl;
}