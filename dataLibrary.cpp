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
#include "globaldef.h"

using namespace std;

pcl::PointCloud<pcl::PointXYZ>::Ptr dataLibrary::cloudxyz(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr dataLibrary::cloudxyzrgb(new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZI>::Ptr dataLibrary::cloudxyzi(new pcl::PointCloud<pcl::PointXYZI>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr dataLibrary::cloudxyzrgb_clusters(new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr dataLibrary::cloudxyzrgb_features(new pcl::PointCloud<pcl::PointXYZRGB>);
sensor_msgs::PointCloud2::Ptr dataLibrary::cloud_blob(new sensor_msgs::PointCloud2);
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
RGSpara dataLibrary::RGSparameter;
TriangulationPara dataLibrary::TriangulationParameter;
std::vector<pcl::PolygonMesh::Ptr> dataLibrary::Fracture_Triangles;
FeaturePara dataLibrary::FeatureParameter;
std::vector<std::string> dataLibrary::contents;
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
clock_t dataLibrary::start;
clock_t dataLibrary::finish;
Vector3f dataLibrary::cloud_centor;

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
	if(cloud_blob->data.size()>0)
	{
		cloud_blob->data.clear();
	}
	if(cloud_blob->fields.size()>0)
	{
		cloud_blob->fields.clear();
	}
	cloud_blob->height=0;
	cloud_blob->width=0;
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
	std::vector<std::string>().swap(contents);
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

bool dataLibrary::CheckClusters(const Eigen::Vector3f &V, const Eigen::Vector3f &xyz_centroid, pcl::PointCloud<pcl::PointXYZ>::Ptr convex_hull, const Eigen::Vector3f &V_i, const Eigen::Vector3f &xyz_centroid_i, pcl::PointCloud<pcl::PointXYZ>::Ptr projected_i, int patchNum, float &length, bool needExLine)
{
    Eigen::Vector3f on_plane_direction = V.cross(V_i);
    float temp_length;
    stringstream ss;
    
    if((on_plane_direction(0) == 0)&&(on_plane_direction(1) == 0)&&(on_plane_direction(2) == 0))
    {
        length=0.0;
        return false;
    }
    else
    {
        float max_value, min_value;
        int max_index, min_index;
        Eigen::Vector3f point;
        point(0) = projected_i->at(0).x;
        point(1) = projected_i->at(0).y;
        point(2) = projected_i->at(0).z;
        max_value = min_value = on_plane_direction.dot(point - xyz_centroid_i);
        max_index = min_index = 0;
        for(int i=1; i<projected_i->size(); i++)
        {
            Eigen::Vector3f point;
            point(0) = projected_i->at(i).x;
            point(1) = projected_i->at(i).y;
            point(2) = projected_i->at(i).z;
            
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
        
        Eigen::Vector3f max_intersection, min_intersection;
        if(PlaneWithLineIntersection(V, xyz_centroid, projected_i->at(max_index).getVector3fMap(), line_direction, max_intersection))
        {
            if(PlaneWithLineIntersection(V, xyz_centroid, projected_i->at(min_index).getVector3fMap(), line_direction, min_intersection))
            {
                Eigen::Vector3f V_x = convex_hull->at(1).getVector3fMap() - convex_hull->at(0).getVector3fMap();
                Eigen::Vector3f V_y = V.cross(V_x);
                std::vector<Eigen::Vector2f> convex_hull_2d;
                
                temp_length=std::sqrt((max_intersection-min_intersection).dot(max_intersection-min_intersection));
                
                if(dataLibrary::projection322(V_x, V_y, convex_hull, convex_hull_2d))
                {
                    Eigen::Vector2f test_point_2d_max, test_point_2d_min;
                    if((dataLibrary::projection322(V_x, V_y, max_intersection, test_point_2d_max))&&(dataLibrary::projection322(V_x, V_y, min_intersection, test_point_2d_min)))
                    {
						ss << patchNum;

						if(needExLine)
						{
							Line NewLine_max, NewLine_min;

							NewLine_max.begin.x = projected_i->at(max_index).getVector3fMap()(0);
							NewLine_max.begin.y = projected_i->at(max_index).getVector3fMap()(1);
							NewLine_max.begin.z = projected_i->at(max_index).getVector3fMap()(2);
							NewLine_max.end.x = max_intersection(0);
							NewLine_max.end.y = max_intersection(1);
							NewLine_max.end.z = max_intersection(2);
							NewLine_max.r = 1;
							NewLine_max.g = 1;
							NewLine_max.b = 1;
							NewLine_max.ID = "Line_auxiliary_max"+ss.str();
							dataLibrary::Lines_max.push_back(NewLine_max);

							NewLine_min.begin.x = projected_i->at(min_index).getVector3fMap()(0);
							NewLine_min.begin.y = projected_i->at(min_index).getVector3fMap()(1);
							NewLine_min.begin.z = projected_i->at(min_index).getVector3fMap()(2);
							NewLine_min.end.x = min_intersection(0);
							NewLine_min.end.y = min_intersection(1);
							NewLine_min.end.z = min_intersection(2);
							NewLine_min.r = 1;
							NewLine_min.g = 1;
							NewLine_min.b = 1;
							NewLine_min.ID = "Line_auxiliary_min"+ss.str();
							dataLibrary::Lines_min.push_back(NewLine_min);
						}

                        if((isInPolygon(convex_hull_2d, test_point_2d_max))||(isInPolygon(convex_hull_2d, test_point_2d_min)))
                        {
                            ss << patchNum;

                            Line NewLine;
                            NewLine.begin.x = max_intersection(0);
                            NewLine.begin.y = max_intersection(1);
                            NewLine.begin.z = max_intersection(2);
                            NewLine.end.x = min_intersection(0);
                            NewLine.end.y = min_intersection(1);
                            NewLine.end.z = min_intersection(2);
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
                            ss << patchNum;

                            Line NewLine;
                            NewLine.begin.x = max_intersection(0);
                            NewLine.begin.y = max_intersection(1);
                            NewLine.begin.z = max_intersection(2);
                            NewLine.end.x = min_intersection(0);
                            NewLine.end.y = min_intersection(1);
                            NewLine.end.z = min_intersection(2);
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
                            ss << patchNum;

                            Line NewLine;
                            NewLine.begin.x = max_intersection(0);
                            NewLine.begin.y = max_intersection(1);
                            NewLine.begin.z = max_intersection(2);
                            NewLine.end.x = min_intersection(0);
                            NewLine.end.y = min_intersection(1);
                            NewLine.end.z = min_intersection(2);
                            NewLine.r = 1;
                            NewLine.g = 0;
                            NewLine.b = 0;
                            NewLine.ID = "Line_out"+ss.str();
                            dataLibrary::Lines.push_back(NewLine);

                            length=temp_length;
                            return false;
                        }
                    }
                }
            }
            else
            {
                length=temp_length;
                return false;
            }
        }
        else
        {
            length=temp_length;
            return false;
        }
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