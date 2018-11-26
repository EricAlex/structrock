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

#pragma once
#include <time.h>
#include <fstream>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/PolygonMesh.h>
#include <Eigen/src/Core/Matrix.h>
#include "ReadFileWorker.h"
#include "checkstatusThread.h"
#include "globaldef.h"

class dataLibrary
{
public:
	static pcl::PointCloud<pcl::PointXYZ>::Ptr cloudxyz;
	static pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudxyzrgb;
	static pcl::PointCloud<pcl::PointXYZI>::Ptr cloudxyzi;
	static pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudxyzrgb_clusters; //for showing the segmentation result
	static pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudxyzrgb_features; //for showing the surface features
	static pcl::PointCloud<pcl::Normal>::Ptr normal;
	static pcl::PointCloud<pcl::PointNormal>::Ptr pointnormals;
	static pcl::PointCloud<pcl::PointXYZ>::Ptr mls_points;
	static pcl::PointCloud<pcl::PointXYZ>::Ptr downsampledxyz;
	static pcl::PointCloud<pcl::PointXYZ>::Ptr outlier_removed_inlier;
	static pcl::PointCloud<pcl::PointXYZ>::Ptr outlier_removed_outlier;
	static pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud;
	static pcl::PointCloud<pcl::PointXYZ>::Ptr segmentation_rem;
	static std::string cloudID;
	static std::vector<pcl::PointIndices> clusters;
	static std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> cluster_patches;			//for reading in the saved clusters data
	static std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> fracture_faces_hull;			//for visualizing fracture oulines
	static std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> fracture_faces_hull_up;			//for visualizing fracture oulines (up side)
	static std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> fracture_faces_hull_down;		//for visualizing fracture oulines (down side)
	static std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> fracture_faces_circle_original;	//for visualizing original fracture circles
	static std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> fracture_faces_expanded;		//for visualizing expanded fracture oulines
	static std::vector<float> dips;
	static std::vector<float> dip_directions;
	static std::vector<float> out_dips;
	static std::vector<float> out_dip_directions;
	static std::vector<float> areas;
	static std::vector<float> roughnesses;
	static std::vector<std::string> patchIDs;
	static int currentPatch;
	static std::vector<int> selectedPatches;
	static int Status;
	static std::vector<pcl::PolygonMesh::Ptr> Fracture_Triangles;
    static pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull_all;
	static std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> fracture_patches;
	static std::vector<Striation> fracture_striations;
	static std::vector<Step> fracture_steps;
	static std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> fractures_with_feature;
    static std::string info_str;
	static Eigen::Vector3f plane_normal_all;
    static std::vector<Line> Lines;
	static std::vector<Line> Lines_max;
	static std::vector<Line> Lines_min;
	static std::vector<WorkLine> Workflow;
	static int current_workline_index;
	static bool have_called_read_file;
	static Vector3f cloud_centor;

public:
	static void getColorBetweenBlueNRed(float value, int &red, int &green, int &blue);
	static void getHeatMapColor(float value, int &red, int &green, int &blue);
	static void checkupflow();
	static bool haveBaseData();
	static void clearall();
	static void clearWorkFlow();
	static bool projection322(const Eigen::Vector3f &xyz_centroid, const Eigen::Vector3f &unit_V1, const Eigen::Vector3f &unit_V2, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, pcl::PointCloud<pcl::PointXY>::Ptr cloud_out);
	static bool projection223(const Eigen::Vector3f &xyz_centroid, const Eigen::Vector3f &unit_V1, const Eigen::Vector3f &unit_V2, pcl::PointCloud<pcl::PointXY>::Ptr cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out);
    static bool projection323(const Eigen::Vector3f &V1, const Eigen::Vector3f &V2, const Eigen::Vector3f &point_in, Eigen::Vector3f &point_out);
    static bool projection322(const Eigen::Vector3f &V1, const Eigen::Vector3f &V2, const Eigen::Vector3f &point_in, Eigen::Vector2f &point_out);
    static bool projection322(const Eigen::Vector3f &V1, const Eigen::Vector3f &V2, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in, std::vector<Eigen::Vector2f> &Points_out);
	static bool dd2pole(float &dip_dir, float &dip, Eigen::Vector3f &pole);
	static bool dd2arc(float &dip_dir, float &dip, std::vector<Eigen::Vector3f> &arc);
	static bool stereonet_project(Eigen::Vector3f &on_sphere, Eigen::Vector2f &projected);
	static bool eqArea_project(Eigen::Vector3f &on_sphere, Eigen::Vector2f &projected);
	static bool PlaneWithLineIntersection(const Eigen::Vector3f &plane_normal, const Eigen::Vector3f &plane_centor, const Eigen::Vector3f &line_point, const Eigen::Vector3f &line_direction, Eigen::Vector3f &point);
	static bool isInPolygon(const std::vector<Eigen::Vector2f> &convex_hull, const Eigen::Vector2f &test_point);
	static float crossMultiply(const Eigen::Vector2f &u, const Eigen::Vector2f &v);
	static bool isSegmentCrossPolygon(const Eigen::Vector2f &point_a, const Eigen::Vector2f &point_b, const std::vector<Eigen::Vector2f> &convex_hull);
	static bool edge_inside_part(const Eigen::Vector3f &V, pcl::PointCloud<pcl::PointXYZ>::Ptr convex_hull, const Eigen::Vector3f &in_begin, const Eigen::Vector3f &in_end, Eigen::Vector3f &out_begin, Eigen::Vector3f &out_end);
	static bool no_trim_edges(const Eigen::Vector3f &V, const Eigen::Vector3f &xyz_centroid, pcl::PointCloud<pcl::PointXYZ>::Ptr convex_hull, const Eigen::Vector3f &max_intersection, const Eigen::Vector3f &min_intersection, int patchNum, double f_p_angle, float &length);
	static bool trim_edges(const Eigen::Vector3f &V, const Eigen::Vector3f &xyz_centroid, pcl::PointCloud<pcl::PointXYZ>::Ptr convex_hull, const Eigen::Vector3f &max_intersection, const Eigen::Vector3f &min_intersection, int patchNum, double f_p_angle, float &length);
	static bool Circular(const Eigen::Vector3f &V, const Eigen::Vector3f &xyz_centroid, pcl::PointCloud<pcl::PointXYZ>::Ptr convex_hull, const Eigen::Vector3f &V_i, const Eigen::Vector3f &xyz_centroid_i, pcl::PointCloud<pcl::PointXYZ>::Ptr convex_hull_i, int patchNum, float &length, const float &expand_ratio, bool is_triming_edges, bool needExLine);
	static bool Rectangular(const Eigen::Vector3f &V, const Eigen::Vector3f &xyz_centroid, pcl::PointCloud<pcl::PointXYZ>::Ptr convex_hull, const Eigen::Vector3f &V_i, const Eigen::Vector3f &xyz_centroid_i, pcl::PointCloud<pcl::PointXYZ>::Ptr convex_hull_i, int patchNum, float &length, const float &expand_ratio, bool is_triming_edges, bool needExLine);
	static bool LowerBound(const Eigen::Vector3f &V, const Eigen::Vector3f &xyz_centroid, pcl::PointCloud<pcl::PointXYZ>::Ptr convex_hull, const Eigen::Vector3f &V_i, const Eigen::Vector3f &xyz_centroid_i, pcl::PointCloud<pcl::PointXYZ>::Ptr convex_hull_i, int patchNum, float &length, bool is_triming_edges, bool needExLine);
	static bool checkContents(std::vector<std::string> contents, std::string query);
	static void write_text_to_log_file( const std::string &text );
	static bool isOnlyDouble(const char* str);
	static void assign_left_with_right(Vector3f &left, const Eigen::Vector3f &right);
	static void assign_left_with_right(Eigen::Vector3f &left, const float* right);
	static void assign_left_with_right(pcl::PointXYZ &left, const Eigen::Vector3f &right);
	static Eigen::Vector4f fitPlaneManually(const pcl::PointCloud<pcl::PointXYZ>& cloud);
	static Eigen::Vector3f compute3DCentroid(const pcl::PointCloud<pcl::PointXYZ>& cloud);
};