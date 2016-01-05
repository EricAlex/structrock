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
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <Eigen/src/Core/Matrix.h>
#include "ReadFileWorker.h"
#include "checkstatusThread.h"
#include "globaldef.h"

class dataLibrary
{
public:
	static pcl::PointCloud<pcl::PointXYZ>::Ptr cloudxyz;
	static pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudxyzrgb;
	static pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudxyzrgb_clusters;
	static sensor_msgs::PointCloud2::Ptr cloud_blob;
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
	static std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> cluster_patches;
	static std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> fracture_faces_hull;
	static std::vector<float> dips;
	static std::vector<float> dip_directions;
	static std::vector<float> out_dips;
	static std::vector<float> out_dip_directions;
	static std::vector<float> areas;
	static std::vector<std::string> patchIDs;
	static int currentPatch;
	static std::vector<int> selectedPatches;
	static int Status;
    static RGSpara RGSparameter;
	static std::vector<std::string> contents;
    static pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull_all;
    static Eigen::Vector3f plane_normal_all;
    static std::vector<Line> Lines;
	static std::vector<Line> Lines_max;
	static std::vector<Line> Lines_min;
	static std::vector<WorkLine> Workflow;
	static int current_workline_index;
	static bool have_called_read_file;

public:
	static void checkupflow();
	static bool haveBaseData();
	static void clearall();
	static void clearWorkFlow();
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
	static bool CheckClusters(const Eigen::Vector3f &V, const Eigen::Vector3f &xyz_centroid, pcl::PointCloud<pcl::PointXYZ>::Ptr convex_hull, const Eigen::Vector3f &V_i, const Eigen::Vector3f &xyz_centroid_i, pcl::PointCloud<pcl::PointXYZ>::Ptr projected_i, int patchNum, float &length, bool needExLine);
	static bool checkContents(std::vector<std::string> contents, std::string query);
};