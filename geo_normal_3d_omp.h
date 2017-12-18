/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
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
 *   * Neither the name of Willow Garage, Inc. nor the names of its
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
 * $Id$
 *
 */

/*
 * Modified by Xin Wang
 * Email : ericrussell@zju.edu.cn
 */

#pragma once
#include "geo_normal_3d.h"


  /** \brief GeoNormalEstimationOMP estimates local surface properties at each 3D point, such as surface normals and
    * curvatures, in parallel, using the OpenMP standard.
    * \author Radu Bogdan Rusu
    * \ingroup features
    */
  template <typename PointInT, typename PointOutT>
  class GeoNormalEstimationOMP: public GeoNormalEstimation<PointInT, PointOutT>
  {
    public:
      using GeoNormalEstimation<PointInT, PointOutT>::feature_name_;
      using GeoNormalEstimation<PointInT, PointOutT>::getClassName;
      using GeoNormalEstimation<PointInT, PointOutT>::indices_;
      using GeoNormalEstimation<PointInT, PointOutT>::input_;
      using GeoNormalEstimation<PointInT, PointOutT>::k_;
      using GeoNormalEstimation<PointInT, PointOutT>::search_parameter_;
      using GeoNormalEstimation<PointInT, PointOutT>::surface_;
      using GeoNormalEstimation<PointInT, PointOutT>::getViewPoint;

      typedef typename GeoNormalEstimation<PointInT, PointOutT>::PointCloudOut PointCloudOut;

    public:
      /** \brief Empty constructor. */
      GeoNormalEstimationOMP () : threads_ (1) 
      {
        feature_name_ = "GeoNormalEstimationOMP";
      };

      /** \brief Initialize the scheduler and set the number of threads to use.
        * \param nr_threads the number of hardware threads to use (-1 sets the value back to automatic)
        */
      GeoNormalEstimationOMP (unsigned int nr_threads) : threads_ (1)
      {
        setNumberOfThreads (nr_threads);
        feature_name_ = "GeoNormalEstimationOMP";
      }

      /** \brief Initialize the scheduler and set the number of threads to use.
        * \param nr_threads the number of hardware threads to use (-1 sets the value back to automatic)
        */
      inline void 
      setNumberOfThreads (unsigned int nr_threads)
      { 
        if (nr_threads == 0)
          nr_threads = 1;
        threads_ = nr_threads; 
      }


    protected:
      /** \brief The number of threads the scheduler should use. */
      unsigned int threads_;

    private:
      /** \brief Estimate normals for all points given in <setInputCloud (), setIndices ()> using the surface in
        * setSearchSurface () and the spatial locator in setSearchMethod ()
        * \param output the resultant point cloud model dataset that contains surface normals and curvatures
        */
      void 
      computeFeature (PointCloudOut &output);

      /** \brief Make the computeFeature (&Eigen::MatrixXf); inaccessible from outside the class
        * \param[out] output the output point cloud 
        */
      void 
      computeFeatureEigen (pcl::PointCloud<Eigen::MatrixXf> &) {}
  };

  /** \brief GeoNormalEstimationOMP estimates local surface properties at each 3D point, such as surface normals and
    * curvatures, in parallel, using the OpenMP standard.
    * \author Radu Bogdan Rusu
    * \ingroup features
    */
  template <typename PointInT>
  class GeoNormalEstimationOMP<PointInT, Eigen::MatrixXf>: public GeoNormalEstimationOMP<PointInT, pcl::Normal>
  {
    public:
      using GeoNormalEstimationOMP<PointInT, pcl::Normal>::indices_;
      using GeoNormalEstimationOMP<PointInT, pcl::Normal>::search_parameter_;
      using GeoNormalEstimationOMP<PointInT, pcl::Normal>::k_;
      using GeoNormalEstimationOMP<PointInT, pcl::Normal>::input_;
      using GeoNormalEstimationOMP<PointInT, pcl::Normal>::surface_;
      using GeoNormalEstimationOMP<PointInT, pcl::Normal>::getViewPoint;
      using GeoNormalEstimationOMP<PointInT, pcl::Normal>::threads_;
      using GeoNormalEstimationOMP<PointInT, pcl::Normal>::compute;

      /** \brief Default constructor.
        */
      GeoNormalEstimationOMP () : GeoNormalEstimationOMP<PointInT, pcl::Normal> () {}

      /** \brief Initialize the scheduler and set the number of threads to use.
        * \param nr_threads the number of hardware threads to use (-1 sets the value back to automatic)
        */
      GeoNormalEstimationOMP (unsigned int nr_threads) : GeoNormalEstimationOMP<PointInT, pcl::Normal> (nr_threads) {}

    private:
      /** \brief Estimate normals for all points given in <setInputCloud (), setIndices ()> using the surface in
        * setSearchSurface () and the spatial locator in setSearchMethod ()
        * \param output the resultant point cloud model dataset that contains surface normals and curvatures
        */
      void 
      computeFeatureEigen (pcl::PointCloud<Eigen::MatrixXf> &output);

      /** \brief Make the compute (&PointCloudOut); inaccessible from outside the class
        * \param[out] output the output point cloud 
        */
      void 
      compute (pcl::PointCloud<pcl::Normal> &) {}
    };

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT> void
GeoNormalEstimationOMP<PointInT, Eigen::MatrixXf>::computeFeatureEigen (pcl::PointCloud<Eigen::MatrixXf> &output)
{
  float vpx, vpy, vpz;
  getViewPoint (vpx, vpy, vpz);
  output.is_dense = true;

  // Resize the output dataset
  output.points.resize (indices_->size (), 4);

  // GCC 4.2.x seems to segfault with "internal compiler error" on MacOS X here
#if defined(_WIN32) || ((__GNUC__ > 4) && (__GNUC_MINOR__ > 2)) 
#pragma omp parallel for schedule (dynamic, threads_)
#endif
  // Iterating over the entire index vector
  for (int idx = 0; idx < static_cast<int> (indices_->size ()); ++idx)
  {
    // Allocate enough space to hold the results
    // \note This resize is irrelevant for a radiusSearch ().
    std::vector<int> nn_indices (k_);
    std::vector<float> nn_dists (k_);

    if (!isFinite ((*input_)[(*indices_)[idx]]) ||
        this->searchForNeighbors ((*indices_)[idx], search_parameter_, nn_indices, nn_dists) == 0)
    {
      output.points (idx, 0) = output.points (idx, 1) = output.points (idx, 2) = output.points (idx, 3) = std::numeric_limits<float>::quiet_NaN ();
      output.is_dense = false;
      continue;
    }

    // 16-bytes aligned placeholder for the XYZ centroid of a surface patch
    Eigen::Vector4f xyz_centroid;
    // Estimate the XYZ centroid
    compute3DCentroid (*surface_, nn_indices, xyz_centroid);

    // Placeholder for the 3x3 covariance matrix at each surface patch
    EIGEN_ALIGN16 Eigen::Matrix3f covariance_matrix;
    // Compute the 3x3 covariance matrix
    computeCovarianceMatrix (*surface_, nn_indices, xyz_centroid, covariance_matrix);

    // Get the plane normal and surface curvature
    pcl::solvePlaneParameters (covariance_matrix,
                          output.points (idx, 0), output.points (idx, 1), output.points (idx, 2), output.points (idx, 3));

    flipNormalTowardsViewpoint (input_->points[(*indices_)[idx]], vpx, vpy, vpz,
                                output.points (idx, 0), output.points (idx, 1), output.points (idx, 2));
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointInT, typename PointOutT> void
GeoNormalEstimationOMP<PointInT, PointOutT>::computeFeature (PointCloudOut &output)
{
  float vpx, vpy, vpz;
  getViewPoint (vpx, vpy, vpz);

  output.is_dense = true;
  // Iterating over the entire index vector
#pragma omp parallel for schedule (dynamic, threads_)
  for (int idx = 0; idx < static_cast<int> (indices_->size ()); ++idx)
  {
    // Allocate enough space to hold the results
    // \note This resize is irrelevant for a radiusSearch ().
    std::vector<int> nn_indices (k_);
    std::vector<float> nn_dists (k_);

    if (!isFinite ((*input_)[(*indices_)[idx]]) ||
        this->searchForNeighbors ((*indices_)[idx], search_parameter_, nn_indices, nn_dists) == 0)
    {
      output.points[idx].normal[0] = output.points[idx].normal[1] = output.points[idx].normal[2] = output.points[idx].curvature = std::numeric_limits<float>::quiet_NaN ();
  
      output.is_dense = false;
      continue;
    }

    // 16-bytes aligned placeholder for the XYZ centroid of a surface patch
    Eigen::Vector4f xyz_centroid;
    // Estimate the XYZ centroid
    compute3DCentroid (*surface_, nn_indices, xyz_centroid);

    // Placeholder for the 3x3 covariance matrix at each surface patch
    EIGEN_ALIGN16 Eigen::Matrix3f covariance_matrix;
    // Compute the 3x3 covariance matrix
    computeCovarianceMatrix (*surface_, nn_indices, xyz_centroid, covariance_matrix);

    // Get the plane normal and surface curvature
    pcl::solvePlaneParameters (covariance_matrix,
                          output.points[idx].normal[0], output.points[idx].normal[1], output.points[idx].normal[2], output.points[idx].curvature);

    flipNormalTowardsViewpoint (input_->points[(*indices_)[idx]], vpx, vpy, vpz,
                                output.points[idx].normal[0], output.points[idx].normal[1], output.points[idx].normal[2]);
  }
}
