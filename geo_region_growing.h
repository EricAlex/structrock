/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
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
 * Author : Sergey Ushakov
 * Email  : mine_all_mine@bk.ru
 *
 */
 
 /*
  * Modified by Xin Wang
  * Email : ericrussell@zju.edu.cn
  */

#pragma once
#include <pcl/pcl_base.h>
#include <pcl/search/search.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <list>
#include <math.h>
#include <time.h>
#include <queue>


/** \brief
    * Implements the well known Region Growing algorithm used for segmentation.
    * Description can be found in the article
    * "Segmentation of point clouds using smoothness constraint"
    * by T. Rabbania, F. A. van den Heuvelb, G. Vosselmanc.
    * In addition to residual test, the possibility to test curvature is added.
    */
  template <typename PointT, typename NormalT>
  class GeoRegionGrowing : public pcl::PCLBase<PointT>
  {
    public:

      typedef pcl::search::Search <PointT> KdTree;
      typedef typename KdTree::Ptr KdTreePtr;
      typedef pcl::PointCloud <NormalT> Normal;
      typedef typename Normal::Ptr NormalPtr;
      typedef pcl::PointCloud <PointT> PointCloud;

      using pcl::PCLBase <PointT>::input_;
      using pcl::PCLBase <PointT>::indices_;
      using pcl::PCLBase <PointT>::initCompute;
      using pcl::PCLBase <PointT>::deinitCompute;

    public:

      /** \brief Constructor that sets default values for member variables. */
      GeoRegionGrowing ();

      /** \brief This destructor destroys the cloud, normals and search method used for
        * finding KNN. In other words it frees memory.
        */
      virtual
      ~GeoRegionGrowing ();

      /** \brief Get the minimum number of points that a cluster needs to contain in order to be considered valid. */
      int
      getMinClusterSize ();

      /** \brief Set the minimum number of points that a cluster needs to contain in order to be considered valid. */
      void
      setMinClusterSize (int min_cluster_size);

      /** \brief Get the maximum number of points that a cluster needs to contain in order to be considered valid. */
      int
      getMaxClusterSize ();

      /** \brief Set the maximum number of points that a cluster needs to contain in order to be considered valid. */
      void
      setMaxClusterSize (int max_cluster_size);

      /** \brief Returns the flag value. This flag signalizes which mode of algorithm will be used.
        * If it is set to true than it will work as said in the article. This means that
        * it will be testing the angle between normal of the current point and it's neighbours normal.
        * Otherwise, it will be testing the angle between normal of the current point
        * and normal of the initial point that was chosen for growing new segment.
        */
      bool
      getSmoothModeFlag () const;

      /** \brief This function allows to turn on/off the smoothness constraint.
        * \param[in] value new mode value, if set to true then the smooth version will be used.
        */
      void
      setSmoothModeFlag (bool value);

      /** \brief Returns the flag that signalize if the curvature test is turned on/off. */
      bool
      getCurvatureTestFlag () const;

      /** \brief Allows to turn on/off the curvature test. Note that at least one test
        * (residual or curvature) must be turned on. If you are turning curvature test off
        * then residual test will be turned on automatically.
        * \param[in] value new value for curvature test. If set to true then the test will be turned on
        */
      virtual void
      setCurvatureTestFlag (bool value);

      /** \brief Returns the flag that signalize if the residual test is turned on/off. */
      bool
      getResidualTestFlag () const;

      /** \brief
        * Allows to turn on/off the residual test. Note that at least one test
        * (residual or curvature) must be turned on. If you are turning residual test off
        * then curvature test will be turned on automatically.
        * \param[in] value new value for residual test. If set to true then the test will be turned on
        */
      virtual void
      setResidualTestFlag (bool value);

      /** \brief Returns smoothness threshold. */
      float
      getSmoothnessThreshold () const;

      /** \brief Allows to set smoothness threshold used for testing the points.
        * \param[in] theta new threshold value for the angle between normals
        */
      void
      setSmoothnessThreshold (float theta);

      /** \brief Returns residual threshold. */
      float
      getResidualThreshold () const;

      /** \brief Allows to set residual threshold used for testing the points.
        * \param[in] residual new threshold value for residual testing
        */
      void
      setResidualThreshold (float residual);

      /** \brief Returns curvature threshold. */
      float
      getCurvatureThreshold () const;

      /** \brief Allows to set curvature threshold used for testing the points.
        * \param[in] curvature new threshold value for curvature testing
        */
      void
      setCurvatureThreshold (float curvature);

      /** \brief Returns the number of nearest neighbours used for KNN. */
      unsigned int
      getNumberOfNeighbours () const;

      /** \brief Allows to set the number of neighbours. For more information check the article.
        * \param[in] neighbour_number number of neighbours to use
        */
      void
      setNumberOfNeighbours (unsigned int neighbour_number);

      /** \brief Returns the pointer to the search method that is used for KNN. */
      KdTreePtr
      getSearchMethod () const;

      /** \brief Allows to set search method that will be used for finding KNN.
        * \param[in] search search method to use
        */
      void
      setSearchMethod (const KdTreePtr& tree);

      /** \brief Returns normals. */
      NormalPtr
      getInputNormals () const;

      /** \brief This method sets the normals. They are needed for the algorithm, so if
        * no normals will be set, the algorithm would not be able to segment the points.
        * \param[in] norm normals that will be used in the algorithm
        */
      void
      setInputNormals (const NormalPtr& norm);

      /** \brief This method launches the segmentation algorithm and returns the clusters that were
        * obtained during the segmentation.
        * \param[out] clusters clusters that were obtained. Each cluster is an array of point indices.
        */
      virtual void
      extract (std::vector <pcl::PointIndices>& clusters);

      /** \brief For a given point this function builds a segment to which it belongs and returns this segment.
        * \param[in] index index of the initial point which will be the seed for growing a segment.
        * \param[out] cluster cluster to which the point belongs.
        */
      virtual void
      getSegmentFromPoint (int index, pcl::PointIndices& cluster);

      /** \brief If the cloud was successfully segmented, then function
        * returns colored cloud. Otherwise it returns an empty pointer.
        * Points that belong to the same segment have the same color.
        * But this function doesn't guarantee that different segments will have different
        * color(it all depends on RNG). Points that were not listed in the indices array will have red color.
        */
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr
      getColoredCloud ();

      /** \brief If the cloud was successfully segmented, then function
        * returns colored cloud. Otherwise it returns an empty pointer.
        * Points that belong to the same segment have the same color.
        * But this function doesn't guarantee that different segments will have different
        * color(it all depends on RNG). Points that were not listed in the indices array will have red color.
        */
      pcl::PointCloud<pcl::PointXYZRGBA>::Ptr
      getColoredCloudRGBA ();

    protected:

      /** \brief This method simply checks if it is possible to execute the segmentation algorithm with
        * the current settings. If it is possible then it returns true.
        */
      virtual bool
      prepareForSegmentation ();

      /** \brief This method finds KNN for each point and saves them to the array
        * because the algorithm needs to find KNN a few times.
        */
      virtual void
      findPointNeighbours ();

      /** \brief This function implements the algorithm described in the article
        * "Segmentation of point clouds using smoothness constraint"
        * by T. Rabbania, F. A. van den Heuvelb, G. Vosselmanc.
        */
      void
      applySmoothRegionGrowingAlgorithm ();

      /** \brief This method grows a segment for the given seed point. And returns the number of its points.
        * \param[in] initial_seed index of the point that will serve as the seed point
        * \param[in] segment_number indicates which number this segment will have
        */
      int
      growRegion (int initial_seed, int segment_number);

      /** \brief This function is checking if the point with index 'nghbr' belongs to the segment.
        * If so, then it returns true. It also checks if this point can serve as the seed.
        * \param[in] initial_seed index of the initial point that was passed to the growRegion() function
        * \param[in] point index of the current seed point
        * \param[in] nghbr index of the point that is neighbour of the current seed
        * \param[out] is_a_seed this value is set to true if the point with index 'nghbr' can serve as the seed
        */
      virtual bool
      validatePoint (int initial_seed, int point, int nghbr, bool& is_a_seed) const;

      /** \brief This function simply assembles the regions from list of point labels.
        * \param[out] clusters clusters that were obtained during the segmentation process.
        * Each cluster is an array of point indices.
        */
      void
      assembleRegions ();

    protected:

      /** \brief Stores the minimum number of points that a cluster needs to contain in order to be considered valid. */
      int min_pts_per_cluster_;

      /** \brief Stores the maximum number of points that a cluster needs to contain in order to be considered valid. */
      int max_pts_per_cluster_;

      /** \brief Flag that signalizes if the smoothness constraint will be used. */
      bool smooth_mode_flag_;

      /** \brief If set to true then curvature test will be done during segmentation. */
      bool curvature_flag_;

      /** \brief If set to true then residual test will be done during segmentation. */
      bool residual_flag_;

      /** \brief Thershold used for testing the smoothness between points. */
      float theta_threshold_;

      /** \brief Thershold used in residual test. */
      float residual_threshold_;

      /** \brief Thershold used in curvature test. */
      float curvature_threshold_;

      /** \brief Number of neighbours to find. */
      unsigned int neighbour_number_;

      /** \brief Serch method that will be used for KNN. */
      KdTreePtr search_;

      /** \brief Contains normals of the points that will be segmented. */
      NormalPtr normals_;

      /** \brief Contains neighbours of each point. */
      std::vector<std::vector<int> > point_neighbours_;

      /** \brief Point labels that tells to which segment each point belongs. */
      std::vector<int> point_labels_;

      /** \brief If set to true then normal/smoothness test will be done during segmentation.
        * It is always set to true for the usual region growing algorithm. It is used for turning on/off the test
        * for smoothness in the child class RegionGrowingRGB.*/
      bool normal_flag_;

      /** \brief Tells how much points each segment contains. Used for reserving memory. */
      std::vector<int> num_pts_in_segment_;

      /** \brief After the segmentation it will contain the segments. */
      std::vector <pcl::PointIndices> clusters_;

      /** \brief Stores the number of segments. */
      int number_of_segments_;

    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

  /** \brief This function is used as a comparator for sorting. */
  inline bool
  comparePair (std::pair<float, int> i, std::pair<float, int> j)
  {
    return (i.first < j.first);
  }


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT>
GeoRegionGrowing<PointT, NormalT>::GeoRegionGrowing () :
  min_pts_per_cluster_ (1),
  max_pts_per_cluster_ (std::numeric_limits<int>::max ()),
  smooth_mode_flag_ (false),
  curvature_flag_ (true),
  residual_flag_ (true),
  theta_threshold_ (30.0f / 180.0f * static_cast<float> (M_PI)),
  residual_threshold_ (0.05f),
  curvature_threshold_ (0.05f),
  neighbour_number_ (30),
  search_ (),
  normals_ (),
  point_neighbours_ (0),
  point_labels_ (0),
  normal_flag_ (true),
  num_pts_in_segment_ (0),
  clusters_ (0),
  number_of_segments_ (0)
{
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT>
GeoRegionGrowing<PointT, NormalT>::~GeoRegionGrowing ()
{
  if (search_ != 0)
    search_.reset ();
  if (normals_ != 0)
    normals_.reset ();

  point_neighbours_.clear ();
  point_labels_.clear ();
  num_pts_in_segment_.clear ();
  clusters_.clear ();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT> int
GeoRegionGrowing<PointT, NormalT>::getMinClusterSize ()
{
  return (min_pts_per_cluster_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT> void
GeoRegionGrowing<PointT, NormalT>::setMinClusterSize (int min_cluster_size)
{
  min_pts_per_cluster_ = min_cluster_size;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT> int
GeoRegionGrowing<PointT, NormalT>::getMaxClusterSize ()
{
  return (max_pts_per_cluster_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT> void
GeoRegionGrowing<PointT, NormalT>::setMaxClusterSize (int max_cluster_size)
{
  max_pts_per_cluster_ = max_cluster_size;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT> bool
GeoRegionGrowing<PointT, NormalT>::getSmoothModeFlag () const
{
  return (smooth_mode_flag_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT> void
GeoRegionGrowing<PointT, NormalT>::setSmoothModeFlag (bool value)
{
  smooth_mode_flag_ = value;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT> bool
GeoRegionGrowing<PointT, NormalT>::getCurvatureTestFlag () const
{
  return (curvature_flag_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT> void
GeoRegionGrowing<PointT, NormalT>::setCurvatureTestFlag (bool value)
{
  curvature_flag_ = value;

  if (curvature_flag_ == false && residual_flag_ == false)
    residual_flag_ = true;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT> bool
GeoRegionGrowing<PointT, NormalT>::getResidualTestFlag () const
{
  return (residual_flag_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT> void
GeoRegionGrowing<PointT, NormalT>::setResidualTestFlag (bool value)
{
  residual_flag_ = value;

  if (curvature_flag_ == false && residual_flag_ == false)
    curvature_flag_ = true;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT> float
GeoRegionGrowing<PointT, NormalT>::getSmoothnessThreshold () const
{
  return (theta_threshold_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT> void
GeoRegionGrowing<PointT, NormalT>::setSmoothnessThreshold (float theta)
{
  theta_threshold_ = theta;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT> float
GeoRegionGrowing<PointT, NormalT>::getResidualThreshold () const
{
  return (residual_threshold_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT> void
GeoRegionGrowing<PointT, NormalT>::setResidualThreshold (float residual)
{
  residual_threshold_ = residual;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT> float
GeoRegionGrowing<PointT, NormalT>::getCurvatureThreshold () const
{
  return (curvature_threshold_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT> void
GeoRegionGrowing<PointT, NormalT>::setCurvatureThreshold (float curvature)
{
  curvature_threshold_ = curvature;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT> unsigned int
GeoRegionGrowing<PointT, NormalT>::getNumberOfNeighbours () const
{
  return (neighbour_number_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT> void
GeoRegionGrowing<PointT, NormalT>::setNumberOfNeighbours (unsigned int neighbour_number)
{
  neighbour_number_ = neighbour_number;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT> typename GeoRegionGrowing<PointT, NormalT>::KdTreePtr
GeoRegionGrowing<PointT, NormalT>::getSearchMethod () const
{
  return (search_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT> void
GeoRegionGrowing<PointT, NormalT>::setSearchMethod (const KdTreePtr& tree)
{
  if (search_ != 0)
    search_.reset ();

  search_ = tree;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT> typename GeoRegionGrowing<PointT, NormalT>::NormalPtr
GeoRegionGrowing<PointT, NormalT>::getInputNormals () const
{
  return (normals_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT> void
GeoRegionGrowing<PointT, NormalT>::setInputNormals (const NormalPtr& norm)
{
  if (normals_ != 0)
    normals_.reset ();

  normals_ = norm;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT> void
GeoRegionGrowing<PointT, NormalT>::extract (std::vector <pcl::PointIndices>& clusters)
{
  clusters_.clear ();
  clusters.clear ();
  point_neighbours_.clear ();
  point_labels_.clear ();
  num_pts_in_segment_.clear ();
  number_of_segments_ = 0;

  bool segmentation_is_possible = initCompute ();
  if ( !segmentation_is_possible )
  {
    deinitCompute ();
    return;
  }

  segmentation_is_possible = prepareForSegmentation ();
  if ( !segmentation_is_possible )
  {
    deinitCompute ();
    return;
  }

  findPointNeighbours ();
  applySmoothRegionGrowingAlgorithm ();
  assembleRegions ();

  clusters.resize (clusters_.size ());
  std::vector<pcl::PointIndices>::iterator cluster_iter_input = clusters.begin ();
  for (std::vector<pcl::PointIndices>::const_iterator cluster_iter = clusters_.begin (); cluster_iter != clusters_.end (); cluster_iter++)
  {
    if ((cluster_iter->indices.size () >= min_pts_per_cluster_) && 
        (cluster_iter->indices.size () <= max_pts_per_cluster_))
    {
      *cluster_iter_input = *cluster_iter;
      cluster_iter_input++;
    }
  }

  clusters_ = std::vector<pcl::PointIndices> (clusters.begin (), cluster_iter_input);
  clusters.resize(clusters_.size());

  deinitCompute ();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT> bool
GeoRegionGrowing<PointT, NormalT>::prepareForSegmentation ()
{
  // if user forgot to pass point cloud or if it is empty
  if ( input_->points.size () == 0 )
    return (false);

  // if user forgot to pass normals or the sizes of point and normal cloud are different
  if ( normals_ == 0 || input_->points.size () != normals_->points.size () )
    return (false);

  // if residual test is on then we need to check if all needed parameters were correctly initialized
  if (residual_flag_)
  {
    if (residual_threshold_ <= 0.0f)
      return (false);
  }

  // if curvature test is on ...
  // if (curvature_flag_)
  // {
  //   in this case we do not need to check anything that related to it
  //   so we simply commented it
  // }

  // from here we check those parameters that are always valuable
  if (neighbour_number_ == 0)
    return (false);

  // if user didn't set search method
  if (!search_)
    search_.reset (new pcl::search::KdTree<PointT>);

  if (indices_)
  {
    if (indices_->empty ())
      PCL_ERROR ("[pcl::RegionGrowing::prepareForSegmentation] Empty given indices!\n");
    search_->setInputCloud (input_, indices_);
  }
  else
    search_->setInputCloud (input_);

  return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT> void
GeoRegionGrowing<PointT, NormalT>::findPointNeighbours ()
{
  int point_number = static_cast<int> (indices_->size ());
  std::vector<int> neighbours;
  std::vector<float> distances;

  point_neighbours_.resize (input_->points.size (), neighbours);

  for (int i_point = 0; i_point < point_number; i_point++)
  {
    int point_index = (*indices_)[i_point];
    neighbours.clear ();
    search_->nearestKSearch (i_point, neighbour_number_, neighbours, distances);
    point_neighbours_[point_index].swap (neighbours);
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT> void
GeoRegionGrowing<PointT, NormalT>::applySmoothRegionGrowingAlgorithm ()
{
  int num_of_pts = static_cast<int> (indices_->size ());
  point_labels_.resize (input_->points.size (), -1);

  std::vector< std::pair<float, int> > point_residual;
  std::pair<float, int> pair;
  point_residual.resize (num_of_pts, pair);

  if (normal_flag_ == true)
  {
    for (int i_point = 0; i_point < num_of_pts; i_point++)
    {
      int point_index = (*indices_)[i_point];
      point_residual[i_point].first = normals_->points[point_index].curvature;
      point_residual[i_point].second = point_index;
    }
    std::sort (point_residual.begin (), point_residual.end (), comparePair);
  }
  else
  {
    for (int i_point = 0; i_point < num_of_pts; i_point++)
    {
      int point_index = (*indices_)[i_point];
      point_residual[i_point].first = 0;
      point_residual[i_point].second = point_index;
    }
  }
  int seed_counter = 0;
  int seed = point_residual[seed_counter].second;

  int segmented_pts_num = 0;
  int number_of_segments = 0;
  while (segmented_pts_num < num_of_pts)
  {
    int pts_in_segment;
    pts_in_segment = growRegion (seed, number_of_segments);
    segmented_pts_num += pts_in_segment;
    num_pts_in_segment_.push_back (pts_in_segment);
    number_of_segments++;

    //find next point that is not segmented yet
    for (int i_seed = seed_counter + 1; i_seed < num_of_pts; i_seed++)
    {
      int index = point_residual[i_seed].second;
      if (point_labels_[index] == -1)
      {
        seed = index;
        break;
      }
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT> int
GeoRegionGrowing<PointT, NormalT>::growRegion (int initial_seed, int segment_number)
{
  std::queue<int> seeds;
  seeds.push (initial_seed);
  point_labels_[initial_seed] = segment_number;

  int num_pts_in_segment = 1;

  while (!seeds.empty ())
  {
    int curr_seed;
    curr_seed = seeds.front ();
    seeds.pop ();

    size_t i_nghbr = 0;
    while ( i_nghbr < neighbour_number_ && i_nghbr < point_neighbours_[curr_seed].size () )
    {
      int index = point_neighbours_[curr_seed][i_nghbr];
      if (point_labels_[index] != -1)
      {
        i_nghbr++;
        continue;
      }

      bool is_a_seed = false;
      bool belongs_to_segment = validatePoint (initial_seed, curr_seed, index, is_a_seed);

      if (belongs_to_segment == false)
      {
        i_nghbr++;
        continue;
      }

      point_labels_[index] = segment_number;
      num_pts_in_segment++;

      if (is_a_seed)
      {
        seeds.push (index);
      }

      i_nghbr++;
    }// next neighbour
  }// next seed

  return (num_pts_in_segment);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT> bool
GeoRegionGrowing<PointT, NormalT>::validatePoint (int initial_seed, int point, int nghbr, bool& is_a_seed) const
{
  is_a_seed = true;

  float cosine_threshold = cosf (theta_threshold_);
  float cosine_residual_threshold = cosf(residual_threshold_);
  float data[4];

  data[0] = input_->points[point].data[0];
  data[1] = input_->points[point].data[1];
  data[2] = input_->points[point].data[2];
  data[3] = input_->points[point].data[3];
  Eigen::Map<Eigen::Vector3f> initial_point (static_cast<float*> (data));
  Eigen::Map<Eigen::Vector3f> initial_normal (static_cast<float*> (normals_->points[point].normal));

  //check the angle between normals
  Eigen::Map<Eigen::Vector3f> nghbr_normal (static_cast<float*> (normals_->points[nghbr].normal));
  float dot_product = fabsf (nghbr_normal.dot (initial_normal));
  if (dot_product < cosine_threshold)
  {
	  return (false);
  }
  
  if (smooth_mode_flag_ == true)
  {
	  // check the curvature if needed
	  if (curvature_flag_ && normals_->points[nghbr].curvature > curvature_threshold_)
	  {
		  is_a_seed = false;
	  }
  }
  else
  {
	  // check the curvature if needed
	  if (curvature_flag_ && normals_->points[nghbr].curvature > curvature_threshold_)
	  {
		  is_a_seed = false;
	  }
	  // check the residual if needed
	  Eigen::Map<Eigen::Vector3f> nghbr_normal (static_cast<float*> (normals_->points[nghbr].normal));
	  Eigen::Map<Eigen::Vector3f> initial_seed_normal (static_cast<float*> (normals_->points[initial_seed].normal));
	  float dot_product = fabsf (nghbr_normal.dot (initial_seed_normal));
	  if (dot_product < cosine_residual_threshold)
	  {
		  is_a_seed = false;
	  }
  }

  return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT> void
GeoRegionGrowing<PointT, NormalT>::assembleRegions ()
{
  int number_of_segments = static_cast<int> (num_pts_in_segment_.size ());
  int number_of_points = static_cast<int> (input_->points.size ());

  pcl::PointIndices segment;
  clusters_.resize (number_of_segments, segment);

  for (int i_seg = 0; i_seg < number_of_segments; i_seg++)
  {
    clusters_[i_seg].indices.resize ( num_pts_in_segment_[i_seg], 0);
  }

  std::vector<int> counter;
  counter.resize (number_of_segments, 0);

  for (int i_point = 0; i_point < number_of_points; i_point++)
  {
    int segment_index = point_labels_[i_point];
    if (segment_index != -1)
    {
      int point_index = counter[segment_index];
      clusters_[segment_index].indices[point_index] = i_point;
      counter[segment_index] = point_index + 1;
    }
  }

  number_of_segments_ = number_of_segments;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT> void
GeoRegionGrowing<PointT, NormalT>::getSegmentFromPoint (int index, pcl::PointIndices& cluster)
{
  cluster.indices.clear ();

  bool segmentation_is_possible = initCompute ();
  if ( !segmentation_is_possible )
  {
    deinitCompute ();
    return;
  }

  // first of all we need to find out if this point belongs to cloud
  bool point_was_found = false;
  int number_of_points = static_cast <int> (indices_->size ());
  for (size_t point = 0; point < number_of_points; point++)
    if ( (*indices_)[point] == index)
    {
      point_was_found = true;
      break;
    }

  if (point_was_found)
  {
    if (clusters_.empty ())
    {
      point_neighbours_.clear ();
      point_labels_.clear ();
      num_pts_in_segment_.clear ();
      number_of_segments_ = 0;

      segmentation_is_possible = prepareForSegmentation ();
      if ( !segmentation_is_possible )
      {
        deinitCompute ();
        return;
      }

      findPointNeighbours ();
      applySmoothRegionGrowingAlgorithm ();
      assembleRegions ();
    }
    // if we have already made the segmentation, then find the segment
    // to which this point belongs
    std::vector <pcl::PointIndices>::iterator i_segment;
    for (i_segment = clusters_.begin (); i_segment != clusters_.end (); i_segment++)
    {
      bool segment_was_found = false;
      for (size_t i_point = 0; i_point < i_segment->indices.size (); i_point++)
      {
        if (i_segment->indices[i_point] == index)
        {
          segment_was_found = true;
          cluster.indices.clear ();
          cluster.indices.reserve (i_segment->indices.size ());
          std::copy (i_segment->indices.begin (), i_segment->indices.end (), std::back_inserter (cluster.indices));
          break;
        }
      }
      if (segment_was_found)
      {
        break;
      }
    }// next segment
  }// end if point was found

  deinitCompute ();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT> pcl::PointCloud<pcl::PointXYZRGB>::Ptr
GeoRegionGrowing<PointT, NormalT>::getColoredCloud ()
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud;

  if (!clusters_.empty ())
  {
    colored_cloud = (new pcl::PointCloud<pcl::PointXYZRGB>)->makeShared ();

    srand (static_cast<unsigned int> (time (0)));
    std::vector<unsigned char> colors;
    for (size_t i_segment = 0; i_segment < clusters_.size (); i_segment++)
    {
      colors.push_back (static_cast<unsigned char> (rand () % 256));
      colors.push_back (static_cast<unsigned char> (rand () % 256));
      colors.push_back (static_cast<unsigned char> (rand () % 256));
    }

    colored_cloud->width = input_->width;
    colored_cloud->height = input_->height;
    colored_cloud->is_dense = input_->is_dense;
    for (size_t i_point = 0; i_point < input_->points.size (); i_point++)
    {
      pcl::PointXYZRGB point;
      point.x = *(input_->points[i_point].data);
      point.y = *(input_->points[i_point].data + 1);
      point.z = *(input_->points[i_point].data + 2);
      point.r = 255;
      point.g = 0;
      point.b = 0;
      colored_cloud->points.push_back (point);
    }

    std::vector< pcl::PointIndices >::iterator i_segment;
    int next_color = 0;
    for (i_segment = clusters_.begin (); i_segment != clusters_.end (); i_segment++)
    {
      std::vector<int>::iterator i_point;
      for (i_point = i_segment->indices.begin (); i_point != i_segment->indices.end (); i_point++)
      {
        int index;
        index = *i_point;
        colored_cloud->points[index].r = colors[3 * next_color];
        colored_cloud->points[index].g = colors[3 * next_color + 1];
        colored_cloud->points[index].b = colors[3 * next_color + 2];
      }
      next_color++;
    }
  }

  return (colored_cloud);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointT, typename NormalT> pcl::PointCloud<pcl::PointXYZRGBA>::Ptr
GeoRegionGrowing<PointT, NormalT>::getColoredCloudRGBA ()
{
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr colored_cloud;

  if (!clusters_.empty ())
  {
    colored_cloud = (new pcl::PointCloud<pcl::PointXYZRGBA>)->makeShared ();

    srand (static_cast<unsigned int> (time (0)));
    std::vector<unsigned char> colors;
    for (size_t i_segment = 0; i_segment < clusters_.size (); i_segment++)
    {
      colors.push_back (static_cast<unsigned char> (rand () % 256));
      colors.push_back (static_cast<unsigned char> (rand () % 256));
      colors.push_back (static_cast<unsigned char> (rand () % 256));
    }

    colored_cloud->width = input_->width;
    colored_cloud->height = input_->height;
    colored_cloud->is_dense = input_->is_dense;
    for (size_t i_point = 0; i_point < input_->points.size (); i_point++)
    {
      pcl::PointXYZRGBA point;
      point.x = *(input_->points[i_point].data);
      point.y = *(input_->points[i_point].data + 1);
      point.z = *(input_->points[i_point].data + 2);
      point.r = 255;
      point.g = 0;
      point.b = 0;
      point.a = 0;
      colored_cloud->points.push_back (point);
    }

    std::vector< pcl::PointIndices >::iterator i_segment;
    int next_color = 0;
    for (i_segment = clusters_.begin (); i_segment != clusters_.end (); i_segment++)
    {
      std::vector<int>::iterator i_point;
      for (i_point = i_segment->indices.begin (); i_point != i_segment->indices.end (); i_point++)
      {
        int index;
        index = *i_point;
        colored_cloud->points[index].r = colors[3 * next_color];
        colored_cloud->points[index].g = colors[3 * next_color + 1];
        colored_cloud->points[index].b = colors[3 * next_color + 2];
      }
      next_color++;
    }
  }

  return (colored_cloud);
}

