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

#include <vector>
#include <algorithm>
#include <string>
#include <math.h>
#include <pcl/point_types.h>
#include <Eigen/src/Core/Matrix.h>
#include "ShowSFeatureWorker.h"
#include "globaldef.h"
#include "dataLibrary.h"

void ShowSFeatureWorker::doWork()
{
	bool is_success(false);

	dataLibrary::Status = STATUS_SHOWSFEATURE;

	dataLibrary::start = clock();

	if(dataLibrary::FeatureParameter.feature_type == FEATURE_ROUGHNESS)
	{
		if(dataLibrary::roughnesses.size() == 0)
		{
			emit showErrors(QString("ShowSFeatures (roughness): Please save the clusters first."));
		}
		else if(dataLibrary::FeatureParameter.percent_out<0.0 || dataLibrary::FeatureParameter.percent_out>0.5)
		{
			emit showErrors(QString("ShowSFeatures (roughness): Percentage Out was not correctly provided."));
		}
		else
		{
			//begin of processing
			//Clear data if needed
			if(!dataLibrary::cloudxyzrgb_features->empty())
				dataLibrary::cloudxyzrgb_features->clear();
			pcl::copyPointCloud(*dataLibrary::cloudxyzrgb_clusters, *dataLibrary::cloudxyzrgb_features);

			for(int i=0; i<dataLibrary::cloudxyzrgb_features->points.size(); i++)
			{
				dataLibrary::cloudxyzrgb_features->at(i).r = 255;
				dataLibrary::cloudxyzrgb_features->at(i).g = 255;
				dataLibrary::cloudxyzrgb_features->at(i).b = 255;
			}

			std::vector<float> temp_roughness(dataLibrary::roughnesses);
			std::sort(temp_roughness.begin(), temp_roughness.end());
			float max_val = temp_roughness[(int)(temp_roughness.size()*(1.0-dataLibrary::FeatureParameter.percent_out))];
			float min_val = temp_roughness[(int)(temp_roughness.size()*dataLibrary::FeatureParameter.percent_out)];
			
			for(int cluster_index = 0; cluster_index < dataLibrary::clusters.size(); cluster_index++)
			{
				unsigned char r, g, b;
				if(dataLibrary::roughnesses[cluster_index]>max_val)
				{
					r=255;g=0;b=0;
				}
				else if(dataLibrary::roughnesses[cluster_index]<min_val)
				{
					r=0;g=0;b=255;
				}
				else
				{
					float a = 4.0*(max_val - dataLibrary::roughnesses[cluster_index])/(max_val-min_val);
					int X = floor(a);
					int Y = floor(255*(a-X));
					switch(X)
					{
					case 0:r=255;g=Y;b=0; break;
					case 1:r=255-Y;g=255;b=0; break;
					case 2:r=0;g=255;b=Y; break;
					case 3:r=0;g=255-Y;b=255; break;
					case 4:r=0;g=0;b=255; break;
					}
				}
				for(int j = 0; j < dataLibrary::clusters[cluster_index].indices.size(); j++)
				{
					dataLibrary::cloudxyzrgb_features->at(dataLibrary::clusters[cluster_index].indices[j]).r = r;
					dataLibrary::cloudxyzrgb_features->at(dataLibrary::clusters[cluster_index].indices[j]).g = g;
					dataLibrary::cloudxyzrgb_features->at(dataLibrary::clusters[cluster_index].indices[j]).b = b;
				}
			}

			is_success = true;
			//end of processing
		}
	}
	else if(dataLibrary::FeatureParameter.feature_type == FEATURE_AREA)
	{
		if(dataLibrary::areas.size() == 0)
		{
			emit showErrors(QString("ShowSFeatures (area): Please save the clusters first."));
		}
		else if(dataLibrary::FeatureParameter.percent_out<0.0 || dataLibrary::FeatureParameter.percent_out>0.5)
		{
			emit showErrors(QString("ShowSFeatures (area): Percentage Out was not correctly provided."));
		}
		else
		{
			//begin of processing
			//Clear data if needed
			if(!dataLibrary::cloudxyzrgb_features->empty())
				dataLibrary::cloudxyzrgb_features->clear();
			pcl::copyPointCloud(*dataLibrary::cloudxyzrgb_clusters, *dataLibrary::cloudxyzrgb_features);

			for(int i=0; i<dataLibrary::cloudxyzrgb_features->points.size(); i++)
			{
				dataLibrary::cloudxyzrgb_features->at(i).r = 255;
				dataLibrary::cloudxyzrgb_features->at(i).g = 255;
				dataLibrary::cloudxyzrgb_features->at(i).b = 255;
			}

			std::vector<float> temp_area(dataLibrary::areas);
			std::sort(temp_area.begin(), temp_area.end());
			float max_val = temp_area[(int)(temp_area.size()*(1.0-dataLibrary::FeatureParameter.percent_out))];
			float min_val = temp_area[(int)(temp_area.size()*dataLibrary::FeatureParameter.percent_out)];
			
			for(int cluster_index = 0; cluster_index < dataLibrary::clusters.size(); cluster_index++)
			{
				unsigned char r, g, b;
				if(dataLibrary::areas[cluster_index]>max_val)
				{
					r=255;g=0;b=0;
				}
				else if(dataLibrary::areas[cluster_index]<min_val)
				{
					r=0;g=0;b=255;
				}
				else
				{
					float a = 4.0*(max_val - dataLibrary::areas[cluster_index])/(max_val-min_val);
					int X = floor(a);
					int Y = floor(255*(a-X));
					switch(X)
					{
					case 0:r=255;g=Y;b=0; break;
					case 1:r=255-Y;g=255;b=0; break;
					case 2:r=0;g=255;b=Y; break;
					case 3:r=0;g=255-Y;b=255; break;
					case 4:r=0;g=0;b=255; break;
					}
				}
				for(int j = 0; j < dataLibrary::clusters[cluster_index].indices.size(); j++)
				{
					dataLibrary::cloudxyzrgb_features->at(dataLibrary::clusters[cluster_index].indices[j]).r = r;
					dataLibrary::cloudxyzrgb_features->at(dataLibrary::clusters[cluster_index].indices[j]).g = g;
					dataLibrary::cloudxyzrgb_features->at(dataLibrary::clusters[cluster_index].indices[j]).b = b;
				}
			}

			is_success = true;
			//end of processing
		}
	}

	dataLibrary::finish = clock();

	if(this->getWriteLogMpde()&&is_success)
	{
		std::string log_text = "\tShowSFeature Costs: ";
		std::ostringstream strs;
		strs << (double)(dataLibrary::finish-dataLibrary::start)/CLOCKS_PER_SEC;
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