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
#include <pcl/io/io.h>
#include <pcl/point_types.h>
#include <Eigen/src/Core/Matrix.h>
#include "ShowSFeatureWorker.h"
#include "globaldef.h"
#include "dataLibrary.h"

bool ShowSFeatureWorker::is_para_satisfying(QString message)
{
	this->setParaSize(1);
	if(dataLibrary::Workflow[dataLibrary::current_workline_index].parameters.size()>0)
	{
		FeaturePara temp_para;
		temp_para.percent_out = 0.0f;
		bool is_feature_defined(false);
		std::string feature_str = dataLibrary::Workflow[dataLibrary::current_workline_index].parameters[0];
		if(feature_str == "roughness")
		{
			temp_para.feature_type = FEATURE_ROUGHNESS;
			is_feature_defined = true;
		}
		else if(feature_str == "area")
		{
			temp_para.feature_type = FEATURE_AREA;
			is_feature_defined = true;
		}
		else if(feature_str == "curvature")
		{
			temp_para.feature_type = FEATURE_CURVATURE;
			is_feature_defined = true;
		}
		else if(feature_str == "fracture_curvature")
		{
			temp_para.feature_type = FEATURE_FRACTURE_CURVATURE;
			is_feature_defined = true;
		}

		if(is_feature_defined)
		{
			this->setParaIndex(this->getParaSize());
			if(dataLibrary::Workflow[dataLibrary::current_workline_index].parameters.size()>this->getParaIndex())
			{
				float percent_out;
				std::stringstream ss(dataLibrary::Workflow[dataLibrary::current_workline_index].parameters[this->getParaIndex()]);
				ss >> percent_out;
				temp_para.percent_out = percent_out;
				this->setParaIndex(this->getParaIndex()+1);
			}
			this->setFPara(temp_para);
			return true;
		}
		else
		{
			message = QString("showsfeatures: The Name of The Surface Feature Not Correctly Provided.");
			return false;
		}
	}
	else
	{
		message = QString("showsfeature: No Feature Name Given.");
		return false;
	}
}
void ShowSFeatureWorker::prepare()
{
	this->setUnmute();
	this->setWriteLog();
}

void ShowSFeatureWorker::doWork()
{
	bool is_success(false);

	dataLibrary::Status = STATUS_SHOWSFEATURE;

	this->timer_start();

	if(this->getFPara().feature_type == FEATURE_ROUGHNESS)
	{
		if(dataLibrary::roughnesses.size() == 0)
		{
			emit showErrors(QString("ShowSFeatures (roughness): Please save the clusters first."));
		}
		else if(this->getFPara().percent_out<0.0 || this->getFPara().percent_out>0.5)
		{
			emit showErrors(QString("ShowSFeatures (roughness): Percentage Out was not correctly provided (0.0<Percentage Out<0.5)."));
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
			float max_val = temp_roughness[(int)(temp_roughness.size()*(1.0-this->getFPara().percent_out))];
			float min_val = temp_roughness[(int)(temp_roughness.size()*this->getFPara().percent_out)];
			
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
	else if(this->getFPara().feature_type == FEATURE_AREA)
	{
		if(dataLibrary::areas.size() == 0)
		{
			emit showErrors(QString("ShowSFeatures (area): Please save the clusters first."));
		}
		else if(this->getFPara().percent_out<0.0 || this->getFPara().percent_out>0.5)
		{
			emit showErrors(QString("ShowSFeatures (area): Percentage Out was not correctly provided (0.0<Percentage Out<0.5)."));
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
			float max_val = temp_area[(int)(temp_area.size()*(1.0-this->getFPara().percent_out))];
			float min_val = temp_area[(int)(temp_area.size()*this->getFPara().percent_out)];
			
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
	else if(this->getFPara().feature_type == FEATURE_CURVATURE)
	{
		if(dataLibrary::pointnormals->empty())
		{
			emit showErrors(QString("ShowSFeatures (curvature): You Haven't Extracted Any Normals Yet! Please Extract Normals First."));
		}
		else if(this->getFPara().percent_out<0.0 || this->getFPara().percent_out>0.5)
		{
			emit showErrors(QString("ShowSFeatures (curvature): Percentage Out was not correctly provided (0.0<Percentage Out<0.5)."));
		}
		else
		{
			//begin of processing
			//Clear data if needed
			if(!dataLibrary::cloudxyzrgb_features->empty())
				dataLibrary::cloudxyzrgb_features->clear();
			pcl::copyPointCloud(*dataLibrary::pointnormals, *dataLibrary::cloudxyzrgb_features);

			std::vector<float> temp_curvature;
			for(int i=0; i<dataLibrary::pointnormals->points.size(); i++)
			{
				temp_curvature.push_back(dataLibrary::pointnormals->points[i].curvature);
			}
			std::sort(temp_curvature.begin(), temp_curvature.end());
			float max_val = temp_curvature[(int)(temp_curvature.size()*(1.0-this->getFPara().percent_out))];
			float min_val = temp_curvature[(int)(temp_curvature.size()*this->getFPara().percent_out)];
			
			for(int j=0; j<dataLibrary::cloudxyzrgb_features->points.size(); j++)
			{
				unsigned char r, g, b;
				if(dataLibrary::pointnormals->points[j].curvature>max_val)
				{
					r=255;g=0;b=0;
				}
				else if(dataLibrary::pointnormals->points[j].curvature<min_val)
				{
					r=0;g=0;b=255;
				}
				else
				{
					float a = 4.0*(max_val - dataLibrary::pointnormals->points[j].curvature)/(max_val-min_val);
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
				dataLibrary::cloudxyzrgb_features->at(j).r = r;
				dataLibrary::cloudxyzrgb_features->at(j).g = g;
				dataLibrary::cloudxyzrgb_features->at(j).b = b;
			}

			is_success = true;
			//end of processing
		}
	}
	else if(this->getFPara().feature_type == FEATURE_FRACTURE_CURVATURE)
	{
		if(dataLibrary::pointnormals->empty())
		{
			emit showErrors(QString("ShowSFeatures (fracture_curvature): You Haven't Extracted Any Normals Yet! Please Extract Normals First."));
		}
		else if(dataLibrary::clusters.empty())
		{
			emit showErrors(QString("ShowSFeatures (fracture_curvature): Please perform the region growing segmentation first."));
		}
		else if(this->getFPara().percent_out<0.0 || this->getFPara().percent_out>0.5)
		{
			emit showErrors(QString("ShowSFeatures (fracture_curvature): Percentage Out was not correctly provided (0.0<Percentage Out<0.5)."));
		}
		else
		{
			//begin of processing
			//Clear data if needed
			if(!dataLibrary::cloudxyzrgb_features->empty())
				dataLibrary::cloudxyzrgb_features->clear();
			pcl::copyPointCloud(*dataLibrary::pointnormals, *dataLibrary::cloudxyzrgb_features);

			for(int i=0; i<dataLibrary::cloudxyzrgb_features->points.size(); i++)
			{
				dataLibrary::cloudxyzrgb_features->at(i).r = 255;
				dataLibrary::cloudxyzrgb_features->at(i).g = 255;
				dataLibrary::cloudxyzrgb_features->at(i).b = 255;
			}

			std::vector<float> temp_curvature;
			for(int cluster_index = 0; cluster_index < dataLibrary::clusters.size(); cluster_index++)
			{
				for(int j = 0; j < dataLibrary::clusters[cluster_index].indices.size(); j++)
				{
					temp_curvature.push_back(dataLibrary::pointnormals->points[dataLibrary::clusters[cluster_index].indices[j]].curvature);
				}
			}
			std::sort(temp_curvature.begin(), temp_curvature.end());
			float max_val = temp_curvature[(int)(temp_curvature.size()*(1.0-this->getFPara().percent_out))];
			float min_val = temp_curvature[(int)(temp_curvature.size()*this->getFPara().percent_out)];

			for(int cluster_index = 0; cluster_index < dataLibrary::clusters.size(); cluster_index++)
			{
				for(int j = 0; j < dataLibrary::clusters[cluster_index].indices.size(); j++)
				{
					unsigned char r, g, b;
					if(dataLibrary::pointnormals->points[dataLibrary::clusters[cluster_index].indices[j]].curvature>max_val)
					{
						r=255;g=0;b=0;
					}
					else if(dataLibrary::pointnormals->points[dataLibrary::clusters[cluster_index].indices[j]].curvature<min_val)
					{
						r=0;g=0;b=255;
					}
					else
					{
						float a = 4.0*(max_val - dataLibrary::pointnormals->points[dataLibrary::clusters[cluster_index].indices[j]].curvature)/(max_val-min_val);
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
					dataLibrary::cloudxyzrgb_features->at(dataLibrary::clusters[cluster_index].indices[j]).r = r;
					dataLibrary::cloudxyzrgb_features->at(dataLibrary::clusters[cluster_index].indices[j]).g = g;
					dataLibrary::cloudxyzrgb_features->at(dataLibrary::clusters[cluster_index].indices[j]).b = b;
				}
			}

			is_success = true;
			//end of processing
		}
	}

	this->timer_stop();

	if(this->getWriteLogMpde()&&is_success)
	{
		std::string log_text = "\tShowSFeature Costs: ";
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