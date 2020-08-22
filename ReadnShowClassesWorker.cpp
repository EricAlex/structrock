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
#include <fstream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/voxel_grid.h>
#include <Eigen/src/Core/Matrix.h>
#include "ReadnShowClassesWorker.h"
#include "globaldef.h"
#include "dataLibrary.h"

bool ReadnShowClassesWorker::is_para_satisfying(QString &message){
	if(dataLibrary::Fracture_Triangles.size() == 0){
		message = QString("readnshowshearfeatures: Please Performed Fracture Triangulation or Read Triangulation PolygonMesh Data First!");
		return false;
	}
	else if(dataLibrary::cloudxyz->empty()&&dataLibrary::cloudxyzrgb->empty()){
		message = QString("readnshowshearfeatures: Please Read Point Cloud Data First (to show fracture shear features)!");
		return false;
	}
	else{
		this->setParaSize(2);
		if(dataLibrary::Workflow[dataLibrary::current_workline_index].parameters.size()<this->getParaSize()){
			message = QString(std::string("readnshowshearfeatures: Not Enough Parameters Given (At Least " + std::to_string(this->getParaSize()) + " Parameters).").c_str());
			return false;
		}
		else{
			std::string feature_str = dataLibrary::Workflow[dataLibrary::current_workline_index].parameters[0];
			if(feature_str == "striations"){
				if(!dataLibrary::fracture_striations.empty()){
					dataLibrary::fracture_striations.clear();
				}
				this->setFeatureType(FRACTURE_FEATURE_STRIATION);
			}
			else if(feature_str == "steps"){
				if(!dataLibrary::fracture_steps.empty()){
					dataLibrary::fracture_steps.clear();
				}
				this->setFeatureType(FRACTURE_FEATURE_STEP);
			}
			else{
				message = QString(std::string("readnshowshearfeatures: Incorrect Feature Type: " + feature_str).c_str());
				return false;
			}
			this->setFileName(QString::fromUtf8(dataLibrary::Workflow[dataLibrary::current_workline_index].parameters[1].c_str()));
			this->setParaIndex(this->getParaSize());
			return true;
		}
	}
}

void ReadnShowClassesWorker::prepare(){
    float percentOut_default = 0.03;
    this->setPercentOut(percentOut_default);
    float ratioThreshold_default = 0.06;
    this->setRatioThreshold(ratioThreshold_default);
	this->setPercentOutRatioThresholdErrorMode(false);
	this->setPercentOutRatioThresholdMessage("Normal");
	int no_match = 0;
    for(int i=1; i<=2; i++){
        if(dataLibrary::Workflow[dataLibrary::current_workline_index].parameters.size()>this->getParaIndex()){
            std::vector<std::string> st;
            boost::split(st, dataLibrary::Workflow[dataLibrary::current_workline_index].parameters[this->getParaIndex()], boost::is_any_of(":"));
            if((st.size()>1)&&(st[0]=="threshold")&&(dataLibrary::isOnlyDouble(st[1].c_str()))){
                this->setRatioThreshold(std::stof(st[1]));
				this->setParaIndex(this->getParaIndex() + 1);
            }
            else if((st.size()>1)&&(st[0]=="percent")&&(dataLibrary::isOnlyDouble(st[1].c_str()))){
                float percent = std::stof(st[1]);
                if((percent>=0)&&(percent<0.5)){
                    this->setPercentOut(percent);
                }
                else{
					this->setPercentOutRatioThresholdErrorMode(true);
					this->setPercentOutRatioThresholdMessage("readnshowshearfeatures: The <Percent Out> Value (" + std::to_string(percent) + ") Not In [0, 0.5), Setting It to the Default Value (" + std::to_string(percentOut_default) + ").");
                }
				this->setParaIndex(this->getParaIndex() + 1);
            }
			else {
				if (i == 1) {
					this->setParaIndex(this->getParaIndex() + 1);
					no_match++;
				}
				if (no_match == 1) {
					this->setParaIndex(this->getParaIndex() - 1);
				}
			}
        }
        else{
            break;
        }
    }
	this->setUnmute();
	this->setWriteLog();
	this->check_mute_nolog();
}

bool ReadnShowClassesWorker::readFeatures(const std::string &filename, std::string err_message){
	ifstream fs;
    fs.open (filename.c_str());
    if (!fs.is_open() || fs.fail()){
		err_message = "Could not open file " + filename + " !";
        fs.close();
        return false;
    }
    
    std::string line;
    std::vector<std::string> st;
    
    while (!fs.eof()){
        getline(fs, line);
        // Ignore empty lines
        if (line == "")
            continue;
        
        // Tokenize the line
        boost::trim(line);
        boost::split(st, line, boost::is_any_of(",\t\r "), boost::token_compress_on);
        
        if (st.size() < 5)
            continue;
        
		if(this->getFeatureType() == FRACTURE_FEATURE_STRIATION){
			Striation temp_striation;
			temp_striation.direction.x = std::stof(st[0]);
			temp_striation.direction.y = std::stof(st[1]);
			temp_striation.direction.z = std::stof(st[2]);
			temp_striation.ratio = std::stof(st[4]);
			dataLibrary::fracture_striations.push_back(temp_striation);
		}
		else if(this->getFeatureType() == FRACTURE_FEATURE_STEP){
			Step temp_step;
			temp_step.facing_direc.x = std::stof(st[0]);
			temp_step.facing_direc.y = std::stof(st[1]);
			temp_step.facing_direc.z = std::stof(st[2]);
			temp_step.ratio = std::stof(st[4]);
			dataLibrary::fracture_steps.push_back(temp_step);
		}
    }
    fs.close();
    return true;
}

void ReadnShowClassesWorker::saveColorBar(const std::string &filename){
    std::ofstream colorbar_out(filename.c_str());
    for(int i=0; i<256; i++){
        int red, green, blue;
        dataLibrary::getHeatMapColor((float)(i)/255.0, red, green, blue);
        colorbar_out<<red<<"\t"<<green<<"\t"<<blue<<"\n";
    }
    colorbar_out<<std::flush;
    colorbar_out.close();
}

void ReadnShowClassesWorker::doWork(){
	bool is_success(false);

	QByteArray ba = this->getFileName().toLocal8Bit();
	std::string* strfilename = new std::string(ba.data());

	dataLibrary::Status = STATUS_READNSHOWCLASSES;

    this->timer_start();

	//begin of processing
	std::string error_msg = "";
	if(!readFeatures(*strfilename,error_msg)){
		emit showErrors(QString(error_msg.c_str()));
	}
	else{
		if (this->getPercentOutRatioThresholdErrorMode()) {
			emit showErrors(QString(this->getPercentOutRatioThresholdMessage().c_str()));
		}
		if(this->getFeatureType() == FRACTURE_FEATURE_STRIATION){
			int striations_size = dataLibrary::fracture_striations.size();
			int polygonmeshs_size = dataLibrary::Fracture_Triangles.size();
			if(striations_size!=polygonmeshs_size){
				emit showErrors(QString(std::string("readnshowshearfeatures: The Length of Fracture Striations Data (" + std::to_string(striations_size) + ") and Fracture PolygonMeshes Data (" + std::to_string(polygonmeshs_size) + ") DO NOT MATCH!").c_str()));
			}
			else{
				if(!dataLibrary::fractures_with_feature.empty()){
					dataLibrary::fractures_with_feature.clear();
				}
				std::vector<float> temp_ratio_vec;
				for(int i=0; i<striations_size; i++){
                    if(dataLibrary::fracture_striations[i].ratio > 0.0){
                        temp_ratio_vec.push_back(dataLibrary::fracture_striations[i].ratio);
                    }
				}
				std::sort(temp_ratio_vec.begin(), temp_ratio_vec.end());
				float Min_Percent = this->getPercentOut();
				float Max_Percent = 1 - Min_Percent;
				float max_ratio = temp_ratio_vec[int((temp_ratio_vec.size()-1)*Max_Percent)];
				float min_ratio = temp_ratio_vec[int((temp_ratio_vec.size()-1)*Min_Percent)];
				dataLibrary::info_str = "Min Ratio: ";
                std::ostringstream min_strs, max_strs;
                min_strs << min_ratio;
                dataLibrary::info_str += (min_strs.str() +"; Max Ratio: ");
                max_strs << max_ratio;
                dataLibrary::info_str += max_strs.str();
                std::string colorbar_file = *strfilename + "_colorbar.txt";
                saveColorBar(colorbar_file);
                for(int i=0; i<polygonmeshs_size; i++){
                    if(dataLibrary::fracture_striations[i].direction.x==0&&dataLibrary::fracture_striations[i].direction.y==0&&dataLibrary::fracture_striations[i].direction.z==0){
                        continue;
                    }
                    else{
                        pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
                        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
                        pcl::fromPCLPointCloud2(dataLibrary::Fracture_Triangles[i]->cloud, *cloud_ptr);
                        Eigen::Vector3f centroid = dataLibrary::compute3DCentroid(*cloud_ptr);
                        Eigen::Vector4f plane_normal_param = dataLibrary::fitPlaneManually(*cloud_ptr);
                        Eigen::Vector3f fracture_normal;
                        fracture_normal << plane_normal_param(0), plane_normal_param(1), plane_normal_param(2);
                        Eigen::Vector3f striation_direc;
                        striation_direc << dataLibrary::fracture_striations[i].direction.x, dataLibrary::fracture_striations[i].direction.y, dataLibrary::fracture_striations[i].direction.z;
                        Eigen::Vector3f value_axis = fracture_normal.cross(striation_direc);
                        value_axis = value_axis/std::sqrt(value_axis.dot(value_axis));
                        float max_value, min_value;
                        std::vector<float> values_vec;
                        max_value  = min_value = 0.0;
                        for(int i=0; i<cloud_ptr->size(); i++){
                            float value = value_axis.dot(cloud_ptr->at(i).getVector3fMap() - centroid);
                            values_vec.push_back(value);
                            if(value>max_value){
                                max_value = value;
                            }
                            if(value<min_value){
                                min_value = value;
                            }
                        }
                        int red, green, blue;
                        if(max_ratio == min_ratio){
                            red = 0.5*255;
                            green = 0.5*255;
                            blue = 0.5*255;
                        }
                        else{
                            float norm_value = (dataLibrary::fracture_striations[i].ratio - min_ratio)/(max_ratio-min_ratio);
                            dataLibrary::getHeatMapColor(norm_value, red, green, blue);
                        }
                        int numofstrips = 6;
                        float color2nocolorRatio = 6.0;
                        float T = (color2nocolorRatio + 1)*(max_value - min_value)/((color2nocolorRatio + 1)*numofstrips + color2nocolorRatio);
                        if(dataLibrary::fracture_striations[i].ratio<this->getRatioThreshold()){
                            for(int i=0; i<cloud_ptr->size(); i++){
                                pcl::PointXYZRGB temp_point;
                                temp_point.x = cloud_ptr->at(i).x;
                                temp_point.y = cloud_ptr->at(i).y;
                                temp_point.z = cloud_ptr->at(i).z;
                                temp_point.r = red;
                                temp_point.g = green;
                                temp_point.b = blue;
                                temp_cloud_ptr->push_back(temp_point);
                            }
                        }
                        else{
                            for(int i=0; i<cloud_ptr->size(); i++){
                                pcl::PointXYZRGB temp_point;
                                temp_point.x = cloud_ptr->at(i).x;
                                temp_point.y = cloud_ptr->at(i).y;
                                temp_point.z = cloud_ptr->at(i).z;
                                float temp_rem = (values_vec[i] - min_value) - floor((values_vec[i] - min_value)/T)*T;
                                if(temp_rem<color2nocolorRatio*(T/(color2nocolorRatio+1))){
                                    temp_point.r = red;
                                    temp_point.g = green;
                                    temp_point.b = blue;
                                }
                                else{
                                    temp_point.r = 255;
                                    temp_point.g = 255;
                                    temp_point.b = 255;
                                }
                                temp_cloud_ptr->push_back(temp_point);
                            }
                        }
                        dataLibrary::fractures_with_feature.push_back(temp_cloud_ptr);
                    }
				}
				is_success = true;
				emit show();
			}
		}
		else if(this->getFeatureType() == FRACTURE_FEATURE_STEP){
			int steps_size = dataLibrary::fracture_steps.size();
			int polygonmeshs_size = dataLibrary::Fracture_Triangles.size();
			if(steps_size!=polygonmeshs_size){
				emit showErrors(QString(std::string("readnshowshearfeatures: The Length of Fracture Steps Data (" + std::to_string(steps_size) + ") and Fracture PolygonMeshes Data (" + std::to_string(polygonmeshs_size) + ") DO NOT MATCH!").c_str()));
			}
			else{
				if(!dataLibrary::fractures_with_feature.empty()){
					dataLibrary::fractures_with_feature.clear();
				}
				std::vector<float> temp_ratio_vec;
				for(int i=0; i<steps_size; i++){
                    if(dataLibrary::fracture_steps[i].ratio > 0.0){
                        temp_ratio_vec.push_back(dataLibrary::fracture_steps[i].ratio);
                    }
				}
				std::sort(temp_ratio_vec.begin(), temp_ratio_vec.end());
                float Min_Percent = this->getPercentOut();
                float Max_Percent = 1 - Min_Percent;
                float max_ratio = temp_ratio_vec[int((temp_ratio_vec.size()-1)*Max_Percent)];
                float min_ratio = temp_ratio_vec[int((temp_ratio_vec.size()-1)*Min_Percent)];
				dataLibrary::info_str = "Min Ratio: ";
                std::ostringstream min_strs, max_strs;
                min_strs << min_ratio;
                dataLibrary::info_str += (min_strs.str() +"; Max Ratio: ");
                max_strs << max_ratio;
                dataLibrary::info_str += max_strs.str();
                std::string colorbar_file = *strfilename + "_colorbar.txt";
                saveColorBar(colorbar_file);
                for(int i=0; i<polygonmeshs_size; i++){
					if(dataLibrary::fracture_steps[i].facing_direc.x==0&&dataLibrary::fracture_steps[i].facing_direc.y==0&&dataLibrary::fracture_steps[i].facing_direc.z==0){
						continue;
					}
					else {
                        pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
                        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
                        pcl::fromPCLPointCloud2(dataLibrary::Fracture_Triangles[i]->cloud, *cloud_ptr);
                        Eigen::Vector3f centroid = dataLibrary::compute3DCentroid(*cloud_ptr);
                        Eigen::Vector4f plane_normal_param = dataLibrary::fitPlaneManually(*cloud_ptr);
                        Eigen::Vector3f fracture_normal;
                        fracture_normal << plane_normal_param(0), plane_normal_param(1), plane_normal_param(2);
                        Eigen::Vector3f step_facing_direc;
                        step_facing_direc << dataLibrary::fracture_steps[i].facing_direc.x, dataLibrary::fracture_steps[i].facing_direc.y, dataLibrary::fracture_steps[i].facing_direc.z;
                        Eigen::Vector3f value_axis = step_facing_direc;
                        value_axis = value_axis / std::sqrt(value_axis.dot(value_axis));
                        float max_value, min_value;
                        std::vector<float> values_vec;
                        max_value = min_value = 0.0;
                        for (int i = 0; i < cloud_ptr->size(); i++) {
                            float value = value_axis.dot(cloud_ptr->at(i).getVector3fMap() - centroid);
                            values_vec.push_back(value);
                            if (value > max_value) {
                                max_value = value;
                            }
                            if (value < min_value) {
                                min_value = value;
                            }
                        }
                        int red, green, blue;
                        if (max_ratio == min_ratio) {
                            red = 0.5 * 255;
                            green = 0.5 * 255;
                            blue = 0.5 * 255;
                        } else {
                            float norm_value = (dataLibrary::fracture_steps[i].ratio - min_ratio) / (max_ratio - min_ratio);
                            dataLibrary::getHeatMapColor(norm_value, red, green, blue);
                        }
                        int numofstrips = 6;
                        float color2nocolorRatio = 1;
                        float T = (color2nocolorRatio + 1)*(max_value - min_value)/((color2nocolorRatio + 1)*numofstrips + color2nocolorRatio);
                        if(dataLibrary::fracture_steps[i].ratio<this->getRatioThreshold()){
                            for(int i = 0; i<cloud_ptr->size(); i++){
                                pcl::PointXYZRGB temp_point;
                                temp_point.x = cloud_ptr->at(i).x;
                                temp_point.y = cloud_ptr->at(i).y;
                                temp_point.z = cloud_ptr->at(i).z;
                                temp_point.r = red;
                                temp_point.g = green;
                                temp_point.b = blue;
                                temp_cloud_ptr->push_back(temp_point);
                            }
                        }
                        else{
                            for(int i = 0; i<cloud_ptr->size(); i++){
                                pcl::PointXYZRGB temp_point;
                                temp_point.x = cloud_ptr->at(i).x;
                                temp_point.y = cloud_ptr->at(i).y;
                                temp_point.z = cloud_ptr->at(i).z;
                                float temp_rem = (values_vec[i] - min_value) - floor((values_vec[i] - min_value)/T)*T;
                                if(temp_rem < color2nocolorRatio*(T/(color2nocolorRatio + 1))){
                                    temp_point.r = red;
                                    temp_point.g = green;
                                    temp_point.b = blue;
                                }
                                else{
                                    float color_weight = (temp_rem - color2nocolorRatio*(T/(color2nocolorRatio + 1)))/(T/(color2nocolorRatio + 1));
                                    temp_point.r = color_weight * red;
                                    temp_point.g = color_weight * green;
                                    temp_point.b = color_weight * blue;
                                }
                                temp_cloud_ptr->push_back(temp_point);
                            }
                        }
                        dataLibrary::fractures_with_feature.push_back(temp_cloud_ptr);
                    }
				}
				is_success = true;
				emit show();
			}
		}
	}
	//end of processing

    this->timer_stop();

    if(this->getWriteLogMode()&&is_success)
    {
        std::string log_text = "\tRead and Show Classes costs: ";
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