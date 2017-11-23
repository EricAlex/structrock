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

#include <fstream>
#include <string>
#include <sstream>
#include <pcl/common/centroid.h>
#include <pcl/point_types.h>
#include <Eigen/src/Core/Matrix.h>
#include "openClustersWorker.h"
#include "globaldef.h"
#include "dataLibrary.h"

using namespace std;

bool openClustersWorker::is_para_satisfying(QString message)
{
	this->setParaSize(1);
	if(dataLibrary::Workflow[dataLibrary::current_workline_index].parameters.size()>0)
	{
		this->setFileName(QString::fromUtf8(dataLibrary::Workflow[dataLibrary::current_workline_index].parameters[0].c_str()));
		this->setParaIndex(this->getParaSize());
		return true;
	}
	else
	{
		message = QString("openclusters: Location of Clusters (bin) File Not Provided.");
		return false;
	}
}
void openClustersWorker::prepare()
{
	this->setUnmute();
	this->setWriteLog();
	this->check_mute_nolog();
}

void openClustersWorker::doWork()
{
	bool is_success(false);

    QByteArray ba = this->getFileName().toLocal8Bit();
	string* strfilename = new std::string(ba.data());
    
    dataLibrary::Status = STATUS_OPENCLUSTERS;

    this->timer_start();

	//begin of processing
    stringstream ss;

    ifstream fbinaryin(strfilename->c_str(), ios::in|ios::binary);
    if(fbinaryin.is_open())
    {
        if(!dataLibrary::cluster_patches.empty())
            dataLibrary::cluster_patches.clear();
        if(!dataLibrary::dips.empty())
            dataLibrary::dips.clear();
        if(!dataLibrary::dip_directions.empty())
            dataLibrary::dip_directions.clear();
        if(!dataLibrary::areas.empty())
            dataLibrary::areas.clear();
        if(!dataLibrary::patchIDs.empty())
            dataLibrary::patchIDs.clear();
        if(!dataLibrary::selectedPatches.empty())
            dataLibrary::selectedPatches.clear();
        
        int num_of_clusters, num_of_points;
        float x, y, z, rgb, dip, dip_direction, area;
        fbinaryin.read(reinterpret_cast<char*>(&num_of_clusters), sizeof(num_of_clusters));
        Eigen::Vector4f centroid;
        float centor_x = 0.0;
		float centor_y = 0.0;
		float centor_z = 0.0;
        for(int i=0; i<num_of_clusters; i++)
        {
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_patch (new pcl::PointCloud<pcl::PointXYZRGB>);
            fbinaryin.read(reinterpret_cast<char*>(&num_of_points), sizeof(num_of_points));
            fbinaryin.read(reinterpret_cast<char*>(&rgb), sizeof(rgb));
            for(int j=0; j<num_of_points; j++)
            {
                fbinaryin.read(reinterpret_cast<char*>(&x), sizeof(x));
                fbinaryin.read(reinterpret_cast<char*>(&y), sizeof(y));
                fbinaryin.read(reinterpret_cast<char*>(&z), sizeof(z));
                pcl::PointXYZRGB point;
                point.x = x;
                point.y = y;
                point.z = z;
                point.rgb = rgb;
                cloud_patch->push_back(point);
            }
            fbinaryin.read(reinterpret_cast<char*>(&dip), sizeof(dip));
            fbinaryin.read(reinterpret_cast<char*>(&dip_direction), sizeof(dip_direction));
            fbinaryin.read(reinterpret_cast<char*>(&area), sizeof(area));
            dataLibrary::cluster_patches.push_back(cloud_patch);
            dataLibrary::dips.push_back(dip);
            dataLibrary::dip_directions.push_back(dip_direction);
            dataLibrary::areas.push_back(area);

            ss << i;
            string patchID = *strfilename += ss.str();
            dataLibrary::patchIDs.push_back(patchID);
            pcl::compute3DCentroid(*cloud_patch, centroid);
            centor_x += centroid[0];
            centor_y += centroid[1];
            centor_z += centroid[2];
        }
        centor_x /= num_of_clusters;
        centor_y /= num_of_clusters;
        centor_z /= num_of_clusters;
        dataLibrary::cloud_centor.x=centor_x;
        dataLibrary::cloud_centor.y=centor_y;
        dataLibrary::cloud_centor.z=centor_z;
        fbinaryin.close();
        if(!this->getMuteMode())
        {
            emit show();
        }
        is_success = true;
    }
    else
    {
        emit showErrors("openclusters: Can Not Open the File!");
    }
	//end of processing

    this->timer_stop();

    if(this->getWriteLogMpde()&&is_success)
    {
		std::string string_filename = this->getFileName().toUtf8().constData();
        std::string log_text = string_filename + "\n\tReading Clusters costs: ";
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