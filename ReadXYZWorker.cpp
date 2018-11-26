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
#include <pcl/point_types.h>
#include "ReadXYZWorker.h"
#include "globaldef.h"
#include "dataLibrary.h"

using namespace std;

bool ReadXYZWorker::is_para_satisfying(QString message)
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
		message = QString("openxyz: Location of XYZ File Not Provided.");
		return false;
	}
}

void ReadXYZWorker::prepare()
{
	this->setUnmute();
	this->setWriteLog();
	this->check_mute_nolog();
}

bool loadCloud (const string &filename, pcl::PointCloud<pcl::PointXYZ> &cloud)
{
    ifstream fs;
    fs.open (filename.c_str (), ios::binary);
    if (!fs.is_open () || fs.fail ())
    {
        PCL_ERROR ("Could not open file '%s'! Error : %s\n", filename.c_str (), strerror (errno));
        fs.close ();
        return (false);
    }
    
    string line;
    vector<string> st;
    
    while (!fs.eof ())
    {
        getline (fs, line);
        // Ignore empty lines
        if (line == "")
            continue;
        
        // Tokenize the line
        boost::trim (line);
        boost::split (st, line, boost::is_any_of (",\t\r "), boost::token_compress_on);
        
        if (st.size () < 3)
            continue;
        
        cloud.push_back (pcl::PointXYZ (float (atof (st[0].c_str ())), float (atof (st[1].c_str ())), float (atof (st[2].c_str ()))));
    }
    fs.close ();
    
    cloud.width = uint32_t (cloud.size ()); cloud.height = 1; cloud.is_dense = true;
    return (true);
}

void ReadXYZWorker::doWork()
{
	bool is_success(false);

    QByteArray ba = this->getFileName().toLocal8Bit();
	string* strfilename = new std::string(ba.data());
	
	dataLibrary::clearall();
    
    dataLibrary::Status = STATUS_OPENXYZ;

    this->timer_start();

	//begin of processing
	if(loadCloud (*strfilename, *dataLibrary::cloudxyz))
	{
		if(!this->getMuteMode())
        {
            emit ReadXYZReady(CLOUDXYZ);
        }
		is_success = true;
	}
	else
	{
		emit showErrors("Error opening xyz file.");
	}

	dataLibrary::cloudID = *strfilename;
	//end of processing

    this->timer_stop();

    if(this->getWriteLogMode()&&is_success)
    {
		std::string string_filename = this->getFileName().toUtf8().constData();
        std::string log_text = string_filename + "\n\tReading XYZ file costs: ";
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