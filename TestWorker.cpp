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
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_types_conversion.h>
#include "TestWorker.h"
#include "globaldef.h"
#include "dataLibrary.h"

using namespace std;

bool TestWorker::is_para_satisfying(QString &message)
{
	this->setParaSize(1);
	if(dataLibrary::Workflow[dataLibrary::current_workline_index].parameters.size()>=this->getParaSize())
	{
		this->setFileName(QString::fromUtf8(dataLibrary::Workflow[dataLibrary::current_workline_index].parameters[0].c_str()));
		this->setParaIndex(this->getParaSize());
		return true;
	}
	else
	{
		message = QString("test: File Path Not Provided.");
		return false;
	}
}

void TestWorker::prepare()
{
	this->setSplitMode(false);
	if(dataLibrary::Workflow[dataLibrary::current_workline_index].parameters.size()>this->getParaIndex())
	{
		if(dataLibrary::Workflow[dataLibrary::current_workline_index].parameters[this->getParaIndex()] == "split")
		{
			this->setSplitMode(true);
			this->setParaIndex(this->getParaIndex()+1);
		}
	}
	this->setUnmute();
	this->setWriteLog();
}

bool transData (const string &filename, const string &savefilename)
{
    ifstream fs;
    fs.open (filename.c_str (), ios::binary);
    if (!fs.is_open () || fs.fail ())
    {
        PCL_ERROR ("Could not open file '%s'! Error : %s\n", filename.c_str (), strerror (errno));
        fs.close ();
        return (false);
    }
    
    ofstream transedData_out;
    transedData_out.open(savefilename.c_str());
    if (!transedData_out.is_open () || transedData_out.fail ())
    {
        PCL_ERROR ("Could not open file '%s'! Error : %s\n", savefilename.c_str (), strerror (errno));
        transedData_out.close ();
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
        
        if (st[0] == "x")
            continue;
        
        transedData_out<<st[0]<<"\t"<<st[1]<<"\t"<<st[2]<<"\n";
    }
    fs.close ();
    transedData_out.close();
    
    return (true);
}

bool splitData (const string &filename, const string &lsavefilename, const string &rsavefilename)
{
    /*to be filled*/
	return true;
}

/*void TestWorker::doWork(const QString &filename)
{
	bool is_success(false);

    QByteArray ba = filename.toLocal8Bit();
	string* strfilename = new std::string(ba.data());
    
    string transedData = strfilename->substr(0, strfilename->size()-13) += "_XYZ.txt";
    
    dataLibrary::Status = STATUS_TESTING;

	//begin of processing
	if(transData (*strfilename, transedData))
	{
		is_success = true;
	}
	else
	{
		emit showErrors("Error transform data.");
	}
	//end of processing

	dataLibrary::Status = STATUS_READY;
    emit showReadyStatus();
	delete strfilename;
	if(this->getWorkFlowMode()&&is_success)
	{
		this->Sleep(1000);
		emit GoWorkFlow();
	}
}*/

void TestWorker::doWork()
{
	bool is_success(false);

    QByteArray ba = this->getFileName().toLocal8Bit();
	string* strfilename = new std::string(ba.data());
    
    dataLibrary::Status = STATUS_TESTING;
	
	//begin of processing
	if(!pcl::io::loadPCDFile (*strfilename, *dataLibrary::cloudxyzrgb))
	{
		dataLibrary::cloudID = *strfilename;
		pcl::PointCloudXYZRGBtoXYZI (*dataLibrary::cloudxyzrgb, *dataLibrary::cloudxyzi);

		if(!this->getMuteMode())
		{
			emit ReadFileReady(CLOUDXYZI);
		}
		is_success = true;
	}
	else
	{
		emit showErrors("Error opening pcd file.");
	}
	//end of processing

	dataLibrary::Status = STATUS_READY;
    emit showReadyStatus();
	delete strfilename;
	if(this->getWorkFlowMode()&&is_success)
	{
		this->Sleep(1000);
		emit GoWorkFlow();
	}
}