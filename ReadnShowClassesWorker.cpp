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
#include "ReadnShowClassesWorker.h"
#include "globaldef.h"
#include "dataLibrary.h"

void ReadnShowClassesWorker::doWork(const QString &filename)
{
	bool is_success(false);

	QByteArray ba = filename.toLocal8Bit();
	std::string* strfilename = new std::string(ba.data());

	dataLibrary::Status = STATUS_READNSHOWCLASSES;

	//begin of processing
	int fracture_class;
	ifstream fracture_classes_in(strfilename->c_str());
	while(fracture_classes_in>>fracture_class)
	{
		dataLibrary::fracture_classes.push_back(fracture_class);
	}
	fracture_classes_in.close();

	std::vector<int> temp_classes(dataLibrary::fracture_classes);
	std::sort(temp_classes.begin(), temp_classes.end());
	float max_val = (float)temp_classes.back();
	float min_val = (float)temp_classes.front()-0.9;
	for(int i=0; i<dataLibrary::fracture_classes.size(); i++)
	{
		unsigned char r, g, b;
		float a = 5.0*(max_val - (float)dataLibrary::fracture_classes[i])/(max_val-min_val);
		int X = floor(a);
		int Y = floor(255*(a-X));
		switch(X)
		{
			case 0:r=255;g=Y;b=0; break;
			case 1:r=255-Y;g=255;b=0; break;
			case 2:r=0;g=255;b=Y; break;
			case 3:r=0;g=255-Y;b=255; break;
			case 4:r=Y;g=0;b=255; break;
			case 5:r=255;g=0;b=255;break;
		}
		Vector3f temp_rgb;
		temp_rgb.x = 0.75*(float)r/255.0;
		temp_rgb.y = 0.75*(float)g/255.0;
		temp_rgb.z = 0.75*(float)b/255.0;
		dataLibrary::fracture_classes_rgb.push_back(temp_rgb);
	}
	is_success = true;
	emit show();
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