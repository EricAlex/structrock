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
#include <iostream>
#include <math.h>
#include <Eigen/Core>
#include "LagrangeTensorWorker.h"
#include "globaldef.h"
#include "dataLibrary.h"

using namespace std;

bool LagrangeTensorWorker::is_para_satisfying(QString &message){
    if(dataLibrary::Fracture_Triangles.size() > 0){
        this->setParaSize(7);
        if(dataLibrary::Workflow[dataLibrary::current_workline_index].parameters.size()>=this->getParaSize()){
            this->setStriationsFileName(QString::fromUtf8(dataLibrary::Workflow[dataLibrary::current_workline_index].parameters[0].c_str()));
            this->setStepsFileName(QString::fromUtf8(dataLibrary::Workflow[dataLibrary::current_workline_index].parameters[1].c_str()));
            this->setSizeFileName(QString::fromUtf8(dataLibrary::Workflow[dataLibrary::current_workline_index].parameters[2].c_str()));
            this->setTensorFileName(QString::fromUtf8(dataLibrary::Workflow[dataLibrary::current_workline_index].parameters[3].c_str()));
            string size_th_string = dataLibrary::Workflow[dataLibrary::current_workline_index].parameters[4];
			transform(size_th_string.begin(), size_th_string.end(), size_th_string.begin(), ::tolower);
            if(size_th_string == "allsize"){
                this->setSizeThMode(false);
                this->setSizeTh(0.0);
            }
            else{
                double SizeTh;
                stringstream ss_SizeTh(dataLibrary::Workflow[dataLibrary::current_workline_index].parameters[4]);
			    ss_SizeTh >> SizeTh;
                this->setSizeThMode(true);
                this->setSizeTh(SizeTh);
            }
            double AngleTh;
            stringstream ss_AngleTh(dataLibrary::Workflow[dataLibrary::current_workline_index].parameters[5]);
			ss_AngleTh >> AngleTh;
            this->setAngleTh(AngleTh);
            string K_string = dataLibrary::Workflow[dataLibrary::current_workline_index].parameters[6];
            transform(K_string.begin(), K_string.end(), K_string.begin(), ::tolower);
            if(K_string == "realk"){
                this->setKfixedMode(false);
            }
            else{
                double K;
                stringstream ss_K(dataLibrary::Workflow[dataLibrary::current_workline_index].parameters[6]);
			    ss_K >> K;
                this->setKfixedMode(true);
                this->setK(K);
            }
            this->setStriationsMTh(0.05);
            this->setStepsMTh(0.0);
            this->setParaIndex(this->getParaSize());
            return true;
        }
        else{
            message = QString("lagrangetensor: No (or not enough) Parameter Given.");
            return false;
        }
    }
    else{
        message = QString("lagrangetensor: Please Perform Fracture Triangulation or Open Triangulation Data First.");
		return false;
    }
}

void LagrangeTensorWorker::prepare(){
	this->setUnmute();
	this->setWriteLog();
	this->check_mute_nolog();
}

bool readStriationsOrSteps(const string &filename, vector<Eigen::Vector3f> &dir, vector<float> &M, string err_message){
	ifstream fs;
    fs.open (filename.c_str());
    if (!fs.is_open() || fs.fail()){
		err_message = "Could not open file " + filename + " !";
        fs.close();
        return false;
    }
    
    string line;
    vector<string> st;
    
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
        
        Eigen::Vector3f temp_dir;
        temp_dir<<stof(st[0]),stof(st[1]),stof(st[2]);
        dir.push_back(temp_dir);
        M.push_back(stof(st[4]));
    }
    fs.close();
    return true;
}

bool readSize(const string &filename, vector<int> &f_size, string err_message){
	ifstream fs;
    fs.open (filename.c_str());
    if (!fs.is_open() || fs.fail()){
		err_message = "Could not open file " + filename + " !";
        fs.close();
        return false;
    }
    
    string line;
    
    while (!fs.eof()){
        getline(fs, line);
        // Ignore empty lines
        if (line == "")
            continue;
        
        boost::trim(line);
        
        f_size.push_back(stoi(line));
    }
    fs.close();
    return true;
}

void LagrangeTensorWorker::doWork(){
	bool is_success(false);

    QByteArray baStriations = this->getStriationsFileName().toLocal8Bit();
	string* strStriationsfilename = new string(baStriations.data());
    QByteArray baSteps = this->getStepsFileName().toLocal8Bit();
	string* strStepsfilename = new string(baSteps.data());
    QByteArray baSize = this->getSizeFileName().toLocal8Bit();
	string* strSizefilename = new string(baSize.data());
    QByteArray baTensor = this->getTensorFileName().toLocal8Bit();
	string* strTensorfilename = new string(baTensor.data());
    
    dataLibrary::Status = STATUS_LAGRANGETENSOR;

    this->timer_start();

	//begin of processing
    vector<Eigen::Vector3f> striations_dir;
    vector<float> striations_M;
    vector<Eigen::Vector3f> steps_dir;
    vector<float> steps_M;
    vector<int> f_size;
	string error_msg = "";
	if(!readStriationsOrSteps(*strStriationsfilename, striations_dir, striations_M, error_msg)){
		emit showErrors(QString(error_msg.c_str()));
	}
    else{
        if(!readStriationsOrSteps(*strStepsfilename, steps_dir, steps_M, error_msg)){
            emit showErrors(QString(error_msg.c_str()));
        }
        else{
            if(!readSize(*strSizefilename, f_size, error_msg)){
                emit showErrors(QString(error_msg.c_str()));
            }
            else{
                ofstream fout(strTensorfilename->c_str());
                for(int i=0; i<dataLibrary::Fracture_Triangles.size(); i++){
                    if((this->getSizeThMode() == false)||((this->getSizeThMode() == true)&&(f_size[i] >= this->getSizeTh()))){
                        if((striations_dir[i].norm() != 0)&&(steps_dir[i].norm() != 0)){
                            if((striations_M[i] >= this->getStriationsMTh())&&(steps_M[i] >= this->getStepsMTh())){
                                Eigen::Vector3f shear_dir;
                                double angle = acos(striations_dir[i].dot(steps_dir[i])/(striations_dir[i].norm()*steps_dir[i].norm()));
                                double angle_th = this->getAngleTh()*TWOPI/360.0;
                                if((angle<=angle_th)||((TWOPI/2-angle)<=angle_th)){
                                    if(angle < TWOPI/4){
                                        shear_dir = striations_dir[i];
                                    }
                                    else if(angle > TWOPI/4){
                                        shear_dir = -striations_dir[i];
                                    }
                                    else{
                                        continue;
                                    }
                                    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
                                    pcl::fromROSMsg(dataLibrary::Fracture_Triangles[i]->cloud, *cloud_ptr);
                                    Eigen::Vector4f plane_normal_param = dataLibrary::fitPlaneManually(*cloud_ptr);
                                    Eigen::Vector3f fracture_normal;
                                    fracture_normal << plane_normal_param(0), plane_normal_param(1), plane_normal_param(2);
                                    Eigen::Vector3f e1 = shear_dir/shear_dir.norm();
                                    Eigen::Vector3f e2 = fracture_normal/fracture_normal.norm();
                                    Eigen::Vector3f e3 = e1.cross(e2);
                                    Eigen::Vector3f e_prime1, e_prime2, e_prime3;
                                    e_prime1<<1,0,0;
                                    e_prime2<<0,1,0;
                                    e_prime3<<0,0,1;
                                    double K;
                                    double M2Kpara = 0.2;
                                    if(this->getKfixedMode()){
                                        K = this->getK();
                                    }
                                    else{
                                        K = (striations_M[i] + steps_M[i])*M2Kpara;
                                    }
                                    Eigen::Matrix3f E_star;
                                    E_star << 0, K/2, 0,
                                            K/2, K*K/2, 0,
                                            0, 0, 0;
                                    Eigen::Matrix3f T;
                                    T << e1.dot(e_prime1), e1.dot(e_prime2), e1.dot(e_prime3),
                                        e2.dot(e_prime1), e2.dot(e_prime2), e2.dot(e_prime3),
                                        e3.dot(e_prime1), e3.dot(e_prime2), e3.dot(e_prime3);
                                    Eigen::Matrix3f E_star_prime = T.transpose()*E_star*T;
                                    fout<<i<<'\t'
                                            <<E_star_prime(0,0)<<'\t'<<E_star_prime(0,1)<<'\t'<<E_star_prime(0,2)<<'\t'
                                            <<E_star_prime(1,0)<<'\t'<<E_star_prime(1,1)<<'\t'<<E_star_prime(1,2)<<'\t'
                                            <<E_star_prime(2,0)<<'\t'<<E_star_prime(2,1)<<'\t'<<E_star_prime(2,2)<<'\n';
                                }
                            }
                        }
                    }
                }
                fout.close();
                is_success = true;
            }
        }
    }
	//end of processing

    this->timer_stop();

    if(this->getWriteLogMode()&&is_success){
        string log_text = "\tLagrange Tensor costs: ";
        ostringstream strs;
        strs << this->getTimer_sec();
        log_text += (strs.str() +" seconds.");
        dataLibrary::write_text_to_log_file(log_text);
    }

	dataLibrary::Status = STATUS_READY;
    emit showReadyStatus();
	delete strStriationsfilename;
    delete strStepsfilename;
    delete strSizefilename;
    delete strTensorfilename;
	if(this->getWorkFlowMode()&&is_success){
		this->Sleep(1000);
		emit GoWorkFlow();
	}
}