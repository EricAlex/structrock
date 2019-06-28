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

#pragma once
#include "Worker.h"
class LagrangeTensorWorker : public Worker{
    Q_OBJECT

private:
	QString _filename_striations;
	QString _filename_steps;
	QString _filename_size;
	QString _filename_tensor;
	bool _has_size_th;
	double _size_th;
	double _angle_th;
	double _striations_M_th;
	double _steps_M_th;
	bool _is_K_fixed;
	double _K;

public:
	void LagrangeTensor(){
		QMetaObject::invokeMethod(this, "doWork");
	}
	void setStriationsFileName(QString name){
		_filename_striations = name;
	}
	QString getStriationsFileName(){
		return _filename_striations;
	}
	void setStepsFileName(QString name){
		_filename_steps = name;
	}
	QString getStepsFileName(){
		return _filename_steps;
	}
	void setSizeFileName(QString name){
		_filename_size = name;
	}
	QString getSizeFileName(){
		return _filename_size;
	}
	void setTensorFileName(QString name){
		_filename_tensor = name;
	}
	QString getTensorFileName(){
		return _filename_tensor;
	}
	void setSizeThMode(bool mode){
		_has_size_th = mode;
	}
	bool getSizeThMode(){
		return _has_size_th;
	}
	void setSizeTh(double size_th){
		_size_th = size_th;
	}
	double getSizeTh(){
		return _size_th;
	}
	void setAngleTh(double angle_th){
		_angle_th = angle_th;
	}
	double getAngleTh(){
		return _angle_th;
	}
	void setStriationsMTh(double striations_M_th){
		_striations_M_th = striations_M_th;
	}
	double getStriationsMTh(){
		return _striations_M_th;
	}
	void setStepsMTh(double steps_M_th){
		_steps_M_th = steps_M_th;
	}
	double getStepsMTh(){
		return _steps_M_th;
	}
	void setKfixedMode(bool mode){
		_is_K_fixed = mode;
	}
	bool getKfixedMode(){
		return _is_K_fixed;
	}
	void setK(double K){
		_K = K;
	}
	double getK(){
		return _K;
	}
	virtual bool is_para_satisfying(QString &message);
	virtual void prepare();

private slots:
    void doWork();
};