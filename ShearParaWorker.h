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

#include <string>
#include "globaldef.h"
#include "Worker.h"
class ShearParaWorker : public Worker
{
    Q_OBJECT

private:
	QString _filename;
	bool _save_screen_mode;
	bool _is_custom_shear_plane;
	Vector3f _custom_shear_plane_normal;
	bool _is_custom_shear_plane_normal_error;
	std::string _custom_shear_plane_normal_message;
    
public:
	void shearpara(){
        QMetaObject::invokeMethod(this, "doWork");
	}
	void setFileName(QString name){
		_filename = name;
	}
	QString getFileName(){
		return _filename;
	}
	void setSaveScreenMode(bool mode){
		_save_screen_mode = mode;
	}
	bool getSaveScreenMode(){
		return _save_screen_mode;
	}
	void setCustomShearPlaneMode(bool mode) {
		_is_custom_shear_plane = mode;
	}
	bool getCustomShearPlaneMode() {
		return _is_custom_shear_plane;
	}
	void setCustomShearPlaneNormal(float nx, float ny, float nz) {
		_custom_shear_plane_normal.x = nx;
		_custom_shear_plane_normal.y = ny;
		_custom_shear_plane_normal.z = nz;
	}
	Vector3f getCustomShearPlaneNormal() {
		return _custom_shear_plane_normal;
	}
	void setCustomShearPlaneNormalErrorMode(bool mode){
		_is_custom_shear_plane_normal_error = mode;
	}
	bool getCustomShearPlaneNormalErrorMode(){
		return _is_custom_shear_plane_normal_error;
	}
	void setCustomShearPlaneNormalMessage(std::string message){
		_custom_shear_plane_normal_message = message;
	}
	std::string getCustomShearPlaneNormalMessage(){
		return _custom_shear_plane_normal_message;
	}
	virtual bool is_para_satisfying(QString &message);
	virtual void prepare();

private slots:
    void doWork();
signals:
	void show();
	void prepare_2_s_f();
	void show_f_save_screen(const QString &filename);
};