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
#include "Worker.h"
class ReadnShowClassesWorker : public Worker
{
	Q_OBJECT

private:
	QString _filename;
	int _feature_type;
	float _percent_out;
	float _ratio_threshold;
	bool _is_percent_out_ratio_threshold_error;
	std::string _percent_out_ratio_threshold_message;

private:
	bool readFeatures(const std::string &filename, std::string err_message);
	void saveColorBar(const std::string &filename);
public:
	void readnshowfeatures(){
		QMetaObject::invokeMethod(this, "doWork");
	}
	void setFileName(QString name){
		_filename = name;
	}
	QString getFileName(){
		return _filename;
	}
	void setFeatureType(int type){
		_feature_type = type;
	}
	int getFeatureType(){
		return _feature_type;
	}
	void setPercentOut(float percent){
		_percent_out = percent;
	}
	float getPercentOut(){
		return _percent_out;
	}
	void setRatioThreshold(float threshold){
        _ratio_threshold = threshold;
	}
	float getRatioThreshold(){
	    return _ratio_threshold;
	}
	void setPercentOutRatioThresholdErrorMode(bool mode) {
		_is_percent_out_ratio_threshold_error = mode;
	}
	bool getPercentOutRatioThresholdErrorMode() {
		return _is_percent_out_ratio_threshold_error;
	}
	void setPercentOutRatioThresholdMessage(std::string message) {
		_percent_out_ratio_threshold_message = message;
	}
	std::string getPercentOutRatioThresholdMessage() {
		return _percent_out_ratio_threshold_message;
	}
	virtual bool is_para_satisfying(QString &message);
	virtual void prepare();

public slots:
	void doWork();

signals:
	void show();
};