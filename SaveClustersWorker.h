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
#include "globaldef.h"

class SaveClustersWorker : public Worker
{
	Q_OBJECT
    
public:
	void saveclusters()
	{
		QMetaObject::invokeMethod(this, "doWork");
	}
private:
	bool _trim_trace_edges;
	QString _filename;
	int _fracture_map_mode;
	double _expand_ratio;
public:
	void setTrimTraceEdgesMode(bool mode)
	{
		_trim_trace_edges = mode;
	}
	bool getTrimTraceEdgesMode()
	{
		return _trim_trace_edges;
	}
	void setDefaltFMAP_Mode()
	{
		_fracture_map_mode = FMAP_LOWER_BOUND;
	}
	void setFMAP_Mode(int mode)
	{
		_fracture_map_mode = mode;
	}
	int getFMAP_Mode()
	{
		return _fracture_map_mode;
	}
	void setExpandRatio(double ratio)
	{
		_expand_ratio = ratio;
	}
	double getExpandRatio()
	{
		return _expand_ratio;
	}
	void setFileName(QString name)
	{
		_filename = name;
	}
	QString getFileName()
	{
		return _filename;
	}
	virtual bool is_para_satisfying(QString &message);
	virtual void prepare();

private slots:
    void doWork();
signals:
    void SaveClustersReady(const QString &filename);
};
