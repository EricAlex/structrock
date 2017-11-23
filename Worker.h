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

#include <time.h>
#include <QObject>
#include <QThread>
#include "MyThread.h"

class Worker : public QObject
{
    Q_OBJECT

public:
	Worker() : QObject()
	{
		_workflow_mode = false;
		_is_mute = false;
		_write_log = true;
		_para_size = 0;
		moveToThread(&t);
		t.start();
	}
	~Worker()
	{
		t.quit();
		t.wait();
	}

signals:
	void GoWorkFlow();
	void showErrors(const QString &errors);
	void showReadyStatus();
private:
	bool _workflow_mode;
	bool _is_mute;
	bool _write_log;
	int _para_size;
	int _para_index;
	clock_t _time_start;
	clock_t _time_finish;
public:
	void setWorkFlowMode(bool mode)
	{
		_workflow_mode = mode;
	}
	bool getWorkFlowMode()
	{
		return _workflow_mode;
	}
	void setMute()
	{
		_is_mute = true;
	}
    void setUnmute()
    {
        _is_mute = false;
    }
	bool getMuteMode()
	{
		return _is_mute;
	}
	void setWriteLog()
	{
		_write_log = true;
	}
	void setUnWriteLog()
	{
		_write_log = false;
	}
	bool getWriteLogMpde()
	{
		return _write_log;
	}
	void Sleep(unsigned long ms)
	{
		t.Sleep(ms);
	}
	void check_mute_nolog();
	bool is_para_satisfying(QString message){}
	void prepare(){}
	void setParaSize(int size)
	{
		_para_size = size;
	}
	int getParaSize()
	{
		return _para_size;
	}
	void setParaIndex(int index)
	{
		_para_index = index;
	}
	int getParaIndex()
	{
		return _para_index;
	}
	void timer_start()
	{
		_time_start = clock();
	}
	void timer_stop()
	{
		_time_finish = clock();
	}
	double getTimer_sec()
	{
		return (double)(_time_finish-_time_start)/CLOCKS_PER_SEC;
	}

private:
	MyThread t;
};