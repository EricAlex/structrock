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

#include <QtGui/QApplication>
#include <QDesktopWidget>
#include "globaldef.h"
#include "structrock.h"


void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event, void* structrock_void)
{
	structrock* structrock_window = static_cast<structrock *>(structrock_void);
	if(event.getKeySym() == "f" && event.keyDown())
	{
		structrock_window->MoveForwardPatch();
	}
	else if(event.getKeySym() == "b" && event.keyDown())
	{
		structrock_window->MoveBackPatch();
	}
	else if(event.getKeySym() == "p" && event.keyDown())
	{
		structrock_window->saveScreen();
	}
}

void pointPickingEvent(const pcl::visualization::PointPickingEvent &event, void* structrock_void)
{
	structrock* structrock_window = static_cast<structrock *>(structrock_void);
	float x, y, z;
	event.getPoint(x, y, z);
	structrock_window->CheckPatchesIfSelected(x, y, z);
}

void mouseEventOccured(const pcl::visualization::MouseEvent &event, void* structrock_void)
{
	structrock* structrock_window = static_cast<structrock *>(structrock_void);
	if(event.getButton() == pcl::visualization::MouseEvent::RightButton && event.getType() == pcl::visualization::MouseEvent::MouseDblClick)
	{
		//in mouse select patches mode, if right button is clicked, show selected patches
		structrock_window->ShowSelectedPatches();
	}
}

int main(int argc, char *argv[])
{
	if((argc>2)&&(strlen(argv[1])==2)&&(argv[1][0]=='-')&&(argv[1][1]=='f'))
	{
		QApplication a(argc, argv);
		structrock w;
		w.setWindowTitle("StructRock");
		w.viewer->registerKeyboardCallback(keyboardEventOccurred, (void*)&w);
		w.viewer->registerPointPickingCallback(pointPickingEvent, (void*)&w);
		w.viewer->registerMouseCallback(mouseEventOccured, (void*)&w);
			
		w.setGeometry(QApplication::desktop()->availableGeometry());
			
		w.show();
		w.OpenWorkFlow(QString(argv[2]));
		return a.exec();
	}
	else if((argc>2)&&(strlen(argv[1])==2)&&(argv[1][0]=='-')&&(argv[1][1]=='c'))
	{
		QApplication a(argc, argv);
		structrock w;
		w.setWindowTitle("StructRock");
		w.viewer->registerKeyboardCallback(keyboardEventOccurred, (void*)&w);
		w.viewer->registerPointPickingCallback(pointPickingEvent, (void*)&w);
		w.viewer->registerMouseCallback(mouseEventOccured, (void*)&w);
			
		w.setGeometry(QApplication::desktop()->availableGeometry());
			
		w.show();
		w.OpenWorkFlow(std::string(argv[2]));
		return a.exec();
	}
	else
	{
		QApplication a(argc, argv);
		structrock w;
		w.setWindowTitle("StructRock");
		w.viewer->registerKeyboardCallback(keyboardEventOccurred, (void*)&w);
		w.viewer->registerPointPickingCallback(pointPickingEvent, (void*)&w);
		w.viewer->registerMouseCallback(mouseEventOccured, (void*)&w);
			
		w.setGeometry(QApplication::desktop()->availableGeometry());
			
		w.show();
		return a.exec();
	}
}
