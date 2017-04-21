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
#include <vector>

#define EXIT_CODE_REBOOT -123456789

#define CLOUDXYZ	1
#define CLOUDXYZRGB	2
#define CLOUDXYZI	3
#define EPSILON 0.0000001
#define TWOPI 6.283185307179586476925287

#define STATUS_READY	100
#define STATUS_OPENPCD	101
#define STATUS_RESAMPLE	102
#define STATUS_DOWNSAMPLE	103
#define STATUS_KNNORMAL	104
#define STATUS_RANORMAL	105
#define STATUS_STATICRO	106
#define STATUS_RGS		107
#define STATUS_SAVECLUSTERS 108
#define STATUS_OPENXYZ	109
#define STATUS_SAVEASCII	110
#define STATUS_SAVEBINARY	111
#define STATUS_SAVENORMALS	112
#define STATUS_SHOWPROCESS	113
#define STATUS_TESTING  114
#define STATUS_SHOWSFEATURE 115

#define FEATURE_ROUGHNESS   1001
#define FEATURE_AREA    1002
#define FEATURE_CURVATURE   1003

struct RGSpara{
    double curvature;
    double smoothness;
    double residual;
    int number_of_neighbors;
    int min_number_of_Points;
    bool IsSmoothMode;
};

struct FeaturePara{
	int feature_type;
	float percent_out;
};

struct Vector3f{
	float x, y, z;
};

struct Line{
    Vector3f begin;
    Vector3f end;
    float r;
    float g;
    float b;
    std::string ID;
};

struct WorkLine{
	std::string command;
	std::vector<std::string> parameters;
};