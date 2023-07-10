/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2014, ISR University of Coimbra.
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
*   * Neither the name of the ISR University of Coimbra nor the names of its
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
* Author: Luca Iocchi (2014) and Luigi Freda (2016-Present) with Mario Gianni (2016)
*********************************************************************/

#include "config.h"
#include <cstring>
#include <cstdlib>

#define STRLEN 200


ConfigFile::ConfigFile(const char *filename)
{
	f.open(filename);
	if (!f.good()) {
		printf("ERROR. Cannot open config file %s.\n",filename);
		return;
	}

	printf("Opened config file %s\n",filename);
	char buf[STRLEN]; char *p; char par[STRLEN], val[STRLEN], profile[STRLEN];
	int state=0;
	while (f.good()) {
		f.getline(buf,STRLEN);
		switch (state) {
			case 0: // find SECTION
			p = strstr(buf,"[SECTION Main]");
			if (p)
				state++;
			break;
			case 1: // find PROFILE
			p = strstr(buf,"PROFILE");
			if (p) {
				sscanf(buf,"%s %s",par,profile);
				printf("Config Profile %s\n",profile);
				state++;
			}
			break;
            
			case 2: // find correct profile
			p = strstr(buf,"PROFILE");
			if (p) {
				sscanf(p,"%s %s",par,val); int n = strlen(val);
				int k=0;
				while (val[k]!=']' && k<n) k++;
				val[k]='\0';
				printf("Read %s\n",val);
				if (strcmp(profile,val)==0) {
					printf("Right profile\n");
					state++;
				}
			}
			break;
			
            case 3: // reading params from profile
			p = strstr(buf,"[END]");
			if (p) {
				state++;
			}
			else {
				if (buf[0]!='#' && buf[0]!=';' && buf[0]!='/') {
					int r = sscanf(buf,"%s %s",par,val);
					if (r>0) {
						char *p = strstr(buf,"\"");
						if (p) {
							char *r = strstr(p+1,"\"");
							if (r) {
								*r='\0';
								params[string(par)] = string(p+1);
								printf("   %s = %s\n",par,p+1);
							}
						}
						else {
							params[string(par)] = string(val);
							printf("   %s = %s\n",par,val);
						}
					}
				}
			}
			break;

			case 4: // end
			break;
		}
		// cout << buf << endl;
	}

	f.close();
}

ConfigFile::~ConfigFile()
{

}

string ConfigFile::getParam(string p) {
	map<string,string>::const_iterator it = params.find(p);
	if (it!=params.end())
		return it->second;
	else
		return string("");
}

string ConfigFile::getParam(const char *p) {
	return getParam(string(p));
}


int ConfigFile::getIParam(const char *p, int def)
{
	map<string,string>::const_iterator it = params.find(string(p));
	if (it!=params.end())
		return atoi(it->second.c_str());
	else
		return def;
}

double ConfigFile::getDParam(const char *p, double def)
{
	map<string,string>::const_iterator it = params.find(string(p));
	if (it!=params.end())
		return atof(it->second.c_str());
	else
		return def;
}

