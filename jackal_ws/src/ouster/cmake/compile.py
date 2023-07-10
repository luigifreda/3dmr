#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Copyright (C) 2008-2014 LAAS-CNRS, JRL AIST-CNRS.
#
# This file is part of jrl-cmakemodules.
# jrl-cmakemodules is free software: you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public License
# as published by the Free Software Foundation, either version 3 of
# the License, or (at your option) any later version.
#
# jrl-cmakemodules is distributed in the hope that it will be useful, but
# WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# General Lesser Public License for more details.  You should have
# received a copy of the GNU Lesser General Public License along with
# jrl-cmakemodules. If not, see <http://www.gnu.org/licenses/>.

import sys, os, py_compile

srcdir = sys.argv[1]
builddir = sys.argv[2]
name = sys.argv[3]

if srcdir[-1] != '/':
    srcdir = srcdir + '/'
if builddir[-1] != '/':
    builddir = builddir + '/'

src = srcdir + name
comp = builddir + name + (__debug__ and 'c' or 'o')

#print("compiling " + src + " into " + comp)

#os.mkdir(os.path.splittext(comp)[0])

try:
    py_compile.compile(src, comp, doraise=True)
except Exception as e:
    print ("Failed to compile python script: {0}".format (repr (src)))
    print ("Exception raised: {0}".format (str(e)))
    sys.exit(1)
except:
    print ("Failed to compile python script: {0}".format (repr (src)))
    sys.exit(1)
