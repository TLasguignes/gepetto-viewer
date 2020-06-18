# Copyright (c) 2018, 2020 CNRS
# Authors: Joseph Mirabel, Guilhem Saurel
#
#
# This file is part of gepetto-viewer
# gepetto-viewer is free software: you can redistribute it
# and/or modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation, either version
# 3 of the License, or (at your option) any later version.
#
# gepetto-viewer is distributed in the hope that it will be
# useful, but WITHOUT ANY WARRANTY; without even the implied warranty
# of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# General Lesser Public License for more details.  You should have
# received a copy of the GNU Lesser General Public License along with
# gepetto-viewer  If not, see
# <http://www.gnu.org/licenses/>.

cmake_minimum_required(VERSION 3.10)
project(gepetto-viewer-octomap LANGUAGES CXX)

find_package(gepetto-viewer REQUIRED)
find_package(octomap REQUIRED)

IF(GEPETTO_VIEWER_QTVERSION VERSION_LESS 5)
  set(USE_QT4 "QT4")
else()
  set(USE_QT4)
endif()

GEPETTO_GUI_PLUGIN(octomapplugin
  ${USE_QT4}

  HEADERS_NO_MOC
  node.hh

  HEADERS
  plugin.hh

  SOURCES
  plugin.cc
  node.cc

  LINK_DEPENDENCIES
  gepetto-viewer::gepetto-viewer
  ${OCTOMAP_LIBRARIES}
  )

target_include_directories(octomapplugin SYSTEM PUBLIC ${OCTOMAP_INCLUDE_DIRS})
