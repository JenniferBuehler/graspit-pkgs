#ifndef GRASP_PLANNING_GRASPIT_SHAREDPTR_H
#define GRASP_PLANNING_GRASPIT_SHAREDPTR_H

/**
   Macros for switching between shared pointer implementations.
   Temprorary fix until it's decided which implementation to use.

   Copyright (C) 2016 Jennifer Buehler

   This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program; if not, write to the Free Software Foundation,
   Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301  USA
*/

#define USE_BOOST_SHARED_PTR

#ifdef USE_BOOST_SHARED_PTR

#include <boost/shared_ptr.hpp>
#define SHARED_PTR boost::shared_ptr

#else  // use C++11 std

#include <memory>
#define SHARED_PTR std::shared_ptr

#endif


#endif  // GRASP_PLANNING_GRASPIT_SHAREDPTR_H
