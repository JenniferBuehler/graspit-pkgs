#ifndef GRASP_PLANNING_GRASPIT_THREADIMPL_H
#define GRASP_PLANNING_GRASPIT_THREADIMPL_H

/**
   Macros for switching between thread implementations.
   Temprorary fix until it's decided which thread implementation to use.

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

#define USE_BOOST_THREAD

#ifdef USE_BOOST_THREAD

#include <boost/thread.hpp>
#define THREAD boost::thread
#define THREAD_CONSTR boost::thread
#define MUTEX boost::mutex
#define RECURSIVE_MUTEX boost::recursive_mutex
#define UNIQUE_LOCK boost::unique_lock<boost::mutex>
#define UNIQUE_RECURSIVE_LOCK boost::unique_lock<boost::recursive_mutex>
#define SLEEP(secs) { boost::this_thread::sleep(boost::posix_time::milliseconds(secs*1000)); }

#else  // use c++11 std

#include <thread>
#include <chrono>
#define THREAD std::thread
#define THREAD_CONSTR std::thread
#define MUTEX std::mutex
#define RECURSIVE_MUTEX std::recursive_mutex
#define UNIQUE_LOCK std::unique_lock<std::mutex>
#define UNIQUE_RECURSIVE_LOCK std::unique_lock<std::recursive_mutex>
#define SLEEP(secs) \
{ \
  std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>(std::floor(secs*1000))));\
}

#endif

#endif  // GRASP_PLANNING_GRASPIT_THREADIMPL_H
