#ifndef GRASP_PLANNING_GRASPIT_LOGBINDING_H
#define GRASP_PLANNING_GRASPIT_LOGBINDING_H
/**
   Logging class to abstract from the type of logging to be used.

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

#include <string>
#include <iostream>
#include <sstream>
#include <grasp_planning_graspit/SharedPtr.h>

namespace GraspIt
{

/**
 * \brief Class to bind to a certain type of logging.
 * \author Jennifer Buehler
 * \date December 2015
 */
class Log
{
public:
    static void print(const std::stringstream& str)
    {
        if (sglOK(str.str().c_str())) Singleton->implPrint(str);
    }
    static void printError(const std::stringstream& str)
    {
        if (sglOK(str.str().c_str())) Singleton->implPrintError(str);
    }
    static void printWarn(const std::stringstream& str)
    {
        if (sglOK(str.str().c_str())) Singleton->implPrintWarn(str);
    }

    static void print(const char * str)
    {
        if (sglOK(str)) Singleton->implPrint(str);
    }
    static void printError(const char * str)
    {
        if (sglOK(str)) Singleton->implPrintError(str);
    }
    static void printWarn(const char * str)
    {
        if (sglOK(str)) Singleton->implPrintWarn(str);
    }

    static void printLn(const std::stringstream& str)
    {
        if (sglOK(str.str().c_str()))
        {
            Singleton->implPrint(str);
            Singleton->printNewLine(false);
        }
    }
    static void printErrorLn(const std::stringstream& str)
    {
        if (sglOK(str.str().c_str()))
        {
            Singleton->implPrintError(str);
            Singleton->printNewLine(false);
        }
    }
    static void printWarnLn(const std::stringstream& str)
    {
        if (sglOK(str.str().c_str()))
        {
            Singleton->implPrintWarn(str);
            Singleton->printNewLine(false);
        }
    }

    static void printLn(const char * str)
    {
        if (sglOK(str))
        {
            Singleton->implPrint(str);
            Singleton->printNewLine(false);
        }
    }
    static void printErrorLn(const char * str)
    {
        if (sglOK(str))
        {
            Singleton->implPrintError(str);
            Singleton->printNewLine(false);
        }
    }
    static void printWarnLn(const char * str)
    {
        if (sglOK(str))
        {
            Singleton->implPrintWarn(str);
            Singleton->printNewLine(false);
        }
    }

    static SHARED_PTR<Log> Singleton;

protected:
    virtual void implPrint(const std::stringstream& str) = 0;
    virtual void implPrintError(const std::stringstream& str) = 0;
    virtual void implPrintWarn(const std::stringstream& str) = 0;
    virtual void implPrint(const char * str) = 0;
    virtual void implPrintError(const char * str) = 0;
    virtual void implPrintWarn(const char * str) = 0;
    /** Subclasses which do NOT automatically make a new line
     * in implPrint* functions need to implement the printing of a new line
     * in this function.
     * \param errorStream set to true if the new line is to be printed in the error
     * stream. Otherwise is printed to the standard stream.
     */
    virtual void printNewLine(bool errorStream) = 0;

private:
    /**
     * If singleton is NULL, prints the message on std::cout and
     * only once prints a warning that Singleton is not inisialized.
     * If singleton is not NULL, this method just returns true.
     */
    static bool sglOK(const char * msg)
    {
        if (Singleton.get()) return true;

        if (!initSglWarningPrinted)
        {
            std::cerr << "WARNING: Initialise Log Singleton to use the proper Logger. Now printing to std out." << std::endl;
            initSglWarningPrinted = true;
        }
        std::cout << msg << std::endl;
        return false;
    }

    // print warning that Singleton is not initialized only once.
    static bool initSglWarningPrinted;
};

/**
 * \brief Simple implementation of a log which just prints the logs on std::cout and std::cerr
 * \author Jennifer Buehler
 * \date December 2015
 */
class StdLog: public Log
{
protected:
    virtual void implPrint(const std::stringstream& str)
    {
        std::cout << str.str();
    }
    virtual void implPrintError(const std::stringstream& str)
    {
        std::cerr << "ERROR:" << str.str();
    }
    virtual void implPrintWarn(const std::stringstream& str)
    {
        std::cout << "WARNING:" << str.str();
    }

    virtual void implPrint(const char* str)
    {
        std::cout << str;
    }
    virtual void implPrintError(const char* str)
    {
        std::cerr << "ERROR: " << str;
    }
    virtual void implPrintWarn(const char* str)
    {
        std::cout << "WARNING: " << str;
    }


    virtual void printNewLine(bool errorStream)
    {
        if (errorStream) std::cerr << std::endl;
        else std::cout << std::endl;
    }
};

}  // namespace GraspIt

extern std::string getFilenameFromPath(const std::string& path);

// in a path to a file name, erase the filename from it and return only directory path
extern std::string getFileDirectory(const std::string& pathToFile);

// initialize the print log singleton to use std::out and std::err
#define PRINT_INIT_STD() \
{\
    if (GraspIt::Log::Singleton) \
    {\
        std::cerr << "Singleton already set, overwriting!" << std::endl;\
    }\
    GraspIt::Log::Singleton = SHARED_PTR<GraspIt::Log>(new GraspIt::StdLog()); \
}


#define PRINTMSG(msg) \
{\
    std::stringstream _str_; \
    _str_ << msg << " - "<< getFilenameFromPath(__FILE__) << ", " << __LINE__; \
    GraspIt::Log::printLn(_str_); \
}

#define PRINTDEBUG(msg) \
{\
    std::stringstream _str_; \
    _str_ << msg << " - "<< getFilenameFromPath(__FILE__) << ", " << __LINE__; \
    GraspIt::Log::printLn(_str_); \
}

#define PRINTERROR(msg) \
{\
    std::stringstream _str_; \
    _str_ << msg << " - "<< getFilenameFromPath(__FILE__) << ", " << __LINE__; \
    GraspIt::Log::printErrorLn(_str_); \
}

#define PRINTWARN(msg) \
{\
    std::stringstream _str_; \
    _str_ << msg << " - "<< getFilenameFromPath(__FILE__) << ", " << __LINE__; \
    GraspIt::Log::printWarnLn(_str_); \
}


#endif  // GRASP_PLANNING_GRASPIT_LOGBINDING_H
