# -*- cmake -*- 
#######################################################################
# Software License Agreement (BSD License)                            #
#                                                                     #
#  Copyright (c) 2015, Frederic Py.                                   #
#  All rights reserved.                                               #
#                                                                     #
#  Redistribution and use in source and binary forms, with or without #
#  modification, are permitted provided that the following conditions #
#  are met:                                                           #
#                                                                     #
#   * Redistributions of source code must retain the above copyright  #
#     notice, this list of conditions and the following disclaimer.   #
#   * Redistributions in binary form must reproduce the above         #
#     copyright notice, this list of conditions and the following     #
#     disclaimer in the documentation and/or other materials provided #
#     with the distribution.                                          #
#   * Neither the name of the TREX Project nor the names of its       #
#     contributors may be used to endorse or promote products derived #
#     from this software without specific prior written permmission.  #
#                                                                     #
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS #
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT   #
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS   #
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE      #
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, #
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,#
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;    #
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER    #
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT  #
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN   #
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE     #
# POSSIBILITY OF SUCH DAMAGE.                                         #
#######################################################################
option(WITH_PYTHON "Enable trex python extension" ON)

if(WITH_PYTHON)
  find_package(Boost QUIET COMPONENTS python)
  message(STATUS "Checking for Boost.Python: ${Boost_PYTHON_LIBRARY}")
  
  if(Boost_PYTHON_LIBRARY)
    # Also need to check for python libraries and paths
    find_package(PythonLibs REQUIRED)
    include_directories(${PYTHON_INCLUDE_DIR})
    message(STATUS "Enabling TREX python adapters.")
    set(py_libs "${PYTHON_LIBRARIES}" CACHE INTERNAL "Python Libraries" FORCE)

    # First we need the library that define trex python_reactor class
    add_library(TREXpython SHARED 
      # C++ source
      exception_helper.cc
      python_reactor.cc
      python_thread.cc
      python_env.cc
      python_listener.cc
      # headers
      python_reactor.hh
      python_thread.hh
      exception_helper.hh
      python_env.hh
      python_listener.hh
      )
    target_link_libraries(TREXpython TREXtransaction ${Boost_PYTHON_LIBRARY} ${PYTHON_LIBRARIES})
    trex_lib(TREXpython core)

    python_add_module(trex trex_pkg.cc 
      utils_decls.cc 
      domain_decls.cc
      transaction_decls.cc
      agent_decls.cc
      )
    add_dependencies(core trex)
    target_link_libraries(trex  ${PYTHON_LIBRARIES} TREXagent TREXpython ${LINK_NO_AS_NEEDED} TREXutils)
    trex_py(trex)
    
  else(Boost_PYTHON_LIBRARY)
    message(WARNING "Did not find Boost.Python : disabling python support")
    set(WITH_PYTHON OFF CACHE BOOL "Disabled due to lack of Boost.Python" FORCE)
  endif(Boost_PYTHON_LIBRARY)
endif(WITH_PYTHON)
