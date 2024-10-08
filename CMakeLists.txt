#
# Copyright (c) 2024,
# Forschungszentrum Juelich GmbH, Germany
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
# INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
# HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
# STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
# OF THE POSSIBILITY OF SUCH DAMAGE.
#

cmake_minimum_required( VERSION 3.10 )

include( cmake/DefineCompilerToolchain.cmake )
project( ompt-printf LANGUAGES C CXX )

set( CMAKE_VERBOSE_MAKEFILE ON CACHE BOOL "Verbose makefile" )
if( CMAKE_VERSION VERSION_GREATER_EQUAL "3.24.0" )
    set( CMAKE_COMPILE_WARNING_AS_ERROR ON CACHE BOOL "Compile warnings as errors" )
endif()

include( cmake/OMPTChecks.cmake )
OMPT_HEADER_CHECK()
if( NOT HAVE_OMP_TOOLS_HEADER )
    message( FATAL_ERROR "omp-tools.h not found, but required for project" )
endif()
OMPT_ADDITIONAL_CFLAGS()
message( "OMPT_C_FLAGS = ${OMPT_C_FLAGS}, OMPT_CXX_FLAGS = ${OMPT_CXX_FLAGS}, required to build interface" )
OMPT_ADDITIONAL_LDFLAGS()
message( "OMPT_LINK_FLAGS = ${OMPT_LINK_FLAGS} required to build interface" )


add_subdirectory( src )

option( OMPT_PRINTF_BUILD_TESTING "Build tests" ON )
if( OMPT_PRINTF_BUILD_TESTING )
    enable_testing()
    add_subdirectory( test )
endif()
