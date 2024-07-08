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

set( COMPILER_TOOLCHAIN GCC CACHE STRING "Set compiler toolchain. Will overwrite native CMake commands" )
if( COMPILER_TOOLCHAIN STREQUAL "GCC" )
    set( CMAKE_C_COMPILER gcc )
    set( CMAKE_CXX_COMPILER g++ )
elseif( COMPILER_TOOLCHAIN STREQUAL "Clang" )
    set( CMAKE_C_COMPILER clang )
    set( CMAKE_CXX_COMPILER clang++ )
elseif( COMPILER_TOOLCHAIN STREQUAL "NVHPC" )
    set( CMAKE_C_COMPILER nvc )
    set( CMAKE_CXX_COMPILER nvc++ )
elseif( COMPILER_TOOLCHAIN STREQUAL "AMDClang" )
    set( CMAKE_C_COMPILER amdclang )
    set( CMAKE_CXX_COMPILER amdclang++ )
elseif( COMPILER_TOOLCHAIN STREQUAL "AOCC" )
    set( CMAKE_C_COMPILER clang )
    set( CMAKE_CXX_COMPILER clang++ )
elseif( COMPILER_TOOLCHAIN STREQUAL "Intel" )
    set( CMAKE_C_COMPILER icc )
    set( CMAKE_CXX_COMPILER icpc )
elseif( COMPILER_TOOLCHAIN STREQUAL "oneAPI" )
    set( CMAKE_C_COMPILER icx )
    set( CMAKE_CXX_COMPILER icpx )
elseif( COMPILER_TOOLCHAIN STREQUAL "Cray" )
    set( CMAKE_C_COMPILER cc )
    set( CMAKE_CXX_COMPILER CC )
else()
    message( FATAL_ERROR "Compiler toolchain not supported. Supported toolchains are: GCC, Clang, NVHPC, AMDClang, AOCC, Intel, oneAPI, Cray" )
endif()
