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

function( add_executable_with_test NAME )
    cmake_parse_arguments( ARG "" "LANGUAGE" "SOURCES;RUNTIME_ARGUMENTS" ${ARGN} )
    add_executable( ${NAME} )
    target_compile_options( ${NAME} PUBLIC ${OMPT_C_FLAGS} ${OMPT_CXX_FLAGS} )
    target_link_options( ${NAME} PUBLIC ${OMPT_LINK_FLAGS} )
    target_link_libraries( ${NAME} PUBLIC m )
    target_sources( ${NAME} PRIVATE ${ARG_SOURCES} )

    add_test( NAME ${NAME} COMMAND "${CMAKE_SOURCE_DIR}/test/test_driver.sh"
              $<TARGET_FILE:${NAME}> ${ARG_RUNTIME_ARGUMENTS} )
    set_property( TEST ${NAME} PROPERTY ENVIRONMENT "OMP_TOOL=enabled"
                  "OMP_TOOL_LIBRARIES=$<TARGET_FILE:ompt-printf>" )
    set_tests_properties( ${NAME} PROPERTIES SKIP_RETURN_CODE 127 )
endfunction()

add_executable_with_test( test_parallel LANGUAGE C SOURCES test_parallel.c )
add_executable_with_test( test_task LANGUAGE C SOURCES test_task.c )
add_executable_with_test( test_sections LANGUAGE C SOURCES test_sections.c )
