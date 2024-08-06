#!/bin/bash

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

# For a given executable, passed via $1, this script should test the available
# options of ompt-printf's modes.The test should return 0 if all tests pass or
# 1 if the test failed. For testing the modes, the script should set
# OMPT_PRINTF_MODE to values between 0 and 3 and run the executable.The modes
# are described in the README.md file.

if [ "$#" -ne 1 ]; then
    echo "Usage: $0 <executable>" > /dev/stderr
    exit 1
fi

executable=$1
if [ ! -x "$executable" ]; then
    echo "Error: $executable is not an executable" > /dev/stderr
    exit 127
fi

for mode in $(seq 0 3); do
    export OMPT_PRINTF_MODE=$mode
    if ! "$executable" "$@" > test_output; then
        echo "Error: $executable failed with \
              OMPT_PRINTF_MODE=$mode" > /dev/stderr
        rm -f test_output
        exit 1
    fi
    if ! grep -q "Chosen printf mode: $mode" test_output; then
        echo "Error: $executable did not print the correct printf mode with \
              OMPT_PRINTF_MODE=$mode" > /dev/stderr
        rm -f test_output
        exit 1
    fi
    # If mode was 0 or 1, we should not get [tool_initialize]
    if [ "$mode" -lt 2 ]; then
        if ! grep -q "[tool_initialize]" test_output ; then
            echo "Error: $executable printed [tool_initialize] with \
                  OMPT_PRINTF_MODE=$mode" > /dev/stderr
            cat test_output
            rm -f test_output
            exit 1
        fi
    fi
done

rm -f test_output
exit 0
