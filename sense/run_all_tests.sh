#!/usr/bin/env bash

#All components of this software are licensed under the BSD 3-Clause
#License.
#
#Copyright (c) 2012-2015, Andrew Barry, all rights reserved.
#
#Redistribution and use in source and binary forms, with or without
#modification, are permitted provided that the following conditions are
#met:
#
#1. Redistributions of source code must retain the above copyright
#notice, this list of conditions and the following disclaimer.
#
#2. Redistributions in binary form must reproduce the above copyright
#notice, this list of conditions and the following disclaimer in the
#documentation and/or other materials provided with the distribution.
#
#3. Neither the name of the copyright holder nor the names of its
#contributors may be used to endorse or promote products derived from
#this software without specific prior written permission.
#
#
#THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
#A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
#HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
#SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
#LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
#DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
#THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
#(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
#OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

# make bash bail out if anything errors
set -e
set -o pipefail

# error out if the date is before the year 2014
# since that indicates the clock isn't set

if [ `date +%s` -lt 1430231759 ]
then
    echo "Error: time/date is not set correctly."
    echo "Current time/date: `date`"
    exit 1
fi

# read the file
FILE=tests.txt

ROOT=`pwd`

while read line; do
    # ignore comments
    case "$line" in \#*) continue ;; esac

    # ignore empty lines
    if [ -z $line ]
    then
        continue
    fi



    # get directory
    dir=$(dirname "$line")
    filename=$(basename "$line")

    echo "----------------------------------------"
    echo "   Running: $line"
    echo "----------------------------------------"

    cd $ROOT/$dir

    # arduino_run test
    ./$filename


done < $FILE