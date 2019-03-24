#!/bin/bash

find . -name *.h -o -name *.cxx -o -name *.hxx -o -name *.c | xargs clang-format -i -style=file
