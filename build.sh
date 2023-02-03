#!/bin/bash
rm -r build/*
pushd build
cmake ..
make
popd
