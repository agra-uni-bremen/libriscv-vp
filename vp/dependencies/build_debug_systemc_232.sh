#!/usr/bin/env bash

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
PREFIX=$DIR/systemc-dist

version=2.3.2
source=systemc-$version.tar.gz

if [ ! -f "$source" ]; then
	wget http://www.accellera.org/images/downloads/standards/systemc/$source
fi

tar xzf $source
# inject custom main function as well as thread and method stubs to avoid catching exceptions
cp debug-patches-systemc-2.3.2/sc_main_main.cpp systemc-$version/src/sysc/kernel/
cp debug-patches-systemc-2.3.2/sc_thread_process.cpp systemc-$version/src/sysc/kernel/
cp debug-patches-systemc-2.3.2/sc_method_process.h systemc-$version/src/sysc/kernel/
cd systemc-$version
mkdir build && cd build
../configure CXXFLAGS='-std=c++14' --prefix=$PREFIX --with-arch-suffix=
make -j4
make install

cd $DIR
