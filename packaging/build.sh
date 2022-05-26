#!/bin/bash

build_dir=$1
dest_dir=$2

cp ${build_dir}/packaging/debian/* ${dest_dir}/DEBIAN/

cd ${build_dir}
mkdir -p ${dest_dir}/usr/bin
cp -f src/mesh_bandwidth_test.py ${dest_dir}/usr/bin/ || exit
chmod +x ${dest_dir}/usr/bin/mesh_bandwidth_test.py || exit

