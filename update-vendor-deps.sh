#!/bin/bash

set -x -e

#bower install

rm -rf vendor
mkdir -p vendor/{css,fonts,js}

cp -r bower_components/bootstrap/dist/js/* vendor/js
wget -P vendor/css https://bootswatch.com/readable/bootstrap.min.css
wget -P vendor/css https://bootswatch.com/readable/bootstrap.css
cp -r bower_components/font-awesome/* vendor

cp -r bower_components/MathJax/unpacked vendor/MathJax
cp -r bower_components/pygments/css vendor/pygments

cp bower_components/jquery/dist/* vendor/js