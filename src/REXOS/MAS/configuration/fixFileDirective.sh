#!/bin/bash
find -name *.java | sed -e 's#\./##'| xargs -n 1 -i sed -i -e 's#@file .*$#@file {}#' {}
