#!/bin/sh

ignore_file=./script/svnignore.txt
svn propset svn:ignore -F $ignore_file . -R