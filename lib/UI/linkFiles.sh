#!/bin/bash
echo 'Linking files from GUISlice folder'

ln -s ../../GUIsliceBuilder/projects/projects.ino ./projects.h
ln -s ../../GUIsliceBuilder/projects/projects_GSLC.h ./projects_GSLC.h

echo "DONE!"