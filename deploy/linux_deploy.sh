#!/bin/bash
# this is a script shell for setting up the application bundle for linux
# It should be run (not sourced) in the meshlab/src/install/linux dir.
 
export DEPLOY_PATH=$(pwd)
cd ../release

#check if we have an exec in distrib
if [ -f SplitAndMill ]
then
  echo "------------------"
else
  echo "ERROR: SplitAndMill bin not found inside release folder"
  exit 1
fi

mkdir usr
mkdir usr/bin
mkdir usr/share
mkdir usr/share/applications
mkdir usr/share/applications/icons/
mkdir usr/share/applications/icons/hicolor
mkdir usr/share/applications/icons/hicolor/256x256
mkdir usr/share/applications/icons/hicolor/256x256/apps

cp SplitAndMill usr/bin/
cp ../res/split_and_mill.png usr/share/applications/icons/hicolor/256x256/apps/
cp $DEPLOY_PATH/SplitAndMill.desktop usr/share/applications/

$DEPLOY_PATH/linuxdeployqt usr/share/applications/SplitAndMill.desktop -appimage

#at this moment, the release folder contains all the files necessary to execute SplitAndMill
echo "release folder is now a self contained SplitAndMill application"