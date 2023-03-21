# readlas2pcd
Read .las file and convert it to .pcd file for use with Point Cloud Library PCL. Also allows for downsampling of the point cloud. 

## Requirements

The code was written in C++ on Linux with the following libraries installed:

+ Point Cloud Library (PCL) (http://www.pointclouds.org/downloads) 
  + see required libraries for PCL
+ libLAS (https://liblas.org) 

(As of 2018, libLAS is in maintenance mode and it does not support LAS 1.4, for which PDAL is used (https://pdal.io/). However, libLAS worked for the version of LAS (LAS 1.0) used for this project.)

## Setup

The following steps need to be followed to setup your LAS to PCL converter:


      git clone https://github.com/svensMPG/readlas2pcd
      cd readlas2pcd
      cmake .
      make


In order to run the most basic conversion on LAS to PCD:


      ./readLAS2PCD <INPUT-FILE>

  
 
