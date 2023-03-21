#ifndef CONVERTLAS_H

#include <fstream>
#include <vector>
#include <string>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/console/time.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/visualization/cloud_viewer.h>

// libLAS library
#include <liblas/liblas.hpp>
#define CONVERTLAS_H


//////////////////////////////////////////
///
///
int toTXT(std::string fileToRead, unsigned int skipPts, unsigned int colorDepth)
{

    std::cout << "debug: writing the point cloud to text file. This will take some time....\n";
    // 1) create a file stream object to access the file
    std::ifstream ifs;
    ifs.open(fileToRead, std::ios::in | std::ios::binary);
    // if the LAS file could not be opend. throw an error (using the PCL_ERROR functionality).
    if (!ifs.is_open())
    {
        PCL_ERROR ("Couldn't read file ");
        return -1;
    }

    // set up ReaderFactory of the LibLAS library for reading in the data.
    std::cout << "Reading in LAS input file: " << fileToRead << std::endl;

    liblas::ReaderFactory f;
    liblas::Reader reader = f.CreateWithStream(ifs);

    liblas::Header const& header = reader.GetHeader();

    long int nPts = header.GetPointRecordsCount();
    std::cout << "Compressed:  " << (header.Compressed() == true) ? "true\n":"false\n";
    std::cout << "\nSignature: " << header.GetFileSignature() << '\n';
    std::cout << "Points count: " << nPts << '\n';

    long long pInfo = static_cast<long long>(nPts * 0.25);

    long long i = 0;

    std::string fileToWrite = fileToRead + ".txt";
    std::ofstream txtfile (fileToWrite);

    unsigned int intensity;
    while (reader.ReadNextPoint()){


        if (i % skipPts == 0){
            liblas::Point const& p = reader.GetPoint();
            txtfile.precision(11);
            // convert intensity from an 8 bits scale to an 16 bits scale.
            if (colorDepth == 16)
                intensity= static_cast<unsigned int>( ( (p.GetIntensity() - 0.0) / (255.0 - 0.0) ) * 65535.0 );
            else
                intensity = static_cast<unsigned int>( p.GetIntensity() );

            txtfile << p.GetX() << " " << p.GetY() << " " << p.GetZ() << " " << intensity << " " << intensity << " " << intensity << std::endl;

            if (i % pInfo == static_cast<long long> (0))
                std::cout << "Point nr. :" <<  i  << "\t\t  x: " << p.GetX() << "  y: " << p.GetY() << "  z: " << p.GetZ() << intensity << "\n";

        }

       i++;
    }

    txtfile.close();

    std::cerr << "Saved " << i << " Points to " << fileToWrite << " \n" << std::endl;


}


template <typename PointInT> int
toPCD(std::string fileToRead,
      typename pcl::PointCloud<PointInT>::Ptr &cloud,
      std::vector<double> &minXYZValues,
      float gridLeafSize,
      bool subtractMinVals,
      std::string ext){


    // 1) create a file stream object to access the file
    std::ifstream ifs;
    ifs.open(fileToRead, std::ios::in | std::ios::binary);
    // if the LAS file could not be opend. throw an error (using the PCL_ERROR functionality).
    if (!ifs.is_open())
    {
        PCL_ERROR ("Couldn't read file ");
        return -1;
    }

    // set up ReaderFactory of the LibLAS library for reading in the data.
    std::cout << "Reading in LAS input file: " << fileToRead << std::endl;

    liblas::ReaderFactory f;
    liblas::Reader reader = f.CreateWithStream(ifs);

    liblas::Header const& header = reader.GetHeader();

    long int nPts = header.GetPointRecordsCount();
    std::cout << "Compressed:  " << (header.Compressed() == true) ? "true\n":"false\n";
    std::cout << "\nSignature: " << header.GetFileSignature() << '\n';
    std::cout << "Points count: " << nPts << '\n';

    int pInfo = nPts * 0.025;

    // Fill in the PCD cloud data
    cloud->width    = nPts;
    cloud->height   = 1;
    cloud->is_dense = true;
    cloud->points.resize (cloud->width * cloud->height);
    int i = 0;

    std::vector<double> PCPointsX;
    std::vector<double> PCPointsY;
    std::vector<double> PCPointsZ;


    double minX = 9999999999, minY= 9999999999, minZ = 9999999999;


    while (reader.ReadNextPoint()){
        liblas::Point const& p = reader.GetPoint();


        if (p.GetX() < minX)
            minX = p.GetX();

        if (p.GetY() < minY)
            minY = p.GetY();

        if (p.GetZ() < minZ)
            minZ = p.GetZ();

        // push in the original double values to subtract the minXYZ values from those
        // befor!! casting them to float.
        PCPointsX.push_back(p.GetX());
        PCPointsY.push_back(p.GetY());
        PCPointsZ.push_back(p.GetZ());

        // in case the user has not specified the -setZero flag. Take the values as they
        // are, without subtracting the min values and just cast them to float.
        cloud->points[i].x = float (p.GetX());
        cloud->points[i].y = float (p.GetY());
        cloud->points[i].z = float (p.GetZ());
        cloud->points[i].intensity = p.GetIntensity();

        if (i % pInfo == 0)
            std::cout << "Point nr. :" <<  i  << "\t\t  x: " << p.GetX() << "  y: " << p.GetY() << "  z: " << p.GetZ() << "\n";


        i++;
    }

    // store the minValues so they can be added in the final processing step to get the true coordinates
    minXYZValues.push_back(minX); minXYZValues.push_back(minY); minXYZValues.push_back(minZ);

    // if true, subtract the min values now. This will normally be 'true'
    if (subtractMinVals){
      pcl::console::print_highlight("\n\nRemoving min x,y,z coordinate values.\n\n");
      pcl::console::print_highlight("minX = %f  , minY = %f , minZ = %f \n\n", minX, minY, minZ);
      for (int i=0; i< cloud->points.size(); ++i){
          cloud->points[i].x = PCPointsX[i] - minX;
          cloud->points[i].y = PCPointsY[i] - minY;
          cloud->points[i].z = PCPointsZ[i] - minZ;

          if (i % pInfo == 0){
            std::cout << "Point nr. :" << i  << "\t\t  x: " << cloud->points[i].x << "  y: " << cloud->points[i].y <<
                         "  z: " << cloud->points[i].z  << "    intensity:  " << cloud->points[i].intensity << std::endl;
          }
      }
    }


    pcl::console::TicToc time;
    time.tic ();

    typename pcl::PointCloud<PointInT>::Ptr cloudFiltered =  boost::make_shared<pcl::PointCloud<PointInT> >();


    if (gridLeafSize > 0.025 && gridLeafSize < 1){
      std::cout << "\nApplying Uniform downsampling with leafSize " << gridLeafSize << ". Processing...";

      pcl::UniformSampling<PointInT> uniform_sampling;
      uniform_sampling.setInputCloud (cloud);
      uniform_sampling.setRadiusSearch (gridLeafSize); //the 3D grid leaf size
      uniform_sampling.filter(*cloudFiltered);
      std::cout << "Downsampled in " << time.toc() / 1000. << " seconds \n" << std::endl;
      pcl::copyPointCloud(*cloudFiltered, *cloud);  // cloud is given by reference so the downsampled cloud has to be copied in there

    }
    else  // else copy original cloud in cloud Filtered and save file...
        pcl::copyPointCloud(*cloud,*cloudFiltered);

    // if (ext.compare("pcd") == 0){
    std::string fileToWrite = fileToRead + ".pcd";
    std::cout << "Writing PCD output file: " << fileToWrite << std::endl;
    pcl::io::savePCDFile (fileToWrite, *cloudFiltered,false);
    std::cerr << "Saved " << cloudFiltered->points.size () << " Points to " << fileToWrite << "\n" << std::endl;
    // }
    // else{
    //     std::string fileToWrite = fileToRead + ".ply";
    //     std::cout << "Writing PLY output file: " << fileToWrite << std::endl;
    //     pcl::io::savePLYFileASCII(fileToWrite, *cloudFiltered);
    //     std::cerr << "Saved " << cloudFiltered->points.size () << " Points to " << fileToWrite << "\n" << std::endl;
    // }




}



template <typename PointInT> void
convertLAS(std::string fileToRead,
            typename pcl::PointCloud<PointInT>::Ptr &cloud,
            std::vector<double> &minXYZValues,
            float gridLeafSize,
            bool subtractMinVals,
            std::string ext,
            unsigned int skipPts,
            unsigned int colorDepth)
{
    // if(ext.compare("pcd") == 0 || ext.compare("ply") == 0)
    toPCD <PointInT> (fileToRead, cloud, minXYZValues, gridLeafSize, subtractMinVals, ext);
    // else
    // toTXT(fileToRead, skipPts, colorDepth);

}









#endif // CONVERTLAS_H
