/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/

#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<opencv2/core/core.hpp>

#include<System.h>

#include <cassert>
#include <cstdlib>
#include <cstdio>
#include <filesystem>

#include <iostream>
#include <list>

#include <iostream>
#include <ctime>
#include <unistd.h>

#include <PvSampleUtils.h>
#include <PvDevice.h>
#include <PvDeviceGEV.h>
#include <PvStream.h>
#include <PvStreamGEV.h>
#include <PvBuffer.h>
#include <opencv2/opencv.hpp>
#include <PvConfigurationWriter.h>

#include <PvPipeline.h>
#include "chunk.hpp"
#include<PvMultiPartSection.h>
#include <PvChunkData.h>

using namespace std;
using namespace cv;
namespace fs = std::filesystem;
char*gDirname;

PV_INIT_SIGNAL_HANDLER();

#define BUFFER_COUNT ( 16 )

///
/// Function Prototypes
///
PvDevice *ConnectToDevice( const PvString &aConnectionID );
PvStream *OpenStream( const PvString &aConnectionID );
void ConfigureStream( PvDevice *aDevice, PvStream *aStream );
PvPipeline* CreatePipeline( PvDevice *aDevice, PvStream *aStream );
void AcquireImages( PvDevice *aDevice, PvStream *aStream, PvPipeline *aPipeline  );

string vocab_file("/home/emslab/ORB_SLAM3/Vocabulary/ORBvoc.txt");
string yaml_file("/home/emslab/ORB_SLAM3/Examples/Monocular/bottlenose5.yaml");

ORB_SLAM3::System SLAM(vocab_file,yaml_file,ORB_SLAM3::System::MONOCULAR, true, 0, string("Testrun"));

float gWbRed, gWbGreen, gWbBlue;
vector<double> ttrackvector;
vector<double> vTimesTrack;
int proccIm = 0;

void SavePair( PvBuffer *aBuffer ) {
  static int count = 0;
  assert(aBuffer->GetMultiPartContainer()->GetPartCount() == 2);
  IPvImage *img0 = aBuffer->GetMultiPartContainer()->GetPart(0)->GetImage();
  IPvImage *img1 = aBuffer->GetMultiPartContainer()->GetPart(1)->GetImage();
  // Note images are YUVY
  Mat leftIn(img0->GetHeight(), img0->GetWidth(), CV_8UC2, img0->GetDataPointer());
  Mat rightIn(img1->GetHeight(), img1->GetWidth(), CV_8UC2, img1->GetDataPointer());
  // Convert to BGR (standard OpenCV format)

  Mat left, right;
  cvtColor(leftIn, left, COLOR_YUV2BGR_YUYV);
  cvtColor(rightIn, right, COLOR_YUV2BGR_YUYV);

  // Save to disk
  stringstream s;
  s << count++;
  string fname_left = string(gDirname) + "/left_" + s.str() + ".png";
  string fname_right = string(gDirname) + "/right_" + s.str() + ".png";
  imwrite(fname_left, left);
  imwrite(fname_right, right);
}

void Saveimg( PvBuffer *aBuffer ) {


  // Vector for tracking time statistics
  //  vector<float> vTimesTrack;
  
  //cout << endl << "-------" << endl;
  cout.precision(17);

  int fps = 20;
  float dT = 1.f/fps;
  
  //ORB_SLAM3::System SLAM(vocab_file,yaml_file,ORB_SLAM3::System::MONOCULAR, true, 0, string("Testrun"));
  //SLAM(vocab_file,yaml_file,ORB_SLAM3::System::MONOCULAR, true, 0, string("Testrun"));
  float imageScale = SLAM.GetImageScale();

  double t_resize = 0.f;
  double t_track = 0.f;

  // Main loop
  //cv::Mat im;
  //int proccIm = 0;
  //int ni=0;
  std::timespec ts;

  static int count = 0;
  IPvImage *img0 = aBuffer->GetImage();
  
  // Note images are YUVY
  Mat leftIn(img0->GetHeight(), img0->GetWidth(), CV_8UC2, img0->GetDataPointer());
  
  // Convert to BGR (standard OpenCV format)
  //Mat left, right;
  Mat im;
  cvtColor(leftIn, im, COLOR_YUV2BGR_YUYV);

  // Save to disk
  stringstream s;
  s << count++;
  string fname_left = string(gDirname) + "/left_" + s.str() + ".png";
  
  //imwrite(fname_left, im);

  keypoints_t* bottlenose_keypoints = (keypoints_t*)aBuffer->GetChunkRawDataByID(CHUNK_ID_FEATURES);
  //cout << "keypoints->count: " << bottlenose_keypoints->count << endl;

  std::vector<cv::KeyPoint> opencv_keypoints;  
  for (uint32_t i = 0; i < bottlenose_keypoints->count; i++) 
  {
    cv::KeyPoint opencv_keypoint(bottlenose_keypoints->points[i].x, bottlenose_keypoints->points[i].y, 0,-1,0,0,0);
    opencv_keypoints.push_back(opencv_keypoint);
  }

  //cv::FileStorage fskptsLeft("keypointsLeft.yml", cv::FileStorage::APPEND);
  //write( fskptsLeft , "img99", opencv_keypoints );
  //fskptsLeft.release();

  descriptors_t* bottlenose_descriptors = (descriptors_t*)aBuffer->GetChunkRawDataByID(CHUNK_ID_DESCRIPTORS);
  //cout << "descriptors->count: " << bottlenose_descriptors->count << endl;

  //cv::Mat opencv_descriptors = cv::Mat::zeros(0,64,CV_8U);
  cv::Mat opencv_descriptors = cv::Mat::zeros(0,32,CV_8U);
  for (uint32_t i = 0; i < bottlenose_descriptors->count; i++) 
  {
    cv::Mat row = cv::Mat(1, 64, CV_8U, bottlenose_descriptors->descriptors[i].data);
    cv::Mat row_r = row.colRange(0,32).clone(); 
    opencv_descriptors.push_back(row_r);
  }

  //cv::FileStorage fsdescsLeft("descriptorsLeft.yml", cv::FileStorage::APPEND);
  //write( fsdescsLeft , "img99", opencv_descriptors );
  //fsdescsLeft.release();


  //exit(0);

  // cout << "DEBUG" << endl;
  // cout << "DEBUG" << endl;
  // cout << "DEBUG" << endl;

  // Read image from file
  //im = cv::imread(vstrImageFilenames[seq][ni],cv::IMREAD_UNCHANGED); //,CV_LOAD_IMAGE_UNCHANGED);
  double tframe;// = vTimestampsCam[seq][ni];
  std::timespec_get(&ts, TIME_UTC);
  tframe = ts.tv_sec*1000000000+ ts.tv_nsec;
  //cout << "timestamp: "<<tframe<<endl;
  if(im.empty())
  {
      cerr << endl << "Failed to load image at time: " <<  tframe << endl;
      exit(1);
  }

  if(imageScale != 1.f)
  {
      std::cout << "Resizing..." << std::endl;
#ifdef REGISTER_TIMES
#ifdef COMPILEDWITHC11
      std::chrono::steady_clock::time_point t_Start_Resize = std::chrono::steady_clock::now();
#else
      std::chrono::monotonic_clock::time_point t_Start_Resize = std::chrono::monotonic_clock::now();
#endif
#endif
      int width = im.cols * imageScale;
      int height = im.rows * imageScale;
      cv::resize(im, im, cv::Size(width, height));
#ifdef REGISTER_TIMES
#ifdef COMPILEDWITHC11
      std::chrono::steady_clock::time_point t_End_Resize = std::chrono::steady_clock::now();
#else
      std::chrono::monotonic_clock::time_point t_End_Resize = std::chrono::monotonic_clock::now();
#endif
      t_resize = std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(t_End_Resize - t_Start_Resize).count();
      SLAM.InsertResizeTime(t_resize);
#endif
  }

#ifdef COMPILEDWITHC11
  std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
  std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

  // Pass the image to the SLAM system
  // cout << "tframe = " << tframe << endl;
  //SLAM.TrackMonocular(im,tframe); // TODO change to monocular_inertial
  SLAM.TrackMonocular(im,opencv_keypoints, opencv_descriptors, tframe);

#ifdef COMPILEDWITHC11
  std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
  std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

#ifdef REGISTER_TIMES
  t_track = t_resize + std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(t2 - t1).count();
  SLAM.InsertTrackTime(t_track);
#endif

  double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

  // average = calcAverage(vdORBExtract_ms);
  //   deviation = calcDeviation(vdORBExtract_ms, average);
  //   std::cout << "ORB Extraction: " << average << "$\\pm$" << deviation << std::endl;
  //   f << "ORB Extraction: " << average << "$\\pm$" << deviation << std::endl;
  //vTimesTrack[ni]=ttrack;

  // sleep till the next frame
  //std::cout << "ttrack: " << ttrack << std::endl;
  proccIm++;
  //vTimesTrack.resize(proccIm);
  //vTimesTrack[proccIm]=ttrack;
  vTimesTrack.push_back(ttrack);
  
  //ttrackvector.push_back(mCurrentFrame.mTimeORB_Ext);

  // if(ttrack<0.05) {
  //     //std::cout << "usleep: " << (0.05-ttrack) << std::endl;
  //     usleep((0.05-ttrack)*1e6); // 1e6
  // }
  /*i dont camera is going to send images any faster*/
  /*
  if(ttrack<0.1) {
       //std::cout << "usleep: " << (0.05-ttrack) << std::endl;
       usleep((0.1-ttrack)*1e6); // 1e6
   }
   */
}

PvDevice *ConnectToDevice( const PvString &aConnectionID )
{
  PvDevice *lDevice;
  PvResult lResult;

  // Connect to the Bottlenose camera
  cout << "Connecting to device." << endl;
  lDevice = PvDevice::CreateAndConnect( aConnectionID, &lResult );
  if ( lDevice == nullptr )
  {
    cout << "Unable to connect to device: "
         << lResult.GetCodeString().GetAscii()
         << " ("
         << lResult.GetDescription().GetAscii()
         << ")" << endl;
  }


  return lDevice;
}

PvStream *OpenStream( const PvString &aConnectionID )
{
  PvStream *lStream;
  PvResult lResult;

  // Open stream to the Bottlenose camera
  cout << "Opening stream from device." << endl;
  lStream = PvStream::CreateAndOpen( aConnectionID, &lResult );
  if ( lStream == nullptr )
  {
    cout << "Unable to stream from device. "
         << lResult.GetCodeString().GetAscii()
         << " ("
         << lResult.GetDescription().GetAscii()
         << ")"
         << endl;
  }

  return lStream;
}


void ConfigureStream( PvDevice *aDevice, PvStream *aStream )
{
  // If this is a GigE Vision device, configure GigE Vision specific streaming parameters
  PvDeviceGEV* lDeviceGEV = dynamic_cast<PvDeviceGEV *>( aDevice );
  PvResult res;

  if ( lDeviceGEV != nullptr )
  {
    PvStreamGEV *lStreamGEV = static_cast<PvStreamGEV *>( aStream );

    // Negotiate packet size
    lDeviceGEV->NegotiatePacketSize();

    // Configure device streaming destination
    lDeviceGEV->SetStreamDestination( lStreamGEV->GetLocalIPAddress(), lStreamGEV->GetLocalPort() );

    uint32_t paramcount;
    PvGenParameterArray *lStreamParams = lStreamGEV->GetParameters();
    paramcount = lStreamParams->GetCount();
    cout << "paramcount: "<< paramcount << endl;

    PvConfigurationWriter lWriter1;
    PvConfigurationWriter lWriter2;
    PvConfigurationWriter lWriter3;
    PvConfigurationWriter lWriter4;

    lWriter1.Store( lStreamParams, "DeviceConfiguration" );
    cout << "Save device configuration" << endl;
    lWriter1.Save( "PersistenceTest1.pvxml" );

    lWriter2.Store( aDevice, "DeviceConfiguration" );
    lWriter2.Save( "PersistenceTest2.pvxml" );
    lWriter3.Store( aStream, "DeviceConfiguration" );
    lWriter3.Save( "PersistenceTest3.pvxml" );

    //PvGenParameterArray *lDeviceGEVParams = lDeviceGEV->GetParameters();
    lWriter4.Store( lDeviceGEV, "DeviceConfiguration" );
    lWriter4.Save( "PersistenceTest4.pvxml" );

    // Tweak GEV timeouts to avoid ABORT and TIMEOUT
    PvGenInteger *lResetOnIdle = dynamic_cast<PvGenInteger *>(lStreamGEV->GetParameters()->Get("ResetOnIdle"));
    assert(lResetOnIdle != nullptr);
    res = lResetOnIdle->SetValue(0);
    assert(res.IsOK());

    // Avoids MISSING_PACKETS
    PvGenInteger *lRequestTimeout = dynamic_cast<PvGenInteger *>(lStreamGEV->GetParameters()->Get("RequestTimeout"));
    assert(lRequestTimeout != nullptr);
    res = lRequestTimeout->SetValue(100000);
    assert(res.IsOK());

    //Force multi-part transmission, this enables stereo image transfer on Bottlenose
    // PvGenBoolean *lMultipart = dynamic_cast<PvGenBoolean *>( lDeviceGEV->GetParameters()->Get( "GevSCCFGMultiPartEnabled" ) );
    // assert(lMultipart != nullptr);
    // res = lMultipart->SetValue(true);
    // assert(res.IsOK());

    // Configure gain and exposure
    PvGenFloat *lExposure = dynamic_cast<PvGenFloat *>( lDeviceGEV->GetParameters()->Get( "exposure" ) );
    assert(lExposure != nullptr);
    res = lExposure->SetValue(5);
    assert(res.IsOK());

    PvGenFloat *lGain = dynamic_cast<PvGenFloat *>( lDeviceGEV->GetParameters()->Get( "gain" ) );
    assert(lGain != nullptr); 
    res = lGain->SetValue(20);
    assert(res.IsOK());

    PvGenInteger *packetsize = dynamic_cast<PvGenInteger *>( lDeviceGEV->GetParameters()->Get( "DeviceStreamChannelPacketSize" ) );
    assert(packetsize != nullptr); 
    res = packetsize->SetValue(9000);
    assert(res.IsOK());

    PvGenInteger *Width = dynamic_cast<PvGenInteger *>( lDeviceGEV->GetParameters()->Get( "Width" ) );
    assert(Width != nullptr);
    res = Width->SetValue(1920);
    assert(res.IsOK());

    PvGenInteger *Height = dynamic_cast<PvGenInteger *>( lDeviceGEV->GetParameters()->Get( "Height" ) );
    assert(Height != nullptr);
    res = Height->SetValue(1080);
    assert(res.IsOK());

    PvGenFloat *lInterval = dynamic_cast<PvGenFloat *>( lDeviceGEV->GetParameters()->Get( "interval" ) );
    assert(lInterval != nullptr);
    //res = lInterval->SetValue(200);
    //res = lInterval->SetValue(50);
    res = lInterval->SetValue(100);
    assert(res.IsOK());

    PvGenBoolean *undistortion = dynamic_cast<PvGenBoolean *>( lDeviceGEV->GetParameters()->Get( "Undistortion" ) );
    assert(undistortion != nullptr);
    res = undistortion->SetValue(false);
    assert(res.IsOK());

    PvGenBoolean *rectification = dynamic_cast<PvGenBoolean *>( lDeviceGEV->GetParameters()->Get( "Rectification" ) );
    assert(rectification != nullptr);
    res = rectification->SetValue(false);
    assert(res.IsOK());

    PvGenInteger *dgainBlue = dynamic_cast<PvGenInteger *>( lDeviceGEV->GetParameters()->Get( "dgainBlue" ) );
    assert(dgainBlue != nullptr);
    res = dgainBlue->SetValue(512);
    assert(res.IsOK());

    PvGenInteger *dgainGB = dynamic_cast<PvGenInteger *>( lDeviceGEV->GetParameters()->Get( "dgainGB" ) );
    assert(dgainGB != nullptr);
    res = dgainGB->SetValue(512);
    assert(res.IsOK());

    PvGenInteger *dgainGR = dynamic_cast<PvGenInteger *>( lDeviceGEV->GetParameters()->Get( "dgainGR" ) );
    assert(dgainGR != nullptr);
    res = dgainGR->SetValue(512);
    assert(res.IsOK());

    PvGenInteger *dgainRed = dynamic_cast<PvGenInteger *>( lDeviceGEV->GetParameters()->Get( "dgainRed" ) );
    assert(dgainRed != nullptr);
    res = dgainRed->SetValue(512);
    assert(res.IsOK());

    /**Enabling keypoint and descriptor generation **/

    PvGenParameterArray* deviceParams = lDeviceGEV->GetParameters();

    PvGenBoolean* chunkEnable = deviceParams->GetBoolean("ChunkModeActive");
    assert(chunkEnable != nullptr);
    res = chunkEnable->SetValue(true);

    //PvGenEnum* sourceSelector = lDeviceGEV->GetEnum("SourceSelector");
    //sourceSelector->FromString("Source0");

    PvGenEnum* chunkSelector = deviceParams->GetEnum("ChunkSelector");
    assert(chunkSelector != nullptr);
    res = chunkSelector->FromString("FeaturePoints");
    assert(res.IsOK());

    /*PvGenBoolean* */chunkEnable = deviceParams->GetBoolean("ChunkEnable");
    assert(chunkEnable != nullptr);
    res = chunkEnable->SetValue(true);
    assert(res.IsOK());

    /*PvGenEnum* */chunkSelector = deviceParams->GetEnum("ChunkSelector");
    assert(chunkSelector != nullptr);
    res = chunkSelector->FromString("FeatureDescriptors");
    assert(res.IsOK());

    /*PvGenBoolean* */chunkEnable = deviceParams->GetBoolean("ChunkEnable");
    assert(chunkEnable != nullptr);
    res = chunkEnable->SetValue(true);
    assert(res.IsOK());

    PvGenInteger *KPMaxNumber = deviceParams->GetInteger("KPMaxNumber");
    assert(KPMaxNumber != nullptr);
    res = KPMaxNumber->SetValue(5000);
    //res = KPMaxNumber->SetValue(10000);
    assert(res.IsOK());

    PvGenInteger* KPThreshold = deviceParams->GetInteger("KPThreshold");
    assert(KPThreshold != nullptr);
    res = KPThreshold->SetValue(3);
    assert(res.IsOK());

    PvGenEnum* AKAZELength = deviceParams->GetEnum("AKAZELength");
    assert(AKAZELength != nullptr);
    res = AKAZELength->SetValue(2);
    assert(res.IsOK());

    PvGenEnum* AKAZEWindow = deviceParams->GetEnum("AKAZEWindow");
    assert(AKAZEWindow != nullptr);
    res = AKAZEWindow->SetValue(4);
    assert(res.IsOK());
  }
}

PvPipeline *CreatePipeline( PvDevice *aDevice, PvStream *aStream )
{
    // Create the PvPipeline object
    PvPipeline* lPipeline = new PvPipeline( aStream );

    if ( lPipeline != NULL )
    {        
        // Reading payload size from device
        uint32_t lSize = aDevice->GetPayloadSize();
    
        // Set the Buffer count and the Buffer size
        lPipeline->SetBufferCount( BUFFER_COUNT );
        lPipeline->SetBufferSize( lSize );
    }
    
    return lPipeline;
}

void AcquireImages( PvDevice *aDevice, PvStream *aStream, PvPipeline *aPipeline )
{
  // Get device parameters need to control streaming
  PvGenParameterArray *lDeviceParams = aDevice->GetParameters();

  // Map the GenICam AcquisitionStart and AcquisitionStop commands
  PvGenCommand *lStart = dynamic_cast<PvGenCommand *>( lDeviceParams->Get( "AcquisitionStart" ) );
  PvGenCommand *lStop = dynamic_cast<PvGenCommand *>( lDeviceParams->Get( "AcquisitionStop" ) );

  // Note: the pipeline must be initialized before we start acquisition
  cout << "Starting pipeline" << endl;
  aPipeline->Start();

  // Get stream parameters
  PvGenParameterArray *lStreamParams = aStream->GetParameters();

  // Map a few GenICam stream stats counters
  PvGenFloat *lFrameRate = dynamic_cast<PvGenFloat *>( lStreamParams->Get( "AcquisitionRate" ) );
  PvGenFloat *lBandwidth = dynamic_cast<PvGenFloat *>( lStreamParams->Get( "Bandwidth" ) );

  // Enable streaming and send the AcquisitionStart command
  cout << "Enabling streaming and sending AcquisitionStart command." << endl;
  aDevice->StreamEnable();
  lStart->Execute();

  char lDoodle[] = "|\\-|-/";
  int lDoodleIndex = 0;
  double lFrameRateVal = 0.0;
  double lBandwidthVal = 0.0;

  // Acquire images until the user instructs us to stop.
  cout << endl << "<press a key to stop streaming>" << endl;
  while ( !PvKbHit() )
  {
    PvBuffer *lBuffer = nullptr;
    PvResult lOperationResult;

    // Retrieve next buffer
    PvResult lResult = aStream->RetrieveBuffer( &lBuffer, &lOperationResult, 1000 );
    if ( lResult.IsOK() )
    {
      if ( lOperationResult.IsOK() )
      {
        //
        // We now have a valid buffer. This is where you would typically process the buffer.
        // -----------------------------------------------------------------------------------------
        // ...

        lFrameRate->GetValue( lFrameRateVal );
        lBandwidth->GetValue( lBandwidthVal );

        //cout << fixed << setprecision( 1 );
        //cout << lDoodle[ lDoodleIndex ];
        //cout << " BlockID: " << uppercase << hex << setfill( '0' ) << setw( 16 ) << lBuffer->GetBlockID();

        switch ( lBuffer->GetPayloadType() )
        {
          case PvPayloadTypeImage:
            //cout << "  W: " << dec << lBuffer->GetImage()->GetWidth() << " H: " << lBuffer->GetImage()->GetHeight();
            Saveimg(lBuffer);
            break;

          case PvPayloadTypeChunkData:
            cout << " Chunk Data payload type" << " with " << lBuffer->GetChunkCount() << " chunks";
            break;

          case PvPayloadTypeRawData:
            cout << " Raw Data with " << lBuffer->GetRawData()->GetPayloadLength() << " bytes";
            break;

          case PvPayloadTypeMultiPart:
            cout << " Multi Part with " << lBuffer->GetMultiPartContainer()->GetPartCount() << " parts";
            // Preserve
            SavePair(lBuffer);
            break;

          default:
            cout << " Payload type not supported by this sample";
            break;
        }
        cout << "  " << lFrameRateVal << " FPS  " << ( lBandwidthVal / 1000000.0 ) << " Mb/s   \r";
      }
      else
      {
        // Non OK operational result
        //cout << lDoodle[ lDoodleIndex ] << " " << lOperationResult.GetCodeString().GetAscii() << "\r";
      }

      // Release the buffer back to the pipeline
      aPipeline->ReleaseBuffer( lBuffer );
    }
    else
    {
      // Retrieve buffer failure
      //cout << lDoodle[ lDoodleIndex ] << " " << lResult.GetCodeString().GetAscii() << "\r";
    }

    ++lDoodleIndex %= 6;
  }

  PvGetChar(); // Flush key buffer for next stop.
  cout << endl << endl;

  // Tell the device to stop sending images.
  cout << "Sending AcquisitionStop command to the device" << endl;
  lStop->Execute();

  // Disable streaming on the device
  cout << "Disable streaming on the controller." << endl;
  aDevice->StreamDisable();

  // Stop the pipeline
  cout << "Stop pipeline" << endl;
  aPipeline->Stop();
}

int main(int argc, char **argv)
{  
    if(argc < 5)
    {
        cerr << endl << "Usage: ./mono_bottlenose: path_to_vocabulary path_to_settings path_to_sequence_folder_1 path_to_times_file_1 (path_to_image_folder_2 path_to_times_file_2 ... path_to_image_folder_N path_to_times_file_N) (trajectory_file_name)" << endl;
        return 1;
    }
/*********************************************************************************/
PvDevice *lDevice = nullptr;
  PvStream *lStream = nullptr;

  gDirname = "./";

  //vocab_file.assign(argv[1]);
  //yaml_file.assign(argv[2]);

  PV_SAMPLE_INIT();

  cout << "Acquisition Sample:" << endl << endl;

  PvString lConnectionID;
  if ( PvSelectDevice( &lConnectionID ) )
  {
    lDevice = ConnectToDevice( lConnectionID );
    if ( nullptr != lDevice )
    {
      PvGenInteger *intval = dynamic_cast<PvGenInteger *>( lDevice->GetParameters()->Get("GevSCPSPacketSize"));
      assert(intval != nullptr);
      int64_t val;
      intval->GetValue(val);
      if(val < 8000) {
        cerr << "Warning: Configure your NICs MTU to be at least 8K to have reliable image transfer -> overriding to 8K" << endl;
        PvResult res = intval->SetValue(8000);
        assert(res.IsOK());
      }
      lStream = OpenStream( lConnectionID );
      if ( nullptr != lStream )
      {
        PvPipeline *lPipeline = NULL;
        ConfigureStream( lDevice, lStream );
        lPipeline = CreatePipeline( lDevice, lStream );
        if( lPipeline )
        {
          AcquireImages( lDevice, lStream, lPipeline );
          delete lPipeline;
        }
        
        // Close the stream
        cout << "Closing stream" << endl;
        lStream->Close();
        PvStream::Free( lStream );
      }

      // Disconnect the device
      cout << "Disconnecting device" << endl;
      lDevice->Disconnect();
      PvDevice::Free( lDevice );
    }
  }
  
  // Stop all threads
  SLAM.Shutdown();

  // Save camera trajectory    
  SLAM.SaveTrajectoryEuRoC("CameraTrajectory.txt");
  SLAM.SaveKeyFrameTrajectoryEuRoC("KeyFrameTrajectory.txt");

  /*********************************************************************************/
    cout << endl;
    cout << "<press a key to exit>" << endl;
    PvWaitForKeyPress();

    PV_SAMPLE_TERMINATE();
  
    cout << "proccIm:" << proccIm << endl;
    vTimesTrack.resize(proccIm);

    std::ofstream file("ttrack.csv");
    for (const auto& value : vTimesTrack) {
        file << value << std::endl;
    }

    sort(vTimesTrack.begin(),vTimesTrack.end());
    float totaltime = 0;
    for(int ni=0; ni<proccIm; ni++)
    {
        totaltime+=vTimesTrack[ni];
        //cout << "ni:" << ni << " vTimesTrack[ni]:" << vTimesTrack[ni] << endl;
    }
    //cout << "-------" << endl;
    //cout << "-------" << endl;
    float mean, median;
    mean = totaltime/proccIm;
    median = vTimesTrack[proccIm/2];
    cout << "median tracking time: " << median << endl;
    cout << "mean tracking time: " << mean << endl;
    //cout <<"proccIm: "<< proccIm << endl;
    //cout <<"nImages[0]: "<< nImages[0] << endl;
    //while(1);
    file << "median tracking time: " << median << std::endl;
    file << "mean tracking time: " << mean << std::endl;
    file.close();

    // double average, deviation;
    // average = calcAverage(ttrackvector);
    // deviation = calcDeviation(ttrackvector, average);
    // std::cout << "ttrack: " << average << "$\\pm$" << deviation << std::endl;

    return 0;
}

/*
double calcAverage(vector<double> v_times)
{
    double accum = 0;
    for(double value : v_times)
    {
        accum += value;
    }

    return accum / v_times.size();
}

double calcDeviation(vector<double> v_times, double average)
{
    double accum = 0;
    for(double value : v_times)
    {
        accum += pow(value - average, 2);
    }
    return sqrt(accum / v_times.size());
}
*/
