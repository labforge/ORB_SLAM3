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
#include<iomanip>
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

using namespace std;
using namespace cv;
namespace fs = std::filesystem;

typedef std::list<PvBuffer *> BufferList;

PV_INIT_SIGNAL_HANDLER();

#define BUFFER_COUNT ( 16 )

///
/// Function Prototypes
///
PvDevice *ConnectToDevice( const PvString &aConnectionID );
PvStream *OpenStream( const PvString &aConnectionID );
void ConfigureStream( PvDevice *aDevice, PvStream *aStream );
void CreateStreamBuffers( PvDevice *aDevice, PvStream *aStream, BufferList *aBufferList );
void AcquireImages( PvDevice *aDevice, PvStream *aStream );
void FreeStreamBuffers( BufferList *aBufferList );

/// Global variables
//int gExposure;
double gExposure;
double gInterval;
int gGain;
char*gDirname;

string vocab_file("/home/emslab/ORB_SLAM3/Vocabulary/ORBvoc.txt");
string yaml_file("/home/emslab/ORB_SLAM3/Examples/Stereo/bottlenose5.yaml");

ORB_SLAM3::System SLAM(vocab_file,yaml_file,ORB_SLAM3::System::STEREO, true, 0, string("Testrun"));

float gWbRed, gWbGreen, gWbBlue;


void SavePair( PvBuffer *aBuffer ) 
{
  std::timespec ts;
  static int count = 0;
  assert(aBuffer->GetMultiPartContainer()->GetPartCount() == 2);
  IPvImage *img0 = aBuffer->GetMultiPartContainer()->GetPart(0)->GetImage();
  IPvImage *img1 = aBuffer->GetMultiPartContainer()->GetPart(1)->GetImage();
  // Note images are YUVY
  Mat leftIn(img0->GetHeight(), img0->GetWidth(), CV_8UC2, img0->GetDataPointer());
  Mat rightIn(img1->GetHeight(), img1->GetWidth(), CV_8UC2, img1->GetDataPointer());
  // Convert to BGR (standard OpenCV format)

  Mat left, right;
  cv::Mat imLeft, imRight;
  cvtColor(leftIn, imLeft, COLOR_YUV2BGR_YUYV);
  cvtColor(rightIn, imRight, COLOR_YUV2BGR_YUYV);

  // Save to disk
  stringstream s;
  s << count++;
  string fname_left = string(gDirname) + "/left_" + s.str() + ".png";
  string fname_right = string(gDirname) + "/right_" + s.str() + ".png";
  //imwrite(fname_left, left);
  //imwrite(fname_right, right);

  imwrite(fname_left, imLeft);
  imwrite(fname_right, imRight);

    float imageScale = SLAM.GetImageScale();

   

    double t_resize = 0;
    double t_rect = 0;
    double t_track = 0;
    int num_rect = 0;
    int proccIm = 0;

    if(imageScale != 1.f)
    {
#ifdef REGISTER_TIMES
    #ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t_Start_Resize = std::chrono::steady_clock::now();
    #else
        std::chrono::monotonic_clock::time_point t_Start_Resize = std::chrono::monotonic_clock::now();
    #endif
#endif
        int width = imLeft.cols * imageScale;
        int height = imLeft.rows * imageScale;
        cv::resize(imLeft, imLeft, cv::Size(width, height));
        cv::resize(imRight, imRight, cv::Size(width, height));
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

        //double tframe = vTimestampsCam[seq][ni];
        double tframe;
  std::timespec_get(&ts, TIME_UTC);
  tframe = ts.tv_sec*1000000000+ ts.tv_nsec;

    #ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    #else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
    #endif

            // Pass the images to the SLAM system
        SLAM.TrackStereo(imLeft,imRight,tframe);

    #ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    #else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
    #endif

#ifdef REGISTER_TIMES
            t_track = t_resize + t_rect + std::chrono::duration_cast<std::chrono::duration<double,std::milli> >(t2 - t1).count();
            SLAM.InsertTrackTime(t_track);
#endif

        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
     
       // sleep till the next frame
  std::cout << "ttrack: " << ttrack << std::endl;

  if(ttrack<0.2) {
      std::cout << "usleep: " << (0.2-ttrack) << std::endl;
      usleep((0.2-ttrack)*1e6); // 1e6
  }
}

void Saveimg( PvBuffer *aBuffer ) {
  static int count = 0;
  IPvImage *img0 = aBuffer->GetImage();
  
  // Note images are YUVY
  Mat leftIn(img0->GetHeight(), img0->GetWidth(), CV_8UC2, img0->GetDataPointer());
  
  // Convert to BGR (standard OpenCV format)
  Mat left, right;
  cvtColor(leftIn, left, COLOR_YUV2BGR_YUYV);

  // Save to disk
  stringstream s;
  s << count++;
  string fname_left = string(gDirname) + "/left_" + s.str() + ".png";
  
  imwrite(fname_left, left);
  //imwrite(fname_left, leftIn);
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

    // Force multi-part transmission, this enables stereo image transfer on Bottlenose
    PvGenBoolean *lMultipart = dynamic_cast<PvGenBoolean *>( lDeviceGEV->GetParameters()->Get( "GevSCCFGMultiPartEnabled" ) );
    assert(lMultipart != nullptr);
    res = lMultipart->SetValue(true);
    assert(res.IsOK());

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
    res = lInterval->SetValue(200);
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

  }
}

void CreateStreamBuffers( PvDevice *aDevice, PvStream *aStream, BufferList *aBufferList )
{
  // Reading payload size from device
  uint32_t lSize = aDevice->GetPayloadSize();

  // Use BUFFER_COUNT or the maximum number of buffers, whichever is smaller
  uint32_t lBufferCount = ( aStream->GetQueuedBufferMaximum() < BUFFER_COUNT ) ?
                          aStream->GetQueuedBufferMaximum() :
                          BUFFER_COUNT;

  // Allocate buffers
  for ( uint32_t i = 0; i < lBufferCount; i++ )
  {
    // Create new buffer object
    PvBuffer *lBuffer = new PvBuffer;

    // Have the new buffer object allocate payload memory
    lBuffer->Alloc( static_cast<uint32_t>( lSize ) );

    // Add to external list - used to eventually release the buffers
    aBufferList->push_back( lBuffer );
  }

  // Queue all buffers in the stream
  BufferList::iterator lIt = aBufferList->begin();
  while ( lIt != aBufferList->end() )
  {
    aStream->QueueBuffer( *lIt );
    lIt++;
  }
}

void AcquireImages( PvDevice *aDevice, PvStream *aStream )
{
  // Get device parameters need to control streaming
  PvGenParameterArray *lDeviceParams = aDevice->GetParameters();

  // Map the GenICam AcquisitionStart and AcquisitionStop commands
  PvGenCommand *lStart = dynamic_cast<PvGenCommand *>( lDeviceParams->Get( "AcquisitionStart" ) );
  PvGenCommand *lStop = dynamic_cast<PvGenCommand *>( lDeviceParams->Get( "AcquisitionStop" ) );

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

        cout << fixed << setprecision( 1 );
        cout << lDoodle[ lDoodleIndex ];
        cout << " BlockID: " << uppercase << hex << setfill( '0' ) << setw( 16 ) << lBuffer->GetBlockID();

        switch ( lBuffer->GetPayloadType() )
        {
          case PvPayloadTypeImage:
            cout << "  W: " << dec << lBuffer->GetImage()->GetWidth() << " H: " << lBuffer->GetImage()->GetHeight();
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
        cout << lDoodle[ lDoodleIndex ] << " " << lOperationResult.GetCodeString().GetAscii() << "\r";
      }

      // Re-queue the buffer in the stream object
      aStream->QueueBuffer( lBuffer );
    }
    else
    {
      // Retrieve buffer failure
      cout << lDoodle[ lDoodleIndex ] << " " << lResult.GetCodeString().GetAscii() << "\r";
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

  // Abort all buffers from the stream and dequeue
  cout << "Aborting buffers still in stream" << endl;
  aStream->AbortQueuedBuffers();
  while ( aStream->GetQueuedBufferCount() > 0 )
  {
    PvBuffer *lBuffer = nullptr;
    PvResult lOperationResult;

    aStream->RetrieveBuffer( &lBuffer, &lOperationResult );
  }
}

void FreeStreamBuffers( BufferList *aBufferList )
{
  // Go through the buffer list
  BufferList::iterator lIt = aBufferList->begin();
  while ( lIt != aBufferList->end() )
  {
    delete *lIt;
    lIt++;
  }

  // Clear the buffer list
  aBufferList->clear();
}


int main(int argc, char **argv)
{  
    cerr << endl << "Usage: ./stereo_bottlenose path_to_vocabulary path_to_settings path_to_sequence_folder_1 path_to_times_file_1 (path_to_image_folder_2 path_to_times_file_2 ... path_to_image_folder_N path_to_times_file_N) (trajectory_file_name)" << endl;

/*********************************************************************************/
  PvDevice *lDevice = nullptr;
  PvStream *lStream = nullptr;
  BufferList lBufferList;

  gDirname = "./";

  vocab_file.assign(argv[1]);
  yaml_file.assign(argv[2]);

  cout << gExposure << " " << gGain << " " << endl;

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
        ConfigureStream( lDevice, lStream );
        CreateStreamBuffers( lDevice, lStream, &lBufferList );
        AcquireImages( lDevice, lStream );
        FreeStreamBuffers( &lBufferList );

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

    return 0;

   
}

