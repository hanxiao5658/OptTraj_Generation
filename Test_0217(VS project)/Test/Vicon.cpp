#include "Vicon.h"
using namespace ViconDataStreamSDK::CPP;

#define output_stream  std::cout 

namespace
{
  std::string Adapt( const bool i_Value )
  {
    return i_Value ? "True" : "False";
  }

  std::string Adapt( const Direction::Enum i_Direction )
  {
    switch( i_Direction )
    {
      case Direction::Forward:
        return "Forward";
      case Direction::Backward:
        return "Backward";
      case Direction::Left:
        return "Left";
      case Direction::Right:
        return "Right";
      case Direction::Up:
        return "Up";
      case Direction::Down:
        return "Down";
      default:
        return "Unknown";
    }
  }

  std::string Adapt( const DeviceType::Enum i_DeviceType )
  {
    switch( i_DeviceType )
    {
      case DeviceType::ForcePlate:
        return "ForcePlate";
      case DeviceType::Unknown:
      default:
        return "Unknown";
    }
  }

  std::string Adapt( const Unit::Enum i_Unit )
  {
    switch( i_Unit )
    {
      case Unit::Meter:
        return "Meter";
      case Unit::Volt:
        return "Volt";
      case Unit::NewtonMeter:
        return "NewtonMeter";
      case Unit::Newton:
        return "Newton";
      case Unit::Kilogram:
        return "Kilogram";
      case Unit::Second:
        return "Second";
      case Unit::Ampere:
        return "Ampere";
      case Unit::Kelvin:
        return "Kelvin";
      case Unit::Mole:
        return "Mole";
      case Unit::Candela:
        return "Candela";
      case Unit::Radian:
        return "Radian";
      case Unit::Steradian:
        return "Steradian";
      case Unit::MeterSquared:
        return "MeterSquared";
      case Unit::MeterCubed:
        return "MeterCubed";
      case Unit::MeterPerSecond:
        return "MeterPerSecond";
      case Unit::MeterPerSecondSquared:
        return "MeterPerSecondSquared";
      case Unit::RadianPerSecond:
        return "RadianPerSecond";
      case Unit::RadianPerSecondSquared:
        return "RadianPerSecondSquared";
      case Unit::Hertz:
        return "Hertz";
      case Unit::Joule:
        return "Joule";
      case Unit::Watt:
        return "Watt";
      case Unit::Pascal:
        return "Pascal";
      case Unit::Lumen:
        return "Lumen";
      case Unit::Lux:
        return "Lux";
      case Unit::Coulomb:
        return "Coulomb";
      case Unit::Ohm:
        return "Ohm";
      case Unit::Farad:
        return "Farad";
      case Unit::Weber:
        return "Weber";
      case Unit::Tesla:
        return "Tesla";
      case Unit::Henry:
        return "Henry";
      case Unit::Siemens:
        return "Siemens";
      case Unit::Becquerel:
        return "Becquerel";
      case Unit::Gray:
        return "Gray";
      case Unit::Sievert:
        return "Sievert";
      case Unit::Katal:
        return "Katal";

      case Unit::Unknown:
      default:
        return "Unknown";
    }
  }
#ifdef WIN32
  bool Hit()
  {
    bool hit = false;
    while( _kbhit() )
    {
      getchar();
      hit = true;
    }
    return hit;
  }
#endif
}
Client MyClient;
Vicon::Vicon(char *ip, unsigned short port)   //initialisation, connecting to the vicon host
{
	std::string HostName = "192.168.10.1";//:801

	std::string LogFile = "";
	std::string MulticastAddress = "244.0.0.0:44801";
	bool ConnectToMultiCast = false;
	bool EnableMultiCast = false;
 
	 // Make a new client
 
	 std::cout << "Connecting to " << HostName << " ..." << std::flush;
    while( !MyClient.IsConnected().Connected )
    {
		// Direct connection

		bool ok = false;
     
		ok =( MyClient.Connect( HostName ).Result == Result::Success );
     
		if(!ok)
		{
		std::cout << "Warning - connect failed..." << std::endl;
		}

		std::cout << ".";
		#ifdef WIN32
		Sleep( 200 );
		#else
		sleep(1);
		#endif
    }
	 // Enable some different data types
    MyClient.EnableSegmentData();

    std::cout << "Segment Data Enabled: "          << Adapt( MyClient.IsSegmentDataEnabled().Enabled )         << std::endl;
	
	//Enable the unlabled maker collection
	MyClient.EnableUnlabeledMarkerData();

    // Set the streaming mode
    MyClient.SetStreamMode( ViconDataStreamSDK::CPP::StreamMode::ServerPush );

    // Set the global up axis
    MyClient.SetAxisMapping( Direction::Forward, 
                             Direction::Left, 
                             Direction::Up ); // Z-up

    Output_GetAxisMapping _Output_GetAxisMapping = MyClient.GetAxisMapping();
    /*std::cout << "Axis Mapping: X-" << Adapt( _Output_GetAxisMapping.XAxis ) 
                           << " Y-" << Adapt( _Output_GetAxisMapping.YAxis ) 
                           << " Z-" << Adapt( _Output_GetAxisMapping.ZAxis ) << std::endl;*/

    // Discover the version number
    Output_GetVersion _Output_GetVersion = MyClient.GetVersion();
    //std::cout << "Version: " << _Output_GetVersion.Major << "." 
    //                         << _Output_GetVersion.Minor << "." 
    //                         << _Output_GetVersion.Point << std::endl;


    size_t FrameRateWindow = 1000; // frames
    size_t Counter = 0;
    clock_t LastTime = clock();

}

void Vicon::GetData(float &x, float &y, float &z, float &aX, float &aY, float &aZ , float &px, float &py, float &pz, float &psticky, float &pstickz)
{
	//output_stream << "Waiting for new frame...";
	while( MyClient.GetFrame().Result != Result::Success )
	{
		// Sleep a little so that we don't lumber the CPU with a busy poll
		#ifdef WIN32
			Sleep( 8 );
		#else
			sleep(1);
		#endif

		//  output_stream << ".";
	}
	Output_GetFrameRate Rate = MyClient.GetFrameRate();
	//frame = Rate.FrameRateHz;
	// Count the number of subjects
	std::string ObjectName = "HummingBird02";// 
	std::string ObjectName_P = "Payload";// 
	std::string ObjectName_Stick = "stick";// 

	// Get the global segment translation
	Output_GetSegmentGlobalTranslation _Output_GetSegmentGlobalTranslation = MyClient.GetSegmentGlobalTranslation( ObjectName, ObjectName );
	x = _Output_GetSegmentGlobalTranslation.Translation[0];//unity: mm
	y = _Output_GetSegmentGlobalTranslation.Translation[1];
	z = _Output_GetSegmentGlobalTranslation.Translation[2];

	// Get the global segment rotation in EulerXYZ co-ordinates
	Output_GetSegmentGlobalRotationEulerXYZ _Output_GetSegmentGlobalRotationEulerXYZ = MyClient.GetSegmentGlobalRotationEulerXYZ( ObjectName, ObjectName );
	aX = _Output_GetSegmentGlobalRotationEulerXYZ.Rotation[ 0 ];
	aY = _Output_GetSegmentGlobalRotationEulerXYZ.Rotation[ 1 ];
	aZ = _Output_GetSegmentGlobalRotationEulerXYZ.Rotation[ 2 ];
    
	// Get the global segment translation
	Output_GetSegmentGlobalTranslation _Output_GetSegmentGlobalTranslation_P = MyClient.GetSegmentGlobalTranslation(ObjectName_P, ObjectName_P);
	px = _Output_GetSegmentGlobalTranslation_P.Translation[0];//unity: mm
	py = _Output_GetSegmentGlobalTranslation_P.Translation[1];
	pz = _Output_GetSegmentGlobalTranslation_P.Translation[2];

	// Get the global segment translation
	Output_GetSegmentGlobalTranslation _Output_GetSegmentGlobalTranslation_Stick = MyClient.GetSegmentGlobalTranslation(ObjectName_Stick, ObjectName_Stick);
	
	psticky = _Output_GetSegmentGlobalTranslation_Stick.Translation[1];
	pstickz = _Output_GetSegmentGlobalTranslation_Stick.Translation[2];
	

	// Get the unlabeled markers
	//unsigned int UnlabeledMarkerCount = MyClient.GetUnlabeledMarkerCount().MarkerCount;
	////output_stream << "    Unlabeled Markers (" << UnlabeledMarkerCount << "):" << std::endl;
	//unsigned int UnlabeledMarkerIndex = 0 ; 
	//// Get the global marker translation
	//Output_GetUnlabeledMarkerGlobalTranslation _Output_GetUnlabeledMarkerGlobalTranslation = MyClient.GetUnlabeledMarkerGlobalTranslation( UnlabeledMarkerIndex );	
	//px = _Output_GetUnlabeledMarkerGlobalTranslation.Translation[ 0 ] ; 
	//py = _Output_GetUnlabeledMarkerGlobalTranslation.Translation[ 1 ] ;
	//pz = _Output_GetUnlabeledMarkerGlobalTranslation.Translation[ 2 ] ;

}
 void Vicon::CloseVicon()
 {
    MyClient.DisableSegmentData();
    MyClient.DisableMarkerData();
    MyClient.DisableUnlabeledMarkerData();
    MyClient.DisableDeviceData();

    // Disconnect and dispose
    int t = clock();
    std::cout << " Disconnecting..." << std::endl;
    MyClient.Disconnect();
    int dt = clock() - t;
    double secs = (double) (dt)/(double)CLOCKS_PER_SEC;
    std::cout << " Disconnect time = " << secs << " secs" << std::endl;
 
 }
