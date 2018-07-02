#include <librealsense2/rs.hpp>
#include "example.hpp"          // Include short list of convenience functions for rendering
#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <librealsense2/rsutil.h>
#include <iostream>
#include <chrono>
#include <ctime>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

//===================================================================
// Global data
//===================================================================

#define NOISY       2.5		// Remove points past NOISY meters

//===================================================================
// Function declarations
//===================================================================

// Looks at RS context info
int printRSContextInfo( rs2::context *rs2Context );

// Configures RS streams
int configureRSStreams(rs2::config *rs2Camera,rs2::pipeline pipe,rs2::pipeline_profile *selection);

// Generats the current frames point cloud data
int generatePointCloud( rs2::pipeline pipe,rs2::context *rs2Camera,rs2::pipeline_profile *selection, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud );


int kbhit(void)
{
  struct termios oldt, newt;
  int ch;
  int oldf;
 
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON | ECHO);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);
  oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
  fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);
 
  ch = getchar();
 
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
  fcntl(STDIN_FILENO, F_SETFL, oldf);
 
  if(ch != EOF)
  {
    ungetc(ch, stdin);
    return 1;
  }
 
  return 0;
}




//===================================================================
// Main
//===================================================================
int main( int argc, char** argv ) try
{

    int err = -1;
    // ==== Cloud Setup ====
    pcl::PointCloud<pcl::PointXYZ>::Ptr rs2CloudPtr( new pcl::PointCloud<pcl::PointXYZ> );

    // Create the RS context and display info about it
    rs2::context rs2Camera;
    printRSContextInfo( &rs2Camera );

    // Create the RS camera and configure streaming and start the streams
    rs2::pipeline pipe;
    rs2::config cfg;
    rs2::pipeline_profile selection;

//    configureRSStreams( &cfg,pipe,&selection );

	
    cfg.enable_stream( RS2_STREAM_DEPTH, 640,480, RS2_FORMAT_ANY, 60);
    cfg.enable_stream( RS2_STREAM_COLOR, 640,480, RS2_FORMAT_BGR8, 60);
    cfg.enable_stream( RS2_STREAM_INFRARED, 1);
    try { cfg.enable_stream(RS2_STREAM_INFRARED, 2); } catch(...) {}
    selection = pipe.start(cfg);


    // ==== Data Grab ====
//    err = generatePointCloud(pipe, &rs2Camera,&selection, rs2CloudPtr );


    pcl::PCDWriter writer;
    int sample_number=100;
    char img_name[80];
    char pcd_name[80];
    char input;

    int i=1;
    do {
    
       rs2::frameset frames = pipe.wait_for_frames();
       rs2::frame frame = frames.first(RS2_STREAM_DEPTH);
       rs2::frame frame_color=frames.first(RS2_STREAM_COLOR);

	const uint16_t * depth_image ;
      // Retrieve our images
       if (frame)
         depth_image    = ( const uint16_t * ) frame.get_data();
        // Retrieve camera parameters for mapping between depth and color
       
	auto depth_stream = selection.get_stream(RS2_STREAM_DEPTH)
                             .as<rs2::video_stream_profile>();
	auto color_stream = selection.get_stream(RS2_STREAM_COLOR)
                             .as<rs2::video_stream_profile>();

	auto resolution = std::make_pair(depth_stream.width(),depth_stream.height());

	auto depth_intrin = depth_stream.get_intrinsics();
	rs2_extrinsics depth_to_color = depth_stream.get_extrinsics_to(color_stream);
	auto color_intrin = color_stream.get_intrinsics();
	auto sensor = selection.get_device().first<rs2::depth_sensor>();
	auto scale =  sensor.get_depth_scale();

	//Disable IR emitter
	sensor.set_option(RS2_OPTION_EMITTER_ENABLED, 0.f);	

	//Possibly turn off laser if present. Verify presence
	if (sensor.supports(RS2_OPTION_LASER_POWER))
 	 {
	  auto range = sensor.get_option_range(RS2_OPTION_LASER_POWER);
//	  sensor.set_option(RS2_OPTION_LASER_POWER, range.max); //Uncomment to Enable laser at max power and comment next line.
	  sensor.set_option(RS2_OPTION_LASER_POWER, 0.f); // Disable laser

         }//

        // Create color image
        cv::Mat rgb(color_intrin.height, color_intrin.width, CV_8UC3, (uchar *)frame_color.get_data() );
 //       cv::cvtColor(rgb, rgb, cv::COLOR_BGR2RGB );


        {
                sprintf(img_name, "img%d.jpg", sample_number);

		if(i==7)
                 cv::imwrite(img_name, rgb);

                // Depth dimension helpers
                int dw  = 0;
                int dh  = 0;
                int dwh = 0;

                dw = depth_intrin.width;
                dh = depth_intrin.height;

                dwh = dw * dh;

                // Set the cloud up to be used
                rs2CloudPtr->clear( );
                rs2CloudPtr->is_dense = false;
                rs2CloudPtr->width = dw;
                rs2CloudPtr->height = dh;
                rs2CloudPtr->resize( dwh );

                static const float nan = std::numeric_limits<float>::quiet_NaN( );

                // Initialize all points with nans
                for( int dy = 0; dy < dh; dy++ )
                {
                    for( int dx = 0; dx < dw; dx++ )
                    {
                        rs2CloudPtr->at(dx, dy).x = (float) nan;
                        rs2CloudPtr->at(dx, dy).y = (float) nan;
                        rs2CloudPtr->at(dx, dy).z = (float) nan;
                    }
                }

                // Iterate the data space
                for( int dy = 0; dy < dh; dy++ )
                {
                    for( int dx = 0; dx < dw; dx++ )
                    {
                        uint i = dy * dw + dx;
                        uint16_t depth_value = depth_image[i];
                        bool color_fail = false;

                        // XYZ input access to cloud
                        float *dp_x;
                        float *dp_y;
                        float *dp_z;
                        float depth_in_meters = depth_value * scale;
			

                        // Map from pixel coordinates in the depth image to pixel coordinates in the color image
                        float depth_pixel[2] = {(float)dx, (float)dy};

			float depth_point[3],color_point[3],color_pixel[2];

			rs2_deproject_pixel_to_point(depth_point,&depth_intrin,depth_pixel,depth_in_meters);

			rs2_transform_point_to_point(color_point,&depth_to_color,depth_point);

			rs2_project_point_to_pixel(color_pixel, &color_intrin ,color_point);



                        // Use the color from the nearest color pixel, or pure white if this point falls outside the color image
                        const int cx = (int)std::round(color_pixel[0]), cy = (int)std::round(color_pixel[1]);
                        if(cx < 0 || cy < 0 || cx >= color_intrin.width || cy >= color_intrin.height) {
                            continue;
                        } 

                        dp_x = &( rs2CloudPtr->at(cx, cy).x );
                        dp_y = &( rs2CloudPtr->at(cx, cy).y );
                        dp_z = &( rs2CloudPtr->at(cx, cy).z );

                        if( depth_value == 0 ) {
                            *dp_x = *dp_y = *dp_z = (float) nan;
                            continue;
                        }

                        if( depth_point[2] > NOISY ) {
                            *dp_x = *dp_y = *dp_z = (float) nan;
                            continue;
                        }

                        // If valid point, add data to cloud
                        // Fill in cloud depth
                        *dp_x = depth_point[0];
                        *dp_y = depth_point[1];
                        *dp_z = depth_point[2];
                    }
                }
		
	      if(i==7)
	       {		
                //cout << "Saving point cloud..." << std::endl;
                sprintf(pcd_name, "points%d.pcd", sample_number);
                writer.write<pcl::PointXYZ>(pcd_name, *rs2CloudPtr,false);
	       }
                i++;
        }

    }while(i<=7);
//i=7 because the first few frames are used to get the right exposure, and hence could either be over or underexposed. 

    if( err != EXIT_SUCCESS )
    {
        std::cout << "Error in getFrame( )\n" << std::endl;
        return err;
    }


    return EXIT_SUCCESS;
}

catch( const rs2::error & e )
{
    std::cerr << "RealSense error calling " << e.get_failed_function( ) << "(" << e.get_failed_args( ) << "):\n    " << e.what( ) << std::endl;
    return EXIT_FAILURE;
}
catch( const std::exception & e )
{
    std::cerr << e.what( ) << std::endl;
    return EXIT_FAILURE;

}

//===================================================================
// provides context info and will exit if no devices
//===================================================================
int printRSContextInfo( rs2::context *c )
{		
//    printf( "There are %d connected RealSense devices.\n", c->query_devices().size() );
    if( c->query_devices().size() == 0 )
        throw std::runtime_error( "No device detected. Is it plugged in?" );

    return EXIT_SUCCESS;
}

 	







