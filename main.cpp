//
// begin license header
//
// This file is part of Pixy CMUcam5 or "Pixy" for short
//
// All Pixy source code is provided under the terms of the
// GNU General Public License v2 (http://www.gnu.org/licenses/gpl-2.0.html).
// Those wishing to use Pixy source code, software and/or
// technologies under different licensing terms should contact us at
// cmucam@cs.cmu.edu. Such licensing terms are available for
// all portions of the Pixy codebase presented here.
//
// end license header
//

#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <unistd.h>
#include "pixy.h"

//ros related includes
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "std_msgs/Int16.h"

//include c header files
#ifdef __cplusplus
extern "C"{
#endif

#include "drone_loc.h"

#ifdef __cplusplus
}
#endif

#define BLOCK_BUFFER_SIZE          4

#define PIXY_X_CENTER              ((PIXY_MAX_X-PIXY_MIN_X)/2)
#define PIXY_Y_CENTER              ((PIXY_MAX_Y-PIXY_MIN_Y)/2)

#define PIXY_RCS_PAN_CHANNEL       0
#define PIXY_RCS_TILT_CHANNEL       1

// PID control parameters //
#define PAN_PROPORTIONAL_GAIN     300
#define PAN_DERIVATIVE_GAIN       90
#define TILT_PROPORTIONAL_GAIN    500
#define TILT_DERIVATIVE_GAIN      400
#define INVERT_IMAGE		  0
#define NONINVERT_IMAGE 1

#define DT 20 //sample interval ms

#define SMOOTH_FILTER_LENGTH 10 // length of the smooth filter

#define SIGNATURE_1 1 //Pixy signature for L, S, R
#define SIGNATURE_2 2 //Pixy signature for M

static image_coordinate_s smoothBuf[SMOOTH_FILTER_LENGTH];
bool flagFilterInitialized(false);

static int filterIndex(0);

static int width = 318;
static int height = 198;
static int distance = 240;
static int distanceOfMo =34;
static int distanceOfLr = 50;
uint16_t blocks_x = 0;
uint16_t blocks_y = 0;
uint16_t blocks_x_ave = 0;
uint16_t blocks_y_ave = 0;

// Pixy Block Buffer //
struct Block  blocks [BLOCK_BUFFER_SIZE];

static bool run_flag = true;



struct Gimbal {
  int32_t position;
  int32_t previous_error;
  int32_t proportional_gain;
  int32_t derivative_gain;
};

// PID control variables //

struct Gimbal pan;
struct Gimbal tilt;

bool enable_pan_tilt(true);

void handle_SIGINT(int unused)
{
  // On CTRL+C - abort! //

  run_flag = false;
}

void initialize_gimbals()
{
  pan.position           = PIXY_RCS_CENTER_POS;
  pan.previous_error     = 0x80000000L;
  pan.proportional_gain  = PAN_PROPORTIONAL_GAIN;
  pan.derivative_gain    = PAN_DERIVATIVE_GAIN;
  tilt.position          = PIXY_RCS_CENTER_POS;
  tilt.previous_error    = 0x80000000L;
  tilt.proportional_gain = TILT_PROPORTIONAL_GAIN;
  tilt.derivative_gain   = TILT_DERIVATIVE_GAIN;
}

void gimbal_update(struct Gimbal *  gimbal, int32_t error)
{
  long int velocity;
  int32_t  error_delta;
  int32_t  P_gain;
  int32_t  D_gain;

  if(gimbal->previous_error != 0x80000000L) {

    error_delta = error - gimbal->previous_error;
    P_gain      = gimbal->proportional_gain;
    D_gain      = gimbal->derivative_gain;

    /* Using the proportional and derivative gain for the gimbal,
       calculate the change to the position.  */
    velocity = (error * P_gain + error_delta * D_gain) >> 10;

    gimbal->position += velocity;

    if (gimbal->position > PIXY_RCS_MAX_POS) {
      gimbal->position = PIXY_RCS_MAX_POS;
    } else if (gimbal->position < PIXY_RCS_MIN_POS) {
      gimbal->position = PIXY_RCS_MIN_POS;
    }
  }

  gimbal->previous_error = error;
}

int main(int argc, char *  argv[])
{
  int     pixy_init_status;
  int     result;
  int     pan_error;
  int     tilt_error;
  int     blocks_copied;

  int image_invert_flag = NONINVERT_IMAGE;
  reference_coordinate_s lastCorr;

  int ch;
  while((ch = getopt(argc,argv,"id:m:l:hD"))!=-1){
    switch(ch){
      case 'i'://invert the image coordinate.
        image_invert_flag = INVERT_IMAGE;
        break;
      case 'd':
        if((distance = atoi(optarg))==0)
          return 0;
	ROS_INFO("lens to image sensor distance is set to:%s",optarg);
          break;
      case 'm':
        if((distanceOfMo = atoi(optarg))==0)
          return 0;
	ROS_INFO("M to O distance is set to:%s",optarg);
        break;
      case 'l':
        if((distanceOfLr = atoi(optarg))==0)
          return 0;
	ROS_INFO("L to R distance is set to:%s",optarg);
        break;
      case 'h':
        printf("usage: rosrun pixy_node pixy_node [-i] [-D] [-d] [-m] [-l] [-h]");
        break;
      case 'D':
        enable_pan_tilt = false;
        break;
      default:
        break;
    }
  }


  ros::init(argc, argv, "Pixy_Node");

  ros::NodeHandle n;

  ros::Publisher pose_pub = n.advertise<geometry_msgs::PoseStamped>("pose",1000);

  ros::Publisher vel_pub = n.advertise<geometry_msgs::Vector3Stamped>("vel",1000);

  ros::Publisher angle_pub = n.advertise<std_msgs::Int16>("angle",1000);

  ros::Rate loop_rate(1000 / DT);

  geometry_msgs::PoseStamped pose;

  geometry_msgs::Vector3Stamped vel;

  std_msgs::Int16 refAngle;

  initialize_gimbals();

  // Catch CTRL+C (SIGINT) signals //
  signal(SIGINT, handle_SIGINT);

  // Connect to Pixy //
  pixy_init_status = pixy_init();

  // Was there an error initializing pixy? //
  if(pixy_init_status != 0)
  {
    // Error initializing Pixy //
    printf("Error: pixy_init() [%d] ", pixy_init_status);
    pixy_error(pixy_init_status);

    return pixy_init_status;
  }

  //set the parameter for the position calculation
  Set_Screen_Len(&width);
  Set_Screen_Wid(&height);
  Set_Camera_Distance(&distance);
  Set_Distance_of_MO(&distanceOfMo);
  Set_Distance_of_LR(&distanceOfLr);

  image_coordinate_s camera_raw_coordinates;
  reference_coordinate_s calculated_position_coordinate;

  while(run_flag && ros::ok()) {

    // Wait for new blocks to be available //
    while(!pixy_blocks_are_new() && run_flag);

    // Get blocks from Pixy //

    blocks_copied = pixy_get_blocks(BLOCK_BUFFER_SIZE, &blocks[0]);
    //ROS_INFO("blocks found:%d",blocks_copied);
    //ROS_INFO("block sum:%d",blocks[0].signature + blocks[1].signature + blocks[2].signature + blocks[3].signature);

    if(blocks_copied < 0) {
      // Error: pixy_get_blocks //
      printf("Error: pixy_get_blocks() [%d] ", blocks_copied);
      pixy_error(blocks_copied);
      fflush(stdout);
    }

    if (blocks_copied>0 && enable_pan_tilt) {
      // Calculate the difference between the   //
      // center of Pixy's focus and the target. //
      blocks_x = 0;
      blocks_y = 0;
      for(int k=0; k<blocks_copied; k++)
      	 {
      		 blocks_x = blocks_x + blocks[k].x;
      		 blocks_y = blocks_y + blocks[k].y;
      	 }
      	 blocks_x_ave = (uint16_t) (blocks_x / blocks_copied);
      	 blocks_y_ave = (uint16_t) (blocks_y / blocks_copied);

        pan_error  = PIXY_X_CENTER - blocks_x_ave;
        tilt_error = blocks_y_ave - PIXY_Y_CENTER;

      // Apply corrections to the pan/tilt with the goal //
      // of putting the target in the center of          //
      // Pixy's focus.                                   //

      gimbal_update(&pan, pan_error);
      gimbal_update(&tilt, tilt_error);

      result = pixy_rcs_set_position(PIXY_RCS_PAN_CHANNEL, (uint16_t) pan.position);
      if (result < 0) {
        printf("Error: pixy_rcs_set_position() [%d] ", result);
        pixy_error(result);
        fflush(stdout);
      }

      result = pixy_rcs_set_position(PIXY_RCS_TILT_CHANNEL, (uint16_t) tilt.position);
      if (result<0) {
        printf("Error: pixy_rcs_set_position() [%d] ", result);
        pixy_error(result);
        fflush(stdout);
      }
    }

    //excute position calculation when found 4 blocks
    int max=0, min=320;
    int max_j=0, min_j=0;
    if(blocks_copied == 4 && (blocks[0].signature + blocks[1].signature + blocks[2].signature + blocks[3].signature)== 3* SIGNATURE_1 + SIGNATURE_2){

      for(int j=0; j<4; j++){
        if(blocks[j].signature == SIGNATURE_1){
          if(blocks[j].x > max){
            max_j = j;
            max = blocks[j].x;
          }
          if(blocks[j].x < min){
            min_j = j;
            min = blocks[j].x;
          }
        }
      }

      if(image_invert_flag == NONINVERT_IMAGE)	//if camera is upright on top
      {
	for(int j=0; j<4; j++){
          if(blocks[j].signature == SIGNATURE_2){
            camera_raw_coordinates.m_coordinate[0] = (int) (blocks[j].x + 0.5 * blocks[j].width);
            camera_raw_coordinates.m_coordinate[1] = (int) (blocks[j].y + 0.5 * blocks[j].height);
          }
          else if(j == min_j){		//then the smallest one is l point
            camera_raw_coordinates.l_coordinate[0] = (int) (blocks[j].x + 0.5 * blocks[j].width);
            camera_raw_coordinates.l_coordinate[1] = (int) (blocks[j].y + 0.5 * blocks[j].height);
          }
          else if(j == max_j){
            camera_raw_coordinates.s_coordinate[0] = (int) (blocks[j].x + 0.5 * blocks[j].width);
            camera_raw_coordinates.s_coordinate[1] = (int) (blocks[j].y + 0.5 * blocks[j].height);
          }
          else{
            camera_raw_coordinates.r_coordinate[0] = (int) (blocks[j].x + 0.5 * blocks[j].width);
            camera_raw_coordinates.r_coordinate[1] = (int) (blocks[j].y + 0.5 * blocks[j].height);
          }

        }
      }
      else	//if the camera is bottom up
      {
        for(int j=0; j<4; j++){
          if(blocks[j].signature == SIGNATURE_2){
            camera_raw_coordinates.m_coordinate[0] = (int) (blocks[j].x + 0.5 * blocks[j].width);
            camera_raw_coordinates.m_coordinate[1] = (int) (blocks[j].y + 0.5 * blocks[j].height);
          }
          else if(j == min_j){		//then the smallest x is the s point
            camera_raw_coordinates.s_coordinate[0] = (int) (blocks[j].x + 0.5 * blocks[j].width);
            camera_raw_coordinates.s_coordinate[1] = (int) (blocks[j].y + 0.5 * blocks[j].height);
          }
          else if(j == max_j){
            camera_raw_coordinates.l_coordinate[0] = (int) (blocks[j].x + 0.5 * blocks[j].width);
            camera_raw_coordinates.l_coordinate[1] = (int) (blocks[j].y + 0.5 * blocks[j].height);
          }
          else{
            camera_raw_coordinates.r_coordinate[0] = (int) (blocks[j].x + 0.5 * blocks[j].width);
            camera_raw_coordinates.r_coordinate[1] = (int) (blocks[j].y + 0.5 * blocks[j].height);
          }

        }
      }

      if(flagFilterInitialized) {
          image_coordinate_s filterdImageCoordinate = {0};//this initializer is only supported in c++ 11
        for(int i = 0; i < SMOOTH_FILTER_LENGTH; i++){
          filterdImageCoordinate.l_coordinate[0] += smoothBuf[i].l_coordinate[0];
          filterdImageCoordinate.l_coordinate[1] += smoothBuf[i].l_coordinate[1];
          filterdImageCoordinate.r_coordinate[0] += smoothBuf[i].r_coordinate[0];
          filterdImageCoordinate.r_coordinate[1] += smoothBuf[i].r_coordinate[1];
          filterdImageCoordinate.m_coordinate[0] += smoothBuf[i].m_coordinate[0];
          filterdImageCoordinate.m_coordinate[1] += smoothBuf[i].m_coordinate[1];
          filterdImageCoordinate.s_coordinate[0] += smoothBuf[i].s_coordinate[0];
          filterdImageCoordinate.s_coordinate[1] += smoothBuf[i].s_coordinate[1];
        }
          filterdImageCoordinate.l_coordinate[0] /= SMOOTH_FILTER_LENGTH;
          filterdImageCoordinate.l_coordinate[1] /= SMOOTH_FILTER_LENGTH;
          filterdImageCoordinate.m_coordinate[0] /= SMOOTH_FILTER_LENGTH;
          filterdImageCoordinate.m_coordinate[1] /= SMOOTH_FILTER_LENGTH;
          filterdImageCoordinate.r_coordinate[0] /= SMOOTH_FILTER_LENGTH;
          filterdImageCoordinate.r_coordinate[1] /= SMOOTH_FILTER_LENGTH;
          filterdImageCoordinate.s_coordinate[0] /= SMOOTH_FILTER_LENGTH;
          filterdImageCoordinate.s_coordinate[1] /= SMOOTH_FILTER_LENGTH;

          smoothBuf[filterIndex] = camera_raw_coordinates;
          filterIndex++;
          filterIndex %= SMOOTH_FILTER_LENGTH;

        Image_To_Reference_Coordinate(filterdImageCoordinate, &calculated_position_coordinate);

        vel.vector.x = (calculated_position_coordinate.x - lastCorr.x) / DT;
        vel.vector.y = (calculated_position_coordinate.y - lastCorr.y) / DT;
        vel.vector.z = (calculated_position_coordinate.z - lastCorr.z) / DT;
        vel.header.stamp = ros::Time::now();
        lastCorr = calculated_position_coordinate;

#ifdef DEBUG
        printf("(%.4f,\t %.4f,\t %.4f)\n", calculated_position_coordinate.x, calculated_position_coordinate.y,
               calculated_position_coordinate.z);
        printf("L:x coordinate %d, y corordinate %d\n", camera_raw_coordinates.l_coordinate[0],
               camera_raw_coordinates.l_coordinate[1]);
        printf("R:x coordinate %d, y corordinate %d\n", camera_raw_coordinates.r_coordinate[0],
               camera_raw_coordinates.r_coordinate[1]);
        printf("S:x coordinate %d, y corordinate %d\n", camera_raw_coordinates.s_coordinate[0],
               camera_raw_coordinates.s_coordinate[1]);
        printf("M:x coordinate %d, y corordinate %d\n", camera_raw_coordinates.m_coordinate[0],
               camera_raw_coordinates.m_coordinate[1]);
#endif

        pose.pose.position.x = calculated_position_coordinate.x;
        pose.pose.position.y = calculated_position_coordinate.y;
        pose.pose.position.z = calculated_position_coordinate.z;
        pose.header.stamp = ros::Time::now();

        refAngle.data = (int16_t) (calculated_position_coordinate.referenceAngle * 180 / 3.1415926 * 1000);

        vel_pub.publish(vel);
        pose_pub.publish(pose);
        angle_pub.publish(refAngle);
      }else{
        smoothBuf[filterIndex] = camera_raw_coordinates;
        filterIndex++;

        if(filterIndex == SMOOTH_FILTER_LENGTH) {
          flagFilterInitialized = true;
          filterIndex = 0;
        }
      }

    }

    ros::spinOnce();

    loop_rate.sleep();
  }
  pixy_close();

  return 0;
}

// LEIMON 2015 //
