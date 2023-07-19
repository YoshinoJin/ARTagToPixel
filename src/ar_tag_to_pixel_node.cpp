#include "ar_tag_to_pixel/ar_tag_to_pixel.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ar_tag_to_pixel");
  
  ARTagtoPixel ar_tag_to_pixel;

  ros::spin();

  return 0;
}