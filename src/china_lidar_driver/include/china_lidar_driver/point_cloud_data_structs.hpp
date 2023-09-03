#pragma once

#include <stdint.h>

// just a structure for Point data (X,Y,Z,D)
//TODO: move lidar interaction structs here and remove duplicate code
typedef struct __attribute__((packed))
{
  float x;
  float y;
  float z;
  uint8_t   intensity;
} PointData;

// Way to use same block of memory so we dont have to copy PointData to byte array
typedef union 
{
  PointData point_data;
  uint8_t bytes[sizeof(PointData)];
} Point_u;