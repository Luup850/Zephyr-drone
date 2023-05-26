#ifndef ARUCO_CONFIG_H
#define ARUCO_CONFIG_H
// Camera Matrix
float mtx[3][3] =   {{809.7256631082, 0.0, 266.4705282079},
                     {0, 817.1412927192, 273.6516853926},
                     {0, 0, 1}};

// Distortion coefficients
float dist[5] =     {0.4273174877, 0.3442052301, -0.0000898197, 0.0276518661, -2.8602810872};
#endif