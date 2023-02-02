#ifndef HORNET_IMU_CONFIG
#define HORNET_IMU_CONFIG

//IMU Selection
#define IMU_JY901
//#define IMU_BUILTIN


#if !(defined IMU_JY901 || defined IMU_BUILTIN)
#error no IMU type selected!
#endif


//! TODO: add sanity check only one IMU type selected
//! TODO: add sanity check make board should have built in IMU if that option is selected

#endif //HORNET_IMU_CONFIG