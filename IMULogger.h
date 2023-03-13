// Avoid IDE problems by defining struct in septate .h file.
// Pad record so size is a power of two for best write performance.
#ifndef IMULogger_h
#define IMULogger_h
const size_t VECTOR_DIM = 4;
struct data_t {// Creates a structure named data_t
  float imu_data[VECTOR_DIM];
  // Maybe integers to store time here
};


#endif  // ExFatLogger_h