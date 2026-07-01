#ifndef gyro_h
#define gyro_h
// timer class
class gyro{
  public:
    
    gyro();
    void init_Gyro();
    double heading();
    double pitch_heading();
    int inverse(int,bool);
    int modulus(int);
    int headingToCardinal(double);
    void reset_accel_filter();
    double opposite_heading(double);
    private:
      bool accelFilterInitialized = false;
      double accelFiltered = 0.0;
      double v;
      unsigned long lastTime;

};
#endif