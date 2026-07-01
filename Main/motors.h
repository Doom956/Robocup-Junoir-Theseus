#ifndef motors_h
#define motors_h
class motors {
  public:
    motors(int, int, int, int, int, int);
    void init_drive();
    void encoder_update_A();
    void encoder_update_B();
    void encoder_update_D();
    void fullstop();
    void drive(double, double, double, double);
    void reset_encoderCount(bool, bool, bool);
    void set_encoderCountA(int);
    void set_encoderCountB(int);
    void set_encoderCountD(int);
    void set_interrupt(bool, bool, bool);
    void fw(int);
    void backward(int);
    void turnleft(int);
    void turnright(int);
    volatile int encoderCountA;
    volatile int encoderCountB;
    volatile int encoderCountD;
  private:
    int _encoderPin_A_A;
    int _encoderPin_A_B;
    int _encoderPin_B_A;
    int _encoderPin_B_B;
    int _encoderPin_D_A;
    int _encoderPin_D_B;
};
#endif