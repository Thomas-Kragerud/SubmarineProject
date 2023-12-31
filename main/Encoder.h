class Encoder {

  public:
    Encoder(int pinA, int pinB);

    /** get the absolute value */
    signed long getValue();

    /** set the absolute value to 0 */
    void resetValue();

    /** get the delta of value since last call to this function. */
    signed long getDeltaValue();

    // Interrupt handlers for A and B encoder outputs
    void encoderInterruptA();
    void encoderInterruptB();

  private:
    int pinA;
    int pinB;
    volatile long value = 0;
    volatile long lastReadValue = 0;
};