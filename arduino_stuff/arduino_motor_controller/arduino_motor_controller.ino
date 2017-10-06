/// This is the code run on the Arduino to actually control the motor controllers
/// As you can, a lot of this was copy and pasted from other examples

#define USE_USBCON
#define USB_CON
/*
 * rosserial Servo Control Example
 *
 * This sketch demonstrates the control of hobby R/C servos
 * using ROS and the arduiono
 * 
 * For the full tutorial write up, visit
 * www.ros.org/wiki/rosserial_arduino_demos
 *
 * For more information on the Arduino Servo Library
 * Checkout :
 * http://www.arduino.cc/en/Reference/Servo
 */

#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif

#include <ros.h>
#include <ArduinoHardware.h>

#include <Servo.h> 

/// Use the two custom messages we have
#include <lk_rover/AllPWMs.h>
#include <lk_rover/AllEncoders.h>

ros::NodeHandle  nh;

/// received is used to check whether a message has made it to the Arduino in
/// the last loop
/// This is so that the Arduino automatically kills the motors if it doesn't
/// receive a ROS command in about half a second (I think)
volatile bool received = false;

/// All the servos
Servo frontLeftWheel, frontRightWheel, backLeftWheel, backRightWheel,
  leftDumpLift, rightDumpLift, leftLadderLift, rightLadderLift,
  ladderSpin, bucketFlap;

/// This converts from the floating points values in the message
/// to microseconds that can be used in the servos
int clamped(float in) {
  int out = (in * 500) + 1500;
  if (out > 2000) out = 2000;
  if (out < 1000) out = 1000;
  return out;
}

/// Whenever the Arduino receives a servo command, it writes the values to
/// the motor controllers and then sets the received flag
void servo_cb( const lk_rover::AllPWMs& cmd_msg){
  // TODO
  frontLeftWheel.writeMicroseconds(clamped(cmd_msg.front_left));
  frontRightWheel.writeMicroseconds(clamped(cmd_msg.front_right));
  backLeftWheel.writeMicroseconds(clamped(cmd_msg.back_left));
  backRightWheel.writeMicroseconds(clamped(cmd_msg.back_right));
  leftDumpLift.writeMicroseconds(clamped(cmd_msg.bucket_left));
  rightDumpLift.writeMicroseconds(clamped(cmd_msg.bucket_right));
  leftLadderLift.writeMicroseconds(clamped(cmd_msg.ladder_left));
  rightLadderLift.writeMicroseconds(clamped(cmd_msg.ladder_right));
  ladderSpin.writeMicroseconds(clamped(cmd_msg.bucket_spin));
  bucketFlap.writeMicroseconds(clamped(cmd_msg.bucket_flap));

  received = true;
  // digitalWrite(13, HIGH-digitalRead(13)); 
}

/// This is used to publish encoder data to ROS
lk_rover::AllEncoders encoderVals;

/// ROS publisher and subscriber stuff
ros::Subscriber<lk_rover::AllPWMs> wheelSub("pwms", servo_cb);
ros::Publisher encoderPub("encoders", &encoderVals);

/// This is where it gets a little complicated
/// First I define a base class with virtual functions
/// This lets me define an interface that will be shared by objects
/// That inherit from Encoder, in this case QuadatureEncoder and PotEncoder
/// This is to simplify code later on by allowing all the input values
/// to be processed by different objects using the same interface
class Encoder {
public:
  virtual void init();
  virtual void readEncoders();
};

/// This is used to collect data from the wheel encoders
/// It is also kind of complicated

/// This uses a really cool feature in C++, which is templates
/// A template is basically a version of a function or class that takes in
/// some set of parameters, and whenever that function is called or an object
/// of that class is made, the C++ compiler makes a new copy of the function
/// or class with those parameters filled in

/// So QuadatureEncoder<1, 2> enc will be a different type of object than
/// QuadatureEncoder<3, 4>

/// Here it matters because we're using a feature
/// that's common in embedded programming called interrupts
/// On each microcontroller, there'll be a set of events that can cause
/// an interrupt
/// When these events happen, the microcontroller stops whatever it's doing,
/// saves the program state, runs the code in its interrupt routine
/// for that particular event, and then goes back to doing what it was before
/// Most of the time you can set what the microcontroller does at least some
/// of the events
/// There's generally certain limitations to the kinds of functions you can
/// use as interrupt code
/// They generally have to have a certain function signature
/// In the case of Arduino, the function signature is void(void),
/// meaning the function must take in no arguments and it must not return
/// anything. It'll also need to not be a method function, so it must either
/// be a static or global function

/// So we could try to write the encoder code like
/// 
/// void encoder0() { /* do stuff for this encoder */ }
/// void encoder1() { /* do stuff for this encoder */ }
/// void encoder2() { /* do stuff for this encoder */ }
/// void encoder3() { /* do stuff for this encoder */ }
/// void encoder4() { /* do stuff for this encoder */ }
/// 
/// But by using templates, we can make the compiler to the repetitive part of
/// this for us
/// For each QuadatureEncoder object, A and B are the two pins that'll be
/// receiving data for that particular encoder
/// Inside the class we declare a static function called encoderISR,
/// so for each QuadatureEncoder object with different A and B, it has its
/// own copy of encoderISR specialized to work with that particular set of A and B,
/// except the compiler generated that for us instead of having to copy and paste it
/// ourselves
template <int A, int B> class QuadatureEncoder: public Encoder {
public:
  QuadatureEncoder(long long unsigned int& ref): count(ref) {}

  /// This is the interrupt function mentioned above
  /// The encoder signal has a certain pattern to it

  /// Because of the way the encoder is designed, it looks like this when the 
  /// encoder is going one way:
  ///      ______________                   ____________
  ///     |              |                 |            |
  /// A   |              |                 |            |
  ///     |              |                 |            |
  ///  ___|              |_________________|            |_________

  ///              ______________                   ____________
  ///             |              |                 |            |
  /// B           |              |                 |            |
  ///             |              |                 |            |
  ///  ___________|              |_________________|            |__


  /// And this when it's going the other:
  ///              ______________                   ____________
  ///             |              |                 |            |
  /// A           |              |                 |            |
  ///             |              |                 |            |
  ///  ___________|              |_________________|            |__

  ///      ______________                   ____________
  ///     |              |                 |            |
  /// B   |              |                 |            |
  ///     |              |                 |            |
  ///  ___|              |_________________|            |_________

  ///             ^                                 ^
  /// The interrupt is triggered on the rising edge of A, which will be at these
  /// points in the signal
  /// You can see that B will be high at those points if the wheel is turning one
  /// way, and low at those points if the wheel is turning the other
  /// So we either decrement or increment based on that

  static void encoderISR() {
    if (digitalRead(B) == HIGH) {
      ++isrCount;
    } else {
      --isrCount;
    }
  }
  
  /// This sets the pins to input so that we can read from them,
  /// sets the counts to zero, and attaches an interrupt to pin A
  void init() {
    pinMode(A, INPUT);
    pinMode(B, INPUT);
    count = 0;
    oldIsrCount = 0;
    attachInterrupt(digitalPinToInterrupt(A), encoderISR, RISING);
  }
  
  /// This finds the change in the interrupt count of the data, and adds
  /// that to count
  void readEncoders() {
    const int curIsrCount = isrCount;
    const int delta = curIsrCount - oldIsrCount;
    count += static_cast<long long int>(delta);
    oldIsrCount = curIsrCount;
  }

  /// This is a reference to a long long int (64-bit) integer
  /// A reference is a lot like a pointer
  /// When you read or write a value here, it actually changes the value
  /// of the thing it points to
  /// The thing it points to is set when the constructor is called
  long long unsigned int &count;
private:
  static volatile int isrCount;
  int oldIsrCount;
};
/// C++ is weird, so isrCount needs to be initialized exactly like this
template <int A, int B> volatile int QuadatureEncoder<A, B>::isrCount = 0;

/// The code for getting data from the potentiometers is much, much simpler
/// It just reads the analog value off of the right pin
class PotSensor: public Encoder {
public:
  PotSensor(int pin, float &output): pin(pin), ref(output) {;}
  void init() {
    pinMode(pin, INPUT); 
  }
  void readEncoders() {
    ref = static_cast<float>(analogRead(pin));
  }

  const int pin;
  float &ref;
};

const int numQuadEncoders = 4;
const int numPotSensors = 4;

/// Here we make all the encoder objects for the wheels
/// and link them to the values in encoderVals
/// 22,23 24,25, 26, 27, and 28,29 are the pins that need to be connected
/// to the wheel encoders
QuadatureEncoder<22, 23> frontLeftEnc(encoderVals.front_left_enc);
QuadatureEncoder<24, 25> frontRightEnc(encoderVals.front_right_enc);
QuadatureEncoder<26, 27> backLeftEnc(encoderVals.back_left_enc);
QuadatureEncoder<28, 29> backRightEnc(encoderVals.back_right_enc);

/// Here we create the potentiometer objects
PotSensor bucketLeftEnc(A0, encoderVals.bucket_left_enc);
PotSensor bucketRightEnc(A1, encoderVals.bucket_right_enc);
PotSensor ladderLeftEnc(A2, encoderVals.ladder_left_enc);
PotSensor ladderRightEnc(A3, encoderVals.ladder_right_enc);

/// Here we stick them all in a big array; we can do this because they all
/// inherit from Encoder
Encoder *encoders[numQuadEncoders + numPotSensors] = {
  &frontLeftEnc, &frontRightEnc, &backLeftEnc, &backRightEnc,
  &bucketLeftEnc, &bucketRightEnc, &ladderLeftEnc, &ladderRightEnc
};

/// Here we use that array of encoders to read all the encoder values at once
/// This is used in the loop() function later to peridically publish the
/// encoder data
void getEncoderVals() {
  for (int i = 0; i < sizeof(encoders)/sizeof(encoders[0]); ++i) {
    Encoder& enc = *encoders[i];
    enc.readEncoders();
  }
}

/// Here we setup the Arduino
void setup(){
  /// Set the resolution to 16, out of the maximum of 20
  analogReadResolution(16);
  /// Not sure what this does
  pinMode(13, OUTPUT);

  /// Set up the ROS stuff
  nh.initNode();
  nh.subscribe(wheelSub);
  nh.advertise(encoderPub);
  
  /// Then initialize all the quadature encoders
  /// This is actually kind of weird, I'm not sure why this isn't also
  /// initializing the potentiometers
  /// Eh whatever, it works at least
  for (int i = 0; i < numQuadEncoders; ++i) {
    Encoder& enc = *encoders[i];
    enc.init();
  }
  /// Set up all the PWMs
  frontLeftWheel.attach(2);
  frontRightWheel.attach(3);
  backLeftWheel.attach(4);
  backRightWheel.attach(5);
  leftDumpLift.attach(6);
  rightDumpLift.attach(7);
  leftLadderLift.attach(8);
  rightLadderLift.attach(9);
  ladderSpin.attach(10);
  bucketFlap.attach(11);
}

/// This writes the 1500 us to all the motors when we need to stop them
/// right now
void killMotors() {
  frontLeftWheel.writeMicroseconds(1500);
  frontRightWheel.writeMicroseconds(1500);
  backLeftWheel.writeMicroseconds(1500);
  backRightWheel.writeMicroseconds(1500);
  leftDumpLift.writeMicroseconds(1500);
  rightDumpLift.writeMicroseconds(1500);
  leftLadderLift.writeMicroseconds(1500);
  rightLadderLift.writeMicroseconds(1500);
  ladderSpin.writeMicroseconds(1500);
}

int timeout = 0;
/// This loops about once a millisecond
void loop() {
  /// It gets the encoder values and stores them in encoderVals
  getEncoderVals();
  /// And then publishes it across the serial connection to ROS
  encoderPub.publish(&encoderVals);
  /// And then does the spinning thing so ROS can do stuff
  nh.spinOnce();
  /// And then it checks to see if it's received any messages from ROS
  /// If the callback was called, received will be sent
  /// Contrawise, if it wasn't called, it wouldn't be set, and it'll have been
  /// one more millisecond since it's received a message
  if (!received) {
    /// which is why timeout is incremented
    ++timeout;
    /// After it's been 500 milliseconds without a message, kill the motors
    if (timeout > 500) {
      killMotors();
    }
  } else {
    /// On the other hand, if it has received a message,
    /// reset the timeout
    timeout = 0;
    /// and reset received so it can be checked next loop
    received = false;
  }
  /// Now wait a millisecond
  delay(1);
}
