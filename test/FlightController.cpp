#include <Arduino.h>
#include <unity.h>
#include <HornetFC.h>

HornetFC* flight_controller;

void test_init(void)
{
    flight_controller = new HornetFC();
    
}

void setup()
{
  delay(2000); // service delay
  UNITY_BEGIN();

  RUN_TEST(test_init);

  UNITY_END(); // stop unit testing
}

void loop()
{
}