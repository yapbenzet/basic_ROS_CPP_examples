#include <stdio.h>
#include "Factorial.h"

int main(int argc, char** argv)
{
  
  cpp_example::Factorial f;
  cpp_example::Factorial g(4);
  
  int result1 = f.compute();
  int result2 = g.compute();
  
  printf("Result1: %d, result2: %d\n", result1, result2);
  
  return 0;
}
