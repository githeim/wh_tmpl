#include <stdio.h>
#include <bitset>
#include <iostream>
int main(int argc, char *argv[]) {
  printf("Project WW_ProjectName_WW\n");
  printf("WW_PrintLetter_WW\n");

  std::bitset<32> mask1;
  std::bitset<32> mask2;
  std::bitset<32> mask3;
  mask1 = 0b1001011;
  mask2 = 0b0001001;
  mask3 = 0b0000100;
  // mask1 &= ~(0b10);
  //mask1 |= (0b100);
  std::cout<<"mask1 :"<<mask1<<"\n";
  std::cout<<"mask2 :"<<mask2<<"\n";

  std::cout<<"\nAdd 0b0000100 to mask1\n";
  mask1 |= mask3;
  std::cout<<"output mask1 :"<<mask1<<"\n";

  std::cout<<"\nremove 0b0000100 from mask1\n";
  mask1 &= ~(mask3);
  std::cout<<"output mask1 :"<<mask1<<"\n";

   
  std::cout<<"\nCheck mask2 from mask1\n";
  
  if ((mask1 & mask2) == mask2) {
     std::cout<<"mask1 include mask2 flags "<<mask2 <<" bit \n";
  }else
  {
     std::cout<<"check fail "<<mask2 <<" bit \n";
  }

  mask2 = 0b0001101;
  std::cout<<"\nmask1 :"<<mask1<<"\n";
  std::cout<<"change mask2 : "<<mask2<<"\n";
  if ((mask1 & mask2) == mask2) {
     std::cout<<"mask1 include mask2 flags "<<mask2 <<" bit \n";
  }else
  {
     std::cout<<"check fail mask2"<<mask2 <<" bit \n";
  }

  return 0;
}
