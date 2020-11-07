#include <stdio.h>
#include <tuple>
#include "libmodule.h"
#define MAXLINE (4096)
int main(int argc, char *argv[]) {
  printf("Project WW_ProjectName_WW\n");
  printf("WW_PrintLetter_WW\n");
  FILE *fp = popen(
      "export TEST_LETTER=\"TEST_LETTER\" ; "
      "ls -al ; "
      "sleep 3 ; "
      "echo $TEST_LETTER", "r");

  char buff[MAXLINE];
  if (fp == NULL)
  {
    perror("erro : ");
    return 0;
  }

  while(fgets(buff, MAXLINE, fp) != NULL)
  {
    printf("%s", buff);
  }
  fclose(fp);

  return 0;
}
