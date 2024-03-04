#pragma once

#include <cstdlib>
#include <sys/wait.h>
#include <unistd.h>

static void BSOD(const char *msg)
{
  pid_t pid = fork();
  if (pid == 0) {
    chdir("/data/openpilot/selfdrive/ui");
    execlp("./text", "text", msg, NULL);
  } else {
    waitpid(pid, NULL, 0);
  }
}
