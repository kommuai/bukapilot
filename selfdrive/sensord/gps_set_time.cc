#include <atomic>
#include <cstring>
#include <iostream>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <unistd.h>
#include <assert.h>
#include <sys/time.h>
#include <sys/cdefs.h>
#include <sys/types.h>
#include <sys/resource.h>

#include <pthread.h>

#include <cutils/log.h>

#include <hardware/gps.h>
#include <utils/Timers.h>

#include "selfdrive/common/timing.h"
#include "selfdrive/common/util.h"

ExitHandler do_exit;

namespace {

const GpsInterface* gGpsInterface = NULL;

std::atomic<char> finish = 0;

char dcmd[64];
char tcmd[64];

bool set_time(char *timestr) {
  if (strlen(timestr) != 6)
    return false;
  char h[3] = {0}, m[3] = {0}, s[3] = {0};
  sscanf(timestr, "%c%c%c%c%c%c", &h[0], &h[1], &m[0], &m[1], &s[0], &s[1]);
  sprintf(tcmd, "date +%%T -s \"%s:%s:%s\"", h, m, s);
  return true;
}

bool set_date(char *datestr) {
  if (strlen(datestr) != 6)
    return false;
  char y[3] = {0}, m[3] = {0}, d[3] = {0};
  sscanf(datestr, "%c%c%c%c%c%c", &d[0], &d[1], &m[0], &m[1], &y[0], &y[1]);
  sprintf(dcmd, "date +%%F -s \"20%s-%s-%s\"", y, m, d);
  return true;
}

void nmea_callback(GpsUtcTime timestamp, const char* nmea, int length) {
  if (finish == 1)
    return;
  if (nmea[3] != 'R' || nmea[4] != 'M' || nmea[5] != 'C')
    return;

  char buf[256];
  strncpy(buf, nmea, 256);
  char *tokp = buf;
  char *tok = strsep(&tokp, ",");
  bool dset = false, tset= false;
  for (int idx = 0; tok; idx++, tok = strsep(&tokp, ",")) {
    switch (idx) {
    case 1:
      tset = set_time(tok);
      break;
    case 9:
      dset = set_date(tok);
      break;
    }
  }
  std::cout << "TIME " << tset << " DATE " << dset << std::endl;
  if (dset && tset) {
    std::cout << dcmd << std::endl;
    std::system(dcmd);
    std::cout << tcmd << std::endl;
    std::system(tcmd);
    finish = 1;
    std::cout << "BYE" << std::endl;
  }
}

void location_callback(GpsLocation* location) {
}

pthread_t create_thread_callback(const char* name, void (*start)(void *), void* arg) {
  pthread_t thread;
  pthread_attr_t attr;
  int err;

  err = pthread_attr_init(&attr);
  err = pthread_create(&thread, &attr, (void*(*)(void*))start, arg);

  return thread;
}

GpsCallbacks gps_callbacks = {
  sizeof(GpsCallbacks),
  location_callback,
  NULL,
  NULL,
  nmea_callback,
  NULL,
  NULL,
  NULL,
  create_thread_callback,
};

void gps_init() {
  hw_module_t* module = NULL;
  hw_get_module(GPS_HARDWARE_MODULE_ID, (hw_module_t const**)&module);
  assert(module);

  static hw_device_t* device = NULL;
  module->methods->open(module, GPS_HARDWARE_MODULE_ID, &device);
  assert(device);

  // ** get gps interface **
  gps_device_t* gps_device = (gps_device_t *)device;
  gGpsInterface = gps_device->get_gps_interface(gps_device);
  assert(gGpsInterface);

  gGpsInterface->init(&gps_callbacks);

  gGpsInterface->start();
  gGpsInterface->set_position_mode(GPS_POSITION_MODE_MS_BASED,
                                   GPS_POSITION_RECURRENCE_PERIODIC,
                                   100, 0, 0);

}

}

int main() {
  dcmd[0] = 0;
  tcmd[0] = 0;
  gps_init();
  std::cout << finish << std::endl;
  while (finish != 1 && !do_exit) {
    sleep(1);
  }

  gGpsInterface->stop();
  gGpsInterface->cleanup();

  return 0;
}
