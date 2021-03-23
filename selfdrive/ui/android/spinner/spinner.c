#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <unistd.h>
#include <assert.h>
#include <ctype.h>

#include <GLES3/gl3.h>
#include <EGL/egl.h>
#include <EGL/eglext.h>

#include "nanovg.h"
#define NANOVG_GLES3_IMPLEMENTATION
#include "nanovg_gl.h"
#include "nanovg_gl_utils.h"

#include "framebuffer.h"
//#include "spinner.h"

#define SPINTEXT_LENGTH 128

// external resources linked in
extern const unsigned char _binary_opensans_semibold_ttf_start[];
extern const unsigned char _binary_opensans_semibold_ttf_end[];

extern const unsigned char _binary_img_spinner_track_png_start[];
extern const unsigned char _binary_img_spinner_track_png_end[];

extern const unsigned char _binary_img_spinner_comma_png_start[];
extern const unsigned char _binary_img_spinner_comma_png_end[];

bool stdin_input_available() {
  struct timeval timeout;
  timeout.tv_sec = 0;
  timeout.tv_usec = 0;

  fd_set fds;
  FD_ZERO(&fds);
  FD_SET(STDIN_FILENO, &fds);
  select(STDIN_FILENO+1, &fds, NULL, NULL, &timeout);
  return (FD_ISSET(0, &fds));
}

int main(int argc, char** argv) {

  bool draw_progress = false;
  float progress_val = 0.0;

  char spintext[SPINTEXT_LENGTH];
  spintext[0] = 0;

  const char* spintext_arg = NULL;
  if (argc >= 2) {
    strncpy(spintext, argv[1], SPINTEXT_LENGTH);
  }

  // spinner
  int fb_w, fb_h;
  FramebufferState *fb = framebuffer_init("spinner", 0x00001000, false,
                                          &fb_w, &fb_h);
  assert(fb);
  framebuffer_set_power(fb, HWC_POWER_MODE_NORMAL);

  NVGcontext *vg = nvgCreateGLES3(NVG_ANTIALIAS | NVG_STENCIL_STROKES);
  assert(vg);

  int font = nvgCreateFontMem(vg, "Bold", (unsigned char*)_binary_opensans_semibold_ttf_start, _binary_opensans_semibold_ttf_end-_binary_opensans_semibold_ttf_start, 0);
  assert(font >= 0);
  uint64_t* header = (uint64_t*) _binary_img_spinner_track_png_start;
	uint64_t count = header[0];
	uint64_t* sizes = &header[1];
	uint8_t* spdata = (uint8_t*) &header[header[0]+1];
  int spin_imgs[30];
  spin_imgs[0] = nvgCreateImageMem(vg, 0, (unsigned char*) spdata, sizes[0]);

  for (int i = 1; i < 30; i++) {
    uint64_t offset = 0;
    for (int j  = 0; j < i; j++) {
      offset += sizes[j];
    }
    spin_imgs[i] = nvgCreateImageMem(vg, 0, (unsigned char*) spdata + offset, sizes[i]);
  }

  for (int cnt = 0; ; cnt++) {
    // Check stdin for new text
    if (stdin_input_available()){
      fgets(spintext, SPINTEXT_LENGTH, stdin);
      spintext[strcspn(spintext, "\n")] = 0;

      // Check if number (update progress bar)
      size_t len = strlen(spintext);
      bool is_number = len > 0;
      for (int i = 0; i < len; i++){
        if (!isdigit(spintext[i])){
          is_number = false;
          break;
        }
      }

      if (is_number) {
        progress_val = (float)(atoi(spintext)) / 100.0;
        progress_val = fmin(1.0, progress_val);
        progress_val = fmax(0.0, progress_val);
      }

      draw_progress = is_number;
    }

    glClearColor(0.1, 0.1, 0.1, 1.0);
    glClear(GL_STENCIL_BUFFER_BIT | GL_COLOR_BUFFER_BIT);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    nvgBeginFrame(vg, fb_w, fb_h, 1.0f);

    // background
    nvgBeginPath(vg);
    NVGpaint bg = nvgLinearGradient(vg, fb_w, 0, fb_w, fb_h,
    nvgRGBA(0, 0, 0, 175), nvgRGBA(0, 0, 0, 255));
    nvgFillPaint(vg, bg);
    nvgRect(vg, 0, 0, fb_w, fb_h);
    nvgFill(vg);

    // kommu spinner
    int adjcnt = (cnt/2) % 30;
    NVGpaint kommu_imgPaint = nvgImagePattern(vg, 0, 0,
          fb_w, fb_h, 0, spin_imgs[adjcnt], 1.0f);
    nvgBeginPath(vg);
    nvgFillPaint(vg, kommu_imgPaint);
    nvgRect(vg, 0, 0, fb_w, fb_h);
    nvgFill(vg);

    if (draw_progress){
      // draw progress bar
      int progress_width = 1000;
      int progress_x = fb_w/2-progress_width/2;
      int progress_y = 775;
      int progress_height = 25;

      NVGpaint paint = nvgBoxGradient(
          vg, progress_x + 1, progress_y + 1,
          progress_width - 2, progress_height, 3, 4, nvgRGB(27, 27, 27), nvgRGB(27, 27, 27));
      nvgBeginPath(vg);
      nvgRoundedRect(vg, progress_x, progress_y, progress_width, progress_height, 12);
      nvgFillPaint(vg, paint);
      nvgFill(vg);

      int bar_pos = ((progress_width - 2) * progress_val);

      paint = nvgBoxGradient(
          vg, progress_x, progress_y,
          bar_pos+1.5f, progress_height-1, 3, 4,
          nvgRGB(245, 245, 245), nvgRGB(105, 105, 105));

      nvgBeginPath(vg);
      nvgRoundedRect(
          vg, progress_x+1, progress_y+1,
          bar_pos, progress_height-2, 12);
      nvgFillPaint(vg, paint);
      nvgFill(vg);
    } else {
      // message
      nvgTextAlign(vg, NVG_ALIGN_CENTER | NVG_ALIGN_TOP);
      nvgFontSize(vg, 96.0f);
      nvgText(vg, fb_w/2, (fb_h*2/3)+24, spintext, NULL);
    }

    nvgEndFrame(vg);
    framebuffer_swap(fb);
    assert(glGetError() == GL_NO_ERROR);
  }

  return 0;
}
