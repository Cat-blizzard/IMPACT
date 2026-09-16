#define _GNU_SOURCE

#include <dlfcn.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

typedef void *EGLDisplay;
typedef unsigned int EGLBoolean;
typedef EGLBoolean (*egl_terminate_fn)(EGLDisplay);

enum
{
  EGL_FALSE = 0,
  EGL_TRUE = 1,
};

EGLBoolean eglTerminate(EGLDisplay display)
{
  const char *guard = getenv("IMPACT_EGL_TERMINATE_GUARD");
  if (guard != NULL && strcmp(guard, "1") == 0)
  {
    static int marker_written = 0;
    if (__atomic_exchange_n(&marker_written, 1, __ATOMIC_RELAXED) == 0)
    {
      static const char marker[] =
        "IMPACT EGL terminate guard active: retaining WSL D3D12 modules until process exit\n";
      const ssize_t written = write(STDERR_FILENO, marker, sizeof(marker) - 1);
      (void)written;
    }
    return EGL_TRUE;
  }

  union
  {
    void *object;
    egl_terminate_fn function;
  } next = {dlsym(RTLD_NEXT, "eglTerminate")};
  return next.function != NULL ? next.function(display) : EGL_FALSE;
}
