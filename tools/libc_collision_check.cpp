// Simulates what BSD and macOS libc declare in <strings.h>. On Linux glibc
// does NOT declare fls, so without this file the collision that broke the
// macOS build would be invisible here.
extern "C" int fls(int);
extern "C" int flsl(long);
extern "C" int ffs(int);
#include "FLS.hpp"
int main() { airmar::Config c; return c.Validate().empty() ? 0 : 1; }
