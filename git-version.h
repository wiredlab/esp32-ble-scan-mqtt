/*
 * Make supplies GIT_VERSION at compile time using:
 *   git describe --tags --always --dirty
 * Keep an honest fallback for builds that do not supply a version.
 */
#ifndef GIT_VERSION
#define GIT_VERSION "unknown"
#endif
