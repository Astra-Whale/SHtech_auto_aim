#include "log.hpp"

Log _logfile("log/"), _screen;
Log_t logfile = &_logfile, screen = &_screen;
