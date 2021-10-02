#ifndef _LOG_HPP_
#define _LOG_HPP_

#include <fstream>
#include <string>
#include <chrono>
#include <sstream>
#include <iomanip>
#include <ctime>
#include <sys/stat.h>
#include <sys/types.h>

class Log
{
public:
    Log operator=(Log &) = delete;
    Log(std::string folder)
    {
        time_t now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
        std::stringstream ss;
        ss << std::put_time(std::localtime(&now), "%Y-%m-%d-%H:%M:%S");
        mkdir(folder.c_str(), S_IRUSR | S_IWUSR | S_IXUSR | S_IRWXG | S_IRWXO);
        if (folder[folder.length() - 1] != '/')
        {
            folder = folder + "/";
        }
        std::string file = folder + ss.str() + ".txt";
        out = fopen(file.c_str(), "w");
        if (!out)
        {
            printf("Open Log File Fail\n");
            //throw "Open Log File Fail";
        }
    }
    Log(void)
    {
        out = stdout;
    }
    ~Log(void)
    {
        if (out != stdout)
        {
            fclose(out);
            printf("Close Log File Success!\n");
        }
    }
    void reopen(std::string file)
    {
        if (out != stdout)
        {
            fclose(out);
        }
        out = fopen(file.c_str(), "w");
        if (!out)
        {
            throw "Repen Log File Fail";
        }
    }
    FILE *out;
};

typedef Log *Log_t;

extern Log_t screen;
extern Log_t logfile;

#include "../timer/timer.hpp"

/************** Define the control code *************/
#define START_CTR "\033[0"
#define END_CTR "m"
#define CLEAR_CODE ";0"
#define LIGHT_CODE ";1"
#define LINE_CODE ";4"
#define BLINK_CODE ";5"
#define REVERSE_CODE ";7"
#define VANISH_CODE ";8"
#define WORD_WHITE_CODE ";30"
#define WORD_RED_CODE ";31"
#define WORD_GREEN_CODE ";32"
#define WORD_YELLOW_CODE ";33"
#define WORD_BLUE_CODE ";34"
#define WORD_PURPLE_CODE ";35"
#define WORD_CYAN_CODE ";36"
#define WORD_GRAY_CODE ";37"
#define BACK_WHITE_CODE ";40"
#define BACK_RED_CODE ";41"
#define BACK_GREEN_CODE ";42"
#define BACK_YELLOW_CODE ";43"
#define BACK_BLUE_CODE ";44"
#define BACK_PURPLE_CODE ";45"
#define BACK_CYAN_CODE ";46"
#define BACK_GRAY_CODE ";47"

#define CTRS(ctrs) START_CTR ctrs END_CTR
#define STR_CTR(ctrs, str) START_CTR ctrs END_CTR str CLEAR_ALL

#define WORD_WHITE WORD_WHITE_CODE
#define WORD_RED WORD_RED_CODE
#define WORD_GREEN WORD_GREEN_CODE
#define WORD_YELLOW WORD_YELLOW_CODE
#define WORD_BLUE WORD_BLUE_CODE
#define WORD_PURPLE WORD_PURPLE_CODE
#define WORD_CYAN WORD_CYAN_CODE
#define WORD_GRAY WORD_GRAY_CODE
#define WORD_LIGHT_WHITE LIGHT_CODE WORD_WHITE
#define WORD_LIGHT_RED LIGHT_CODE WORD_RED
#define WORD_LIGHT_GREEN LIGHT_CODE WORD_GREEN
#define WORD_LIGHT_YELLOW LIGHT_CODE WORD_YELLOW
#define WORD_LIGHT_BLUE LIGHT_CODE WORD_BLUE
#define WORD_LIGHT_PURPLE LIGHT_CODE WORD_PURPLE
#define WORD_LIGHT_CYAN LIGHT_CODE WORD_CYAN
#define WORD_LIGHT_GRAY LIGHT_CODE WORD_GRAY
#define CLEAR_ALL CTRS(CLEAR_CODE)

/******* Ensure the color corresponding to the level ******/
#ifndef LOG_ERR_COLOR
#define LOG_ERR_COLOR WORD_RED
#endif
#ifndef LOG_WARN_COLOR
#define LOG_WARN_COLOR WORD_YELLOW
#endif
#ifndef LOG_MSG_COLOR
#define LOG_MSG_COLOR WORD_GRAY
#endif
#ifndef LOG_LINK_COLOR
#define LOG_LINK_COLOR LINE_CODE WORD_BLUE
#endif

#define MLOG(tar, level, fmt, ...) fprintf((tar)->out, "%s " level fmt "\n", systime.getTimeStr().c_str(), ##__VA_ARGS__)
#define LOGM(tar, fmt, ...) MLOG((tar), STR_CTR(LOG_MSG_COLOR, "<MSG>: "), STR_CTR(LOG_MSG_COLOR, fmt), ##__VA_ARGS__)
#define LOGW(tar, fmt, ...) MLOG((tar), STR_CTR(LOG_WARN_COLOR, "<WARN>: "), STR_CTR(LOG_WARN_COLOR, fmt), ##__VA_ARGS__)
#define LOGE(tar, fmt, ...) MLOG((tar), STR_CTR(LOG_ERR_COLOR, "<ERR>: "), STR_CTR(LOG_ERR_COLOR, fmt), ##__VA_ARGS__)
#define LOGM_S(fmt, ...) LOGM(screen, fmt, ##__VA_ARGS__)
#define LOGW_S(fmt, ...) LOGW(screen, fmt, ##__VA_ARGS__)
#define LOGE_S(fmt, ...) LOGE(screen, fmt, ##__VA_ARGS__)
#define LOGM_F(fmt, ...) LOGM(logfile, fmt, ##__VA_ARGS__)
#define LOGW_F(fmt, ...) LOGW(logfile, fmt, ##__VA_ARGS__)
#define LOGE_F(fmt, ...) LOGE(logfile, fmt, ##__VA_ARGS__)
#define LOGAvar(var_name, fmt, var, index) LOGM_F(var_name ":(" fmt ",%d)", var, index)
#define LOGAstate(var_name, index) LOGM_F(var_name ":(1,%d)", index)

#endif
