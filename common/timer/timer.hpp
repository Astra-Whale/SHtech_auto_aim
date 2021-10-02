#ifndef _TIMER_HPP_
#define _TIMER_HPP_
#include <chrono>
#include <vector>
#include <string>
#include <sstream>
#include <iomanip>
#include <memory>
#include "../log/log.hpp"

typedef std::chrono::steady_clock::time_point tp;
typedef double systime_t;     //ms
typedef double sysduration_t; //ms

class systimer
{
public:
    systimer(void);
    systime_t getTime(void) const;
    std::string getTimeStr(void) const;
    std::string getTimeStr(systime_t t) const;

private:
    tp time_base;
};

class timer
{
public:
    timer() = default;
    void tick(void);
    double tock(void);

private:
    systime_t begin;
};

class avg_timer : public timer
{
public:
    avg_timer(const std::string &name, int up = 50, Log_t log = screen);
    bool tock_avg(void);

private:
    std::string name;
    std::vector<double> time_seq;
    unsigned int up;
    Log_t log;
};

class fps_counter : private timer
{
public:
    fps_counter(const std::string &_name, systime_t _period = 500, Log_t l = screen);
    bool count(void);

private:
    std::string name;
    systime_t period;
    Log_t log;
    int c{0};
};

extern const systimer systime;

#ifndef DO_NOT_CNT_TIM

#define CNT_TIM_AVG(_avg_timer, codes, logs) \
    do                                       \
    {                                        \
        _avg_timer.tick();                   \
        codes;                               \
        _avg_timer.tock();                   \
        if (_avg_timer.tock_avg())           \
        {                                    \
            logs;                            \
        }                                    \
    } while (0);

#define CNT_FPS(_fps_counter, logs) \
    do                              \
    {                               \
        if (_fps_counter.count())   \
        {                           \
            logs;                   \
        }                           \
    } while (0);

#else

#define CNT_TIM(_timer, codes, ...) \
    do                              \
    {                               \
        codes;                      \
    } while (0);

#endif

#endif
