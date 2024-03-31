/*C requirements includes*/
#include <stdio.h>

/*system includes*/

/*utilitys includes*/

/*internal drivers includes*/

/*external component includes*/

/*global includes*/

/*defines*/
#define ONE_TIME_EXECUTION
//#define LIMITED_REPITION_LOOP
//#define INFINITE_LOOP
//#define SINGLE_PASS_TASK_EXECUTION
//#define LIMITED_REPITION_TASK_EXECUTION
//#define INFITE_TASK_LOOP

/*global function prototypes*/
#ifdef _TIME_H_
static void compile_time_set(void);
#endif

/*variables*/

void app_main(void)
{
#ifdef _TIME_H_
    compile_time_set();
#endif


}

#ifdef _TIME_H_
static void compile_time_set(void)
{
    time_t t_set;
    struct tm tmFw;
    char strftime_buf[32];
    
    /*Get time at compile*/
    strptime(__DATE__, "%b %d %Y", &tmFw);
    strptime(__TIME__, "%T", &tmFw);
    strftime(strftime_buf, sizeof(strftime_buf), "%c", &tmFw);
    t_set = mktime(&tmFw);
    struct timeval set_now = { .tv_sec = t_set };
    settimeofday(&set_now, NULL);
    ESP_LOGI(TAG, "Compile local date/time is: %s", strftime_buf);
}
#endif
