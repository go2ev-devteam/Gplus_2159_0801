/******************************************************************
 *
 * cvr monitor
 *
 * author: deyueli
 * date: 2011-01-21 21:04:01
 */
 
#include <stdio.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <stdlib.h>
#include <pthread.h>
#include <sys/ioctl.h>

#define GP_WDT_MAGIC		'W'
#define WDIOC_CTRL		_IO(GP_WDT_MAGIC,0x01)		/*!< @brief watchdog enable/disable control */	
#define WDIOC_KEEPALIVE		_IO(GP_WDT_MAGIC,0x02)		/*!< @brief watchdog feed */	
#define WDIOC_FORCERESET	_IO(GP_WDT_MAGIC,0x03)		/*!< @brief watchdog force reset system */	
#define WDIOC_SETTIMEROUT	_IOW(GP_WDT_MAGIC,0x04,unsigned int)	/*!< @brief watchdog set timerout */
#define WDIOC_GETTIMEROUT	_IOR(GP_WDT_MAGIC,0x05,unsigned int)	/*!< @brief watchdog get timerout */


static pthread_t wdt_Thread = 0;

int wdt_process(void *param)
{
	int wdt_fd = -1;
	wdt_fd = open("/dev/watchdog", 0);
	ioctl(wdt_fd, WDIOC_CTRL, 0); //disable watchdog
	printf("===============enable watchdog(2sec)================\n");
	while(1){
		ioctl(wdt_fd, WDIOC_CTRL, 0); //disable watchdog
		ioctl(wdt_fd, WDIOC_SETTIMEROUT, 2); //set timeout 2 sec
		ioctl(wdt_fd, WDIOC_CTRL, 1);
		sleep(1);
	}

	close(wdt_fd);
	wdt_fd = 0;

	return 0;
}

 
int main(int argc, char **argv)
{
    int ret, i, status;
    char *child_argv[100] = {0};
    pid_t pid;
	pid_t ch_pid;
	int reboot_count = 0;
	struct sched_param sch_param;
    if (argc < 2) {
 
        fprintf(stderr, "Usage:%s <exe_path> <args...>\n", argv[0]);
        return -1;
    }
    for (i = 1; i < argc; ++i) {
        child_argv[i-1] = (char *)malloc(strlen(argv[i])+1);
        strncpy(child_argv[i-1], argv[i], strlen(argv[i]));
        child_argv[i-1][strlen(argv[i])] = '\0';
    }

	if(wdt_Thread == 0) {
		pthread_create(&wdt_Thread, NULL, wdt_process, 0);
		sch_param.sched_priority = 20;
		pthread_setschedparam(wdt_Thread, SCHED_RR, &sch_param);
	}

    while(1){
 
        pid = fork(); 
        if (pid == -1) {
            fprintf(stderr, "fork() error.errno:%d error:%s\n", errno, strerror(errno));
            break;
        }
        if (pid == 0) {
            ret = execv(child_argv[0], (char **)child_argv);
            if (ret < 0) {
                fprintf(stderr, "execv ret:%d errno:%d error:%s\n", ret, errno, strerror(errno));
                continue;
            }
            exit(0);
        }
 
        if (pid > 0) {
            pid = wait(&status); 
			fprintf(stdout, "kill camcorde or cvr_player");

			system("killall -2 camcorder");
			system("killall -2 cvrplayer");
 
			reboot_count++;
            fprintf(stdout, "wait reboot and reboot_count(%d)\n", reboot_count);
        }
 
    }

	if(wdt_Thread) {
		pthread_join(wdt_Thread, NULL);
		wdt_Thread = 0;
	}
 
    return 0;
}
