
# 1 "util/build_time.c"

# 4 "C:/Program Files/Microchip/MPLABX/v5.50/packs/Microchip/PIC18F-J_DFP/1.4.41/xc8\pic\include\__size_t.h"
typedef unsigned size_t;

# 8 "C:\Program Files\Microchip\xc8\v2.32\pic\include\c90\time.h"
typedef long time_t;
struct tm {
int tm_sec;
int tm_min;
int tm_hour;
int tm_mday;
int tm_mon;
int tm_year;
int tm_wday;
int tm_yday;
int tm_isdst;
};

# 25
extern int time_zone;

# 30
extern time_t time(time_t *);
extern int stime(time_t *);

# 47
extern char * asctime(const struct tm *) ;
extern char * ctime(const time_t *) ;
extern struct tm * gmtime(const time_t *) ;
extern struct tm * localtime(const time_t *) ;
extern size_t strftime(char *, size_t, const char *, const struct tm *) ;
extern time_t mktime(struct tm *);

# 79 "util/build_time.h"
time_t build_time(void);

# 4 "util/build_time.c"
time_t build_time(void)
{
struct tm tm = { 0 };
time_t timestamp;

tm.tm_year = ((("Feb 02 2023"[0] == '?')) ? 99 : ( ("Feb 02 2023"[ 7] - '0') * 1000 + ("Feb 02 2023"[ 8] - '0') * 100 + ("Feb 02 2023"[ 9] - '0') * 10 + ("Feb 02 2023"[10] - '0') )) - 1900;
tm.tm_mon = ((("Feb 02 2023"[0] == '?')) ? 99 : ( (("Feb 02 2023"[0] == 'J' && "Feb 02 2023"[1] == 'a' && "Feb 02 2023"[2] == 'n')) ? 1 : (("Feb 02 2023"[0] == 'F')) ? 2 : (("Feb 02 2023"[0] == 'M' && "Feb 02 2023"[1] == 'a' && "Feb 02 2023"[2] == 'r')) ? 3 : (("Feb 02 2023"[0] == 'A' && "Feb 02 2023"[1] == 'p')) ? 4 : (("Feb 02 2023"[0] == 'M' && "Feb 02 2023"[1] == 'a' && "Feb 02 2023"[2] == 'y')) ? 5 : (("Feb 02 2023"[0] == 'J' && "Feb 02 2023"[1] == 'u' && "Feb 02 2023"[2] == 'n')) ? 6 : (("Feb 02 2023"[0] == 'J' && "Feb 02 2023"[1] == 'u' && "Feb 02 2023"[2] == 'l')) ? 7 : (("Feb 02 2023"[0] == 'A' && "Feb 02 2023"[1] == 'u')) ? 8 : (("Feb 02 2023"[0] == 'S')) ? 9 : (("Feb 02 2023"[0] == 'O')) ? 10 : (("Feb 02 2023"[0] == 'N')) ? 11 : (("Feb 02 2023"[0] == 'D')) ? 12 : 99 )) - 1;
tm.tm_mday = ((("Feb 02 2023"[0] == '?')) ? 99 : ( (("Feb 02 2023"[4] >= '0') ? ("Feb 02 2023"[4] - '0') * 10 : 0) + ("Feb 02 2023"[5] - '0') ));
tm.tm_hour = ((("10:03:47"[0] == '?')) ? 99 : (("10:03:47"[0] - '0') * 10 + "10:03:47"[1] - '0')) - 9;
tm.tm_min = ((("10:03:47"[0] == '?')) ? 99 : (("10:03:47"[3] - '0') * 10 + "10:03:47"[4] - '0'));
tm.tm_sec = ((("10:03:47"[0] == '?')) ? 99 : (("10:03:47"[6] - '0') * 10 + "10:03:47"[7] - '0'));
timestamp = mktime(&tm);

return timestamp;
}
