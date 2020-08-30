#include <stdio.h>
#include <dirent.h>
#include <sys/stat.h>
#include <time.h>
#include <limits.h>

int main(int argc, char *argv[])
{
	DIR *d;
	struct dirent *dir;
	d = opendir("/dev/pts");
	long int idle = LONG_MAX;

	if (d) {
		while ((dir = readdir(d)) != NULL) {
			long int t;
			char file_name[16];
			struct stat f_info;
			sprintf(file_name, "/dev/pts/%s", dir->d_name);

			stat(file_name, &f_info);
			t = time(NULL) - f_info.st_atime;
			if (t < idle)
				idle = t;
		}
		closedir(d);
	}

	printf("%ld\n", idle);

	return 0;
}