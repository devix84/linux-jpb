#include <errno.h>
#include <pthread.h>
#include <signal.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include "smmute-lib.h"

enum loglevel loglevel = LOG_DEBUG;

struct smmute_bind_child {
	pid_t		pid;
	pthread_t	thread;
	int		pasid;
};

/*
 * At the moment set the max bind at 118 tasks. Two ASIDs per task -> 128 ASIDs
 * available, minus 8+2 for reserve. Rubbish, but I don't make the rules.
 */
//#define NR_CHILDREN	118
#define NR_CHILDREN	32

static struct smmute_dev dev;
static struct smmute_bind_child children[NR_CHILDREN];

static void *kill_unbind_child(void *arg)
{
	int ret;
	struct smmute_bind_child *child = arg;

	kill(child->pid, SIGINT);
	usleep(500000);
	ret = smmute_unbind(&dev, child->pid, child->pasid);
	if (ret)
		pr_warn("unbind %d failed with %d\n", child->pid, ret);

	return NULL;
}

static int test_unbind_exit_race(struct smmute_dev *dev)
{
	int i;
	int ret;
	pid_t pid;
	struct smmute_bind_child *child;

	/* Fork a process, bind it, kill it, unbind it */

	for (i = 0; i < NR_CHILDREN; i++) {
		pid = fork();

		if (pid < 0) {
			goto err_kill_children;
		} else if (pid == 0) {
			/* Child process idles */
			while (true)
				asm volatile ("wfe" ::: "memory");
			return 0;
		}

		/* Parent process */
		child = &children[i];
		child->pid = pid;

		ret = smmute_bind(dev, pid, &child->pasid);
		if (ret) {
			pr_warn("bind %d failed with %d\n", i, ret);
			goto err_kill_children;
		}
		pr_debug("child pid=%d pasid=%d\n", child->pid, child->pasid);
	}

	for (i = 0; i < NR_CHILDREN; i++) {
		child = &children[i];
		pthread_create(&child->thread, NULL, kill_unbind_child, child);
	}

	for (i = 0; i < NR_CHILDREN; i++) {
		child = &children[i];
		pthread_join(child->thread, NULL);
	}

	return 0;

err_kill_children:
	pr_info("Kill all!\n");
	kill(0, SIGKILL);
	return EINVAL;
}

int main(int argc, char **argv)
{
	int ret;
	const char *dev_path = "/dev/smmute0";

	ret = smmute_backend_init(SMMUTE_BACKEND_KERNEL, NULL);
	if (ret)
		return ret;

	dev.backend = SMMUTE_BACKEND_KERNEL;

	ret = smmute_device_open(&dev, dev_path, 0);
	if (ret) {
		pr_err("Could not open '%s'\n", dev_path);
		return ret;
	}

	return test_unbind_exit_race(&dev);
}
