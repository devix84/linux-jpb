#define _GNU_SOURCE

#include <fcntl.h>
#include <errno.h>
#include <getopt.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/stat.h>

/* Log messages on stderr, since stdout is used for splicing */
#define STREAM_OUT stderr
#include "smmute-lib.h"

enum loglevel loglevel = LOG_INFO;

enum transaction_mode {
	NONE		= 0,
	SPLICE_IN	= 1,
	SPLICE_OUT	= 2,
};

struct program_options {
	char			*device;
	size_t			size;
	enum transaction_mode	mode;
};

static int do_splice(int fd, struct program_options *opts)
{
	int ret = 0;
	ssize_t len = opts->size;
	ssize_t n_spliced_in = 0;
	ssize_t n_spliced_out = 0;
	unsigned int splice_flags = 0;
	enum transaction_mode mode = opts->mode;

	/*
	 * TODO: API ought to be improved to pass more info.
	 * ioctls:
	 *   set_default_transaction_params, set_devpipe
	 *   get_all_results
	 */
	int devpipe = (mode & SPLICE_IN) && (mode & SPLICE_OUT);
	if (devpipe)
		splice_flags |= SPLICE_F_MOVE; // XXX hack

	if (mode & SPLICE_IN) {
		while (len) {
			ret = splice(STDIN_FILENO, NULL, fd, NULL, len,
				     splice_flags);
			if (ret <= 0)
				break;

			n_spliced_in += ret;
			len -= ret;
		}

		pr_info("%zd bytes spliced in\n", n_spliced_in);

		if (ret < 0) {
			perror("splice(STDIN, fd)");
			return 1;
		}
		len = n_spliced_in;
	}

	if (!(mode & SPLICE_OUT))
		return 0;

	while (len) {
		ret = splice(fd, NULL, STDOUT_FILENO, NULL, len,
				splice_flags);
		if (ret <= 0)
			break;

		n_spliced_out += ret;
		len -= ret;
	}

	if (ret < 0) {
		perror("splice(fd, STDOUT)");
	} else {
		/* stdout might be busy, output to stderr */
		pr_info("%zd bytes spliced out\n", n_spliced_out);
		ret = 0;
	}

	return ret;
}

static const char *help_string =
"Usage: ... | %s [opts]           SUM64 on input                           \n"
"             %s [opts] | ...     RAND48 into output                       \n"
"       ... | %s [opts] | ...     MEMCPY from input into output          \n\n"
"splice(2) operation on pipes. Map input/output pages served by the kernel \n"
"and execute transaction on them with the test engine                    \n\n"
"  OPTION           DESCRIPTION                                     DEFAULT\n"
"  <dev>            smmute device                              /dev/smmute0\n"
"  -d               show debug messages                                    \n"
"  -q               only show error messages                               \n"
"  -s <size>        Number of bytes to splice                              \n";

static void print_usage(char *progname)
{
	fprintf(stderr, help_string, progname, progname, progname);
}

static int parse_options(int argc, char *argv[], struct program_options *opts)
{
	const char *optstring = "dqs:";
	int ret;

	/* optind, opterr, optopt and optarg are defined in unistd */
	optind = 1;
	/* Print error messages */
	opterr = 1;
	while ((ret = getopt(argc, argv, optstring)) != -1) {
		switch (ret) {
		case 'd':
			loglevel = LOG_DEBUG;
			break;
		case 'q':
			loglevel = LOG_ERROR;
			break;
		case 's':
			errno = 0;
			opts->size = strtoul(optarg, NULL, 0);
			if (errno) {
				pr_err("invalid number '%s'\n", optarg);
				return 1;
			}
			break;
		default:
			print_usage(argv[0]);
			return 1;
		}
	}

	if (optind < argc) {
		opts->device = argv[optind];
	}

	return 0;
};

/*
 * Returns 1 if fd is a pipe
 */
static int is_fifo(int fd)
{
	struct stat stat;

	if (fstat(fd, &stat) < 0)
		return 0;

	if (S_ISFIFO(stat.st_mode))
		return 1;

	return 0;
}

int main(int argc, char **argv) {
	int fd;
	struct program_options opts = {
		.device	= "/dev/smmute0",
		.size	= 0x1000,
		.mode	= NONE,
	};

	if (parse_options(argc, argv, &opts))
		return 1;

	fd = open(opts.device, O_RDWR);
	if (fd < 0) {
		perror("cannot open smmute file");
		return 1;
	}

	if (is_fifo(STDIN_FILENO))
		opts.mode |= SPLICE_IN;

	if (is_fifo(STDOUT_FILENO))
		opts.mode |= SPLICE_OUT;

	if (opts.mode == NONE) {
		print_usage(argv[0]);
		return 1;
	}

	return do_splice(fd, &opts);
}
