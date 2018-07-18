#include <libgen.h>
#include <limits.h>
#include <linux/vfio.h>
#include <poll.h>
#include <pthread.h>
#include <stdlib.h>
#include <string.h>
#include <sys/eventfd.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

#include <linux/list.h>

#include "smmute-lib.h"
#include "smmute-vfio.h"

#define to_mdev(dev)	((struct smmute_mdev *)(dev)->private)

struct smmute_mdev {
	int				fd;
	char				*path;
	char				*name;
	struct smmute_iommu_group	*group;
	struct smmute_vfio_transactions	transactions;
	struct smmute_vfio_frames	*frames;
	int				frames_offset;
	int				eventfd;
};

static void *smmute_mdev_alloc_buffer(struct smmute_dev *dev, size_t size,
				      int prot, struct smmute_mem_options *opts)
{
	size = PAGE_ALIGN(size);

	return smmute_lib_alloc_buffer(-1, size, prot, opts);
}

static void smmute_mdev_free_buffer(struct smmute_dev *dev, void *buf, size_t size,
				    struct smmute_mem_options *opts)
{
	smmute_lib_free_buffer(buf, size, opts);
}

static int smmute_mdev_map_buffer(struct smmute_dev *dev, void *va,
				  dma_addr_t *iova, size_t size, int prot,
				  struct smmute_mem_options *opts)
{
	struct smmute_mdev *mdev = to_mdev(dev);

	return smmute_vfio_container_map(mdev->group->container->fd, va, iova,
					 size, prot, opts);
}

static int smmute_mdev_unmap_buffer(struct smmute_dev *dev, void *va,
				    dma_addr_t iova, size_t size,
				    struct smmute_mem_options *opts)
{
	struct smmute_mdev *mdev = to_mdev(dev);

	return smmute_vfio_container_unmap(mdev->group->container->fd, va, iova,
					   size, opts);
}

static int smmute_mdev_launch_transaction(struct smmute_dev *dev, int cmd,
					  union smmute_transaction_params *params)
{
	static int transaction_id;

	int ret, frame_nr;
	uint32_t smmute_cmd;
	uint64_t input_start, output_start;
	struct smmute_mdev *mdev = to_mdev(dev);
	struct smmute_vfio_transactions *transactions = &mdev->transactions;
	struct smmute_vfio_transaction *transaction;
	volatile struct smmute_vfio_uframe *uframe;

	frame_nr = smmute_vfio_alloc_frame(mdev->frames);
	if (frame_nr < 0)
		return -frame_nr;

	transaction = calloc(1, sizeof(*transaction));
	if (!transaction) {
		ret = errno;
		goto err_free_frame;
	}

	input_start = params->common.input_start;
	output_start = params->memcpy.output_start;
	if (params->common.flags & SMMUTE_FLAG_FAULT) {
		input_start = ~input_start;
		output_start = ~output_start;
	}

	transaction->frame = frame_nr;
	transaction->params = *params;

	switch (cmd) {
	case SMMUTE_IOCTL_MEMCPY:
		smmute_cmd = ENGINE_MEMCPY;
		break;
	case SMMUTE_IOCTL_RAND48:
		smmute_cmd = ENGINE_RAND48;
		break;
	case SMMUTE_IOCTL_SUM64:
		smmute_cmd = ENGINE_SUM64;
		break;
	default:
		ret = EINVAL;
		pr_err("unknown command\n");
		goto err_free_transaction;
	}

	pthread_mutex_lock(&transactions->lock);
	transaction->id = ++transaction_id;
	list_add(&transaction->list, &transactions->list);
	pthread_mutex_unlock(&transactions->lock);

	params->common.transaction_id = transaction->id;

	uframe = smmute_vfio_get_uframe(mdev->frames->pages, frame_nr);

	pr_debug("Launching transaction %llu on frame %u\n", transaction->id,
		 frame_nr);

	uframe->cmd		= ENGINE_HALTED;

	uframe->uctrl		= 0;
	uframe->begin		= input_start;
	uframe->end_incl	= input_start + params->common.size - 1;
	uframe->stride		= 1;
	uframe->seed		= params->common.seed;
	uframe->udata[0]	= output_start;
	uframe->udata[1]	= 0;
	uframe->udata[2]	= 0;

	uframe->cmd		= smmute_cmd;

	return 0;

err_free_transaction:
	free(transaction);
err_free_frame:
	smmute_vfio_free_frame(mdev->frames, frame_nr);
	return ret;
}

static int smmute_mdev_get_result(struct smmute_dev *dev,
				  struct smmute_transaction_result *result)
{
	int ret;
	uint32_t cmd;
	uint64_t event;
	volatile struct smmute_vfio_uframe *uframe;
	struct smmute_vfio_transaction *tmp;
	struct smmute_vfio_transaction *transaction = NULL;
	struct smmute_mdev *mdev = to_mdev(dev);
	struct smmute_vfio_transactions *transactions = &mdev->transactions;
	unsigned long long poll_delay = 60000; // ms

	struct pollfd pollfd = {
		.fd = mdev->eventfd,
		.events = POLLIN,
	};

	pthread_mutex_lock(&transactions->lock);
	list_for_each_entry(tmp, &transactions->list, list) {
		if (tmp->id == result->transaction_id) {
			transaction = tmp;
			break;
		}
	}
	pthread_mutex_unlock(&transactions->lock);

	if (!transaction)
		return EINVAL;

	uframe = smmute_vfio_get_uframe(mdev->frames->pages, transaction->frame);

	ret = poll(&pollfd, 1, poll_delay);
	if (ret < 0)
		perror("poll");
	else if (ret == 0)
		pr_err("poll timed out\n");
	else if (pollfd.revents & ~POLLIN || !pollfd.revents)
		pr_err("poll failed with 0x%x\n", pollfd.revents);

	if (ret > 0 && pollfd.revents & POLLIN)
		read(mdev->eventfd, &event, 8);
	cmd = uframe->cmd;

	switch (cmd) {
	case ENGINE_FRAME_MISCONFIGURED:
		result->status = EINVAL;
		break;
	case ENGINE_ERROR:
		result->status = EIO;
		result->value = uframe->udata[2];
		break;
	case ENGINE_HALTED:
		result->status = 0;
		break;
	default:
		result->status = EFAULT;
		break;
	}

	if (!result->status)
		/* Result of a SUM64 */
		result->value = uframe->udata[1];

	smmute_vfio_free_frame(mdev->frames, transaction->frame);

	pthread_mutex_lock(&transactions->lock);
	list_del(&transaction->list);
	pthread_mutex_unlock(&transactions->lock);

	free(transaction);

	return 0;
}

static int smmute_mdev_init_irq(struct smmute_mdev *mdev)
{
	int ret;
	struct vfio_irq_info info = {
		.argsz	= sizeof(info),
	};
	struct {
		struct vfio_irq_set	hdr;
		s32			eventfd;
	} irq_set = {
		.hdr.argsz		= sizeof(irq_set),
		.hdr.flags		= VFIO_IRQ_SET_DATA_EVENTFD |
					  VFIO_IRQ_SET_ACTION_TRIGGER,
		.hdr.count		= 1,
	};

	ret = ioctl(mdev->fd, VFIO_DEVICE_GET_IRQ_INFO, &info);
	if (ret) {
		perror("VFIO_DEVICE_GET_IRQ_INFO");
		return ENODEV;
	}

	if (info.count != 1 || info.flags != VFIO_IRQ_INFO_EVENTFD) {
		pr_err("invalid IRQ info\n");
		return EINVAL;
	}

	irq_set.eventfd = mdev->eventfd = eventfd(0, 0);
	if (mdev->eventfd < 0)
		return errno;

	ret = ioctl(mdev->fd, VFIO_DEVICE_SET_IRQS, &irq_set);
	if (ret) {
		perror("VFIO_DEVICE_SET_IRQS");
		return ENODEV;
	}

	return 0;
}

static int smmute_mdev_init_dev(struct smmute_mdev *mdev)
{
	int ret;
	size_t frames_in_long;
	struct smmute_vfio_frames *frames;
	struct vfio_device_info device_info = {
		.argsz = sizeof(device_info),
	};
	struct vfio_region_info frames_info = {
		.argsz = sizeof(frames_info),
	};

	mdev->frames = frames = calloc(1, sizeof(*frames));
	if (!frames)
		return errno;

	ret = ioctl(mdev->fd, VFIO_DEVICE_GET_INFO, &device_info);
	if (ret) {
		perror("VFIO_DEVICE_GET_INFO");
		return errno;
	}

	if (device_info.num_regions < 1) {
		pr_err("invalid number of regions %d\n", device_info.num_regions);
		return EINVAL;
	}

	if (device_info.flags & VFIO_DEVICE_FLAGS_RESET)
		ioctl(mdev->fd, VFIO_DEVICE_RESET);

	ret = ioctl(mdev->fd, VFIO_DEVICE_GET_REGION_INFO, &frames_info);
	if (ret) {
		perror("VFIO_DEVICE_GET_REGION_INFO");
		return errno;
	}

	if (!(frames_info.flags & VFIO_REGION_INFO_FLAG_MMAP)) {
		pr_err("mmap unsupported\n");
		return EINVAL;
	}

	mdev->frames_offset = frames_info.offset;

	*frames = (struct smmute_vfio_frames) {
		.size	= frames_info.size,
		.nr	= frames_info.size / SMMUTE_VFIO_FRAME_SIZE,
		.cursor	= 0,
	};
	pthread_mutex_init(&frames->lock, NULL);

	frames->pages = mmap(NULL, frames_info.size, PROT_READ | PROT_WRITE,
			     MAP_SHARED, mdev->fd, frames_info.offset);
	if (frames->pages == MAP_FAILED) {
		pr_err("cannot mmap frames\n");
		return errno;
	}
	pr_debug("Mapped %zu pairs of frames at %p (%llx)\n", frames->nr,
		 frames->pages, frames_info.size);

	/* Round up to the nearest long */
	frames_in_long = (frames->nr + BITS_PER_LONG - 1) / BITS_PER_LONG;

	frames->bitmap = calloc(frames_in_long, sizeof(long));
	if (!frames->bitmap) {
		ret = errno;
		pr_err("cannot allocate frame bitmap\n");
		goto err_unmap_pages;
	}

	ret = smmute_mdev_init_irq(mdev);
	if (ret)
		goto err_free_bitmap;

	INIT_LIST_HEAD(&mdev->transactions.list);
	pthread_mutex_init(&mdev->transactions.lock, NULL);

	return 0;

err_free_bitmap:
	free(frames->bitmap);
err_unmap_pages:
	munmap(frames->pages, frames->size);

	return ret;
}

static void smmute_mdev_destroy_dev(struct smmute_mdev *mdev)
{
	struct smmute_vfio_frames *frames = mdev->frames;

	free(frames->bitmap);
	munmap(frames->pages, frames->size);
}

static int smmute_mdev_open(struct smmute_dev *dev, const char *path, int flags)
{
	struct smmute_mdev *mdev;
	long group_id;
	int ret;

	mdev = calloc(1, sizeof(*mdev));
	if (!mdev)
		return ENOMEM;

	mdev->path = realpath(path, NULL);
	if (!mdev->path) {
		ret = errno;
		pr_err("invalid device path %s\n", path);
		goto err_free_mdev;
	}

	group_id = smmute_vfio_group_id(mdev->path);
	if (group_id < 0) {
		ret = -group_id;
		goto err_free_path;
	}

	mdev->group = smmute_vfio_group_get(group_id);
	if (!mdev->group) {
		ret = errno;
		goto err_free_path;
	}

	mdev->name = strdup(basename(mdev->path));
	if (!mdev->name) {
		ret = errno;
		goto err_put_group;
	}
	pr_debug("MDEV name: %s\n", mdev->name);

	mdev->fd = ioctl(mdev->group->fd, VFIO_GROUP_GET_DEVICE_FD, mdev->name);
	if (mdev->fd < 0) {
		ret = errno;
		pr_err("could not open mdev\n");
		goto err_free_name;
	}

	ret = smmute_mdev_init_dev(mdev);
	if (ret)
		goto err_close_dev;

	dev->private = mdev;

	return 0;

err_close_dev:
	close(mdev->fd);
err_free_name:
	free(mdev->name);
err_put_group:
	smmute_vfio_group_put(mdev->group);
err_free_path:
	free(mdev->path);
err_free_mdev:
	free(mdev);
	return ret;
}

static void smmute_mdev_close(struct smmute_dev *dev)
{
	struct smmute_mdev *mdev = to_mdev(dev);

	if (!mdev)
		return;

	smmute_mdev_destroy_dev(mdev);
	close(mdev->fd);
	free(mdev->name);
	smmute_vfio_group_put(mdev->group);
	free(mdev->path);
	free(mdev);
}

static int smmute_mdev_init(struct smmute_backend_options *opts)
{
	return 0;
}

static void smmute_mdev_exit(void)
{
}

struct smmute_device_ops mdev_ops = {
	.init			= smmute_mdev_init,
	.exit			= smmute_mdev_exit,

	.open			= smmute_mdev_open,
	.close			= smmute_mdev_close,

//	.bind			= smmute_mdev_bind,
//	.unbind			= smmute_mdev_unbind,

	.alloc_buffer		= smmute_mdev_alloc_buffer,
	.free_buffer		= smmute_mdev_free_buffer,

	.map_buffer		= smmute_mdev_map_buffer,
	.unmap_buffer		= smmute_mdev_unmap_buffer,

	.launch_transaction	= smmute_mdev_launch_transaction,
	.get_result		= smmute_mdev_get_result,
};
