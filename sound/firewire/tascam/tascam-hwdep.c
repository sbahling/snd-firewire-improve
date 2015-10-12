/*
 * tascam-hwdep.c - a part of driver for TASCAM FireWire series
 *
 * Copyright (c) 2015 Takashi Sakamoto
 *
 * Licensed under the terms of the GNU General Public License, version 2.
 */

/*
 * This codes give three functionality.
 *
 * 1.get firewire node information
 * 2.get notification about starting/stopping stream
 * 3.lock/unlock stream
 */

#include "tascam.h"

static long hwdep_read(struct snd_hwdep *hwdep, char __user *buf, long count,
		       loff_t *offset)
{
	struct snd_tscm *tscm = hwdep->private_data;
	DEFINE_WAIT(wait);
	union snd_firewire_event event = {
		.lock_status.type = SNDRV_FIREWIRE_EVENT_LOCK_STATUS,
	};

	spin_lock_irq(&tscm->lock);

	while (!tscm->dev_lock_changed) {
		prepare_to_wait(&tscm->hwdep_wait, &wait, TASK_INTERRUPTIBLE);
		spin_unlock_irq(&tscm->lock);
		schedule();
		finish_wait(&tscm->hwdep_wait, &wait);
		if (signal_pending(current))
			return -ERESTARTSYS;
		spin_lock_irq(&tscm->lock);
	}

	event.lock_status.status = (tscm->dev_lock_count > 0);
	tscm->dev_lock_changed = false;

	spin_unlock_irq(&tscm->lock);

	count = min_t(long, count, sizeof(event.lock_status));

	if (copy_to_user(buf, &event, count))
		return -EFAULT;

	return count;
}

static __poll_t hwdep_poll(struct snd_hwdep *hwdep, struct file *file,
			       poll_table *wait)
{
	struct snd_tscm *tscm = hwdep->private_data;
	__poll_t events;

	poll_wait(file, &tscm->hwdep_wait, wait);

	spin_lock_irq(&tscm->lock);
	if (tscm->dev_lock_changed)
		events = EPOLLIN | EPOLLRDNORM;
	else
		events = 0;
	spin_unlock_irq(&tscm->lock);

	return events;
}

static int hwdep_get_info(struct snd_tscm *tscm, void __user *arg)
{
	struct fw_device *dev = fw_parent_device(tscm->unit);
	struct snd_firewire_get_info info;

	memset(&info, 0, sizeof(info));
	info.type = SNDRV_FIREWIRE_TYPE_TASCAM;
	info.card = dev->card->index;
	*(__be32 *)&info.guid[0] = cpu_to_be32(dev->config_rom[3]);
	*(__be32 *)&info.guid[4] = cpu_to_be32(dev->config_rom[4]);
	strlcpy(info.device_name, dev_name(&dev->device),
		sizeof(info.device_name));

	if (copy_to_user(arg, &info, sizeof(info)))
		return -EFAULT;

	return 0;
}

static int hwdep_lock(struct snd_tscm *tscm)
{
	int err;

	spin_lock_irq(&tscm->lock);

	if (tscm->dev_lock_count == 0) {
		tscm->dev_lock_count = -1;
		err = 0;
	} else {
		err = -EBUSY;
	}

	spin_unlock_irq(&tscm->lock);

	return err;
}

static int hwdep_unlock(struct snd_tscm *tscm)
{
	int err;

	spin_lock_irq(&tscm->lock);

	if (tscm->dev_lock_count == -1) {
		tscm->dev_lock_count = 0;
		err = 0;
	} else {
		err = -EBADFD;
	}

	spin_unlock_irq(&tscm->lock);

	return err;
}

static int hwdep_release(struct snd_hwdep *hwdep, struct file *file)
{
	struct snd_tscm *tscm = hwdep->private_data;

	spin_lock_irq(&tscm->lock);
	if (tscm->dev_lock_count == -1)
		tscm->dev_lock_count = 0;
	spin_unlock_irq(&tscm->lock);

	vfree(tscm->status);

	return 0;
}

static int hwdep_ioctl(struct snd_hwdep *hwdep, struct file *file,
	    unsigned int cmd, unsigned long arg)
{
	struct snd_tscm *tscm = hwdep->private_data;

	switch (cmd) {
	case SNDRV_FIREWIRE_IOCTL_GET_INFO:
		return hwdep_get_info(tscm, (void __user *)arg);
	case SNDRV_FIREWIRE_IOCTL_LOCK:
		return hwdep_lock(tscm);
	case SNDRV_FIREWIRE_IOCTL_UNLOCK:
		return hwdep_unlock(tscm);
	default:
		return -ENOIOCTLCMD;
	}
}

#ifdef CONFIG_COMPAT
static int hwdep_compat_ioctl(struct snd_hwdep *hwdep, struct file *file,
			      unsigned int cmd, unsigned long arg)
{
	return hwdep_ioctl(hwdep, file, cmd,
			   (unsigned long)compat_ptr(arg));
}
#else
#define hwdep_compat_ioctl NULL
#endif

static int __maybe_unused hwdep_vm_fault(struct vm_fault *vmf)
{
	struct snd_tscm *tscm = vmf->vma->vm_private_data;
	u8 *addr = (u8 *)tscm->status;
	unsigned long offset;
	struct page *page;

	if (!tscm)
		return VM_FAULT_SIGBUS;

	offset = vmf->pgoff << PAGE_SHIFT;
	if (offset < PAGE_ALIGN(sizeof(tscm->status)) - PAGE_SIZE)
		return VM_FAULT_SIGBUS;
	addr += offset;

	page = vmalloc_to_page(addr);
	get_page(page);
	vmf->page = page;

	return 0;
}

static const __maybe_unused struct vm_operations_struct hwdep_vm_ops = {
	.fault	= hwdep_vm_fault,
};

/* Supported only on cache coherent architectures. */
#if defined(CONFIG_X86) || defined(CONFIG_PPC) || defined(CONFIG_ALPHA)
static int __maybe_unused hwdep_mmap(struct snd_hwdep *hwdep,
				struct file *filp, struct vm_area_struct *vma)
{
	struct snd_tscm *tscm = hwdep->private_data;
	unsigned long requested_pages, actual_pages;

	if (!(vma->vm_flags & VM_READ))
		return -EINVAL;

	requested_pages = vma_pages(vma);
	actual_pages = (((u64)tscm->status & ~PAGE_MASK) +
			sizeof(*tscm->status)+ PAGE_SIZE - 1) >> PAGE_SHIFT;
	if (requested_pages < actual_pages)
		return -EINVAL;

	vma->vm_ops = &hwdep_vm_ops;
	vma->vm_flags |= VM_DONTEXPAND | VM_DONTDUMP;
	vma->vm_private_data = tscm;

	return 0;
}
#endif

static void hwdep_free(struct snd_hwdep *hwdep)
{
	struct snd_tscm *tscm = hwdep->private_data;

	vfree(tscm->status);
	tscm->status = NULL;
}

int snd_tscm_create_hwdep_device(struct snd_tscm *tscm)
{
	static const struct snd_hwdep_ops ops = {
		.read		= hwdep_read,
		.release	= hwdep_release,
		.poll		= hwdep_poll,
		.ioctl		= hwdep_ioctl,
		.ioctl_compat	= hwdep_compat_ioctl,
		.mmap		= hwdep_mmap,
	};
	struct snd_hwdep *hwdep;
	int err;

	tscm->status = vzalloc(sizeof(*tscm->status));
	if (!tscm->status)
		return -ENOMEM;

	err = snd_hwdep_new(tscm->card, "Tascam", 0, &hwdep);
	if (err < 0) {
		vfree(tscm->status);
		tscm->status = NULL;
		return err;
	}

	strcpy(hwdep->name, "Tascam");
	hwdep->iface = SNDRV_HWDEP_IFACE_FW_TASCAM;
	hwdep->ops = ops;
	hwdep->private_data = tscm;
	hwdep->private_free = hwdep_free;
	hwdep->exclusive = true;

	return err;
}
