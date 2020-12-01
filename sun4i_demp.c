// SPDX-License-Identifier: GPL-2.0+

/*
 * Copyright (c) 2020 Luc Verhaegen <libv@skynet.be>
 */

#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>

#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-mem2mem.h>
#include <media/videobuf2-dma-contig.h>

#define MODULE_NAME	"sun4i-demp"

struct demp {
	struct device *dev;

	struct mutex mutex[1];

	struct v4l2_device v4l2_dev[1];
	struct video_device slashdev[1];

	struct v4l2_m2m_dev *m2m_dev;
};

struct demp_context {
	struct v4l2_fh fh[1];
	struct demp *demp;
};

static int demp_vb2_queue_setup(struct vb2_queue *vb2_queue,
				unsigned int *buffer_count,
				unsigned int *plane_count,
				unsigned int sizes[],
				struct device *alloc_devs[])
{
	struct demp_context *context = vb2_get_drv_priv(vb2_queue);
	struct demp *demp = context->demp;
	enum v4l2_buf_type type = vb2_queue->type;

	if (type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		dev_info(demp->dev, "%s(input);\n", __func__);
	} else if (type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
		dev_info(demp->dev, "%s(output);\n", __func__);
	} else {
		dev_err(demp->dev, "%s(): wrong type %d\n", __func__, type);
		return -EINVAL;
	}

	return 0;
}

static int demp_vb2_buffer_prepare(struct vb2_buffer *vb2_buffer)
{
	struct demp_context *context = vb2_get_drv_priv(vb2_buffer->vb2_queue);
	struct demp *demp = context->demp;
	enum v4l2_buf_type type = vb2_buffer->vb2_queue->type;

	if (type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		dev_info(demp->dev, "%s(input);\n", __func__);
	} else if (type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
		dev_info(demp->dev, "%s(output);\n", __func__);
	} else {
		dev_err(demp->dev, "%s(): wrong type %d\n", __func__, type);
		return -EINVAL;
	}

	return 0;
}

static void demp_vb2_buffer_queue(struct vb2_buffer *vb2_buffer)
{
	struct demp_context *context = vb2_get_drv_priv(vb2_buffer->vb2_queue);
	struct vb2_v4l2_buffer *v4l2_buffer = to_vb2_v4l2_buffer(vb2_buffer);

	dev_info(context->demp->dev, "%s();\n", __func__);

	v4l2_m2m_buf_queue(context->fh->m2m_ctx, v4l2_buffer);
}

static const struct vb2_ops demp_vb2_queue_ops = {
	.queue_setup = demp_vb2_queue_setup,
	.buf_prepare = demp_vb2_buffer_prepare,
	.buf_queue = demp_vb2_buffer_queue,
	.wait_prepare = vb2_ops_wait_prepare,
	.wait_finish = vb2_ops_wait_finish,
};

static int
demp_vb2_queue_init(void *priv,
		    struct vb2_queue *source, struct vb2_queue *dest)
{
	struct demp_context *context = priv;
	struct demp *demp = context->demp;
	int ret;

	dev_info(demp->dev, "%s();\n", __func__);

	source->type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
	source->io_modes = VB2_MMAP | VB2_DMABUF;
	source->drv_priv = context;
	source->buf_struct_size = sizeof(struct v4l2_m2m_buffer);
	source->min_buffers_needed = 1;
	source->ops = &demp_vb2_queue_ops;
	source->mem_ops = &vb2_dma_contig_memops;
	source->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	source->lock = demp->mutex;
	source->dev = demp->dev;

	ret = vb2_queue_init(source);
	if (ret) {
		dev_err(demp->dev, "Error: %s(): vb2_queue_init(source): %d\n",
			__func__, ret);
		return ret;
	}

	dest->type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	dest->io_modes = VB2_MMAP | VB2_DMABUF;
	dest->drv_priv = context;
	dest->buf_struct_size = sizeof(struct v4l2_m2m_buffer);
	dest->min_buffers_needed = 1;
	dest->ops = &demp_vb2_queue_ops;
	dest->mem_ops = &vb2_dma_contig_memops;
	dest->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	dest->lock = demp->mutex;
	dest->dev = demp->dev;

	ret = vb2_queue_init(dest);
	if (ret) {
		dev_err(demp->dev, "Error: %s(): vb2_queue_init(dest): %d\n",
			__func__, ret);
		return ret;
	}

	return 0;
}

static int demp_v4l2_fop_open(struct file *file)
{
	struct demp *demp = video_drvdata(file);
	struct video_device *vdev = video_devdata(file);
	struct demp_context *context = NULL;
	struct v4l2_fh *fh = NULL;
	int ret;

	dev_info(demp->dev, "%s();\n", __func__);

	if (mutex_lock_interruptible(demp->mutex))
		return -ERESTARTSYS;

	context = kzalloc(sizeof(struct demp_context), GFP_KERNEL);
	if (!context) {
		ret = -ENOMEM;
		goto error;
	}

	file->private_data = context;
	context->demp = demp;

	fh = context->fh;
	v4l2_fh_init(fh, vdev);

	fh->m2m_ctx = v4l2_m2m_ctx_init(demp->m2m_dev, context,
					&demp_vb2_queue_init);
	if (IS_ERR(fh->m2m_ctx)) {
		ret = PTR_ERR(fh->m2m_ctx);
		dev_err(demp->dev, "%s(): v4l2_m2m_ctx_init(): %d.\n",
			__func__, ret);
		goto error;
	}

	v4l2_fh_add(fh);

	mutex_unlock(demp->mutex);
	return 0;
 error:
	v4l2_fh_exit(fh);
	kfree(context);
	mutex_unlock(demp->mutex);
	return ret;
}

static int demp_v4l2_fop_release(struct file *file)
{
	struct demp *demp = video_drvdata(file);
	struct demp_context *context = file->private_data;
	struct v4l2_fh *fh = context->fh;

	dev_info(demp->dev, "%s();\n", __func__);

	if (!context)
		return 0;

	mutex_lock(demp->mutex);

	v4l2_m2m_ctx_release(fh->m2m_ctx);
	v4l2_fh_del(fh);
	v4l2_fh_exit(fh);
	kfree(context);

	mutex_unlock(demp->mutex);

	return 0;
}

static const struct v4l2_file_operations demp_slashdev_fops = {
	.owner = THIS_MODULE,
	.open = demp_v4l2_fop_open,
	.release = demp_v4l2_fop_release,
	.unlocked_ioctl = video_ioctl2,
	.mmap = v4l2_m2m_fop_mmap,
	.poll = v4l2_m2m_fop_poll,
};

static void demp_m2m_device_run(void *priv)
{
	struct demp_context *context = priv;
	struct demp *demp = context->demp;

	dev_info(demp->dev, "%s();\n", __func__);

	/*
	 * Since we do not do any real work yet, act as if we did immediately.
	 */
	v4l2_m2m_buf_done_and_job_finish(demp->m2m_dev, context->fh->m2m_ctx,
					 VB2_BUF_STATE_DONE);
}

/*
 * We need to provide this so that we do not go into wait_event() forever
 * when closing our fd, which turns our userspace into a sleepwalking zombie.
 * This wait_event() happens in v4l2_m2m_cancel_job() called from
 * v4l2_m2m_ctx_release() from our fop_release()).
 * Perhaps this call should be made mandatory.
 */
static void demp_m2m_job_abort(void *priv)
{
	struct demp_context *context = priv;
	struct demp *demp = context->demp;

	dev_info(demp->dev, "%s();\n", __func__);

	/*
	 * TODO: Figure out whether the engine is busy first, and then return
	 * either DONE or ERROR.
	 */
	v4l2_m2m_buf_done_and_job_finish(demp->m2m_dev, context->fh->m2m_ctx,
					 VB2_BUF_STATE_ERROR);
}

static const struct v4l2_m2m_ops demp_m2m_ops = {
	.device_run = demp_m2m_device_run,
	.job_abort = demp_m2m_job_abort,
};

static int demp_ioctl_capability_query(struct file *file, void *handle,
				       struct v4l2_capability *capability)
{
	struct demp *demp = video_drvdata(file);

	dev_info(demp->dev, "%s();\n", __func__);

	strscpy(capability->driver, "sun4i_demp", sizeof(capability->driver));
	strscpy(capability->card, demp->slashdev->name,
		sizeof(capability->card));

	snprintf(capability->bus_info, sizeof(capability->bus_info),
		 "platform:%s", demp->dev->of_node->name);

	return 0;
}

static int demp_ioctl_format_input_enumerate(struct file *file,
					     void *handle,
					     struct v4l2_fmtdesc *descriptor)
{
	struct demp *demp = video_drvdata(file);

	dev_info(demp->dev, "%s();\n", __func__);

	return 0;
}

static int demp_ioctl_format_input_get(struct file *file, void *handle,
				       struct v4l2_format *format)
{
	struct demp *demp = video_drvdata(file);

	dev_info(demp->dev, "%s();\n", __func__);

	return 0;
}

static int demp_ioctl_format_input_set(struct file *file, void *handle,
				       struct v4l2_format *format_new)
{
	struct demp *demp = video_drvdata(file);

	dev_info(demp->dev, "%s();\n", __func__);

	return 0;
}

static int demp_ioctl_format_input_try(struct file *file, void *handle,
				       struct v4l2_format *format_try)
{
	struct demp *demp = video_drvdata(file);

	dev_info(demp->dev, "%s();\n", __func__);

	return 0;
}

static int demp_ioctl_format_output_enumerate(struct file *file,
					      void *handle,
					      struct v4l2_fmtdesc *descriptor)
{
	struct demp *demp = video_drvdata(file);

	dev_info(demp->dev, "%s();\n", __func__);

	return 0;
}

static int demp_ioctl_format_output_get(struct file *file, void *handle,
					struct v4l2_format *format)
{
	struct demp *demp = video_drvdata(file);

	dev_info(demp->dev, "%s();\n", __func__);

	return 0;
}

static int demp_ioctl_format_output_set(struct file *file, void *handle,
					struct v4l2_format *format_new)
{
	struct demp *demp = video_drvdata(file);

	dev_info(demp->dev, "%s();\n", __func__);

	return 0;
}

static int demp_ioctl_format_output_try(struct file *file, void *handle,
					struct v4l2_format *format_try)
{
	struct demp *demp = video_drvdata(file);

	dev_info(demp->dev, "%s();\n", __func__);

	return 0;
}

static const struct v4l2_ioctl_ops demp_ioctl_ops = {
	.vidioc_querycap = demp_ioctl_capability_query,

	.vidioc_enum_fmt_vid_cap = demp_ioctl_format_output_enumerate,
	.vidioc_g_fmt_vid_cap_mplane = demp_ioctl_format_output_get,
	.vidioc_s_fmt_vid_cap_mplane =  demp_ioctl_format_output_set,
	.vidioc_try_fmt_vid_cap_mplane = demp_ioctl_format_output_try,

	.vidioc_enum_fmt_vid_out = demp_ioctl_format_input_enumerate,
	.vidioc_g_fmt_vid_out_mplane = demp_ioctl_format_input_get,
	.vidioc_s_fmt_vid_out_mplane =  demp_ioctl_format_input_set,
	.vidioc_try_fmt_vid_out_mplane = demp_ioctl_format_input_try,

	.vidioc_reqbufs	= v4l2_m2m_ioctl_reqbufs,
	.vidioc_querybuf = v4l2_m2m_ioctl_querybuf,
	.vidioc_qbuf = v4l2_m2m_ioctl_qbuf,
	.vidioc_dqbuf = v4l2_m2m_ioctl_dqbuf,
	.vidioc_prepare_buf = v4l2_m2m_ioctl_prepare_buf,
	.vidioc_create_bufs = v4l2_m2m_ioctl_create_bufs,
	.vidioc_expbuf = v4l2_m2m_ioctl_expbuf,

	.vidioc_streamon = v4l2_m2m_ioctl_streamon,
	.vidioc_streamoff = v4l2_m2m_ioctl_streamoff,
};

static int demp_slashdev_initialize(struct demp *demp)
{
	struct video_device *slashdev = demp->slashdev;
	int ret;

	video_set_drvdata(slashdev, demp);

	strscpy(slashdev->name, KBUILD_MODNAME, sizeof(slashdev->name));

	slashdev->vfl_type = VFL_TYPE_VIDEO;
	slashdev->vfl_dir = VFL_DIR_M2M;
	slashdev->device_caps =
		V4L2_CAP_STREAMING | V4L2_CAP_VIDEO_M2M_MPLANE;

	slashdev->v4l2_dev = demp->v4l2_dev;

	slashdev->release = video_device_release_empty;
	slashdev->fops = &demp_slashdev_fops;
	slashdev->ioctl_ops = &demp_ioctl_ops;

	slashdev->lock = demp->mutex;

	ret = video_register_device(slashdev, VFL_TYPE_VIDEO, -1);
	if (ret) {
		dev_err(demp->dev, "%s():video_register_device(): %d\n",
			__func__, ret);
		return ret;
	}

	return 0;
}

static void demp_slashdev_free(struct demp *demp)
{
	video_unregister_device(demp->slashdev);
}

static int demp_v4l2_initialize(struct demp *demp)
{
	struct device *dev = demp->dev;
	struct v4l2_m2m_dev *m2m_dev;
	int ret;

	dev_info(dev, "%s();\n", __func__);

	ret = v4l2_device_register(dev, demp->v4l2_dev);
	if (ret) {
		dev_err(dev, "%s(): v4l2_device_register() failed: %d.\n",
			__func__, ret);
		return ret;
	}

	ret = demp_slashdev_initialize(demp);
	if (ret)
		goto error;

	m2m_dev = v4l2_m2m_init(&demp_m2m_ops);
	if (IS_ERR(m2m_dev)) {
		ret = PTR_ERR(m2m_dev);
		dev_err(demp->dev, "%s():v4l2_m2m_init(): %d\n",
			__func__, ret);
		goto error;
	}
	demp->m2m_dev = m2m_dev;

	return 0;

 error:
	demp_slashdev_free(demp);
	v4l2_device_unregister(demp->v4l2_dev);
	return ret;
}

static int demp_v4l2_cleanup(struct demp *demp)
{
	struct device *dev = demp->dev;

	dev_info(dev, "%s();\n", __func__);

	v4l2_m2m_release(demp->m2m_dev);
	demp_slashdev_free(demp);
	v4l2_device_unregister(demp->v4l2_dev);

	return 0;
}

static int demp_probe(struct platform_device *platform_dev)
{
	struct device *dev = &platform_dev->dev;
	struct demp *demp;
	int ret;

	dev_info(dev, "%s();\n", __func__);

	demp = devm_kzalloc(dev, sizeof(struct demp), GFP_KERNEL);
	if (!demp)
		return -ENOMEM;
	demp->dev = dev;

	mutex_init(demp->mutex);

	platform_set_drvdata(platform_dev, demp);

	ret = demp_v4l2_initialize(demp);
	if (ret)
		return ret;

	return 0;
}

static int demp_remove(struct platform_device *platform_dev)
{
	struct device *dev = &platform_dev->dev;
	struct demp *demp = platform_get_drvdata(platform_dev);
	int ret;

	dev_info(dev, "%s();\n", __func__);

	ret = demp_v4l2_cleanup(demp);
	if (ret)
		return ret;

	return 0;
}

/* We are currently only testing on sun7i, but should work for sun4i as well */
static const struct of_device_id demp_of_match[] = {
	{ .compatible = "allwinner,sun4i-a10-demp", },
	{ .compatible = "allwinner,sun7i-a20-demp", },
	{},
};
MODULE_DEVICE_TABLE(of, demp_of_match);

static struct platform_driver demp_platform_driver = {
	.probe = demp_probe,
	.remove = demp_remove,
	.driver = {
		.name = MODULE_NAME,
		.of_match_table = of_match_ptr(demp_of_match),
	},
};

module_platform_driver(demp_platform_driver);
MODULE_DESCRIPTION("Allwinner A10/A20 Mixer Processor V4L2 driver");
MODULE_AUTHOR("Luc Verhaegen <libv@skynet.be>");
MODULE_LICENSE("GPL v2");
