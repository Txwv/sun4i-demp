// SPDX-License-Identifier: GPL-2.0+

/*
 * Copyright (c) 2020 Luc Verhaegen <libv@skynet.be>
 */

#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/reset.h>
#include <linux/io.h>
#include <linux/interrupt.h>

#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-mem2mem.h>
#include <media/videobuf2-dma-contig.h>

#include "sun4i_demp_reg.h"

#define MODULE_NAME	"sun4i-demp"

#define DEMP_DEFAULT_WIDTH 640
#define DEMP_DEFAULT_HEIGHT 480

struct demp {
	struct device *dev;

	struct clk *clk_bus;
	struct clk *clk_ram;
	struct clk *clk_de;
	struct reset_control *reset;

	void __iomem *mmio;
	struct spinlock io_lock[1];

	int irq;

	int usage_count;
	struct mutex mutex[1];

	struct v4l2_device v4l2_dev[1];
	struct video_device slashdev[1];

	struct v4l2_m2m_dev *m2m_dev;
};

struct demp_context {
	struct v4l2_fh fh[1];
	struct demp *demp;

	struct v4l2_pix_format_mplane format_input[1];
	struct v4l2_pix_format_mplane format_output[1];
};

static void __maybe_unused demp_reg_write(struct demp *demp,
					  int address, uint32_t value)
{
	writel(value, demp->mmio + address);
}

static void __maybe_unused demp_reg_write_spin(struct demp *demp,
					       int address, uint32_t value)
{
	unsigned long flags;

	spin_lock_irqsave(demp->io_lock, flags);

	writel(value, demp->mmio + address);

	spin_unlock_irqrestore(demp->io_lock, flags);
}

static uint32_t __maybe_unused demp_reg_read(struct demp *demp,
					     int address)
{
	return readl(demp->mmio + address);
}

static uint32_t __maybe_unused demp_reg_read_spin(struct demp *demp,
						  int address)
{
	unsigned long flags;
	uint32_t ret;

	spin_lock_irqsave(demp->io_lock, flags);

	ret = readl(demp->mmio + address);

	spin_unlock_irqrestore(demp->io_lock, flags);

	return ret;
}

static void __maybe_unused demp_reg_mask(struct demp *demp, int address,
					 uint32_t value, uint32_t mask)
{
	uint32_t temp = readl(demp->mmio + address);

	temp &= ~mask;
	value &= mask;

	writel(value | temp, demp->mmio + address);
}

static void __maybe_unused demp_reg_mask_spin(struct demp *demp, int address,
					      uint32_t value, uint32_t mask)
{
	unsigned long flags;
	uint32_t temp;

	spin_lock_irqsave(demp->io_lock, flags);

	temp = readl(demp->mmio + address);

	temp &= ~mask;
	value &= mask;

	writel(value | temp, demp->mmio + address);

	spin_unlock_irqrestore(demp->io_lock, flags);
}

static void __maybe_unused demp_registers_print(struct demp *demp)
{
	unsigned long flags;

	spin_lock_irqsave(demp->io_lock, flags);

	pr_info("DEMP_REG_CONTROL: 0x%08X\n",
		demp_reg_read(demp, DEMP_REG_CONTROL));

	pr_info("DEMP_REG_STATUS: 0x%08X\n",
		demp_reg_read(demp, DEMP_REG_STATUS));

	pr_info("DEMP_REG_IDMA_GLOBAL_CONTROL: 0x%08X\n",
		demp_reg_read(demp, DEMP_REG_IDMA_GLOBAL_CONTROL));
	pr_info("DEMP_REG_IDMA_ADDRESS_HIGH: 0x%08X\n",
		demp_reg_read(demp, DEMP_REG_IDMA_ADDRESS_HIGH));
	pr_info("DEMP_REG_IDMA0_ADDRESS_LOW: 0x%08X\n",
		demp_reg_read(demp, DEMP_REG_IDMA0_ADDRESS_LOW));
	pr_info("DEMP_REG_IDMA1_ADDRESS_LOW: 0x%08X\n",
		demp_reg_read(demp, DEMP_REG_IDMA1_ADDRESS_LOW));
	pr_info("DEMP_REG_IDMA2_ADDRESS_LOW: 0x%08X\n",
		demp_reg_read(demp, DEMP_REG_IDMA2_ADDRESS_LOW));
	pr_info("DEMP_REG_IDMA3_ADDRESS_LOW: 0x%08X\n",
		demp_reg_read(demp, DEMP_REG_IDMA3_ADDRESS_LOW));
	pr_info("DEMP_REG_IDMA0_PITCH: 0x%08X\n",
		demp_reg_read(demp, DEMP_REG_IDMA0_PITCH));
	pr_info("DEMP_REG_IDMA1_PITCH: 0x%08X\n",
		demp_reg_read(demp, DEMP_REG_IDMA1_PITCH));
	pr_info("DEMP_REG_IDMA2_PITCH: 0x%08X\n",
		demp_reg_read(demp, DEMP_REG_IDMA2_PITCH));
	pr_info("DEMP_REG_IDMA3_PITCH: 0x%08X\n",
		demp_reg_read(demp, DEMP_REG_IDMA3_PITCH));
	pr_info("DEMP_REG_IDMA0_SIZE: 0x%08X\n",
		demp_reg_read(demp, DEMP_REG_IDMA0_SIZE));
	pr_info("DEMP_REG_IDMA1_SIZE: 0x%08X\n",
		demp_reg_read(demp, DEMP_REG_IDMA1_SIZE));
	pr_info("DEMP_REG_IDMA2_SIZE: 0x%08X\n",
		demp_reg_read(demp, DEMP_REG_IDMA2_SIZE));
	pr_info("DEMP_REG_IDMA3_SIZE: 0x%08X\n",
		demp_reg_read(demp, DEMP_REG_IDMA3_SIZE));
	pr_info("DEMP_REG_IDMA0_COORD: 0x%08X\n",
		demp_reg_read(demp, DEMP_REG_IDMA0_COORD));
	pr_info("DEMP_REG_IDMA1_COORD: 0x%08X\n",
		demp_reg_read(demp, DEMP_REG_IDMA1_COORD));
	pr_info("DEMP_REG_IDMA2_COORD: 0x%08X\n",
		demp_reg_read(demp, DEMP_REG_IDMA2_COORD));
	pr_info("DEMP_REG_IDMA3_COORD: 0x%08X\n",
		demp_reg_read(demp, DEMP_REG_IDMA3_COORD));
	pr_info("DEMP_REG_IDMA0_CONTROL: 0x%08X\n",
		demp_reg_read(demp, DEMP_REG_IDMA0_CONTROL));
	pr_info("DEMP_REG_IDMA1_CONTROL: 0x%08X\n",
		demp_reg_read(demp, DEMP_REG_IDMA1_CONTROL));
	pr_info("DEMP_REG_IDMA2_CONTROL: 0x%08X\n",
		demp_reg_read(demp, DEMP_REG_IDMA2_CONTROL));
	pr_info("DEMP_REG_IDMA3_CONTROL: 0x%08X\n",
		demp_reg_read(demp, DEMP_REG_IDMA3_CONTROL));
	pr_info("DEMP_REG_IDMA0_FILLCOLOR: 0x%08X\n",
		demp_reg_read(demp, DEMP_REG_IDMA0_FILLCOLOR));
	pr_info("DEMP_REG_IDMA_SORT: 0x%08X\n",
		demp_reg_read(demp, DEMP_REG_IDMA_SORT));

	pr_info("DEMP_REG_CSC0_CONTROL: 0x%08X\n",
		demp_reg_read(demp, DEMP_REG_CSC0_CONTROL));
	pr_info("DEMP_REG_CSC1_CONTROL: 0x%08X\n",
		demp_reg_read(demp, DEMP_REG_CSC1_CONTROL));
	pr_info("DEMP_REG_SCALE_CONTROL: 0x%08X\n",
		demp_reg_read(demp, DEMP_REG_SCALE_CONTROL));

	pr_info("DEMP_REG_ROP_CONTROL: 0x%08X\n",
		demp_reg_read(demp, DEMP_REG_ROP_CONTROL));
	pr_info("DEMP_REG_ROP_CH3_INDEX0_CONTROL: 0x%08X\n",
		demp_reg_read(demp, DEMP_REG_ROP_CH3_INDEX0_CONTROL));
	pr_info("DEMP_REG_ROP_CH3_INDEX1_CONTROL: 0x%08X\n",
		demp_reg_read(demp, DEMP_REG_ROP_CH3_INDEX1_CONTROL));
	pr_info("DEMP_REG_ALPHA_COLORKEY: 0x%08X\n",
		demp_reg_read(demp, DEMP_REG_ALPHA_COLORKEY));
	pr_info("DEMP_REG_COLORKEY_MIN: 0x%08X\n",
		demp_reg_read(demp, DEMP_REG_COLORKEY_MIN));
	pr_info("DEMP_REG_COLORKEY_MAX: 0x%08X\n",
		demp_reg_read(demp, DEMP_REG_COLORKEY_MAX));
	pr_info("DEMP_REG_ROP_OUT_FILLCOLOR: 0x%08X\n",
		demp_reg_read(demp, DEMP_REG_ROP_OUT_FILLCOLOR));

	pr_info("DEMP_REG_CSC2_CONTROL: 0x%08X\n",
		demp_reg_read(demp, DEMP_REG_CSC2_CONTROL));

	pr_info("DEMP_REG_OUTPUT_CONTROL: 0x%08X\n",
		demp_reg_read(demp, DEMP_REG_OUTPUT_CONTROL));
	pr_info("DEMP_REG_OUTPUT_SIZE: 0x%08X\n",
		demp_reg_read(demp, DEMP_REG_OUTPUT_SIZE));
	pr_info("DEMP_REG_OUTPUT_ADDRESS_HIGH: 0x%08X\n",
		demp_reg_read(demp, DEMP_REG_OUTPUT_ADDRESS_HIGH));
	pr_info("DEMP_REG_OUTPUT_ADDRESS_CH0: 0x%08X\n",
		demp_reg_read(demp, DEMP_REG_OUTPUT_ADDRESS_CH0));
	pr_info("DEMP_REG_OUTPUT_ADDRESS_CH1: 0x%08X\n",
		demp_reg_read(demp, DEMP_REG_OUTPUT_ADDRESS_CH1));
	pr_info("DEMP_REG_OUTPUT_ADDRESS_CH2: 0x%08X\n",
		demp_reg_read(demp, DEMP_REG_OUTPUT_ADDRESS_CH2));
	pr_info("DEMP_REG_OUTPUT_PITCH_CH0: 0x%08X\n",
		demp_reg_read(demp, DEMP_REG_OUTPUT_PITCH_CH0));
	pr_info("DEMP_REG_OUTPUT_PITCH_CH1: 0x%08X\n",
		demp_reg_read(demp, DEMP_REG_OUTPUT_PITCH_CH1));
	pr_info("DEMP_REG_OUTPUT_PITCH_CH2: 0x%08X\n",
		demp_reg_read(demp, DEMP_REG_OUTPUT_PITCH_CH2));
	pr_info("DEMP_REG_OUTPUT_ALPHA: 0x%08X\n",
		demp_reg_read(demp, DEMP_REG_OUTPUT_ALPHA));

	pr_info("DEMP_REG_OUTPUT_CSC_YG_GY_COEFF: 0x%08X\n",
		demp_reg_read(demp, DEMP_REG_OUTPUT_CSC_YG_GY_COEFF));
	pr_info("DEMP_REG_OUTPUT_CSC_YG_RU_COEFF: 0x%08X\n",
		demp_reg_read(demp, DEMP_REG_OUTPUT_CSC_YG_RU_COEFF));
	pr_info("DEMP_REG_OUTPUT_CSC_YG_BV_COEFF: 0x%08X\n",
		demp_reg_read(demp, DEMP_REG_OUTPUT_CSC_YG_BV_COEFF));
	pr_info("DEMP_REG_OUTPUT_CSC_YG_CONSTANT: 0x%08X\n",
		demp_reg_read(demp, DEMP_REG_OUTPUT_CSC_YG_CONSTANT));
	pr_info("DEMP_REG_OUTPUT_CSC_UR_GY_COEFF: 0x%08X\n",
		demp_reg_read(demp, DEMP_REG_OUTPUT_CSC_UR_GY_COEFF));
	pr_info("DEMP_REG_OUTPUT_CSC_UR_RU_COEFF: 0x%08X\n",
		demp_reg_read(demp, DEMP_REG_OUTPUT_CSC_UR_RU_COEFF));
	pr_info("DEMP_REG_OUTPUT_CSC_UR_BV_COEFF: 0x%08X\n",
		demp_reg_read(demp, DEMP_REG_OUTPUT_CSC_UR_BV_COEFF));
	pr_info("DEMP_REG_OUTPUT_CSC_UR_CONSTANT: 0x%08X\n",
		demp_reg_read(demp, DEMP_REG_OUTPUT_CSC_UR_CONSTANT));
	pr_info("DEMP_REG_OUTPUT_CSC_VB_GY_COEFF: 0x%08X\n",
		demp_reg_read(demp, DEMP_REG_OUTPUT_CSC_VB_GY_COEFF));
	pr_info("DEMP_REG_OUTPUT_CSC_VB_RU_COEFF: 0x%08X\n",
		demp_reg_read(demp, DEMP_REG_OUTPUT_CSC_VB_RU_COEFF));
	pr_info("DEMP_REG_OUTPUT_CSC_VB_BV_COEFF: 0x%08X\n",
		demp_reg_read(demp, DEMP_REG_OUTPUT_CSC_VB_BV_COEFF));
	pr_info("DEMP_REG_OUTPUT_CSC_VB_CONSTANT: 0x%08X\n",
		demp_reg_read(demp, DEMP_REG_OUTPUT_CSC_VB_CONSTANT));

	spin_unlock_irqrestore(demp->io_lock, flags);
}

static irqreturn_t demp_isr(int irq, void *dev_id)
{
	struct demp *demp = (struct demp *) dev_id;
	struct demp_context *context = v4l2_m2m_get_curr_priv(demp->m2m_dev);
	enum vb2_buffer_state state;
	uint32_t value;

	if (!context) {
		dev_err(demp->dev, "%s(): no context!\n", __func__);
		return IRQ_NONE;
	}

	spin_lock(demp->io_lock);

	value = demp_reg_read(demp, DEMP_REG_STATUS);

	/* ack. */
	demp_reg_write(demp, DEMP_REG_STATUS, 0x300);
	/*
	 * some brokenness it seems, it needs this to reinit future
	 * interrupts.
	 */
	demp_reg_write(demp, DEMP_REG_CONTROL, 0);

	spin_unlock(demp->io_lock);

	if (value & 0x100) /* finished. */
		state = VB2_BUF_STATE_DONE;
	else if (value & 0x200) /* error */
		state = VB2_BUF_STATE_ERROR;
	else {
		dev_err(demp->dev, "%s(): invalid value: 0x%08X\n",
			__func__, value);
		return IRQ_NONE;
	}

	v4l2_m2m_buf_done_and_job_finish(demp->m2m_dev, context->fh->m2m_ctx,
					 state);
	return IRQ_HANDLED;
}

static int demp_poweron(struct demp *demp)
{
	struct device *dev = demp->dev;
	int ret;

	dev_info(dev, "%s();\n", __func__);

	clk_set_rate(demp->clk_de, 300000000);

	ret = reset_control_deassert(demp->reset);
	if (ret) {
		dev_err(dev, "%s(): reset_control_deassert() failed: %d.\n",
			__func__, ret);
		goto err_reset;
	}

	ret = clk_prepare_enable(demp->clk_bus);
	if (ret) {
		dev_err(dev, "%s(): clk_prepare_enable(bus) failed: %d.\n",
			__func__, ret);
		goto err_bus;
	}

	ret = clk_prepare_enable(demp->clk_de);
	if (ret) {
		dev_err(dev, "%s(): clk_prepare_enable(de) failed: %d.\n",
			__func__, ret);
		goto err_de;
	}

	ret = clk_prepare_enable(demp->clk_ram);
	if (ret) {
		dev_err(dev, "%s(): clk_prepare_enable(ram) failed: %d.\n",
			__func__, ret);
		goto err_ram;
	}

	return 0;

 err_ram:
	clk_disable_unprepare(demp->clk_de);
 err_de:
	clk_disable_unprepare(demp->clk_bus);
 err_bus:
	reset_control_assert(demp->reset);
 err_reset:
	return 0;
}

/*
 * We do not bother with checking return values here, we are powering
 * down anyway.
 */
static int demp_poweroff(struct demp *demp)
{
	struct device *dev = demp->dev;

	dev_info(dev, "%s();\n", __func__);

	clk_disable_unprepare(demp->clk_ram);

	clk_disable_unprepare(demp->clk_de);

	clk_disable_unprepare(demp->clk_bus);

	reset_control_assert(demp->reset);

	return 0;
}

/*
 * We might want to power up/down depending on actual usage though.
 */
static int demp_resume(struct device *dev)
{
	struct demp *demp = dev_get_drvdata(dev);

	dev_info(dev, "%s();\n", __func__);

	if (!demp->usage_count)
		return 0;

	return demp_poweron(demp);
}

static int demp_suspend(struct device *dev)
{
	struct demp *demp = dev_get_drvdata(dev);

	dev_info(dev, "%s();\n", __func__);

	if (!demp->usage_count)
		return 0;

	return demp_poweroff(demp);
}

static const struct dev_pm_ops demp_pm_ops = {
	SET_RUNTIME_PM_OPS(demp_suspend, demp_resume, NULL)
};

static int demp_resources_get(struct demp *demp,
			      struct platform_device *platform_dev)
{
	struct device *dev = demp->dev;
	struct resource *resource;
	int irq, ret;

	dev_info(dev, "%s();\n", __func__);

	demp->clk_bus = devm_clk_get(dev, "bus");
	if (IS_ERR(demp->clk_bus)) {
		dev_err(dev, "%s(): devm_clk_get(bus) failed: %ld.\n",
			__func__, PTR_ERR(demp->clk_bus));
		return PTR_ERR(demp->clk_bus);
	}

	demp->clk_de = devm_clk_get(dev, "de");
	if (IS_ERR(demp->clk_de)) {
		dev_err(dev, "%s(): devm_clk_get(de) failed: %ld.\n",
			__func__, PTR_ERR(demp->clk_de));
		return PTR_ERR(demp->clk_de);
	}

	demp->clk_ram = devm_clk_get(dev, "ram");
	if (IS_ERR(demp->clk_ram)) {
		dev_err(dev, "%s(): devm_clk_get(ram) failed: %ld.\n",
			__func__, PTR_ERR(demp->clk_ram));
		return PTR_ERR(demp->clk_ram);
	}

	demp->reset = devm_reset_control_get(dev, NULL);
	if (IS_ERR(demp->reset)) {
		dev_err(dev, "%s(): devm_reset_control_get() failed: %ld.\n",
			__func__, PTR_ERR(demp->reset));
		return PTR_ERR(demp->reset);
	}

	resource = platform_get_resource(platform_dev, IORESOURCE_MEM, 0);
	if (!resource) {
		dev_err(dev, "%s(): platform_get_resource() failed.\n",
			__func__);
		return EINVAL;
	}

	demp->mmio = devm_ioremap_resource(dev, resource);
	if (IS_ERR(demp->mmio)) {
		dev_err(dev, "%s(): devm_ioremap_resource() failed: %ld.\n",
			__func__, PTR_ERR(demp->mmio));
		return PTR_ERR(demp->mmio);
	}

	irq = platform_get_irq(platform_dev, 0);
	if (irq < 0) {
		dev_err(dev, "%s(): platform_get_irq() failed: %d.\n",
			__func__, -irq);
		return -irq;
	}

	ret = devm_request_irq(dev, irq, demp_isr, 0, MODULE_NAME, demp);
	if (ret) {
		dev_err(dev, "%s(): devm_request_irq(\"%s\") failed: %d.\n",
			__func__, "demp", ret);
		return ret;
	}
	demp->irq = irq;

	return 0;
}

static int demp_vb2_queue_setup(struct vb2_queue *vb2_queue,
				unsigned int *buffer_count,
				unsigned int *plane_count,
				unsigned int sizes[],
				struct device *alloc_devs[])
{
	struct demp_context *context = vb2_get_drv_priv(vb2_queue);
	struct demp *demp = context->demp;
	enum v4l2_buf_type type = vb2_queue->type;
	struct v4l2_pix_format_mplane *format;
	int i;

	if (type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		dev_info(demp->dev, "%s(input);\n", __func__);

		format = context->format_input;
	} else if (type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
		dev_info(demp->dev, "%s(output);\n", __func__);

		format = context->format_output;
	} else {
		dev_err(demp->dev, "%s(): wrong type %d\n", __func__, type);
		return -EINVAL;
	}

	*plane_count = format->num_planes;
	for (i = 0; i < format->num_planes; i++)
		sizes[i] = format->plane_fmt[i].sizeimage;

	return 0;
}

static int demp_vb2_buffer_prepare(struct vb2_buffer *vb2_buffer)
{
	struct demp_context *context = vb2_get_drv_priv(vb2_buffer->vb2_queue);
	struct demp *demp = context->demp;
	enum v4l2_buf_type type = vb2_buffer->vb2_queue->type;
	struct v4l2_pix_format_mplane *format;
	int i;

	if (type == V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE) {
		dev_info(demp->dev, "%s(input);\n", __func__);

		format = context->format_input;
	} else if (type == V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE) {
		dev_info(demp->dev, "%s(output);\n", __func__);

		format = context->format_output;
	} else {
		dev_err(demp->dev, "%s(): wrong type %d\n", __func__, type);
		return -EINVAL;
	}

	for (i = 0; i < format->num_planes; i++)
		vb2_set_plane_payload(vb2_buffer, i,
				      format->plane_fmt[i].sizeimage);

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

static struct v4l2_pix_format_mplane format_default_rgba[1] = {{
	.width = DEMP_DEFAULT_WIDTH,
	.height = DEMP_DEFAULT_HEIGHT,
	.pixelformat = V4L2_PIX_FMT_RGBA32,
	.field = V4L2_FIELD_NONE,
	.colorspace = V4L2_COLORSPACE_SRGB,

	.num_planes = 1,
	.plane_fmt[0].sizeimage = DEMP_DEFAULT_WIDTH * DEMP_DEFAULT_HEIGHT * 4,
	.plane_fmt[0].bytesperline = DEMP_DEFAULT_WIDTH * 4,
}};

static struct v4l2_pix_format_mplane format_default_prgb[1] = {{
	.width = DEMP_DEFAULT_WIDTH,
	.height = DEMP_DEFAULT_HEIGHT,
	.pixelformat = V4L2_PIX_FMT_R8_G8_B8,
	.field = V4L2_FIELD_NONE,
	.colorspace = V4L2_COLORSPACE_SRGB,

	.num_planes = 3,
	.plane_fmt[0].sizeimage = DEMP_DEFAULT_WIDTH * DEMP_DEFAULT_HEIGHT,
	.plane_fmt[0].bytesperline = DEMP_DEFAULT_WIDTH,
	.plane_fmt[1].sizeimage = DEMP_DEFAULT_WIDTH * DEMP_DEFAULT_HEIGHT,
	.plane_fmt[1].bytesperline = DEMP_DEFAULT_WIDTH,
	.plane_fmt[2].sizeimage = DEMP_DEFAULT_WIDTH * DEMP_DEFAULT_HEIGHT,
	.plane_fmt[2].bytesperline = DEMP_DEFAULT_WIDTH,
}};

static struct v4l2_pix_format_mplane format_default_ayuv[1] = {{
	.width = DEMP_DEFAULT_WIDTH,
	.height = DEMP_DEFAULT_HEIGHT,
	.pixelformat = V4L2_PIX_FMT_AYUV32,
	.field = V4L2_FIELD_NONE,
	.colorspace = V4L2_COLORSPACE_REC709,

	.num_planes = 1,
	.plane_fmt[0].sizeimage = DEMP_DEFAULT_WIDTH * DEMP_DEFAULT_HEIGHT * 4,
	.plane_fmt[0].bytesperline = DEMP_DEFAULT_WIDTH * 4,
}};

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

	*context->format_input = format_default_rgba[0];
	*context->format_output = format_default_rgba[0];

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

	demp->usage_count++;
	if (demp->usage_count == 1) {
		ret = demp_poweron(demp);
		if (ret) {
			dev_err(demp->dev, "%s(): demp_poweron(): %d.\n",
				__func__, ret);
			goto error;
		}
	}

	mutex_unlock(demp->mutex);
	return 0;
 error:
	v4l2_fh_del(fh);
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

	demp->usage_count--;
	if (!demp->usage_count)
		demp_poweroff(demp);

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

static void demp_input_rgba(struct demp *demp, struct vb2_buffer *buffer,
			    struct v4l2_pix_format_mplane *format)
{
	dma_addr_t addr = vb2_dma_contig_plane_dma_addr(buffer, 0);
	uint16_t width = format->width - 1;
	uint16_t height = format->height - 1;
	uint32_t pitch = format->plane_fmt[0].bytesperline << 3;

	demp_reg_mask(demp, DEMP_REG_IDMA_ADDRESS_HIGH, addr >> 29, 0x07);
	demp_reg_write(demp, DEMP_REG_IDMA0_ADDRESS_LOW, addr << 3);

	demp_reg_write(demp, DEMP_REG_IDMA0_PITCH, pitch);

	demp_reg_mask(demp, DEMP_REG_IDMA0_SIZE,
		      width | (height << 16), 0x1FFF1FFF);

	demp_reg_write(demp, DEMP_REG_IDMA0_COORD, 0);
	/* format is 0, and enable. */
	demp_reg_write(demp, DEMP_REG_IDMA0_CONTROL, 0x01);

	/* disable others. */
	demp_reg_write(demp, DEMP_REG_IDMA1_CONTROL, 0);
	demp_reg_write(demp, DEMP_REG_IDMA2_CONTROL, 0);
	demp_reg_write(demp, DEMP_REG_IDMA3_CONTROL, 0);

	/* bypass all channels. */
	demp_reg_write(demp, DEMP_REG_ROP_CONTROL, 0xF0);
}

static void demp_input_prgb(struct demp *demp, struct vb2_buffer *buffer,
			    struct v4l2_pix_format_mplane *format)
{
	dma_addr_t addr;
	uint32_t high = 0;
	uint16_t width = format->width - 1;
	uint16_t height = format->height - 1;
	uint32_t pitch = format->plane_fmt[0].bytesperline << 3;

	addr = vb2_dma_contig_plane_dma_addr(buffer, 0);
	high |= (addr >> 29) & 0x07;
	demp_reg_write(demp, DEMP_REG_IDMA0_ADDRESS_LOW, addr << 3);

	addr = vb2_dma_contig_plane_dma_addr(buffer, 1);
	high |= (addr >> 21) & 0x0700;
	demp_reg_write(demp, DEMP_REG_IDMA1_ADDRESS_LOW, addr << 3);

	addr = vb2_dma_contig_plane_dma_addr(buffer, 2);
	high |= (addr >> 13) & 0x070000;
	demp_reg_write(demp, DEMP_REG_IDMA2_ADDRESS_LOW, addr << 3);

	demp_reg_write(demp, DEMP_REG_IDMA_ADDRESS_HIGH, high);

	demp_reg_write(demp, DEMP_REG_IDMA0_PITCH, pitch);
	demp_reg_write(demp, DEMP_REG_IDMA1_PITCH, pitch);
	demp_reg_write(demp, DEMP_REG_IDMA2_PITCH, pitch);

	demp_reg_mask(demp, DEMP_REG_IDMA0_SIZE,
		      width | (height << 16), 0x1FFF1FFF);
	demp_reg_mask(demp, DEMP_REG_IDMA1_SIZE,
		      width | (height << 16), 0x1FFF1FFF);
	demp_reg_mask(demp, DEMP_REG_IDMA2_SIZE,
		      width | (height << 16), 0x1FFF1FFF);

	demp_reg_write(demp, DEMP_REG_IDMA0_COORD, 0);
	demp_reg_write(demp, DEMP_REG_IDMA1_COORD, 0);
	demp_reg_write(demp, DEMP_REG_IDMA2_COORD, 0);

	/* format is greyscale, alpha defaults to 0xFF, enable. */
	demp_reg_write(demp, DEMP_REG_IDMA0_CONTROL, 0x0701);
	demp_reg_write(demp, DEMP_REG_IDMA1_CONTROL, 0x0701);
	demp_reg_write(demp, DEMP_REG_IDMA2_CONTROL, 0x0701);

	/* disable channel 3 */
	demp_reg_write(demp, DEMP_REG_IDMA3_CONTROL, 0);

	/* bypass all channels, r -> 0, g -> 1, b -> 2, a -> 0 */
	demp_reg_write(demp, DEMP_REG_ROP_CONTROL, 0x06F0);
}

static void demp_output_rgba(struct demp *demp, struct vb2_buffer *buffer,
			     struct v4l2_pix_format_mplane *format)
{
	dma_addr_t addr = vb2_dma_contig_plane_dma_addr(buffer, 0);
	uint16_t width = format->width - 1;
	uint16_t height = format->height - 1;
	uint32_t pitch = format->plane_fmt[0].bytesperline << 3;

	/* BGRA */
	demp_reg_write(demp, DEMP_REG_OUTPUT_CONTROL, 0x800);

	demp_reg_mask(demp, DEMP_REG_OUTPUT_SIZE,
		      width | (height << 16), 0x1FFF1FFF);

	demp_reg_mask(demp, DEMP_REG_OUTPUT_ADDRESS_HIGH, addr >> 29, 0x07);
	demp_reg_write(demp, DEMP_REG_OUTPUT_ADDRESS_CH0, addr << 3);

	demp_reg_write(demp, DEMP_REG_OUTPUT_PITCH_CH0, pitch);

	/* clear other channels */
	demp_reg_write(demp, DEMP_REG_OUTPUT_ADDRESS_CH1, 0);
	demp_reg_write(demp, DEMP_REG_OUTPUT_PITCH_CH1, 0);
	demp_reg_write(demp, DEMP_REG_OUTPUT_ADDRESS_CH2, 0);
	demp_reg_write(demp, DEMP_REG_OUTPUT_PITCH_CH2, 0);
}

static void demp_output_ayuv(struct demp *demp, struct vb2_buffer *buffer,
			     struct v4l2_pix_format_mplane *format)
{
	dma_addr_t addr = vb2_dma_contig_plane_dma_addr(buffer, 0);
	uint16_t width = format->width - 1;
	uint16_t height = format->height - 1;
	uint32_t pitch = format->plane_fmt[0].bytesperline << 3;

	demp_reg_write(demp, DEMP_REG_OUTPUT_CONTROL, 0);

	demp_reg_mask(demp, DEMP_REG_OUTPUT_SIZE,
		      width | (height << 16), 0x1FFF1FFF);

	demp_reg_mask(demp, DEMP_REG_OUTPUT_ADDRESS_HIGH, addr >> 29, 0x07);
	demp_reg_write(demp, DEMP_REG_OUTPUT_ADDRESS_CH0, addr << 3);

	demp_reg_write(demp, DEMP_REG_OUTPUT_PITCH_CH0, pitch);

	/* clear other channels */
	demp_reg_write(demp, DEMP_REG_OUTPUT_ADDRESS_CH1, 0);
	demp_reg_write(demp, DEMP_REG_OUTPUT_PITCH_CH1, 0);
	demp_reg_write(demp, DEMP_REG_OUTPUT_ADDRESS_CH2, 0);
	demp_reg_write(demp, DEMP_REG_OUTPUT_PITCH_CH2, 0);
}

static void demp_csc2_disable(struct demp *demp)
{
	demp_reg_write(demp, DEMP_REG_CSC2_CONTROL, 0);
}

static void demp_csc2_enable_rec709(struct demp *demp)
{
	demp_reg_write(demp, DEMP_REG_CSC2_CONTROL, 0x01);

	demp_reg_write(demp, DEMP_REG_OUTPUT_CSC_YG_GY_COEFF, 0x0275);
	demp_reg_write(demp, DEMP_REG_OUTPUT_CSC_YG_RU_COEFF, 0x00BB);
	demp_reg_write(demp, DEMP_REG_OUTPUT_CSC_YG_BV_COEFF, 0x003F);
	demp_reg_write(demp, DEMP_REG_OUTPUT_CSC_YG_CONSTANT, 0x0100);
	demp_reg_write(demp, DEMP_REG_OUTPUT_CSC_UR_GY_COEFF, 0x1EA6);
	demp_reg_write(demp, DEMP_REG_OUTPUT_CSC_UR_RU_COEFF, 0x1F99);
	demp_reg_write(demp, DEMP_REG_OUTPUT_CSC_UR_BV_COEFF, 0x01C2);
	demp_reg_write(demp, DEMP_REG_OUTPUT_CSC_UR_CONSTANT, 0x0800);
	demp_reg_write(demp, DEMP_REG_OUTPUT_CSC_VB_GY_COEFF, 0x1E67);
	demp_reg_write(demp, DEMP_REG_OUTPUT_CSC_VB_RU_COEFF, 0x01C2);
	demp_reg_write(demp, DEMP_REG_OUTPUT_CSC_VB_BV_COEFF, 0x1FD7);
	demp_reg_write(demp, DEMP_REG_OUTPUT_CSC_VB_CONSTANT, 0x0800);
}

static void demp_m2m_device_run(void *priv)
{
	struct demp_context *context = priv;
	struct demp *demp = context->demp;
	struct vb2_v4l2_buffer *source_v4l2 =
		v4l2_m2m_next_src_buf(context->fh->m2m_ctx);
	struct vb2_buffer *source_vb2 = &source_v4l2->vb2_buf;
	struct vb2_v4l2_buffer *dest_v4l2 =
		v4l2_m2m_next_dst_buf(context->fh->m2m_ctx);
	struct vb2_buffer *dest_vb2 = &dest_v4l2->vb2_buf;
	unsigned long flags;

	dev_info(demp->dev, "%s();\n", __func__);

	spin_lock_irqsave(demp->io_lock, flags);

	demp_reg_write(demp, DEMP_REG_CONTROL, 0x301);

	switch (context->format_input->pixelformat) {
	case V4L2_PIX_FMT_RGBA32:
		demp_input_rgba(demp, source_vb2, context->format_input);
		break;
	case V4L2_PIX_FMT_R8_G8_B8:
		demp_input_prgb(demp, source_vb2, context->format_input);
		break;
	default:
		dev_err(demp->dev, "%s(): unhandled input format: 0x%X\n",
			__func__, context->format_input->pixelformat);
		break;
	}

	switch (context->format_output->pixelformat) {
	case V4L2_PIX_FMT_RGBA32:
		demp_output_rgba(demp, dest_vb2, context->format_output);
		demp_csc2_disable(demp);
		break;
	case V4L2_PIX_FMT_AYUV32:
		demp_output_ayuv(demp, dest_vb2, context->format_output);
		demp_csc2_enable_rec709(demp);
		break;
	default:
		dev_err(demp->dev, "%s: unhandled output format %d\n",
			__func__, context->format_output->pixelformat);
		break;
	}

	dev_info(demp->dev, "%s(): Go!", __func__);
	demp_reg_write(demp, DEMP_REG_CONTROL, 0x303); /* GO! */

	spin_unlock_irqrestore(demp->io_lock, flags);
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

	/* shut down. */
	demp_reg_write_spin(demp, DEMP_REG_CONTROL, 0);

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

	switch (descriptor->index) {
	case 0:
		descriptor->pixelformat = V4L2_PIX_FMT_RGBA32;
		return 0;
	case 1:
		descriptor->pixelformat = V4L2_PIX_FMT_R8_G8_B8;
		return 0;
	default:
		return -EINVAL;
	}
}

static int demp_ioctl_format_input_get(struct file *file, void *handle,
				       struct v4l2_format *format)
{
	struct demp *demp = video_drvdata(file);
	struct demp_context *context = file->private_data;

	dev_info(demp->dev, "%s();\n", __func__);

	format->type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
	format->fmt.pix_mp = context->format_input[0];

	return 0;
}

#define DEMP_MAX_WIDTH 8192
#define DEMP_MAX_HEIGHT DEMP_MAX_WIDTH

/*
 * This sets dimensions for both in and output.
 */
static int demp_ioctl_format_input_set(struct file *file, void *handle,
				       struct v4l2_format *format_new)
{
	struct demp *demp = video_drvdata(file);
	struct demp_context *context = file->private_data;
	struct v4l2_pix_format_mplane *new =
		&format_new->fmt.pix_mp;
	struct v4l2_pix_format_mplane *input = context->format_input;
	struct v4l2_pix_format_mplane *output = context->format_output;
	int width = new->width;
	int height = new->height;
	int size;

	dev_info(demp->dev, "%s();\n", __func__);

	/* 2-align, as we might be converting to NV12 */
	width = ALIGN(width, 2);
	if (width > DEMP_MAX_WIDTH)
		width = DEMP_MAX_WIDTH;

	height = ALIGN(height, 2);
	if (height > DEMP_MAX_HEIGHT)
		height = DEMP_MAX_HEIGHT;

	size = width * height;

	switch (new->pixelformat) {
	case V4L2_PIX_FMT_RGBA32:
		*input = format_default_rgba[0];
		input->width = width;
		input->height = height;
		input->plane_fmt[0].bytesperline = width * 4;
		input->plane_fmt[0].sizeimage = size * 4;
		break;
	case V4L2_PIX_FMT_R8_G8_B8:
		*input = format_default_prgb[0];
		input->width = width;
		input->height = height;
		input->plane_fmt[0].bytesperline = width;
		input->plane_fmt[0].sizeimage = size;
		input->plane_fmt[1].bytesperline = width;
		input->plane_fmt[1].sizeimage = size;
		input->plane_fmt[2].bytesperline = width;
		input->plane_fmt[2].sizeimage = size;
		break;
	default:
		dev_err(demp->dev, "%s(): unhandled input format: 0x%X\n",
			__func__, new->pixelformat);
		return -EINVAL;
	}

	/* now force the same size on the output format */
	switch (output->pixelformat) {
	case V4L2_PIX_FMT_RGBA32:
	case V4L2_PIX_FMT_AYUV32:
		output->width = width;
		output->height = height;
		output->plane_fmt[0].bytesperline = width * 4;
		output->plane_fmt[0].sizeimage = size * 4;
		break;
	default:
		dev_err(demp->dev, "%s(): unhandled output format: 0x%X\n",
			__func__, output->pixelformat);
		break;
	}

	format_new->type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
	format_new->fmt.pix_mp = context->format_input[0];

	return 0;
}

static int demp_ioctl_format_input_try(struct file *file, void *handle,
				       struct v4l2_format *format_try)
{
	struct demp *demp = video_drvdata(file);
	struct v4l2_pix_format_mplane *try = &format_try->fmt.pix_mp;
	int width = try->width;
	int height = try->height;
	int size;

	dev_info(demp->dev, "%s();\n", __func__);

	width = ALIGN(try->width, 2);
	if (width > DEMP_MAX_WIDTH)
		width = DEMP_MAX_WIDTH;

	height = ALIGN(try->height, 2);
	if (height > DEMP_MAX_HEIGHT)
		height = DEMP_MAX_HEIGHT;

	size = width * height;

	switch (try->pixelformat) {
	case V4L2_PIX_FMT_RGBA32:
		*try = format_default_rgba[0];
		try->width = width;
		try->height = height;
		try->plane_fmt[0].bytesperline = width * 4;
		try->plane_fmt[0].sizeimage = size * 4;
		break;
	case V4L2_PIX_FMT_R8_G8_B8:
		*try = format_default_prgb[0];
		try->width = width;
		try->height = height;
		try->plane_fmt[0].bytesperline = width;
		try->plane_fmt[0].sizeimage = size;
		try->plane_fmt[1].bytesperline = width;
		try->plane_fmt[1].sizeimage = size;
		try->plane_fmt[2].bytesperline = width;
		try->plane_fmt[2].sizeimage = size;
		break;
	default:
		dev_err(demp->dev, "%s(): unhandled input format: 0x%X\n",
			__func__, try->pixelformat);
		return -EINVAL;
	}

	return 0;
}

static int demp_ioctl_format_output_enumerate(struct file *file,
					      void *handle,
					      struct v4l2_fmtdesc *descriptor)
{
	struct demp *demp = video_drvdata(file);

	dev_info(demp->dev, "%s();\n", __func__);

	switch (descriptor->index) {
	case 0:
		descriptor->pixelformat = V4L2_PIX_FMT_RGBA32;
		return 0;
	case 1:
		descriptor->pixelformat = V4L2_PIX_FMT_AYUV32;
		return 0;
	default:
		return -EINVAL;
	}
}

static int demp_ioctl_format_output_get(struct file *file, void *handle,
					struct v4l2_format *format)
{
	struct demp *demp = video_drvdata(file);
	struct demp_context *context = file->private_data;

	dev_info(demp->dev, "%s();\n", __func__);

	format->type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	format->fmt.pix_mp = context->format_output[0];

	return 0;
}

/*
 * Ignore everything, except pixelformat changes, and use input width/height
 */
static int demp_ioctl_format_output_set(struct file *file, void *handle,
					struct v4l2_format *format_new)
{
	struct demp *demp = video_drvdata(file);
	struct demp_context *context = file->private_data;
	struct v4l2_pix_format_mplane *new =
		&format_new->fmt.pix_mp;
	struct v4l2_pix_format_mplane *input = context->format_input;
	struct v4l2_pix_format_mplane *output = context->format_output;
	int width = input->width;
	int height = input->height;
	int size = width * height;

	dev_info(demp->dev, "%s();\n", __func__);

	/* width/height was already aligned and clamped when setting input */

	switch (new->pixelformat) {
	case V4L2_PIX_FMT_RGBA32:
		*output = format_default_rgba[0];
		output->width = width;
		output->height = height;
		output->plane_fmt[0].bytesperline = width * 4;
		output->plane_fmt[0].sizeimage = size * 4;
		break;
	case V4L2_PIX_FMT_AYUV32:
		*output = format_default_ayuv[0];
		output->width = width;
		output->height = height;
		output->plane_fmt[0].bytesperline = width * 4;
		output->plane_fmt[0].sizeimage = size * 4;
		break;
	default:
		dev_err(demp->dev, "%s(): unhandled output format: 0x%X\n",
			__func__, new->pixelformat);
		return -EINVAL;
	}

	format_new->type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	format_new->fmt.pix_mp = context->format_output[0];

	return 0;
}

static int demp_ioctl_format_output_try(struct file *file, void *handle,
					struct v4l2_format *format_try)
{
	struct demp *demp = video_drvdata(file);
	struct v4l2_pix_format_mplane *try = &format_try->fmt.pix_mp;
	int width = try->width;
	int height = try->height;
	int size;

	dev_info(demp->dev, "%s();\n", __func__);

	width = ALIGN(width, 2);
	if (width > DEMP_MAX_WIDTH)
		width = DEMP_MAX_WIDTH;

	height = ALIGN(height, 2);
	if (height > DEMP_MAX_HEIGHT)
		height = DEMP_MAX_HEIGHT;

	size = width * height;

	switch (try->pixelformat) {
	case V4L2_PIX_FMT_RGBA32:
		*try = format_default_rgba[0];
		try->width = width;
		try->height = height;
		try->plane_fmt[0].bytesperline = width * 4;
		try->plane_fmt[0].sizeimage = size * 4;
		break;
	case V4L2_PIX_FMT_AYUV32:
		*try = format_default_ayuv[0];
		try->width = width;
		try->height = height;
		try->plane_fmt[0].bytesperline = width * 4;
		try->plane_fmt[0].sizeimage = size * 4;
		break;
	default:
		dev_err(demp->dev, "%s(): unhandled output format: 0x%X\n",
			__func__, try->pixelformat);
		return -EINVAL;
	}

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
	spin_lock_init(demp->io_lock);

	ret = demp_resources_get(demp, platform_dev);
	if (ret)
		return ret;

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
		.pm = &demp_pm_ops,
	},
};

module_platform_driver(demp_platform_driver);
MODULE_DESCRIPTION("Allwinner A10/A20 Mixer Processor V4L2 driver");
MODULE_AUTHOR("Luc Verhaegen <libv@skynet.be>");
MODULE_LICENSE("GPL v2");
