// SPDX-License-Identifier: GPL-2.0+ OR BSD-3-Clause
/* Copyright 2023 NXP */
#include "llce_can_core.h"

#define PATH_DIR(P, D)			P "/" D

#define DEBUGFS_ROOT			"/sys/kernel/debug"

#define LLCE_CAN_CORE_DIR		"llce_can_core"
#define LLCE_CAN_CORE_PATH		PATH_DIR(DEBUGFS_ROOT, LLCE_CAN_CORE_DIR)

#define LLCE_CAN_DEST_DIR		"can_destinations"
#define ADD_CMD_DIR			"add"
#define REMOVE_CMD_DIR			"remove"
#define SET_STATUS_CMD_DIR		"set_status"
#define README_FILE			"README"
#define LIST_FILE			"list"
#define EXECUTE_FILE			"execute"

#define LLCE_CAN_DEST_RPATH		PATH_DIR(LLCE_CAN_CORE_PATH,\
						 LLCE_CAN_DEST_DIR)
#define LLCE_CAN_DEST_PATH(D)		PATH_DIR(LLCE_CAN_DEST_RPATH, D)
#define ADD_DEST_CMD_PATH(D)		PATH_DIR(LLCE_CAN_DEST_PATH(ADD_CMD_DIR), D)

#define DEST_ID_FLD			"id"
#define DEST_ROUTING_OPS_FLD		"routing_opts"
#define DEST_CAN_ID_REMAP_FLD		"can_id_remap"
#define DEST_CH_LIST_FLD		"ch_list"

#define LLCE_CAN_FILTERS_DIR		"can_filters"
#define FILTERS_UTAG_FLD		"tag"
#define FILTERS_ENTRY_FLD		"entry"
#define FILTERS_MB_TYPE_FLD		"mb_type"
#define FILTERS_MB_COUNT_FLD		"mb_count"
#define FILTERS_ADDR_FLD		"addr"
#define FILTERS_FIFO_FLD		"rx_intf"
#define FILTERS_CAN_ID_FLD		"can_mask_id"
#define FILTERS_MSG_ID_FLD		"msg_id"
#define FILTERS_HW_CTRL_FLD		"hw_ctrl"
#define FILTERS_AUTH_FLD		"opt_auth"
#define FILTERS_HOST_FLD		"opt_host"
#define FILTERS_LOGGING_FLD		"logging"
#define FILTERS_CUSTOM_FLD		"opt_custom"
#define FILTERS_CAN_ROUTE_FLD		"can_dest"
#define FILTERS_STATUS_FLD		"status"

#define LLCE_CAN_FILTER_RPATH		PATH_DIR(LLCE_CAN_CORE_PATH, \
						 LLCE_CAN_FILTERS_DIR)
#define LLCE_CAN_FILTER_PATH(D)		PATH_DIR(LLCE_CAN_FILTER_RPATH, D)
#define ADD_FILTER_CMD_PATH(D)		PATH_DIR(LLCE_CAN_FILTER_PATH(ADD_CMD_DIR), D)
#define RM_FILTER_CMD_PATH(D)		PATH_DIR(LLCE_CAN_FILTER_PATH(REMOVE_CMD_DIR), D)
#define STS_FILTER_CMD_PATH(D)		PATH_DIR(LLCE_CAN_FILTER_PATH(SET_STATUS_CMD_DIR), D)

#define ENABLED_OPT_NAME		"enabled"
#define DISABLED_OPT_NAME		"disabled"

#define ENTRY_EXACT_MATCH		"exact"
#define ENTRY_MASK_MATCH		"mask"
#define ENTRY_RANGE_MATCH		"range"

#define MB_TYPE_LONG			"long"
#define MB_TYPE_SHORT			"short"

#define AUTH_DISABLED			DISABLED_OPT_NAME
#define AUTH_ENABLED			ENABLED_OPT_NAME
#define AUTH_NOT_SUPP			"not_supp"

#define HOST_RCV_DISABLED		DISABLED_OPT_NAME
#define HOST_RCV_ENABLED		ENABLED_OPT_NAME

#define LOGGING_DISABLED	DISABLED_OPT_NAME
#define LOGGING_ENABLED		ENABLED_OPT_NAME

#define CPROCESS_DISABLED	DISABLED_OPT_NAME
#define CPROCESS_ENABLED	ENABLED_OPT_NAME

#define STATUS_DISABLED		DISABLED_OPT_NAME
#define STATUS_ENABLED		ENABLED_OPT_NAME

struct u8_array_meta {
	struct debugfs_u8_array *array;
	size_t alloc_size;
	char *buffer;
};

struct debugfs_dir_create {
	struct dentry **parent, **dir;
	const char *name;
	char **readme;
};

/* Tentative definitions for README files content */
static char *root_readme;
static char *can_dest_readme;
static char *can_dest_add_cmd_readme;
static char *can_filters_readme;
static char *can_filters_add_readme;
static char *can_filters_remove_readme;
static char *can_filters_set_status_readme;

static struct debugfs_choice entry_choices[] = {
	{ .id = LLCE_CAN_ENTRY_EXACT_MATCH, .name = ENTRY_EXACT_MATCH, },
	{ .id = LLCE_CAN_ENTRY_CFG_MASKED, .name = ENTRY_MASK_MATCH, },
	{ .id = LLCE_CAN_ENTRY_CFG_RANGED, .name = ENTRY_RANGE_MATCH, },
};

static struct debugfs_choice mb_choices[] = {
	{ .id = USE_LONG_MB, .name = MB_TYPE_LONG, },
	{ .id = USE_SHORT_MB, .name = MB_TYPE_SHORT, },
};

static struct debugfs_choice auth_choices[] = {
	{ .id = LLCE_AF_AUTHENTICATION_DISABLED, .name = AUTH_DISABLED, },
	{ .id = LLCE_AF_AUTHENTICATION_ENABLED, .name = AUTH_ENABLED, },
	{ .id = LLCE_AF_AUTHENTICATION_NOT_SUPPORTED, .name = AUTH_NOT_SUPP, },
};

static struct debugfs_choice host_choices[] = {
	{ .id = LLCE_AF_HOSTRECEIVE_DISABLED, .name = HOST_RCV_DISABLED, },
	{ .id = LLCE_AF_HOSTRECEIVE_ENABLED, .name = HOST_RCV_ENABLED, },
};

static struct debugfs_choice logging_choices[] = {
	{ .id = LLCE_AF_LOGGING_DISABLED, .name = LOGGING_DISABLED, },
	{ .id = LLCE_AF_LOGGING_ENABLED, .name = LOGGING_ENABLED, },
};

static struct debugfs_choice custom_choices[] = {
	{ .id = LLCE_AF_CUSTOMPROCESSING_DISABLED, .name = CPROCESS_DISABLED, },
	{ .id = LLCE_AF_CUSTOMPROCESSING_ENABLED, .name = CPROCESS_ENABLED, },
};

static struct debugfs_choice status_choices[] = {
	{ .id = false, .name = STATUS_DISABLED, },
	{ .id = true, .name = STATUS_ENABLED, },
};

static void debugfs_create_readme(struct dentry *parent, char **value)
{
	debugfs_create_str(README_FILE, 0400, parent, value);
}

static int debugfs_cmd_set(void *data, u64 val)
{
	struct debugfs_cmd_block *blk = data;

	if (!blk || !blk->clb)
		return -EINVAL;

	return blk->clb(blk, val);
}

static int debugfs_cmd_get(void *data, u64 *val)
{
	struct debugfs_cmd_block *blk = data;

	if (!blk || !blk->get_val)
		return -EINVAL;

	return blk->get_val(blk, val);
}

DEFINE_DEBUGFS_ATTRIBUTE(fops_cmd, debugfs_cmd_get, debugfs_cmd_set, "%llu\n");

static int debugfs_create_cmd(struct device *dev, const char *name,
			      struct dentry *parent,
			      struct debugfs_cmd_block *blk)
{
	struct dentry *file;

	file = debugfs_create_file_unsafe(name, 0600,
					  parent, blk, &fops_cmd);
	if (IS_ERR(file)) {
		dev_err(dev, "Failed to create the file '%s'\n", name);
		return PTR_ERR(file);
	}

	return 0;
}

static ssize_t debugfs_read_file_choice(struct file *file,
					char __user *user_buf,
					size_t count, loff_t *ppos)
{
	struct dentry *dentry = file->f_path.dentry;
	struct debugfs_choice_block *choice_blk;
	const char *option_name;
	ssize_t ret;
	size_t len;
	char *copy;

	ret = debugfs_file_get(dentry);
	if (unlikely(ret))
		return ret;

	choice_blk = file->private_data;
	option_name = choice_blk->selected->name;

	/* Two extra chars: \n + \0 */
	if (unlikely(check_add_overflow(strlen(option_name), (size_t)2, &len)))
		return -E2BIG;

	copy = kmalloc(len, GFP_KERNEL);
	if (!copy) {
		debugfs_file_put(dentry);
		return -ENOMEM;
	}

	memcpy(copy, option_name, len - 2);
	copy[len - 2] = '\n';
	copy[len - 1] = '\0';

	debugfs_file_put(dentry);

	ret = simple_read_from_buffer(user_buf, count, ppos, copy, len);
	kfree(copy);

	return ret;
}

static const struct debugfs_choice *get_choice(struct debugfs_choice_block *blk,
					       char *option)
{
	size_t i;

	for (i = 0; i < blk->n_choices; i++)
		if (!strcmp(option, blk->choices[i].name))
			return &blk->choices[i];

	return NULL;
}

static ssize_t debugfs_write_file_choice(struct file *file,
					 const char __user *user_buf,
					 size_t count, loff_t *ppos)
{
	struct debugfs_choice_block *choice_blk;
	const struct debugfs_choice *selection;
	struct dentry *dentry = file->f_path.dentry;
	char *buffer, *value;
	ssize_t ret;

	if (!count)
		return 0;

	choice_blk = file->private_data;

	buffer = kmalloc(count, GFP_KERNEL);
	if (!buffer)
		return -ENOMEM;

	ret = simple_write_to_buffer(buffer, count, ppos, user_buf, count);
	if (ret < 0)
		goto release_buffer;

	buffer[count - 1] = '\0';

	value = strim(buffer);

	ret = debugfs_file_get(dentry);
	if (unlikely(ret))
		goto release_buffer;

	selection = get_choice(choice_blk, value);
	if (!selection) {
		ret = -EINVAL;
		goto dentry_put;
	}

	choice_blk->selected = selection;

	ret = count;

dentry_put:
	debugfs_file_put(dentry);

release_buffer:
	kfree(buffer);

	return ret;
}

static const struct file_operations fops_choice = {
	.read =		debugfs_read_file_choice,
	.write =	debugfs_write_file_choice,
	.open =		simple_open,
	.llseek =	no_llseek,
};

static int debugfs_create_choice(struct device *dev, const char *name,
				 struct dentry *parent,
				 struct debugfs_choice_block *blk)
{
	struct dentry *file;

	file = debugfs_create_file_unsafe(name, 0600, parent, blk,
					  &fops_choice);
	if (IS_ERR(file)) {
		dev_err(dev, "Failed to create the file '%s'\n", name);
		return PTR_ERR(file);
	}

	return 0;
}

static int u8_array_open(struct inode *inode, struct file *file)
{
	struct debugfs_u8_array *data = inode->i_private;
	struct u8_array_meta *meta;
	int ret;

	meta = kmalloc(sizeof(*meta), GFP_KERNEL);
	if (!meta)
		return -ENOMEM;

	meta->array = data;

	/* Max size:
	 *  - 10 bytes per number
	 *  - terminating NUL character
	 */
	meta->alloc_size = data->max_elems * 10 + 1;
	meta->buffer = kmalloc(meta->alloc_size, GFP_KERNEL);
	if (!meta->buffer) {
		ret = -ENOMEM;
		goto release_meta;
	}

	meta->buffer[meta->alloc_size - 1] = 0;

	file->private_data = meta;

	ret = nonseekable_open(inode, file);
	if (ret)
		kfree(meta->buffer);

release_meta:
	if (ret)
		kfree(meta);

	return ret;
}

static int u8_array_release(struct inode *inode, struct file *file)
{
	struct u8_array_meta *meta = file->private_data;

	kfree(meta->buffer);
	kfree(meta);

	return 0;
}

static ssize_t u8_array_read(struct file *file, char __user *buf, size_t len,
			     loff_t *ppos)
{
	struct u8_array_meta *meta = file->private_data;
	struct debugfs_u8_array *data = meta->array;
	u8 *array = data->array;
	u8 n_elements = data->n_elements;
	size_t size, bufsize;
	char *str, term;
	int elem_len;

	bufsize = meta->alloc_size;

	meta->buffer[0] = 0;
	str = meta->buffer;

	while (n_elements > 0) {
		term = (n_elements > 1) ? ' ' : '\n';

		elem_len = snprintf(str, bufsize, "%hhu%c", *array++, term);
		if (elem_len < 0) {
			pr_err("Failed to read an array member from string: '%s'\n",
			       str);
			return elem_len;
		}

		if (bufsize < elem_len)
			return -EINVAL;

		str += elem_len;
		bufsize -= elem_len;
		n_elements--;
	}

	size = str - meta->buffer;

	return simple_read_from_buffer(buf, len, ppos, meta->buffer, size);
}

static ssize_t u8_array_write(struct file *file, const char __user *user_buf,
			      size_t count, loff_t *ppos)
{
	struct u8_array_meta *meta = file->private_data;
	struct debugfs_u8_array *data = meta->array;
	u8 *array = data->array;
	int pos;
	char *buf;
	ssize_t ret;

	if (count > meta->alloc_size)
		return -E2BIG;

	data->n_elements = 0;

	ret = simple_write_to_buffer(meta->buffer, meta->alloc_size, ppos,
				     user_buf, count);
	if (ret < 0)
		return ret;

	buf = meta->buffer;
	while (sscanf(buf, "%hhu%n", array, &pos) == 1) {
		if (data->n_elements >= data->max_elems)
			return -E2BIG;

		array++;
		data->n_elements++;
		buf += pos;
	}

	return ret;
}

static const struct file_operations u8_array_fops = {
	.owner	 = THIS_MODULE,
	.open	 = u8_array_open,
	.release = u8_array_release,
	.read	 = u8_array_read,
	.write	 = u8_array_write,
	.llseek	 = no_llseek,
};

static int debugfs_create_u8_array(struct device *dev, const char *name,
				   struct dentry *parent,
				   struct debugfs_u8_array *array)
{
	struct dentry *file;

	file = debugfs_create_file_unsafe(name, 0600, parent, array,
					  &u8_array_fops);
	if (IS_ERR(file)) {
		dev_err(dev, "Failed to create the file '%s'\n", name);
		return PTR_ERR(file);
	}

	return 0;
}

static struct llce_can_core *get_can_core(struct llce_can_core_debugfs *root)
{
	return container_of(root, struct llce_can_core, debugfs);
}

static struct device *get_llce_dbgfs_device(struct llce_can_core_debugfs *root)
{
	struct llce_can_core *core = get_can_core(root);

	return get_can_core_dev(core);
}

static void *dest_list_seq_start(struct seq_file *seq, loff_t *pos)
{
	struct llce_can_core *can_core = seq->private;

	mutex_lock(&can_core->can_dest_lock);
	return seq_list_start_head(&can_core->can_dest_list, *pos);
}

static void *dest_list_seq_next(struct seq_file *seq, void *v, loff_t *pos)
{
	struct llce_can_core *can_core = seq->private;

	return seq_list_next(v, &can_core->can_dest_list, pos);
}

static void dest_list_seq_stop(struct seq_file *seq, void *v)
{
	struct llce_can_core *can_core = seq->private;

	mutex_unlock(&can_core->can_dest_lock);
}

static int dest_list_seq_show(struct seq_file *seq, void *v)
{
	const struct can_destination *can_dest;
	struct llce_can_core *can_core = seq->private;
	struct llce_can_word0 word0;
	u8 idx;

	can_dest = list_entry(v, struct can_destination, link);

	if (v == &can_core->can_dest_list) {
		seq_printf(seq, "%3s %s %s %s\n",
			   DEST_ID_FLD, DEST_ROUTING_OPS_FLD,
			   DEST_CAN_ID_REMAP_FLD, DEST_CH_LIST_FLD);
		return 0;
	}

	word0 = llce_can_unpack_word0(can_dest->dest.can_id_remap_value);
	seq_printf(seq, "%3hhu   0x%08x   0x%08x", can_dest->id,
		   can_dest->dest.can2can_routing_options,
		   word0.id);

	for (idx = 0; idx < can_dest->dest.dest_hw_ch_list_count; idx++)
		seq_printf(seq, " %hhu", can_dest->dest.dest_hw_ch_list[idx]);

	seq_puts(seq, "\n");

	return 0;
}

static void *filters_list_seq_start(struct seq_file *seq, loff_t *pos)
{
	struct llce_can_core *can_core = seq->private;

	mutex_lock(&can_core->filters_lock);
	return seq_list_start_head(&can_core->filters_list, *pos);
}

static void *filters_list_seq_next(struct seq_file *seq, void *v, loff_t *pos)
{
	struct llce_can_core *can_core = seq->private;

	return seq_list_next(v, &can_core->filters_list, pos);
}

static void filters_list_seq_stop(struct seq_file *seq, void *v)
{
	struct llce_can_core *can_core = seq->private;

	mutex_unlock(&can_core->filters_lock);
}

static const char *get_option_name(struct debugfs_choice *choices,
				   size_t num_choices,
				   u64 id)
{
	u64 i;

	for (i = 0; i < num_choices; i++)
		if (id == choices[i].id)
			return choices[i].name;

	return NULL;
}

static int filters_list_seq_show(struct seq_file *seq, void *v)
{
	struct filter_state *filter;
	struct llce_can_advanced_feature *adv;
	struct llce_can_core *can_core = seq->private;
	struct llce_can_rx_filter *base;
	const char *entry, *mb, *auth, *host, *logging, *custom, *status;

	filter = list_entry(v, struct filter_state, link);

	if (v == &can_core->filters_list) {
		seq_printf(seq, "%7s %8s %11s %10s %6s %8s %5s %7s %5s %7s | ",
			   FILTERS_HW_CTRL_FLD, FILTERS_STATUS_FLD, FILTERS_CAN_ID_FLD,
			   FILTERS_MSG_ID_FLD, FILTERS_UTAG_FLD, FILTERS_MB_COUNT_FLD,
			   FILTERS_ADDR_FLD, FILTERS_FIFO_FLD, FILTERS_ENTRY_FLD,
			   FILTERS_MB_TYPE_FLD);
		seq_printf(seq, "%8s %8s %8s %10s %8s\n",
			   FILTERS_AUTH_FLD, FILTERS_HOST_FLD, FILTERS_LOGGING_FLD,
			   FILTERS_CUSTOM_FLD, FILTERS_CAN_ROUTE_FLD);
		return 0;
	}

	base = get_base_filter(filter);
	adv = &filter->f.advanced.llce_can_advanced_feature;

	entry = get_option_name(entry_choices, ARRAY_SIZE(entry_choices),
				base->entry_type);
	mb = get_option_name(mb_choices, ARRAY_SIZE(mb_choices),
			     base->filter_mb_length);
	status = get_option_name(status_choices,
				 ARRAY_SIZE(status_choices),
				 filter->enabled);
	if (filter->advanced) {
		auth = get_option_name(auth_choices, ARRAY_SIZE(auth_choices),
				       adv->can_authentication_feature);
		host = get_option_name(host_choices, ARRAY_SIZE(host_choices),
				       adv->host_receive);
		logging = get_option_name(logging_choices,
					  ARRAY_SIZE(logging_choices),
					  adv->can_logging_feature);
		custom = get_option_name(custom_choices,
					 ARRAY_SIZE(custom_choices),
					 adv->can_custom_processing);
	} else {
		auth = "";
		host = "";
		logging = "";
		custom = "";
	}

	seq_printf(seq, "%7u ", filter->hw_ctrl);
	seq_printf(seq, "%8s ", status);
	seq_printf(seq, " 0x%08x ", base->id_mask);
	seq_printf(seq, "0x%08x ", base->message_id & ~LLCE_CAN_MB_IDE);
	seq_printf(seq, "0x%04x ", base->filter_id);
	seq_printf(seq, "  0x%04x ", base->mb_count);
	seq_printf(seq, "%5u ", base->filter_addr);
	seq_printf(seq, "%7u ", base->rx_dest_interface);
	seq_printf(seq, "%5s ", entry);
	seq_printf(seq, "%7s ", mb);

	seq_puts(seq, "| ");
	seq_printf(seq, "%8s ", auth);
	seq_printf(seq, "%8s ", host);
	seq_printf(seq, "%8s ", logging);
	seq_printf(seq, "%10s ", custom);

	if (filter->advanced)
		seq_printf(seq, "%8u", adv->can2can_routing_table_idx);

	seq_puts(seq, "\n");

	return 0;
}

static const struct seq_operations filters_list_seq_ops = {
	.start = filters_list_seq_start,
	.next  = filters_list_seq_next,
	.stop  = filters_list_seq_stop,
	.show  = filters_list_seq_show,
};

static const struct seq_operations dest_list_seq_ops = {
	.start = dest_list_seq_start,
	.next  = dest_list_seq_next,
	.stop  = dest_list_seq_stop,
	.show  = dest_list_seq_show,
};

struct list_files_entry {
	const char *parent_name;
	const struct seq_operations *seq_ops;
};

static struct list_files_entry list_map[] = {
	{
		.parent_name = LLCE_CAN_DEST_DIR,
		.seq_ops = &dest_list_seq_ops,
	},
	{
		.parent_name = LLCE_CAN_FILTERS_DIR,
		.seq_ops = &filters_list_seq_ops,
	},
};

static int can_core_list_open(struct inode *inode, struct file *file)
{
	const struct seq_operations *seq_ops = NULL;
	const char *folder_name = (const char *)file->f_path.dentry->d_parent->d_iname;
	void *data = inode->i_private;
	struct seq_file *seq;
	size_t i;
	int ret;

	for (i = 0; i < ARRAY_SIZE(list_map); i++) {
		if (strcmp(folder_name, list_map[i].parent_name))
			continue;

		seq_ops = list_map[i].seq_ops;
		break;
	}

	if (!seq_ops)
		return -EINVAL;

	ret = seq_open(file, seq_ops);
	if (!ret) {
		seq = file->private_data;
		seq->private = data;
	}

	return ret;
}

static int can_core_list_release(struct inode *inode, struct file *file)
{
	return seq_release(inode, file);
}

static const struct file_operations can_core_list_debugfs_fops = {
	.open		= can_core_list_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= can_core_list_release,
};

void llce_remove_debugs(struct llce_can_core_debugfs *root)
{
	debugfs_remove_recursive(root->root_dir);
}

static int can_get_last_dest_id(void *data, u64 *val)
{
	struct debugfs_cmd_block *blk = data;
	struct llce_can_dest_debugfs *can_dest;

	can_dest = container_of(blk, struct llce_can_dest_debugfs, add_exec);

	*val = can_dest->id;

	return 0;
}

static int validate_can_add_dest_params(struct device *dev,
					struct llce_can_dest_debugfs *can_dest)
{
	struct debugfs_u8_array *channels;
	u8 i;

	channels = &can_dest->ch_list;

	if (channels->n_elements > LLCE_CAN_CONFIG_MAXCTRL_COUNT) {
		dev_err(dev, "The number of channels exceeds the allowed limit of %u elements\n",
			LLCE_CAN_CONFIG_MAXCTRL_COUNT);
		return -ENOSPC;
	}

	for (i = 0; i < channels->n_elements; i++) {
		if (channels->array[i] < LLCE_CAN_CONFIG_MAXCTRL_COUNT)
			continue;

		dev_err(dev, "Channel id %u is invalid\n",
			channels->array[i]);

		return -EINVAL;
	}

	return 0;
}

static u64 get_selected_id(struct debugfs_choice_block *blk)
{
	return blk->selected->id;
}

static bool is_advanced_filter(struct llce_can_advanced_feature *opts)
{
	if (opts->can_authentication_feature == LLCE_AF_AUTHENTICATION_ENABLED)
		return true;

	if (opts->host_receive == LLCE_AF_HOSTRECEIVE_ENABLED)
		return true;

	if (opts->can_logging_feature == LLCE_AF_LOGGING_ENABLED)
		return true;

	if (opts->can_custom_processing == LLCE_AF_CUSTOMPROCESSING_ENABLED)
		return true;

	if (opts->can2can_routing_table_idx !=
	    LLCE_CAN_ADVANCED_FILTER_NOT_USED)
		return true;

	if (opts->can2eth_routing_table_idx !=
	    LLCE_CAN_ADVANCED_FILTER_NOT_USED)
		return true;

	if (opts->other_routing_table_idx !=
	    LLCE_CAN_ADVANCED_FILTER_NOT_USED)
		return true;

	return false;
}

static inline bool is_can_extended_id(u32 message_id)
{
	return message_id > CAN_SFF_MASK;
}

static void debugfs2filter(struct llce_can_filters_debugfs *filters_dfs,
			   struct filter_state *filter)
{
	struct llce_can_advanced_feature *adv_opts;
	struct llce_can_rx_filter *base_opts;

	filter->enabled = get_selected_id(&filters_dfs->status);

	adv_opts = &filter->f.advanced.llce_can_advanced_feature;
	base_opts = &filter->f.advanced.llce_can_rx_filter;

	/* Non-string/non-enum options */
	*base_opts = filters_dfs->base_opts;
	*adv_opts = filters_dfs->adv_opts;

	/* Basic enum options */
	base_opts->entry_type = get_selected_id(&filters_dfs->entry);
	base_opts->filter_mb_length = get_selected_id(&filters_dfs->mb_type);
	if (is_can_extended_id(base_opts->message_id))
		base_opts->message_id |= LLCE_CAN_MB_IDE;

	/* Advanced enum options */
	adv_opts->can_authentication_feature =
	    get_selected_id(&filters_dfs->auth);
	adv_opts->host_receive = get_selected_id(&filters_dfs->host);
	adv_opts->can_logging_feature = get_selected_id(&filters_dfs->logging);
	adv_opts->can_custom_processing =
	    get_selected_id(&filters_dfs->custom_proc);

	if (is_advanced_filter(adv_opts)) {
		filter->advanced = true;
	} else {
		filter->advanced = false;
		memmove(&filter->f.base, &filter->f.advanced.llce_can_rx_filter,
			sizeof(filter->f.base));
	}
}

static int can_add_filter_func(void *data, u64 val)
{
	struct debugfs_cmd_block *blk = data;
	struct llce_can_filters_debugfs *can_filters;
	struct llce_can_core_debugfs *core_debugfs;
	struct llce_can_core *can_core;
	struct device *dev;
	struct filter_state filter;
	int ret;

	can_filters = container_of(blk, struct llce_can_filters_debugfs,
				   add_exec);
	core_debugfs = container_of(can_filters, struct llce_can_core_debugfs,
				    can_filters);
	can_core = get_can_core(core_debugfs);
	dev = get_llce_dbgfs_device(core_debugfs);

	if (can_filters->hw_ctrl > LLCE_CAN_CONFIG_MAXCTRL_COUNT) {
		dev_err(dev, "Invalid controller %hhu\n", can_filters->hw_ctrl);
		return -EINVAL;
	}

	debugfs2filter(can_filters, &filter);

	ret = llce_add_can_filter(can_core, can_filters->hw_ctrl, &filter);
	if (ret)
		return ret;

	/* Save the address of the filter for later */
	ret = get_filter_addr(&filter, &can_filters->base_opts.filter_addr);

	return ret;
}

static int can_get_last_filter_id(void *data, u64 *val)
{
	struct debugfs_cmd_block *blk = data;
	struct llce_can_filters_debugfs *can_filters;

	can_filters = container_of(blk, struct llce_can_filters_debugfs,
				   add_exec);

	*val = can_filters->base_opts.filter_addr;

	return 0;
}

static int can_set_filter_status_func(void *data, u64 val)
{
	struct debugfs_cmd_block *blk = data;
	struct llce_can_filters_debugfs *can_filters;
	struct llce_can_core_debugfs *core_debugfs;
	struct llce_can_core *can_core;
	bool enabled;
	u16 filter_addr;

	can_filters = container_of(blk, struct llce_can_filters_debugfs,
				   set_status_exec);
	core_debugfs = container_of(can_filters, struct llce_can_core_debugfs,
				    can_filters);
	can_core = get_can_core(core_debugfs);

	enabled = get_selected_id(&can_filters->status);
	filter_addr = can_filters->base_opts.filter_addr;

	return llce_set_filter_status(can_core, filter_addr, enabled);
}

static int can_remove_filter_func(void *data, u64 val)
{
	struct debugfs_cmd_block *blk = data;
	struct llce_can_filters_debugfs *can_filters;
	struct llce_can_core_debugfs *core_debugfs;
	struct llce_can_core *can_core;
	u16 filter_addr;

	can_filters = container_of(blk, struct llce_can_filters_debugfs,
				   remove_exec);
	core_debugfs = container_of(can_filters, struct llce_can_core_debugfs,
				    can_filters);
	can_core = get_can_core(core_debugfs);

	filter_addr = can_filters->base_opts.filter_addr;

	return llce_remove_filter(can_core, filter_addr);
}

static int can_add_dest_func(void *data, u64 val)
{
	struct debugfs_cmd_block *blk = data;
	struct llce_can_can2can_routing_table route_dest;
	struct llce_can_dest_debugfs *can_dest;
	struct llce_can_core_debugfs *core_debugfs;
	struct llce_can_core *can_core;
	struct device *dev;
	struct llce_can_word0 word0;
	int ret;

	can_dest = container_of(blk, struct llce_can_dest_debugfs, add_exec);
	core_debugfs = container_of(can_dest, struct llce_can_core_debugfs,
				    can_dest);
	can_core = get_can_core(core_debugfs);
	dev = get_llce_dbgfs_device(core_debugfs);

	word0 = (struct llce_can_word0) {
		.id = can_dest->can_id_remap,
		.rtr = 0,
		.ide = is_can_extended_id(can_dest->can_id_remap),
	};

	route_dest = (struct llce_can_can2can_routing_table) {
		.can2can_routing_options = can_dest->routing_opts,
		.can_id_remap_value = llce_can_pack_word0(&word0),
		.dest_hw_ch_list_count = can_dest->ch_list.n_elements,
	};

	ret = validate_can_add_dest_params(dev, can_dest);
	if (ret)
		return ret;

	if (route_dest.dest_hw_ch_list_count >
	    ARRAY_SIZE(route_dest.dest_hw_ch_list)) {
		dev_err(dev, "Too many channels\n");
		return -ENOSPC;
	}

	memcpy(&route_dest.dest_hw_ch_list,
	       can_dest->ch_list.array,
	       sizeof(route_dest.dest_hw_ch_list[0]) *
	       route_dest.dest_hw_ch_list_count);

	ret = llce_add_can_dest(can_core, &route_dest, &can_dest->id);
	if (ret)
		dev_err(dev, "Failed to add a CAN2CAN route\n");

	return ret;
}

static int debugfs_create_dirs(struct device *dev,
			       struct debugfs_dir_create *dirs, size_t n_dirs)
{
	struct dentry *folder;
	struct dentry **ptr_folder;
	size_t i;

	for (i = 0; i < n_dirs; i++) {
		if (!dirs[i].dir)
			ptr_folder = &folder;
		else
			ptr_folder = dirs[i].dir;

		*ptr_folder = debugfs_create_dir(dirs[i].name, *dirs[i].parent);
		if (IS_ERR(*ptr_folder)) {
			dev_err(dev, "Failed to create %s directory\n",
				dirs[i].name);
			return PTR_ERR(*ptr_folder);
		}

		debugfs_create_readme(*ptr_folder, dirs[i].readme);
	}

	return 0;
}

static int debugfs_create_list(struct llce_can_core *can_core,
			       struct dentry *parent)
{
	struct dentry *list_file;

	list_file = debugfs_create_file(LIST_FILE, 0400, parent, can_core,
					&can_core_list_debugfs_fops);
	if (IS_ERR(list_file))
		return PTR_ERR(list_file);

	return 0;
}

static int create_can_dest_tree(struct llce_can_core_debugfs *root)
{
	struct dentry *add_folder, *dest_folder;
	struct device *dev = get_llce_dbgfs_device(root);
	struct llce_can_core *can_core = get_can_core(root);
	struct llce_can_dest_debugfs *can_dest = &root->can_dest;
	struct debugfs_dir_create folders[] = {
		{
			.parent = &root->root_dir,
			.name = LLCE_CAN_DEST_DIR,
			.dir = &dest_folder,
			.readme = &can_dest_readme
		},
		{
			.parent = &dest_folder,
			.name = ADD_CMD_DIR,
			.dir = &add_folder,
			.readme = &can_dest_add_cmd_readme,
		},
	};
	int ret;

	ret = debugfs_create_dirs(dev, folders, ARRAY_SIZE(folders));
	if (ret)
		return ret;

	can_dest->ch_list.array = &can_dest->dest_hw_ch_list[0];
	can_dest->ch_list.max_elems = ARRAY_SIZE(can_dest->dest_hw_ch_list);
	can_dest->ch_list.n_elements = 0;

	ret = debugfs_create_list(can_core, dest_folder);
	if (ret)
		return ret;

	debugfs_create_x32(DEST_ROUTING_OPS_FLD, 0600,
			   add_folder, &can_dest->routing_opts);

	debugfs_create_x32(DEST_CAN_ID_REMAP_FLD, 0600,
			   add_folder,
			   &can_dest->can_id_remap);

	ret = debugfs_create_u8_array(dev, DEST_CH_LIST_FLD, add_folder,
				      &can_dest->ch_list);
	if (ret)
		return ret;

	can_dest->add_exec.clb = can_add_dest_func;
	can_dest->add_exec.get_val = can_get_last_dest_id;

	ret = debugfs_create_cmd(dev, EXECUTE_FILE, add_folder,
				 &can_dest->add_exec);
	if (ret)
		return ret;

	return 0;
}

static void init_filters_choices(struct device *dev,
				 struct llce_can_filters_debugfs *can_filters)
{
	can_filters->status.choices = status_choices;
	can_filters->status.n_choices = ARRAY_SIZE(status_choices);
	can_filters->status.selected = &status_choices[0];

	can_filters->entry.choices = entry_choices;
	can_filters->entry.n_choices = ARRAY_SIZE(entry_choices);
	can_filters->entry.selected = &entry_choices[0];

	can_filters->mb_type.choices = mb_choices;
	can_filters->mb_type.n_choices = ARRAY_SIZE(mb_choices);
	can_filters->mb_type.selected = &mb_choices[0];

	can_filters->auth.choices = auth_choices;
	can_filters->auth.n_choices = ARRAY_SIZE(auth_choices);
	can_filters->auth.selected = &auth_choices[0];

	can_filters->host.choices = host_choices;
	can_filters->host.n_choices = ARRAY_SIZE(host_choices);
	can_filters->host.selected = &host_choices[0];

	can_filters->logging.choices = logging_choices;
	can_filters->logging.n_choices = ARRAY_SIZE(logging_choices);
	can_filters->logging.selected = &logging_choices[0];

	can_filters->custom_proc.choices = custom_choices;
	can_filters->custom_proc.n_choices = ARRAY_SIZE(custom_choices);
	can_filters->custom_proc.selected = &custom_choices[0];
}

static int create_filter_status_files(struct device *dev,
				      struct llce_can_filters_debugfs *can_filters,
				      struct dentry *parent)
{
	int ret;

	ret = debugfs_create_choice(dev, FILTERS_STATUS_FLD, parent,
				    &can_filters->status);
	if (ret)
		return ret;

	debugfs_create_u16(FILTERS_ADDR_FLD, 0600, parent,
			   &can_filters->base_opts.filter_addr);

	return 0;
}

static int create_rx_filter_files(struct device *dev,
				  struct llce_can_filters_debugfs *can_filters,
				  struct dentry *parent)
{
	struct {
		const char *name;
		struct debugfs_choice_block *blk;
	} choices[] = {
		{ .name = FILTERS_STATUS_FLD, .blk = &can_filters->status },
		{ .name = FILTERS_ENTRY_FLD, .blk = &can_filters->entry },
		{ .name = FILTERS_MB_TYPE_FLD, .blk = &can_filters->mb_type },
		{ .name = FILTERS_AUTH_FLD, .blk =  &can_filters->auth },
		{ .name = FILTERS_HOST_FLD, .blk =  &can_filters->host },
		{ .name = FILTERS_LOGGING_FLD, .blk =  &can_filters->logging },
		{ .name = FILTERS_CUSTOM_FLD, .blk =  &can_filters->custom_proc },
	};
	int ret;
	size_t i;

	debugfs_create_x32(FILTERS_CAN_ID_FLD, 0600,  parent,
			   &can_filters->base_opts.id_mask);

	debugfs_create_x32(FILTERS_MSG_ID_FLD, 0600, parent,
			   &can_filters->base_opts.message_id);

	debugfs_create_x16(FILTERS_UTAG_FLD, 0600, parent,
			   &can_filters->base_opts.filter_id);

	debugfs_create_x16(FILTERS_MB_COUNT_FLD, 0600, parent,
			   &can_filters->base_opts.mb_count);

	debugfs_create_u8(FILTERS_HW_CTRL_FLD, 0600, parent,
			  &can_filters->hw_ctrl);

	debugfs_create_u8(FILTERS_CAN_ROUTE_FLD, 0600, parent,
			  &can_filters->adv_opts.can2can_routing_table_idx);

	can_filters->adv_opts.can2eth_routing_table_idx =
	    LLCE_CAN_ADVANCED_FILTER_NOT_USED;
	can_filters->adv_opts.other_routing_table_idx =
	    LLCE_CAN_ADVANCED_FILTER_NOT_USED;

	for (i = 0; i < ARRAY_SIZE(choices); i++) {
		ret = debugfs_create_choice(dev, choices[i].name, parent,
					    choices[i].blk);
		if (ret)
			return ret;
	}

	return 0;
}

static int create_can_filters_tree(struct llce_can_core_debugfs *root)
{
	struct dentry *filters_folder, *add_folder, *remove_folder,
		      *set_status_folder;
	struct device *dev = get_llce_dbgfs_device(root);
	struct llce_can_core *can_core = get_can_core(root);
	struct llce_can_filters_debugfs *can_filters = &root->can_filters;
	struct debugfs_dir_create folders[] = {
		{
			.parent = &root->root_dir, .name = LLCE_CAN_FILTERS_DIR,
			.dir = &filters_folder, .readme = &can_filters_readme,
		},
		{
			.parent = &filters_folder, .name = ADD_CMD_DIR,
			.dir = &add_folder, .readme = &can_filters_add_readme,
		},
		{
			.parent = &filters_folder, .name = SET_STATUS_CMD_DIR,
			.dir = &set_status_folder,
			.readme = &can_filters_set_status_readme,
		},
		{
			.parent = &filters_folder, .name = REMOVE_CMD_DIR,
			.dir = &remove_folder,
			.readme = &can_filters_remove_readme,
		},
	};
	int ret;

	init_filters_choices(dev, can_filters);

	ret = debugfs_create_dirs(dev, folders, ARRAY_SIZE(folders));
	if (ret)
		return ret;

	ret = debugfs_create_list(can_core, filters_folder);
	if (ret)
		return ret;

	/* Add */
	ret = create_rx_filter_files(dev, can_filters, add_folder);
	if (ret)
		return ret;

	can_filters->add_exec.clb = can_add_filter_func;
	can_filters->add_exec.get_val = can_get_last_filter_id;

	ret = debugfs_create_cmd(dev, EXECUTE_FILE, add_folder,
				 &can_filters->add_exec);
	if (ret)
		return ret;

	/* Set status */
	ret = create_filter_status_files(dev, can_filters, set_status_folder);
	if (ret)
		return ret;

	can_filters->set_status_exec.clb = can_set_filter_status_func;
	can_filters->set_status_exec.get_val = NULL;

	ret = debugfs_create_cmd(dev, EXECUTE_FILE, set_status_folder,
				 &can_filters->set_status_exec);
	if (ret)
		return ret;

	/* Remove */
	debugfs_create_u16(FILTERS_ADDR_FLD, 0600, remove_folder,
			   &can_filters->base_opts.filter_addr);

	can_filters->remove_exec.clb = can_remove_filter_func;
	can_filters->remove_exec.get_val = NULL;

	ret = debugfs_create_cmd(dev, EXECUTE_FILE, remove_folder,
				 &can_filters->remove_exec);
	if (ret)
		return ret;

	return 0;
}

int llce_create_debugfs(struct llce_can_core_debugfs *root)
{
	struct device *dev = get_llce_dbgfs_device(root);
	int ret;

	root->root_dir = debugfs_create_dir(LLCE_CAN_CORE_DIR, NULL);
	if (IS_ERR(root->root_dir)) {
		dev_err(dev, "Failed to create %s directory\n",
			LLCE_CAN_CORE_DIR);
		return PTR_ERR(root->root_dir);
	}

	debugfs_create_readme(root->root_dir, &root_readme);

	ret = create_can_dest_tree(root);
	if (ret)
		goto remove_debugfs_root;

	ret = create_can_filters_tree(root);
	if (ret)
		goto remove_debugfs_root;

remove_debugfs_root:
	if (ret)
		llce_remove_debugs(root);

	return ret;
}

static char *root_readme = "\n"
"This folder contains various configuration files of the LLCE CAN subsystem\n"
"that do not have an alternative in classic SocketCAN tools. Each functionality\n"
"is placed into a subfolder to isolate the context and ease navigation. README\n"
"files can be found in each directory to provide more details about the context\n"
"and usage of the offered settings.\n"
"\n"
"This is a summary of the subfolders:\n"
"- " LLCE_CAN_DEST_DIR "\n"
"    Manages CAN2CAN destinations used for frames accepted by LLCE CAN advanced\n"
"    filters.\n"
"- " LLCE_CAN_FILTERS_DIR "\n"
"    Manages CAN2CAN filters.\n"
"\n"
"Some subdirectories are responsible for various LLCE CAN commands. These"
"sub-folders contain field files whose format may vary from file to file. The\n"
"content format can be found by inspecting the README in the same folder as the\n"
"field or by checking the file contents.\n";

static char *can_dest_readme = "\n"
"The LLCE CAN destinations are rules to be used for frames accepted by LLCE CAN\n"
"advanced filters. These rules can be either added to the firmware's table or\n"
"inspected. The formation of a new destination is managed by files from '"
ADD_CMD_DIR "'\n"
"subfolder. After successfully creating the destination, it may be attached to\n"
"one or more advanced filters. Finally, the listing of all added rules can be\n"
"performed by inspecting the content of the '" LIST_FILE "' file:\n"
"    cat " LLCE_CAN_DEST_PATH(LIST_FILE) "\n";

static char *can_dest_add_cmd_readme = "\n"
"This folder contains files designated for LLCE CAN destinations creation.\n"
"\n"
"The '" ADD_CMD_DIR "' command inserts a new entry into the LLCE firmware destinations table\n"
"for CAN2CAN routes. It is a wrapper for the LLCE firmware's\n"
"LLCE_CAN_CMD_CREATE_AF_DESTINATION command and requires three parameters:\n"
"    - '" DEST_ROUTING_OPS_FLD "'\n"
"    - '" DEST_CAN_ID_REMAP_FLD "'\n"
"    - '" DEST_CH_LIST_FLD "'\n"
"\n"
"Each parameter will be written into its dedicated file, part of this folder.\n"
"The files mentioned above keep the state of a destination that was not submitted\n"
"yet. The content of the field files will be interpreted and the command will be\n"
"sent to the firmware after a write into '" EXECUTE_FILE "'.\n"
"\n"
"An example showing its use:\n"
"    echo '0x1' > " ADD_DEST_CMD_PATH(DEST_ROUTING_OPS_FLD) "\n"
"    echo '1 2 3 4' > " ADD_DEST_CMD_PATH(DEST_CH_LIST_FLD) "\n"
"    echo '1' > " ADD_DEST_CMD_PATH(EXECUTE_FILE) "\n"
"\n"
"    The firmware returns a rule index in case the destination was successfully\n"
"    added. It can be inspected by dumping the content of the '" EXECUTE_FILE "' file:\n"
"    cat "  ADD_DEST_CMD_PATH(EXECUTE_FILE) "\n"
"\n"
"More details about each parameter can be found in the LLCE Firmware User Guide.\n"
"Here are some references that can be used as a starting point.\n"
"\n"
"......................................................\n"
":  Parameter   :          LLCE FW reference          :\n"
":..............:.....................................:\n"
": " DEST_ROUTING_OPS_FLD " : u32Can2CanRoutingOptions            :\n"
": " DEST_CAN_ID_REMAP_FLD " : u32CanIdRemapValue                  :\n"
": " DEST_CH_LIST_FLD "      : u8DestHwChList, u8DestHwChListCount :\n"
":..............:.....................................:\n"
"\n"
"NOTE:\n"
"    LLCE Firmware expects u32CanIdRemapValue to be sent in WORD0 format.\n"
"    Linux takes care of converting '" DEST_CAN_ID_REMAP_FLD "' into the requested\n"
"    format so the user should write the remap ID value, without any changes applied.\n";

static char *can_filters_readme = "\n"
"The LLCE CAN filters are used to direct incoming CAN frames. This traffic\n"
"can be either sent to HIF (termination traffic) or routed to another LLCE CAN\n"
"interface (CAN2CAN), or routed to an ethernet interface (CAN2ETH).\n"
"\n"
"This folder contains files and subdirectories allowing to create, remove, change\n"
"the status of a filter and list all the registered filters.\n"
"\n"
"......................................................\n"
":  Folder/File :                  Role               :\n"
":..............:.....................................:\n"
": " ADD_CMD_DIR "          : Adds a new filter                   :\n"
": " REMOVE_CMD_DIR "       : Removes a registered filter         :\n"
": " SET_STATUS_CMD_DIR "   : Changes the status of a filter      :\n"
":..............:.....................................:\n";

static char *can_filters_add_readme = "\n"
"An LLCE CAN filter can be created using ADD command. This folder contains\n"
"field files used to send LLCE_CAN_CMD_SET(ADVANCED)FILTER commands to LLCE\n"
"firmware. The selection between basic and advanced filters is performed based\n"
"on enabled/filled fields. Additionally, a filter can be added in disabled state\n"
"if writing 'disabled' in '" FILTERS_STATUS_FLD "' file. The content of the field "
"files will be\n"
"interpreted and the command will be sent to the firmware after a write into\n"
"'" EXECUTE_FILE "'.\n"
"\n"
"......................................................\n"
":  Parameter   :          LLCE FW reference          :\n"
":..............:.....................................:\n"
": " FILTERS_CAN_ID_FLD "  : uIdMask                             :\n"
": " FILTERS_MSG_ID_FLD "       : uMessageId                          :\n"
": " FILTERS_UTAG_FLD "          : u16FilterId                         :\n"
": " FILTERS_MB_COUNT_FLD "     : u16MbCount                          :\n"
": " FILTERS_ENTRY_FLD "        : eEntryType                          :\n"
": " FILTERS_MB_TYPE_FLD "      : eFilterMbLength                     :\n"
": " FILTERS_AUTH_FLD "     : eCanAuthenticationFeature           :\n"
": " FILTERS_HOST_FLD "     : eHostReceive                        :\n"
": " FILTERS_LOGGING_FLD "      : eCanLoggingFeature                  :\n"
": " FILTERS_CUSTOM_FLD "   : eCanCustomProcessing                :\n"
": " FILTERS_CAN_ROUTE_FLD "     : u8Can2CanRoutingTableIdx            :\n"
":..............:.....................................:\n"
": " FILTERS_HW_CTRL_FLD "      : Hardware controller ID (0-15)       :\n"
": " FILTERS_STATUS_FLD "       : Status of the added filter          :\n"
":              : (enabled/disabled)                  :\n"
":..............:.....................................:\n"
"\n"
"Some field files may contain a value from a predefined list. The following\n"
"tables list all possible values for this field type.\n"
"\n"
"......................................................\n"
":   '" FILTERS_ENTRY_FLD "'    :                                     :\n"
":    values    :          LLCE FW reference          :\n"
":..............:.....................................:\n"
": " ENTRY_EXACT_MATCH "        : LLCE_CAN_ENTRY_EXACT_MATCH          :\n"
": " ENTRY_MASK_MATCH "         : LLCE_CAN_ENTRY_CFG_MASKED           :\n"
": " ENTRY_RANGE_MATCH "        : LLCE_CAN_ENTRY_CFG_RANGED           :\n"
":..............:.....................................:\n"
"\n"
"......................................................\n"
":  '" FILTERS_MB_TYPE_FLD "'   :                                     :\n"
":    values    :          LLCE FW reference          :\n"
":..............:.....................................:\n"
": " MB_TYPE_LONG "         : USE_LONG_MB                         :\n"
": " MB_TYPE_SHORT "        : USE_SHORT_MB                        :\n"
":..............:.....................................:\n"
"\n"
"......................................................\n"
":  '" FILTERS_AUTH_FLD "'  :                                     :\n"
":    values    :          LLCE FW reference          :\n"
":..............:.....................................:\n"
": " AUTH_DISABLED "      : LLCE_AF_AUTHENTICATION_DISABLED     :\n"
": " AUTH_ENABLED "     : LLCE_AF_AUTHENTICATION_ENABLED      :\n"
": " AUTH_NOT_SUPP "     : LLCE_AF_AUTHENTICATION_NOT_SUPPORTED:\n"
":..............:.....................................:\n"
"\n"
"......................................................\n"
":  '" FILTERS_HOST_FLD "'  :                                     :\n"
":    values    :          LLCE FW reference          :\n"
":..............:.....................................:\n"
": " HOST_RCV_DISABLED "     : LLCE_AF_HOSTRECEIVE_DISABLED        :\n"
": " HOST_RCV_ENABLED "      : LLCE_AF_HOSTRECEIVE_ENABLED         :\n"
":..............:.....................................:\n"
"\n"
"......................................................\n"
":   '" FILTERS_LOGGING_FLD "'  :                                     :\n"
":    values    :          LLCE FW reference          :\n"
":..............:.....................................:\n"
": " LOGGING_DISABLED "     : LLCE_AF_LOGGING_DISABLED            :\n"
": " LOGGING_ENABLED "      : LLCE_AF_LOGGING_ENABLED             :\n"
":..............:.....................................:\n"
"\n"
"......................................................\n"
": '" FILTERS_CUSTOM_FLD "' :                                     :\n"
":    values    :          LLCE FW reference          :\n"
":..............:.....................................:\n"
": " CPROCESS_DISABLED "     : LLCE_AF_CUSTOMPROCESSING_DISABLED   :\n"
": " CPROCESS_ENABLED "      : LLCE_AF_CUSTOMPROCESSING_ENABLED    :\n"
":..............:.....................................:\n"
"\n"
"......................................................\n"
":   '" FILTERS_STATUS_FLD "'   :                                     :\n"
":    values    :          Short description          :\n"
":..............:.....................................:\n"
": " STATUS_DISABLED "     : Adds a disabled filter              :\n"
": " STATUS_ENABLED "      : Adds the filter enabled             :\n"
":..............:.....................................:\n"
"\n"
"An example showing how to add a filter to LLCE CAN1 interface, which routes\n"
"the messages with CAN ID 0x77 to LLCE CAN destination 0.\n"
"\n"
"    echo 0 >  " ADD_FILTER_CMD_PATH(FILTERS_CAN_ROUTE_FLD) "\n"
"    echo 0x0 > " ADD_FILTER_CMD_PATH(FILTERS_CAN_ID_FLD) "\n"
"    echo " ENTRY_EXACT_MATCH " > " ADD_FILTER_CMD_PATH(FILTERS_ENTRY_FLD) "\n"
"    echo 1 > " ADD_FILTER_CMD_PATH(FILTERS_HW_CTRL_FLD) "\n"
"    echo 0x0 > " ADD_FILTER_CMD_PATH(FILTERS_UTAG_FLD) "\n"
"    echo 0x10 > " ADD_FILTER_CMD_PATH(FILTERS_MB_COUNT_FLD) "\n"
"    echo long > " ADD_FILTER_CMD_PATH(FILTERS_MB_TYPE_FLD) "\n"
"    echo 0x77 > " ADD_FILTER_CMD_PATH(FILTERS_MSG_ID_FLD) "\n"
"    echo enabled > " ADD_FILTER_CMD_PATH(FILTERS_STATUS_FLD) "\n"
"    echo disabled > " ADD_FILTER_CMD_PATH(FILTERS_AUTH_FLD) "\n"
"    echo disabled > " ADD_FILTER_CMD_PATH(FILTERS_CUSTOM_FLD) "\n"
"    echo disabled > " ADD_FILTER_CMD_PATH(FILTERS_HOST_FLD) "\n"
"    echo disabled > " ADD_FILTER_CMD_PATH(FILTERS_LOGGING_FLD) "\n"
"    echo 1 > " ADD_FILTER_CMD_PATH(EXECUTE_FILE) "\n"
"\n"
"NOTES:\n"
"   1. A filter can only be added to an LLCE CAN initialized interface. The\n"
"      operation will fail if the target interface is not under Linux control\n"
"      or is not initialized.\n"
"   2. Llce_Can_ReceiveFilterType.u8RxDestInterface field is populated by the\n"
"      driver based on HW FIFO mapping on controllers.\n"
"   3. LLCE Firmware expects a value of uMessageId in which bit 30 (IDE) is set\n"
"      if an extended ID is used. The IDE bit enablement is handled by the Linux\n"
"      LLCE drivers, so " FILTERS_MSG_ID_FLD " should contain the message ID, without any\n"
"      changes applied.\n";

static char *can_filters_remove_readme = "\n"
"This command folder should be used to remove an LLCE CAN filter that was added\n"
"using the '" ADD_CMD_DIR "' command. The removal operation is performed in two steps.\n"
"The address of the filter to be removed is placed in '" FILTERS_ADDR_FLD "' field file,\n"
"followed by a write into '" EXECUTE_FILE "' file. For example:\n"
"    echo 512 > " RM_FILTER_CMD_PATH(FILTERS_ADDR_FLD) "\n"
"    echo 1 > " RM_FILTER_CMD_PATH(EXECUTE_FILE) "\n"
"\n";

static char *can_filters_set_status_readme = "\n"
"This command folder should be used to enable or disable an existing LLCE CAN\n"
"filter. Before executing the command, two fields must be filled: '" FILTERS_ADDR_FLD
"' and\n"
"'" FILTERS_STATUS_FLD "'. The former refers to the address of the filter and"
"the latter to the\n"
"state, which can be either 'enabled' or 'disabled'.\n"
"\n"
"An example showing its use:\n"
"    echo 513 > " STS_FILTER_CMD_PATH(FILTERS_ADDR_FLD) "\n"
"    echo enabled > " STS_FILTER_CMD_PATH(FILTERS_STATUS_FLD) "\n"
"    echo 1 > " STS_FILTER_CMD_PATH(EXECUTE_FILE) "\n"
"\n";
